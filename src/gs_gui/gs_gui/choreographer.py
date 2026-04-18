#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
表演编排器 — 自然语言 → 集群表演 XML

核心流程：
1. 用户输入自然语言描述（如 "3艘USV画五角星，半径6米，红色灯光"）
2. LLM 解析为结构化编排计划 (JSON)
3. shape_library 计算精确坐标
4. 生成标准 XML 路径文件，可直接由 ClusterTaskManager 执行

支持的 LLM 后端：
- 本地 Ollama (默认)
- OpenAI 兼容 API（vLLM / llama.cpp / LM Studio）
"""

import json
import logging
import math
import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field, asdict
from typing import List, Dict, Tuple, Optional, Any
from xml.dom import minidom

from gs_gui.shape_library import (
    SHAPE_REGISTRY, FORMATION_REGISTRY,
    Waypoint, StepAssignment,
    circle, ellipse, star, polygon, figure_eight, heart,
    zigzag, spiral, line, wave,
    formation_circle, formation_star, formation_line, formation_v, formation_grid,
    distribute_paths_to_usvs, formation_to_steps,
    rotate_points, scale_points, translate_points,
)

logger = logging.getLogger(__name__)


# ────────────────────── 编排计划数据结构 ──────────────────────

@dataclass
class SegmentPlan:
    """单个编排段落。"""
    type: str                    # "path_trace" | "formation" | "maneuver"
    shape: str = ""              # SHAPE_REGISTRY 或 FORMATION_REGISTRY 中的 key
    params: Dict[str, Any] = field(default_factory=dict)
    velocity: float = 0.5
    nav_mode: str = "async"
    led: str = "0"
    sync_timeout: float = 10.0
    yaw_mode: str = "auto"
    maneuver_type: Optional[str] = None
    maneuver_circles: Optional[float] = None
    maneuver_direction: Optional[str] = None
    description: str = ""


@dataclass
class ChoreographyPlan:
    """完整编排计划。"""
    title: str = "未命名表演"
    usv_ids: List[str] = field(default_factory=lambda: ["usv_01", "usv_02", "usv_03"])
    segments: List[SegmentPlan] = field(default_factory=list)


# ────────────────────── 系统提示词 ──────────────────────

SYSTEM_PROMPT = """你是一个水面无人船（USV）集群表演编排助手。用户会用自然语言描述想要的表演动作，你需要将其转换为结构化的 JSON 编排计划。

## 可用图形 (shape)

路径图形（USV沿路径移动）:
- "circle": 圆形。参数: center=[x,y], radius=float, num_points=int
- "ellipse": 椭圆。参数: center=[x,y], a=float(x半轴), b=float(y半轴), num_points=int
- "star": 五角星。参数: center=[x,y], radius=float, num_points=int(角数)
- "polygon": 正多边形。参数: center=[x,y], radius=float, num_sides=int
- "figure_eight": 8字形。参数: center=[x,y], radius=float, num_points=int
- "heart": 心形。参数: center=[x,y], size=float, num_points=int
- "zigzag": Z字形。参数: start=[x,y], end=[x,y], amplitude=float, num_zigs=int
- "spiral": 螺旋。参数: center=[x,y], start_radius=float, end_radius=float, num_turns=float
- "line": 直线。参数: start=[x,y], end=[x,y], num_points=int
- "wave": 波浪。参数: start=[x,y], end=[x,y], amplitude=float, wavelength=float

队形图形（多USV同时到达组成图案）:
- "circle": 圆形队列。参数: center=[x,y], radius=float
- "star": 星形队列。参数: center=[x,y], radius=float, num_points=int
- "line": 一字排列。参数: center=[x,y], spacing=float, heading=float(弧度)
- "v_shape": V字形队列。参数: center=[x,y], spacing_along=float, spacing_cross=float, heading=float
- "grid": 网格排列。参数: center=[x,y], spacing=float, cols=int, rows=int

## LED 灯效
- "0": 关闭
- "rainbow": 彩虹色
- "red", "green", "blue", "white", "yellow", "purple": 纯色
- "breath_red", "breath_blue", "breath_green": 呼吸灯

## 导航模式 (nav_mode)
- "async": 异步，各USV独立导航不等待
- "sync": 同步，所有USV到达后才进行下一步
- "rotate": 到达后原地旋转
- "terminal": 终止导航

## 机动动作 (maneuver)
- "rotate": 原地旋转。circles=圈数, direction="clockwise"/"ccw"
- "spin": 高速旋转。circles=圈数, direction="clockwise"/"ccw"

## 输出格式

严格输出以下 JSON 格式，不要输出其他内容：

```json
{
  "title": "表演标题",
  "usv_ids": ["usv_01", "usv_02", "usv_03"],
  "segments": [
    {
      "type": "path_trace",
      "shape": "star",
      "params": {"center": [0, 0], "radius": 6.0, "num_points": 5},
      "velocity": 0.5,
      "nav_mode": "async",
      "led": "rainbow",
      "description": "三艘船各自沿五角星路径巡航"
    },
    {
      "type": "formation",
      "shape": "circle",
      "params": {"center": [0, 0], "radius": 4.0},
      "velocity": 0.5,
      "nav_mode": "sync",
      "led": "red",
      "sync_timeout": 15.0,
      "description": "集合成圆形队列"
    },
    {
      "type": "maneuver",
      "maneuver_type": "rotate",
      "maneuver_circles": 2,
      "maneuver_direction": "clockwise",
      "nav_mode": "sync",
      "description": "原地旋转两圈"
    }
  ]
}
```

## 注意事项
1. 坐标单位是米，原点 (0,0) 是表演区域中心
2. 速度单位 m/s，无人船安全速度范围 0.2-1.0
3. 半径建议不超过 15m
4. 选择合理的点数让轨迹平滑但不过于密集（建议每米1-2个点）
5. path_trace 类型中，多艘USV可以跟踪同一形状但起始偏移不同，或跟踪不同形状
6. 参数中的 center/start/end 用 [x, y] 列表格式
"""


# ────────────────────── LLM 客户端 ──────────────────────

class LLMClient:
    """OpenAI 兼容 API 客户端，支持 Ollama / vLLM / LM Studio 等。"""

    def __init__(self, base_url: str = "http://localhost:11434/v1",
                 model: str = "gemma3:27b",
                 api_key: str = "ollama",
                 timeout: float = 60.0):
        self.base_url = base_url.rstrip("/")
        self.model = model
        self.api_key = api_key
        self.timeout = timeout

    def chat(self, system_prompt: str, user_message: str) -> str:
        """发送聊天请求并返回 LLM 响应文本。"""
        import urllib.request
        import urllib.error

        url = f"{self.base_url}/chat/completions"
        payload = {
            "model": self.model,
            "messages": [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_message}
            ],
            "temperature": 0.3,
            "max_tokens": 4096,
        }

        data = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(
            url,
            data=data,
            headers={
                "Content-Type": "application/json",
                "Authorization": f"Bearer {self.api_key}"
            },
            method="POST"
        )

        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                body = json.loads(resp.read().decode("utf-8"))
                return body["choices"][0]["message"]["content"]
        except urllib.error.URLError as e:
            raise ConnectionError(f"LLM 服务连接失败 ({self.base_url}): {e}")
        except (KeyError, IndexError, json.JSONDecodeError) as e:
            raise ValueError(f"LLM 响应格式错误: {e}")


# ────────────────────── 计划解析 ──────────────────────

def parse_plan_json(raw: str) -> ChoreographyPlan:
    """从 LLM 输出中提取并解析 JSON 编排计划。"""
    # 尝试提取 JSON 块
    text = raw.strip()
    # 处理 markdown 代码块
    if "```json" in text:
        start = text.index("```json") + 7
        end = text.index("```", start)
        text = text[start:end].strip()
    elif "```" in text:
        start = text.index("```") + 3
        end = text.index("```", start)
        text = text[start:end].strip()

    # 尝试找到 JSON 对象
    brace_start = text.find("{")
    brace_end = text.rfind("}")
    if brace_start >= 0 and brace_end > brace_start:
        text = text[brace_start:brace_end + 1]

    data = json.loads(text)

    plan = ChoreographyPlan(
        title=data.get("title", "未命名表演"),
        usv_ids=data.get("usv_ids", ["usv_01", "usv_02", "usv_03"]),
    )

    for seg_data in data.get("segments", []):
        # 标准化 params 中的 list → tuple
        params = seg_data.get("params", {})
        for key in ("center", "start", "end"):
            if key in params and isinstance(params[key], list):
                params[key] = tuple(params[key])

        segment = SegmentPlan(
            type=seg_data.get("type", "path_trace"),
            shape=seg_data.get("shape", ""),
            params=params,
            velocity=float(seg_data.get("velocity", 0.5)),
            nav_mode=seg_data.get("nav_mode", "async"),
            led=str(seg_data.get("led", "0")),
            sync_timeout=float(seg_data.get("sync_timeout", 10.0)),
            yaw_mode=seg_data.get("yaw_mode", "auto"),
            maneuver_type=seg_data.get("maneuver_type"),
            maneuver_circles=seg_data.get("maneuver_circles"),
            maneuver_direction=seg_data.get("maneuver_direction"),
            description=seg_data.get("description", ""),
        )
        plan.segments.append(segment)

    return plan


# ────────────────────── 坐标生成 ──────────────────────

def _generate_path_trace_steps(
    segment: SegmentPlan,
    usv_ids: List[str]
) -> List[StepAssignment]:
    """将 path_trace 段落转为 StepAssignment 列表。"""
    shape_name = segment.shape
    if shape_name not in SHAPE_REGISTRY:
        logger.warning(f"未知形状 '{shape_name}'，跳过")
        return []

    shape_info = SHAPE_REGISTRY[shape_name]
    func = shape_info["func"]

    # 合并默认参数和用户参数
    params = dict(shape_info["params"])
    params.update(segment.params)

    # 生成基础路径
    base_path = func(**params)

    if not base_path:
        return []

    num_usvs = len(usv_ids)
    num_points = len(base_path)

    # 为每艘 USV 生成偏移的路径（在同一条路径上起始点偏移）
    paths = []
    for i in range(num_usvs):
        offset = int(num_points * i / num_usvs)
        path = base_path[offset:] + base_path[:offset]
        paths.append(path)

    steps = distribute_paths_to_usvs(paths, usv_ids)

    # 设置每步的参数
    for step in steps:
        step.nav_mode = segment.nav_mode
        step.velocity = segment.velocity
        step.led = segment.led
        step.yaw_mode = segment.yaw_mode

    return steps


def _generate_formation_steps(
    segment: SegmentPlan,
    usv_ids: List[str]
) -> List[StepAssignment]:
    """将 formation 段落转为 StepAssignment 列表。"""
    shape_name = segment.shape
    if shape_name not in FORMATION_REGISTRY:
        logger.warning(f"未知队形 '{shape_name}'，跳过")
        return []

    func = FORMATION_REGISTRY[shape_name]["func"]
    params = dict(segment.params)
    params["count"] = len(usv_ids)

    points = func(**params)

    formation_list = [points]  # 单步队形
    return formation_to_steps(
        formation_list, usv_ids,
        nav_mode=segment.nav_mode,
        velocity=segment.velocity,
        led=segment.led,
        sync_timeout=segment.sync_timeout,
    )


def _generate_maneuver_steps(
    segment: SegmentPlan,
    usv_ids: List[str]
) -> List[StepAssignment]:
    """将 maneuver 段落转为 StepAssignment。"""
    # 原地机动不需要改变坐标，使用 (0, 0) 占位
    # 实际坐标会在发送时使用当前位置
    positions = {uid: Waypoint(0.0, 0.0) for uid in usv_ids}

    circles = segment.maneuver_circles or 1
    if segment.maneuver_direction == "ccw":
        circles = -circles

    return [StepAssignment(
        positions=positions,
        nav_mode=segment.nav_mode,
        velocity=segment.velocity,
        led=segment.led,
        sync_timeout=segment.sync_timeout,
        maneuver_type=segment.maneuver_type,
        maneuver_circles=circles,
        maneuver_direction=segment.maneuver_direction,
    )]


def plan_to_steps(plan: ChoreographyPlan) -> List[StepAssignment]:
    """将编排计划转换为完整的步骤列表。"""
    all_steps = []
    for segment in plan.segments:
        if segment.type == "path_trace":
            steps = _generate_path_trace_steps(segment, plan.usv_ids)
        elif segment.type == "formation":
            steps = _generate_formation_steps(segment, plan.usv_ids)
        elif segment.type == "maneuver":
            steps = _generate_maneuver_steps(segment, plan.usv_ids)
        else:
            logger.warning(f"未知段落类型 '{segment.type}'，跳过")
            continue
        all_steps.extend(steps)
    return all_steps


# ────────────────────── XML 生成 ──────────────────────

def steps_to_xml(steps: List[StepAssignment]) -> str:
    """将步骤列表转换为标准集群路径 XML 字符串。"""
    root = ET.Element("cluster", type="formation")

    for idx, step in enumerate(steps, 1):
        step_elem = ET.SubElement(root, "step",
                                  number=str(idx),
                                  nav_mode=step.nav_mode)
        if step.nav_mode == "sync":
            step_elem.set("sync_timeout", f"{step.sync_timeout:.1f}")

        usvs_elem = ET.SubElement(step_elem, "usvs")

        for usv_id, wp in sorted(step.positions.items()):
            usv_elem = ET.SubElement(usvs_elem, "usv", led=step.led)

            id_elem = ET.SubElement(usv_elem, "usv_id")
            id_elem.text = usv_id

            pos_elem = ET.SubElement(usv_elem, "position")
            x_elem = ET.SubElement(pos_elem, "x")
            x_elem.text = f"{wp.x:.3f}"
            y_elem = ET.SubElement(pos_elem, "y")
            y_elem.text = f"{wp.y:.3f}"
            z_elem = ET.SubElement(pos_elem, "z")
            z_elem.text = f"{wp.z:.3f}"

            yaw_elem = ET.SubElement(usv_elem, "yaw", mode=step.yaw_mode)
            if step.yaw_value is not None:
                val_elem = ET.SubElement(yaw_elem, "value")
                val_elem.text = f"{step.yaw_value:.1f}"

            vel_elem = ET.SubElement(usv_elem, "velocity")
            val_elem = ET.SubElement(vel_elem, "value")
            val_elem.text = f"{step.velocity:.3f}"

            if step.maneuver_type:
                maneuver_attrs = {"type": step.maneuver_type}
                if step.maneuver_circles is not None:
                    maneuver_attrs["circles"] = str(abs(step.maneuver_circles))
                if step.maneuver_direction:
                    maneuver_attrs["direction"] = step.maneuver_direction
                ET.SubElement(usv_elem, "maneuver", **maneuver_attrs)

    # 格式化为带缩进的 XML
    rough_string = ET.tostring(root, encoding="unicode")
    parsed = minidom.parseString(rough_string)
    pretty = parsed.toprettyxml(indent="  ", encoding=None)
    # 去掉 minidom 添加的 XML 声明
    lines = pretty.split("\n")
    if lines and lines[0].startswith("<?xml"):
        lines[0] = '<?xml version="1.0" encoding="utf-8"?>'
    return "\n".join(ln for ln in lines if ln.strip())


def save_xml(xml_string: str, filepath: str) -> str:
    """保存 XML 到文件。"""
    os.makedirs(os.path.dirname(filepath) or ".", exist_ok=True)
    with open(filepath, "w", encoding="utf-8") as f:
        f.write(xml_string)
    return filepath


# ────────────────────── 主编排器 ──────────────────────

class Choreographer:
    """表演编排器 — 连接自然语言输入和 XML 输出的桥梁。"""

    def __init__(self, llm_base_url: str = "http://localhost:11434/v1",
                 llm_model: str = "gemma3:27b",
                 llm_api_key: str = "ollama"):
        self.llm = LLMClient(
            base_url=llm_base_url,
            model=llm_model,
            api_key=llm_api_key,
        )
        self._last_plan: Optional[ChoreographyPlan] = None
        self._last_xml: Optional[str] = None

    @property
    def last_plan(self) -> Optional[ChoreographyPlan]:
        return self._last_plan

    @property
    def last_xml(self) -> Optional[str]:
        return self._last_xml

    def generate_from_text(self, user_input: str,
                           usv_ids: Optional[List[str]] = None) -> str:
        """
        自然语言 → XML 全流程。

        Args:
            user_input: 用户的自然语言描述
            usv_ids: 可用的 USV ID 列表，会追加到提示词中

        Returns:
            生成的 XML 字符串
        """
        # 补充 USV 信息到用户输入
        enhanced_input = user_input
        if usv_ids:
            enhanced_input += f"\n\n可用的USV: {', '.join(usv_ids)}"

        # LLM 生成编排计划
        raw_response = self.llm.chat(SYSTEM_PROMPT, enhanced_input)
        logger.info(f"LLM 原始响应:\n{raw_response}")

        # 解析 JSON 计划
        plan = parse_plan_json(raw_response)

        # 如果用户指定了 USV IDs，覆盖 LLM 的选择
        if usv_ids:
            plan.usv_ids = usv_ids

        self._last_plan = plan

        # 计算坐标并生成 XML
        steps = plan_to_steps(plan)
        if not steps:
            raise ValueError("编排计划未生成任何有效步骤")

        xml_str = steps_to_xml(steps)
        self._last_xml = xml_str
        return xml_str

    def generate_from_plan(self, plan: ChoreographyPlan) -> str:
        """从编排计划直接生成 XML（跳过 LLM）。"""
        self._last_plan = plan
        steps = plan_to_steps(plan)
        if not steps:
            raise ValueError("编排计划未生成任何有效步骤")
        xml_str = steps_to_xml(steps)
        self._last_xml = xml_str
        return xml_str

    def quick_shape(self, shape: str, usv_ids: List[str],
                    **kwargs) -> str:
        """快速生成单个形状的 XML（不经过 LLM）。"""
        plan = ChoreographyPlan(
            title=f"{shape} 表演",
            usv_ids=usv_ids,
            segments=[SegmentPlan(
                type="path_trace",
                shape=shape,
                params=kwargs,
            )]
        )
        return self.generate_from_plan(plan)

    def quick_formation(self, formation: str, usv_ids: List[str],
                        **kwargs) -> str:
        """快速生成单个队形的 XML（不经过 LLM）。"""
        plan = ChoreographyPlan(
            title=f"{formation} 队形",
            usv_ids=usv_ids,
            segments=[SegmentPlan(
                type="formation",
                shape=formation,
                params=kwargs,
                nav_mode="sync",
            )]
        )
        return self.generate_from_plan(plan)
