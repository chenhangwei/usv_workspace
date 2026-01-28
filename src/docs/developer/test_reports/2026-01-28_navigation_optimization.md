# USV 导航控制优化测试报告

| 项目 | 内容 |
|------|------|
| **文档编号** | TEST-2026-01-28-001 |
| **日期** | 2026年1月28日 |
| **版本** | v2.0 |
| **作者** | chenhangwei |
| **状态** | 待验证 |
| **更新时间** | 2026-01-28 |

---

## 1. 测试背景

### 1.1 测试目的

基于 2026年1月28日 的实船测试日志分析，对 USV 导航控制系统进行优化，解决以下问题：

1. 直线航行时走弧线（核心问题）
2. 拐弯时冲出航线（惯性过大）
3. 低速/静止时原地绕圈
4. 航行不够平滑

### 1.2 测试环境

| 项目 | 配置 |
|------|------|
| 硬件平台 | Raspberry Pi 5 + Pixhawk 飞控 |
| 通信协议 | MAVROS (MAVLink over UDP/Serial) |
| 控制模式 | 速度模式 (Velocity Mode) |
| 控制算法 | Pure Pursuit + Stanley 混合控制 |
| 日志文件 | `nav_log_20260128_085616.csv` |

### 1.3 日志分析结果

通过分析导航日志，发现以下关键问题：

| 问题 | 现象 | 根因分析 |
|------|------|----------|
| 走弧线 | 直线段呈现外凸圆弧 | Pure Pursuit 追航点而非航线，Stanley增益不足 |
| 冲出航线 | 急弯时轨迹偏离 | 无提前减速机制，惯性甩尾 |
| 原地绕圈 | 低速时反复打转 | GPS速度航向噪声大，无回退机制 |
| 航向混乱 | 侧风时偏离直线 | 未区分 Heading 与 Course |

---

## 2. 优化方案

### 2.1 修改文件清单

| 序号 | 文件路径 | 修改类型 | 状态 |
|------|----------|----------|------|
| 1 | `usv_control/usv_control/velocity_path_tracker.py` | 核心算法 | ✅ 已完成 |
| 2 | `usv_control/usv_control/velocity_controller_node.py` | ROS节点 | ✅ 已完成 |
| 3 | `usv_bringup/config/usv_params.yaml` | 参数配置 | ✅ 已完成 |

### 2.2 参数调整

修改文件：`usv_bringup/config/usv_params.yaml`

| 参数名 | 修改前 | 修改后 | 说明 |
|--------|--------|--------|------|
| `stanley_gain` | 2.0 | 3.5 | 加快横向误差纠偏速度 |
| `lookahead_distance` | 2.5 m | 1.5 m | 减少切角，走更直的线 |
| `goal_tolerance` | 0.8 m | 1.5 m | 防止终点附近反复调整 |
| `min_speed` | 0.05 m/s | 0.15 m/s | 提高最低速度避免停滞 |
| `min_lookahead` | 1.2 m | 1.0 m | 配合前视距离减小 |
| `cruise_speed` | 0.5 m/s | 0.4 m/s | 略降低提高精度 |
| `max_angular_velocity` | 0.5 rad/s | 0.4 rad/s | 减少急转 |
| `switch_tolerance` | 1.5 m | 2.0 m | 中间航点切换容差 |

### 2.3 代码修改详情

#### 2.3.1 数据结构扩展 (velocity_path_tracker.py)

**修改：扩展 `Pose2D` 类**

```python
@dataclass
class Pose2D:
    """2D 位姿"""
    x: float
    y: float
    yaw: float  # 船头朝向 (Heading), 弧度
    course: Optional[float] = None  # 航迹角 (Course over Ground), 弧度
    speed: Optional[float] = None   # 对地速度 (m/s)
```

**目的**：严格分离船头朝向 (Heading) 与实际运动方向 (Course)，为 L1 算法提供数据基础。

#### 2.3.2 L1 航向选择算法 (velocity_path_tracker.py)

**新增：`_get_control_heading()` 方法**

```python
def _get_control_heading(self, pose: Pose2D) -> float:
    """
    获取用于控制的航向 (Course vs Heading)
    
    借鉴 L1 导航算法：
    当有足够速度时，优先使用航迹角(Course)代替船头朝向(Heading)，
    以此抵抗风流干扰，实现"蟹行"走直线。
    """
    if (pose.course is not None and 
        pose.speed is not None and 
        pose.speed > 0.3):  # 0.3 m/s 阈值
        return pose.course
    return pose.yaw
```

**算法逻辑**：

```
┌─────────────────────────────────────────────────────┐
│  当前速度 > 0.3 m/s?                                │
│      ├── 是 → 使用 Course (GPS速度向量方向)         │
│      │        效果：自动抵消侧风/水流               │
│      └── 否 → 使用 Heading (磁罗盘/EKF)             │
│               效果：静止时稳定，无GPS噪声           │
└─────────────────────────────────────────────────────┘
```

#### 2.3.3 弯道自适应减速 (velocity_path_tracker.py)

**新增：`_calculate_corner_speed_limit()` 方法**

```python
def _calculate_corner_speed_limit(self, current_pose: Pose2D, current_wp: Waypoint) -> float:
    """
    计算过弯限速
    
    逻辑：
    1. 检查队列里有没有下一个航点
    2. 计算当前航段和下一航段的夹角
    3. 夹角越尖锐（急弯），允许通过的速度越低
    """
```

**速度限制公式**：

$$V_{limit} = V_{corner} + (V_{cruise} - V_{corner}) \times \frac{d}{R}$$

其中：
- $V_{corner} = V_{cruise} \times (1 - \frac{\theta}{\pi})$ —— 基于转角的目标速度
- $d$ —— 到拐点的距离
- $R = 5.0m$ —— 影响半径
- $\theta$ —— 转向角（弧度）

**角度-速度映射**：

| 转向角度 | 速度因子 | 说明 |
|----------|----------|------|
| 0° (直行) | 100% | 不减速 |
| 45° | 75% | 轻微减速 |
| 90° (直角) | 50% | 中等减速 |
| 135° | 25% | 大幅减速 |
| 180° (掉头) | 30% | 最大减速 |

#### 2.3.4 航线跟踪模式 (velocity_path_tracker.py) - 核心改进

**新增：航线跟踪相关方法（类似飞控 L1 算法）**

| 方法 | 功能 |
|------|------|
| `_project_to_path()` | 计算船在航线上的垂直投影点 |
| `_get_lookahead_on_path()` | 在航线上计算前视点 |
| `_pure_pursuit_on_path()` | 航线跟踪版 Pure Pursuit |

**核心逻辑**：

```python
def _pure_pursuit_on_path(self, pose, target):
    """
    航线跟踪版 Pure Pursuit
    
    前视点在航线上，而不是直接指向目标航点。
    这样可以让船先回到航线再沿线行驶，实现真正的直线。
    """
    # 1. 获取前序航点，构成航线段
    prev_wp = self._path_history[-1]
    
    # 2. 计算船到航线的投影点
    proj_x, proj_y, t = self._project_to_path(pose, prev_wp, target)
    
    # 3. 从投影点沿航线方向前进 L 米，得到前视点
    lh_x = proj_x + lookahead_dist * dir_x
    lh_y = proj_y + lookahead_dist * dir_y
    
    # 4. 用航线上的前视点计算曲率
    return curvature_to_lookahead_point
```

**对比图**：

```
修改前（追航点）：                    修改后（追航线）：

起点 A ─────────── 终点 B            起点 A ════════════ 终点 B
                    ↗                         │      ◎ 前视点
                  ↗  弧线                     │     ↗
                ↗                             │   ↗  更直
              ↗                               │ ↗
           ⛵                                ⛵
```

**安全机制**：

| 边界情况 | 处理方式 |
|----------|----------|
| 无前序航点（第一个点） | 回退到普通 Pure Pursuit |
| 船过了目标点 (t > 1) | 回退到普通模式 |
| 横向偏差 > 5m | 回退到普通模式，先回归 |
| 距离目标 < 3m | 降低 PP 权重，确保精确到达 |
| 前视点超出航线 | 限制在终点位置 |

#### 2.3.5 ROS 节点适配 (velocity_controller_node.py)

**修改：`_pose_callback()` 方法**

```python
def _pose_callback(self, msg: PoseStamped):
    # 分离航向数据
    heading_yaw = magnetometer_yaw  # 始终使用磁罗盘/EKF
    course_yaw = None
    current_speed = self._current_speed
    
    # 速度有效时获取航迹角
    if self._use_velocity_heading and self._velocity_yaw_valid:
        course_yaw = self._velocity_based_yaw
        
    # 打包完整位姿
    new_pose = Pose2D(
        x=msg.pose.position.x,
        y=msg.pose.position.y,
        yaw=heading_yaw,      # 船头朝向
        course=course_yaw,    # 航迹方向
        speed=current_speed   # 对地速度
    )
```

---

## 3. 技术评估

### 3.1 算法对比

| 特性 | 修改前 | 修改后 |
|------|--------|--------|
| 跟踪目标 | 航点 (Waypoint) | 航线段 (Path Segment) |
| 前视点位置 | 目标航点本身 | 航线上的投影点 + L |
| 航向数据源 | 固定使用速度向量 | 智能切换(速度/罗盘) |
| 侧风抗扰 | 无 | L1算法自动蟹行 |
| 弯道速度 | 全程匀速 | 基于角度自适应减速 |
| 低速稳定性 | GPS噪声影响大 | 自动回退罗盘 |

### 3.2 风险评估

| 风险项 | 等级 | 缓解措施 |
|--------|------|----------|
| L1 阈值不当 | 低 | 0.3 m/s 可根据GPS精度调整 |
| 减速距离过短 | 低 | INFLUENCE_RADIUS 可增至 8m |
| 航点间距小于5m | 低 | 返回巡航速度，不影响正常航行 |
| 第一个航点无航线 | 低 | 自动回退到普通 Pure Pursuit |
| 船过了目标点 | 低 | 自动检测并回退到航点跟踪 |

### 3.3 预期效果

| 场景 | 预期改善 |
|------|----------|
| 直线航行 | 轨迹显著更直，类似飞控效果 |
| 急弯过弯 | 提前3-5秒减速，无甩尾 |
| 侧风环境 | 船头自动偏向上游，轨迹笔直 |
| 起点/终点 | 平稳启动/停止，无绕圈 |
| 中远距离 | 沿航线行驶而非切向目标 |

---

## 4. 测试验证计划

### 4.1 验证场景

| 序号 | 场景 | 验证内容 | 通过标准 |
|------|------|----------|----------|
| 1 | 直线航行 (50m) | 横向偏差 | < 0.5m（显著改善） |
| 2 | 90° 直角弯 | 是否提前减速 | 弯前5m开始减速 |
| 3 | 连续S弯 | 航行流畅度 | 无急刹、无甩尾 |
| 4 | 静止启动 | 起步方向 | 无绕圈，正确朝向目标 |
| 5 | 终点停止 | 停止稳定性 | 无反复震荡 |
| 6 | 侧风/水流 | 轨迹直线度 | 船头可偏，轨迹笔直 |
| 7 | 第一个航点 | 普通追踪 | 正常到达（回退模式） |
| 8 | 多航点路径 | 航线跟踪 | 中间段走直线 |

### 4.2 数据采集

测试时需记录以下日志字段：

```
timestamp, pose_x, pose_y, heading, course, speed, 
target_x, target_y, distance, cmd_vx, cmd_wz
```

### 4.3 分析命令

```bash
# 下载最新日志
./download_usv_logs.sh

# 分析日志
python3 ~/usv_workspace/src/usv_control/scripts/analyze_nav_log.py ~/usv_logs/nav_log_xxx.csv
```

---

## 5. 可调参数速查表

### 5.1 配置文件参数 (usv_params.yaml)

| 参数 | 当前值 | 范围 | 作用 |
|------|--------|------|------|
| `stanley_gain` | 3.5 | 1.0 ~ 5.0 | 纠偏力度，↑更快回归直线 |
| `lookahead_distance` | 1.5 m | 0.5 ~ 5.0 | 前视距离，↓减少切角 |
| `goal_tolerance` | 1.5 m | 0.5 ~ 3.0 | 到达判定距离 |
| `min_speed` | 0.15 m/s | 0.05 ~ 0.3 | 最低航行速度 |
| `cruise_speed` | 0.4 m/s | 0.3 ~ 1.0 | 巡航速度 |
| `max_angular_velocity` | 0.4 rad/s | 0.3 ~ 1.0 | 最大转向角速度 |

### 5.2 代码常量 (velocity_path_tracker.py)

| 常量 | 当前值 | 位置 | 作用 |
|------|--------|------|------|
| `INFLUENCE_RADIUS` | 5.0 m | `_calculate_corner_speed_limit()` | 弯道减速开始距离 |
| `MIN_CORNER_RATIO` | 0.3 | `_calculate_corner_speed_limit()` | 急弯最低速度比 |
| 速度阈值 | 0.3 m/s | `_get_control_heading()` | L1航向切换阈值 |
| `SAFE_DISTANCE` | 3.0 m | `_hybrid_control()` | 航线/航点模式切换距离 |
| 横向偏差阈值 | 5.0 m | `_get_lookahead_on_path()` | 超过此值回退航点跟踪 |

---

## 6. 代码验证状态

### 6.1 语法检查

| 文件 | 状态 |
|------|------|
| `velocity_path_tracker.py` | ✅ 通过 |
| `velocity_controller_node.py` | ✅ 通过 |
| `usv_params.yaml` | ✅ 配置正确 |

### 6.2 新增方法清单

| 文件 | 新增方法 | 功能 |
|------|----------|------|
| `velocity_path_tracker.py` | `_get_control_heading()` | L1航向选择 |
| `velocity_path_tracker.py` | `_calculate_corner_speed_limit()` | 弯道限速计算 |
| `velocity_path_tracker.py` | `_project_to_path()` | 航线投影计算 |
| `velocity_path_tracker.py` | `_get_lookahead_on_path()` | 航线上前视点 |
| `velocity_path_tracker.py` | `_pure_pursuit_on_path()` | 航线跟踪 PP |

---

## 7. 附录

### 7.1 相关文件

| 文件 | 用途 |
|------|------|
| [velocity_path_tracker.py](../../../usv_control/usv_control/velocity_path_tracker.py) | 核心路径跟踪算法 |
| [velocity_controller_node.py](../../../usv_control/usv_control/velocity_controller_node.py) | ROS 控制节点 |
| [usv_params.yaml](../../../usv_bringup/config/usv_params.yaml) | 参数配置文件 |
| [analyze_nav_log.py](../../../usv_control/scripts/analyze_nav_log.py) | 日志分析脚本 |

### 7.2 参考资料

1. **L1 导航算法**：ArduPilot/PX4 飞控标准算法
2. **Pure Pursuit**：[PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics)
3. **Stanley Controller**：Stanford DARPA Grand Challenge

### 7.3 修订历史

| 版本 | 日期 | 作者 | 修改内容 |
|------|------|------|----------|
| v1.0 | 2026-01-28 | chenhangwei | 初始版本：参数优化+L1航向+弯道减速 |
| v2.0 | 2026-01-28 | chenhangwei | 新增航线跟踪模式，解决走弧线问题 |

---

## 8. 总结

本次优化主要实现了四大改进：

### 8.1 核心改进清单

| 序号 | 改进项 | 目标问题 | 预期效果 |
|------|--------|----------|----------|
| 1 | **航线跟踪模式** | 直线走弧线 | 前视点在航线上，真正走直 |
| 2 | **L1 航向智能切换** | 侧风偏航、低速绕圈 | 高速用航迹抗风，低速用罗盘稳定 |
| 3 | **弯道自适应减速** | 惯性甩尾 | 基于角度/距离的线性减速 |
| 4 | **参数精细调优** | 回正慢、切角大 | Stanley增益↑、前视距离↓ |

### 8.2 技术亮点

- **航线跟踪**：与飞控 L1 算法原理一致，让船先回归航线再沿线行驶
- **智能回退**：边界情况自动回退到普通模式，确保安全
- **保守策略**：近距离（<3m）强制使用航点跟踪，确保精确到达

### 8.3 下一步

1. 实船测试验证
2. 重点观察直线段中段是否更直
3. 收集日志进行对比分析

---

*文档生成时间：2026-01-28*  
*语法检查：✅ 全部通过*
