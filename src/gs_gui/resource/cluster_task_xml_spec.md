# USV 集群任务 XML 规范说明 v2.0

> 更新日期: 2026-01-21  
> 适用于: 地面站 GS GUI 任务导入/导出模块

---

## 1. 文件结构概览

```xml
<?xml version="1.0" encoding="utf-8"?>
<cluster type="home">
  <step number="1" nav_mode="async">
    <usvs>
      <usv led="led_off">
        <!-- USV 配置 -->
      </usv>
    </usvs>
  </step>
  <!-- 更多步骤... -->
</cluster>
```

---

## 2. 根节点 `<cluster>`

| 属性 | 类型 | 必填 | 默认值 | 说明 |
|------|------|------|--------|------|
| `type` | string | 否 | - | 任务类型标识，如 `home`, `patrol`, `formation` |

---

## 3. 步骤节点 `<step>`

### 3.1 属性定义

| 属性 | 类型 | 必填 | 默认值 | 说明 |
|------|------|------|--------|------|
| `number` | int | **是** | - | 步骤编号，从 1 开始递增 |
| `nav_mode` | string | 否 | `async` | 导航模式（见下表） |
| `sync_timeout` | float | 否 | `10.0` | 同步模式超时时间（秒） |
| `arrival_quality` | float | 否 | `0.8` | 到达质量阈值（0.0-1.0） |

### 3.2 导航模式 `nav_mode`

| 值 | 数值 | 说明 | 使用场景 |
|----|------|------|---------|
| `async` | 0 | **异步模式** - 到达阈值距离后立即前往下一航点 | 中间过渡点、快速巡航 |
| `sync` | 1 | **同步模式** - 等待到达质量确认（80%样本在阈值内） | 关键检查点、编队集结点 |
| `rotate` | 2 | **旋转模式** - 到达后原地旋转指定圈数 | 扫描、展示动作 |
| `terminal` | 3 | **终止模式** - 到达后停止，任务结束 | 最终目标点、返航点 |

### 3.3 同步模式参数说明

当 `nav_mode="sync"` 时，可配置以下参数：

```xml
<step number="6" nav_mode="sync" sync_timeout="15.0" arrival_quality="0.8">
```

- `sync_timeout`: 等待确认的最大时间（秒），超时后自动降级为异步模式
- `arrival_quality`: 到达质量阈值，表示在检测窗口内有多少比例的样本需要在到达阈值内
  - `0.8` = 80% 的采样点需要在到达阈值距离内

---

## 4. USV 列表节点 `<usvs>`

包含当前步骤涉及的所有 USV 配置。

---

## 5. 单个 USV 节点 `<usv>`

### 5.1 属性定义

| 属性 | 类型 | 必填 | 默认值 | 说明 |
|------|------|------|--------|------|
| `led` | string | 否 | - | LED 控制命令，如 `led_off`, `led_red`, `led_green` |

### 5.2 子节点定义

#### 5.2.1 `<usv_id>` (必填)

USV 唯一标识符。

```xml
<usv_id>usv_01</usv_id>
```

#### 5.2.2 `<position>` (必填)

目标位置坐标（相对于任务区域中心）。

```xml
<position>
  <x>-1.000</x>   <!-- X 坐标（米） -->
  <y>-5.000</y>   <!-- Y 坐标（米） -->
  <z>0.000</z>    <!-- Z 坐标（米），水面任务通常为 0 -->
</position>
```

#### 5.2.3 `<yaw>` (可选)

航向控制配置。

**方式一：自动航向（推荐）**
```xml
<yaw mode="auto" />
```

**方式二：固定航向**
```xml
<yaw mode="fixed">
  <value>90.0</value>  <!-- 航向角（度），0=北，90=东 -->
</yaw>
```

| 属性/子节点 | 说明 |
|------------|------|
| `mode="auto"` | 自动朝向目标点方向 |
| `mode="fixed"` | 使用指定航向角 |
| `<value>` | 航向角度值（仅 fixed 模式需要） |

#### 5.2.4 `<velocity>` (可选)

速度配置。

```xml
<velocity>
  <value>0.500</value>  <!-- 速度（米/秒） -->
</velocity>
```

#### 5.2.5 `<maneuver>` (可选)

特殊机动动作配置。

```xml
<maneuver type="spin" circles="3.0" direction="ccw" />
```

| 属性 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `type` | string | **是** | 机动类型：`spin`(原地旋转), `none`(无) |
| `circles` | float | 是(spin) | 旋转圈数，如 `3.0` |
| `direction` | string | 否 | 旋转方向：`cw`(顺时针), `ccw`(逆时针) |

---

## 6. 完整示例

```xml
<?xml version="1.0" encoding="utf-8"?>
<cluster type="patrol">
  
  <!-- 步骤1: 异步模式 - 快速前往起始位置 -->
  <step number="1" nav_mode="async">
    <usvs>
      <usv led="led_off">
        <usv_id>usv_01</usv_id>
        <position>
          <x>0.000</x>
          <y>-10.000</y>
          <z>0.000</z>
        </position>
        <yaw mode="auto" />
        <velocity>
          <value>0.800</value>
        </velocity>
      </usv>
      <usv led="led_off">
        <usv_id>usv_02</usv_id>
        <position>
          <x>2.000</x>
          <y>-10.000</y>
          <z>0.000</z>
        </position>
        <yaw mode="auto" />
        <velocity>
          <value>0.800</value>
        </velocity>
      </usv>
    </usvs>
  </step>

  <!-- 步骤2: 同步模式 - 等待所有USV到达集结点 -->
  <step number="2" nav_mode="sync" sync_timeout="20.0" arrival_quality="0.8">
    <usvs>
      <usv led="led_green">
        <usv_id>usv_01</usv_id>
        <position>
          <x>0.000</x>
          <y>0.000</y>
          <z>0.000</z>
        </position>
        <yaw mode="fixed">
          <value>0.0</value>
        </yaw>
        <velocity>
          <value>0.500</value>
        </velocity>
      </usv>
      <usv led="led_green">
        <usv_id>usv_02</usv_id>
        <position>
          <x>2.000</x>
          <y>0.000</y>
          <z>0.000</z>
        </position>
        <yaw mode="fixed">
          <value>0.0</value>
        </yaw>
        <velocity>
          <value>0.500</value>
        </velocity>
      </usv>
    </usvs>
  </step>

  <!-- 步骤3: 终止模式 + 旋转 - 到达终点后旋转展示 -->
  <step number="3" nav_mode="terminal">
    <usvs>
      <usv led="led_red">
        <usv_id>usv_01</usv_id>
        <position>
          <x>-5.000</x>
          <y>10.000</y>
          <z>0.000</z>
        </position>
        <maneuver type="spin" circles="2.0" direction="ccw" />
        <velocity>
          <value>0.300</value>
        </velocity>
      </usv>
      <usv led="led_red">
        <usv_id>usv_02</usv_id>
        <position>
          <x>5.000</x>
          <y>10.000</y>
          <z>0.000</z>
        </position>
        <maneuver type="spin" circles="2.0" direction="cw" />
        <velocity>
          <value>0.300</value>
        </velocity>
      </usv>
    </usvs>
  </step>

</cluster>
```

---

## 7. 导航模式选择指南

| 场景 | 推荐模式 | 说明 |
|------|---------|------|
| 中间过渡点 | `async` | 快速通过，不停留 |
| 编队集结点 | `sync` | 确保所有船只稳定到达后再继续 |
| 展示/扫描动作 | `rotate` (配合 `<maneuver>`) | 到达后执行旋转 |
| 任务最终目标 | `terminal` | 任务结束，停止导航 |
| 返航点 | `terminal` | 返回后停止 |

---

## 8. 注意事项

1. **步骤编号连续性**: `number` 属性应从 1 开始连续递增
2. **USV ID 一致性**: 同一任务中的 `usv_id` 应与系统注册的 USV 标识符一致
3. **坐标系说明**: 
   - X 轴: 正向为东
   - Y 轴: 正向为北
   - Z 轴: 正向为上（水面任务通常为 0）
4. **角度单位**: 航向角使用度数（0-360），0° 为正北
5. **速度单位**: 米/秒 (m/s)
6. **距离单位**: 米 (m)

---

## 9. 版本历史

| 版本 | 日期 | 变更内容 |
|------|------|---------|
| v2.0 | 2026-01-21 | 新增 `nav_mode` 导航模式支持 |
| v1.0 | 2025-11-19 | 初始版本，支持基础导航和旋转机动 |
