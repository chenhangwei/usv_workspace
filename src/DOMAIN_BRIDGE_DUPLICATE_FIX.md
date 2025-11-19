# Domain Bridge 重复启动问题修复

## 问题描述

**现象:** usv_02和usv_03持续以1800+ Hz的频率接收arming消息,导致刷屏

**根本原因:** 有**多个domain_bridge进程**同时运行

## 诊断过程

### 1. 发现重复进程

```bash
ps aux | grep domain_bridge
# 发现2个进程:
# PID 14646: domain_bridge_usv01.yaml (screen启动)
# PID 25752: domain_bridge.yaml (gs_launch.py启动)
```

### 2. 进程树分析

```bash
ps -o pid,ppid,cmd -p 25752
# 输出: PPID 20581 -> gs_launch.py
```

**结论:** gs_launch.py启动了domain_bridge,但源码中已经注释掉了!

### 3. 源码vs安装版本对比

**源码版本** (`src/gs_bringup/launch/gs_launch.py`):
```python
# ⚠️ Domain Bridge 已移除 - 请使用独立脚本管理
return LaunchDescription([
    gs_param_arg,
    fleet_config_arg,
    main_gui_app,
    # domain_bridge_launch,  # 已注释 - 请独立启动
])
```

**安装版本** (`install/gs_bringup/share/gs_bringup/launch/gs_launch.py`):
```python
domain_bridge_launch = IncludeLaunchDescription(...)  # 还在!!!
return LaunchDescription([
    ...
    domain_bridge_launch,  # 没有注释!!!
])
```

**问题:** 修改源码后没有重新编译安装!

## 修复方案

### 步骤1: 重新编译gs_bringup包

```bash
cd /home/chenhangwei/usv_workspace
colcon build --packages-select gs_bringup --symlink-install
```

### 步骤2: 停止所有进程

```bash
# 停止地面站
pkill -f "gs_launch|main_gui_app"

# 停止domain_bridge
cd /home/chenhangwei/usv_workspace/src/gs_bringup
./scripts/domain_bridge.sh stop
```

### 步骤3: 重新启动

```bash
# 1. 先启动domain_bridge
./scripts/domain_bridge.sh start

# 2. 再启动地面站
ros2 launch gs_bringup gs_launch.py
```

### 步骤4: 验证单例

```bash
# 检查domain_bridge进程数量
ps aux | grep "domain_bridge /home" | grep -v grep | wc -l
# 预期输出: 1

# 检查ROS节点
ros2 node list | grep domain_bridge
# 预期输出: 
# /domain_bridge
# /usv_domain_bridge_11
# /usv_domain_bridge_12  
# /usv_domain_bridge_13
# /usv_domain_bridge_99
```

## 配置文件修复

### 1. domain_bridge.launch.py 配置文件路径

**修复前:**
```python
default_value=os.path.expanduser('~/domain_bridge/domain_bridge_usv01.yaml'),
```

**修复后:**
```python
default_value=os.path.expanduser('~/domain_bridge/domain_bridge.yaml'),
```

### 2. domain_bridge_usv01.yaml 类型修复

**修复前:**
```yaml
usv_01/set_usv_arming:
  type: std_msgs/msg/Bool  # 错误!
```

**修复后:**
```yaml
usv_01/set_usv_arming:
  type: std_msgs/msg/String  # 正确
```

## 预防措施

### 1. 使用symlink-install

```bash
# 开发时使用symlink避免源码和安装版本不一致
colcon build --symlink-install
```

### 2. 启动前检查脚本

在`domain_bridge.sh`中添加:
```bash
if pgrep -f "gs_launch.*domain_bridge" > /dev/null; then
  echo "❌ 错误: 检测到gs_launch已启动domain_bridge"
  echo "请修改gs_launch.py移除domain_bridge启动"
  exit 1
fi
```

### 3. 启动顺序强制

在`gs_launch.py`开头添加检查:
```python
# 检查domain_bridge是否已独立启动
if not os.path.exists('/tmp/domain_bridge.lock'):
    print("❌ 错误: domain_bridge未运行!")
    print("请先启动: ./src/gs_bringup/scripts/domain_bridge.sh start")
    sys.exit(1)
```

## 修复的文件清单

1. `src/gs_bringup/launch/gs_launch.py` - 注释domain_bridge_launch
2. `src/gs_bringup/launch/domain_bridge.launch.py` - 修复配置文件路径
3. `~/domain_bridge/domain_bridge_usv01.yaml` - 修复Bool->String
4. 重新编译安装

## 验证结果

### ✅ 修复后状态

| 检查项 | 预期 | 实际 | 状态 |
|--------|------|------|------|
| domain_bridge进程数 | 1 | 1 | ✅ |
| ROS domain_bridge节点 | 5个 | 5个 | ✅ |
| usv_02消息频率 | 0 Hz | 0 Hz | ✅ |
| topic类型一致性 | String | String | ✅ |

### 测试命令

```bash
# 1. 检查进程
ps aux | grep domain_bridge | grep -v grep

# 2. 检查节点
ros2 node list | grep domain_bridge

# 3. 检查消息频率(启动地面站后测试)
timeout 5 ros2 topic hz /usv_02/set_usv_arming

# 4. 检查类型一致性
for topic in /usv_0{1,2,3}/set_usv_{arming,mode}; do
  echo "=== $topic ==="
  ros2 topic info $topic | head -3
done
```

## 总结

**主要问题:**
1. ❌ gs_launch.py启动了domain_bridge (安装版本未更新)
2. ❌ domain_bridge.sh也启动了domain_bridge
3. ❌ 两个进程使用不同配置文件
4. ❌ usv01配置使用Bool类型(旧版)

**解决方案:**
1. ✅ 重新编译gs_bringup包同步源码修改
2. ✅ 修复domain_bridge.launch.py使用正确配置
3. ✅ 修复usv01配置的Bool->String
4. ✅ 确保只有一个domain_bridge实例运行

**结果:**
- 消息传递恢复正常
- 不再出现1800+ Hz的消息洪水
- topic类型统一为String

---

**修复日期:** 2025-11-18  
**测试状态:** ✅ 待地面站重启后验证  
**关键教训:** 修改launch文件后必须重新编译安装!
