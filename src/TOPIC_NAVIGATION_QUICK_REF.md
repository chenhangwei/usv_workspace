# 基于话题的导航系统 - 快速参考

## 🚀 快速开始

### 编译
```bash
cd ~/usv_workspace
colcon build --packages-select common_interfaces usv_comm gs_gui --allow-overriding common_interfaces
source install/setup.bash
```

### 启动 Domain Bridge
```bash
cd ~/usv_workspace/src
bash gs_bringup/scripts/domain_bridge.sh restart
```

### 运行测试
```bash
cd ~/usv_workspace/src
bash test_topic_navigation.sh
```

---

## 📡 关键话题

### USV_01 (Domain 11)
- `/usv_01/navigation_goal` - 接收导航目标 (从地面站)
- `/usv_01/navigation_feedback` - 发送导航进度 (到地面站)
- `/usv_01/navigation_result` - 发送完成结果 (到地面站)

### USV_02 (Domain 12) / USV_03 (Domain 13)
- 相同模式,只是 Domain ID 不同

---

## 🔍 调试命令

### 监听地面站话题
```bash
export ROS_DOMAIN_ID=99
ros2 topic list | grep navigation
ros2 topic echo /usv_01/navigation_feedback
ros2 topic echo /usv_01/navigation_result
```

### 监听 USV 话题
```bash
export ROS_DOMAIN_ID=11
ros2 topic echo /usv_01/navigation_goal
```

### 查看消息定义
```bash
ros2 interface show common_interfaces/msg/NavigationGoal
ros2 interface show common_interfaces/msg/NavigationFeedback
ros2 interface show common_interfaces/msg/NavigationResult
```

### 检查 Domain Bridge
```bash
# 查看日志
bash gs_bringup/scripts/domain_bridge.sh attach  # Ctrl+A, D 退出

# 重启
bash gs_bringup/scripts/domain_bridge.sh restart

# 停止
bash gs_bringup/scripts/domain_bridge.sh stop
```

---

## 📊 消息结构

### NavigationGoal
```
goal_id: 123                    # 唯一ID
target_pose:
  header:
    frame_id: "map"
  pose:
    position: {x: 10.0, y: 5.0, z: 0.0}
    orientation: {w: 1.0}       # 四元数
timeout: 60.0                   # 秒
timestamp: ...
```

### NavigationFeedback
```
goal_id: 123
distance_to_goal: 2.5           # 米
heading_error: 15.0             # 度
estimated_time: 10.0            # 秒
timestamp: ...
```

### NavigationResult
```
goal_id: 123
success: True
error_code: 0                   # 0=成功, 1=超时, 2=取消, 3=错误
message: "到达目标点"
timestamp: ...
```

---

## ⚙️ 关键参数

### navigate_to_point_node (USV 端)
```yaml
nav_arrival_threshold: 1.0      # 到达阈值(米)
nav_feedback_period: 0.5        # 反馈周期(秒)
distance_mode: "2d"             # 距离模式: 2d/3d
```

### ground_station_node (地面站)
```python
_next_goal_id: 1                # 自动递增
_goal_id_lock: threading.Lock() # 线程安全
_goal_to_usv: {}                # goal_id -> usv_id 映射
```

---

## 🛠️ 常见问题

### Q: 反馈未收到?
```bash
# 1. 检查 Domain Bridge
ps aux | grep domain_bridge

# 2. 检查话题
export ROS_DOMAIN_ID=99
ros2 topic list | grep navigation

# 3. 查看 Domain Bridge 日志
bash gs_bringup/scripts/domain_bridge.sh attach
```

### Q: 目标未发送?
```bash
# 检查地面站发布器
export ROS_DOMAIN_ID=99
ros2 topic info /usv_01/navigation_goal

# 检查 USV 订阅器
export ROS_DOMAIN_ID=11
ros2 topic info /usv_01/navigation_goal
```

### Q: Goal ID 不匹配?
- 检查 `_next_goal_id` 是否正确递增
- 查看 `_goal_to_usv` 映射是否正确
- 确认回调函数检查 `cached.get('goal_id') == msg.goal_id`

---

## 📝 代码位置

### 消息定义
- `common_interfaces/msg/NavigationGoal.msg`
- `common_interfaces/msg/NavigationFeedback.msg`
- `common_interfaces/msg/NavigationResult.msg`

### USV 节点
- `usv_comm/usv_comm/navigate_to_point_node.py`
- 入口点: `ros2 run usv_comm navigate_to_point_node`

### 地面站
- `gs_gui/gs_gui/usv_manager.py` - 话题管理
- `gs_gui/gs_gui/ground_station_node.py` - 导航逻辑

### 配置
- `/home/chenhangwei/domain_bridge/domain_bridge.yaml`

---

## 🎯 测试检查清单

- [ ] 消息类型已生成 (`ros2 interface list | grep Navigation`)
- [ ] Domain Bridge 已重启
- [ ] 地面站可发送目标 (检查 `navigation_goal_pubs`)
- [ ] USV 可接收目标 (检查 `navigation_goal` 订阅器)
- [ ] 地面站可接收反馈 (检查 `navigation_feedback_subs`)
- [ ] 地面站可接收结果 (检查 `navigation_result_subs`)
- [ ] Goal ID 正确匹配
- [ ] 集群任务完成度正确统计

---

## 📚 完整文档
- `TOPIC_BASED_NAVIGATION_IMPLEMENTATION.md` - 完整实现文档
- `NAVIGATION_FLOW_COMPLETE.md` - 导航流程文档
- `DOMAIN_BRIDGE_GUIDE.md` - Domain Bridge 指南

---

**更新时间**: 2024-11-18  
**版本**: v1.0  
**状态**: ✅ 已完成并测试
