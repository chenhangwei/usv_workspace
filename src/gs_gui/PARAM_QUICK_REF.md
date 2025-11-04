# 参数管理功能 - 快速参考

## ⚠️ 重要提示

**当前版本的参数管理功能暂未完全实现**，主要受限于 MAVROS param 插件的复杂性：

1. **ParamPull 服务会阻塞 30-60 秒**，在后台线程中无法正常工作
2. **参数存储在 MAVROS 内部**，需要订阅 topic 才能获取完整列表
3. **实现复杂度较高**，需要更多开发时间

## 🔧 当前推荐方案

### 方案 1: 使用 QGroundControl (推荐)

**QGroundControl** 是功能最完善的地面站软件：

1. 下载 QGC: https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html
2. 连接 USV 的 MAVLink 端口（如 UDP 14550）
3. 打开 **Vehicle Setup → Parameters**
4. 搜索、修改、保存参数

### 方案 2: 使用 Mission Planner

**Mission Planner** (仅 Windows)：

1. 下载: https://ardupilot.org/planner/
2. 连接 USV
3. 打开 **Config/Tuning → Full Parameter List**
4. 修改参数

### 方案 3: 使用命令行（高级用户）

```bash
# 获取单个参数
ros2 service call /usv_01/mavros/param/get mavros_msgs/srv/ParamGet \
  "{param_id: 'ARMING_CHECK'}"

# 设置单个参数
ros2 service call /usv_01/mavros/param/set mavros_msgs/srv/ParamSet \
  "{param_id: 'ARMING_CHECK', value: {integer: 1, real: 0.0}}"

# 拉取所有参数（会阻塞 30-60 秒）
ros2 service call /usv_01/mavros/param/pull mavros_msgs/srv/ParamPull "{}"

# 推送所有参数到飞控
ros2 service call /usv_01/mavros/param/push mavros_msgs/srv/ParamPush "{}"
```

## 📋 后续开发计划

参数管理功能将在后续版本中完善：

**Phase 2** (计划中):
- [ ] 订阅 `/mavros/param/param_value` topic 获取参数列表
- [ ] 实现非阻塞的参数加载机制
- [ ] 添加参数缓存功能

**Phase 3** (长期):
- [ ] 参数元数据（单位、范围、描述）
- [ ] 参数导入/导出
- [ ] 参数模板

## 🚀 快速启用（暂不可用）

~~### 1. 启用 param 插件~~

~~编辑 `usv_bringup/launch/usv_launch.py`（约280行）：~~

```python
# 暂时不需要启用，因为功能未完成
# 'plugin_allowlist': [
#     # ... 现有插件 ...
#     'param',  # ← 不要添加
# ],
```

## ❓ 常见问题

**Q: 为什么一直在加载？**
- ❌ 当前版本的参数加载功能尚未完成实现
- ✅ 请使用 QGroundControl 或 Mission Planner

**Q: 什么时候能用？**
- 📅 计划在后续版本中完善
- � 需要重构参数加载逻辑

**Q: 有其他替代方案吗？**
- ✅ QGroundControl（最推荐）
- ✅ Mission Planner
- ✅ 命令行方式（见上方）

## 📚 相关文档

- **ArduPilot 参数文档**: https://ardupilot.org/rover/docs/parameters.html
- **QGC 下载**: https://docs.qgroundcontrol.com/
- **MAVROS 参数文档**: https://github.com/mavlink/mavros

## 🆘 需要帮助？

- 📧 邮箱: chenhangwei77777@hotmail.com
- 🐛 GitHub: https://github.com/chenhangwei/usv_workspace/issues

---

**版本**: v1.0.0 (功能暂不可用) | **更新**: 2025-11-04
