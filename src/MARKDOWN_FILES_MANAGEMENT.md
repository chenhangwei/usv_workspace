# Markdown 文档管理指南

## 📚 文档分类和作用

### 🔴 核心文档（不可删除）

这些文档是**项目的核心知识库**，包含重要的架构设计、使用指南和开发流程：

#### 1. 项目级别
- **`.github/copilot-instructions.md`** ⭐⭐⭐⭐⭐
  - **作用**: AI Agent 的项目上下文指令
  - **重要性**: 极高（GitHub Copilot 依赖此文件）
  - **删除影响**: AI 无法理解项目架构和约定
  - **建议**: **必须保留**

- **`QUICK_START.md`**
  - **作用**: 项目快速入门指南
  - **重要性**: 高（新开发者首次使用）
  - **建议**: 保留

#### 2. GUI 模块架构文档
- **`gs_gui/MODULE_ARCHITECTURE.md`** ⭐⭐⭐⭐⭐
  - **作用**: GUI 模块化架构设计文档
  - **重要性**: 极高（理解代码结构的关键）
  - **建议**: **必须保留**

- **`gs_gui/REFACTOR_SUMMARY.md`**
  - **作用**: GUI 重构总结（70KB→7个模块）
  - **重要性**: 高（重构历史记录）
  - **建议**: 保留

- **`gs_gui/QUICK_REFERENCE.md`**
  - **作用**: GUI 常用操作快速参考
  - **重要性**: 高（日常开发查询）
  - **建议**: 保留

- **`gs_gui/TEST_GUIDE.md`**
  - **作用**: 测试指南和示例
  - **重要性**: 中高（测试开发）
  - **建议**: 保留

### 🟡 功能实现文档（建议保留）

这些文档记录了特定功能的实现细节和使用方法：

#### 优化相关
- **`OPTIMIZATION_GUIDE.md`** - 性能优化总指南
- **`OPTIMIZATION_SUMMARY.md`** - 优化总结
- **`usv_bringup/MAVROS_STARTUP_OPTIMIZATION.md`** - MAVROS 启动优化（97s→12s）
- **`MAVLINK_COMMAND_TIMEOUT_GUIDE.md`** - MAVLink 命令超时问题指南
- **`gs_gui/OFFLINE_DETECTION_OPTIMIZATION.md`** - 离线检测优化
- **`gs_gui/FLEET_LAUNCHER_OPTIMIZATION.md`** ⭐⭐ - 集群启动器性能优化（详细方案）
- **`gs_gui/FLEET_LAUNCHER_OPTIMIZATION_SUMMARY.md`** ⭐⭐ - 集群启动器优化快速总结

#### 功能实现
- **`gs_gui/GRACEFUL_SHUTDOWN.md`** - 优雅关闭功能详细文档
- **`gs_gui/LED_INFECTION_MODE.md`** - LED 传染模式实现
- **`gs_gui/AREA_OFFSET_GUIDE.md`** - 坐标偏移功能指南
- **`HOME_POSITION_SETTING_GUIDE.md`** ⭐⭐⭐ - Home Position 设置功能完整实现指南
- **`HOME_POSITION_QUICK_REF.md`** ⭐ - Home Position 快速参考
- **`HOME_POSITION_CHECKLIST.md`** - 实现检查清单
- **`HOME_POSITION_TEST_GUIDE.md`** ⭐⭐ - 功能测试步骤和验证方法
- **`gs_gui/USV_INFO_PANEL_GUIDE.md`** - USV 信息面板使用指南
- **`gs_gui/UI_MODERNIZATION_GUIDE.md`** - UI 现代化设计指南
- **`gs_gui/UI_RESPONSIVE_DESIGN.md`** - 响应式设计文档
- **`gs_gui/UI_SCROLL_IMPLEMENTATION.md`** - 滚动条实现
- **`gs_gui/REBOOT_BUTTON_FIX.md`** ⭐ - 飞控重启按钮修复（服务名称错误）
- **`usv_comm/BATTERY_PERCENTAGE_FIX.md`** - 电池百分比修复
- **`usv_sound/LOW_BATTERY_SOUND_FIX.md`** ⭐⭐ - 低电量声音自动播放修复
- **`LOW_BATTERY_MODE_ANALYSIS.md`** ⭐⭐⭐ - 低电量模式完整分析（LED/Sound 协同）
- **`usv_comm/STARTUP_LOW_BATTERY_FALSE_TRIGGER_FIX.md`** ⭐⭐⭐ - 启动时低电量误触发修复

#### 参数管理功能
- **`gs_gui/PARAM_MANAGER_DESIGN.md`** - 参数管理器架构设计（MAVROS 模式）
- **`gs_gui/PARAM_MANAGER_GUIDE.md`** - 参数管理使用指南（MAVROS 模式）
- **`gs_gui/PARAM_MANAGER_SUMMARY.md`** - 参数管理实现总结
- **`gs_gui/PARAM_QUICK_REF.md`** - 参数管理快速参考
- **`gs_gui/PARAM_BUTTON_FIX.md`** - 参数按钮修复文档
- **`gs_gui/PARAM_LOADING_ISSUE.md`** ⭐ - 参数加载问题详细说明（含 Phase 2 完成状态）
- **`gs_gui/PARAM_PHASE2_SUMMARY.md`** ⭐ - Phase 2 实现总结（异步加载）
- **`gs_gui/PARAM_PHASE3_SUMMARY.md`** ⭐ - Phase 3 实现总结（参数缓存）
- **`gs_gui/PARAM_USAGE_GUIDE.md`** ⭐ - 参数管理快速使用指南
- **`gs_gui/PARAM_UI_IMPROVEMENT.md`** ⭐ - 参数窗口界面显示优化（字体、间距、对比度）
- **`gs_gui/PARAM_SERVICE_TROUBLESHOOTING.md`** ⭐⭐ - 参数服务故障排查指南（含实际诊断案例）
- **`gs_gui/PARAM_SERIAL_MODE_GUIDE.md`** ⭐⭐⭐ - 串口直连参数模式完整指南（pymavlink 模式，推荐）

#### MAVROS 通信和配置
- **`usv_bringup/MAVROS_STARTUP_OPTIMIZATION.md`** - MAVROS 启动优化（97s→12s）
- **`usv_bringup/BAUDRATE_CONFIGURATION_GUIDE.md`** ⭐⭐ - 波特率配置指南（含实测案例和推荐值）
- **`MAVLINK_COMMAND_TIMEOUT_GUIDE.md`** - MAVLink 命令超时问题指南

#### 问题分析
- **`USV_STARTUP_DIFFERENCES_ANALYSIS.md`** - USV 启动差异分析
- **`gs_gui/TEMPERATURE_HYSTERESIS_UPDATE.md`** - 温度滞后更新

### 🟢 快速参考文档（可合并或删除）

这些是**重复性的快速参考**，与详细文档内容重复：

- **`gs_gui/GRACEFUL_SHUTDOWN_QUICK_REF.md`** ← 与 GRACEFUL_SHUTDOWN.md 重复
- **`gs_gui/GRACEFUL_SHUTDOWN_IMPLEMENTATION.md`** ← 与上面两个重复
- **`gs_gui/LED_INFECTION_QUICK_START.md`** ← 与 LED_INFECTION_MODE.md 重复
- **`gs_gui/LED_INFECTION_IMPLEMENTATION_SUMMARY.md`** ← 重复
- **`gs_gui/TEMPERATURE_HYSTERESIS_QUICK_REF.md`** ← 与 TEMPERATURE_HYSTERESIS_UPDATE.md 重复
- **`USV_STARTUP_DIFFERENCES_QUICK_REF.md`** ← 与 USV_STARTUP_DIFFERENCES_ANALYSIS.md 重复
- **`gs_gui/QUICK_REFERENCE_USV_INFO.md`** ← 可能与 USV_INFO_PANEL_GUIDE.md 重复

**建议**: 可以合并到主文档中，删除单独的快速参考版本。

### 🔵 历史记录文档（可归档）

这些文档记录了已完成的迁移/修复工作，主要用于历史追溯：

- **`gs_gui/BOOT_POSE_EXPLAINED.md`** - Boot Pose 概念说明
- **`gs_gui/BOOT_POSE_MIGRATION.md`** - Boot Pose 迁移过程
- **`gs_gui/BOOT_POSE_REMOVAL_SUMMARY.md`** - Boot Pose 移除总结
- **`gs_gui/BUGFIX_STEERING_BUTTON.md`** - 转向按钮修复
- **`gs_gui/INTEGRATION_COMPLETE.md`** - 集成完成记录
- **`gs_gui/TEMPERATURE_DISPLAY_UPDATE.md`** - 温度显示更新

**建议**: 
- 如果不需要追溯历史，可以删除
- 或者创建 `docs/history/` 目录归档

### ⚪ 自动生成（可删除）

- **`gs_gui/.pytest_cache/README.md`** - pytest 缓存说明
  - **建议**: 可以删除（.pytest_cache 目录本身可以加入 .gitignore）

### 📖 README 文档（保留）

- **`gs_gui/README.md`** - gs_gui 包说明
- **`gs_gui/REFACTOR_README.md`** - 重构说明

## 🗑️ 删除建议

### 方案 1: 激进清理（删除 ~40%）

**可以安全删除的文件**（主要是重复的快速参考）:

```bash
cd /home/chenhangwei/usv_workspace/src

# 删除重复的快速参考文档
rm gs_gui/GRACEFUL_SHUTDOWN_QUICK_REF.md
rm gs_gui/GRACEFUL_SHUTDOWN_IMPLEMENTATION.md
rm gs_gui/LED_INFECTION_QUICK_START.md
rm gs_gui/LED_INFECTION_IMPLEMENTATION_SUMMARY.md
rm gs_gui/TEMPERATURE_HYSTERESIS_QUICK_REF.md
rm USV_STARTUP_DIFFERENCES_QUICK_REF.md

# 删除历史记录文档（如果不需要）
rm gs_gui/BOOT_POSE_EXPLAINED.md
rm gs_gui/BOOT_POSE_MIGRATION.md
rm gs_gui/BOOT_POSE_REMOVAL_SUMMARY.md
rm gs_gui/BUGFIX_STEERING_BUTTON.md
rm gs_gui/INTEGRATION_COMPLETE.md
rm gs_gui/TEMPERATURE_DISPLAY_UPDATE.md

# 删除 pytest 缓存
rm -rf gs_gui/.pytest_cache/
```

**节省**: 约 12-15 个文件

### 方案 2: 保守整理（归档 ~30%）

创建归档目录，保留但整理：

```bash
cd /home/chenhangwei/usv_workspace/src

# 创建归档目录
mkdir -p docs/archive/quick-refs
mkdir -p docs/archive/history

# 移动快速参考到归档
mv gs_gui/GRACEFUL_SHUTDOWN_QUICK_REF.md docs/archive/quick-refs/
mv gs_gui/LED_INFECTION_QUICK_START.md docs/archive/quick-refs/
mv gs_gui/TEMPERATURE_HYSTERESIS_QUICK_REF.md docs/archive/quick-refs/
mv USV_STARTUP_DIFFERENCES_QUICK_REF.md docs/archive/quick-refs/

# 移动历史记录到归档
mv gs_gui/BOOT_POSE_*.md docs/archive/history/
mv gs_gui/BUGFIX_STEERING_BUTTON.md docs/archive/history/
mv gs_gui/INTEGRATION_COMPLETE.md docs/archive/history/
```

### 方案 3: 合并优化（推荐）

将重复内容合并到主文档：

1. **优雅关闭功能**: 只保留 `GRACEFUL_SHUTDOWN.md`
2. **LED 传染模式**: 只保留 `LED_INFECTION_MODE.md`
3. **温度滞后**: 只保留 `TEMPERATURE_HYSTERESIS_UPDATE.md`
4. **USV 启动差异**: 只保留 `USV_STARTUP_DIFFERENCES_ANALYSIS.md`

然后删除其他重复文件。

## 📊 统计

当前文档数量: **38 个 .md 文件**

| 类别 | 数量 | 建议 |
|------|------|------|
| 核心文档 | ~8 | ✅ 必须保留 |
| 功能实现文档 | ~15 | ✅ 建议保留 |
| 快速参考（重复） | ~7 | ⚠️ 可删除/合并 |
| 历史记录 | ~6 | ⚠️ 可归档/删除 |
| 自动生成 | ~1 | ⚠️ 可删除 |
| README | ~2 | ✅ 保留 |

**删除后可减少**: 30-50% 的文档数量（约 12-18 个文件）

## 🎯 推荐操作

### 立即执行（安全删除）

```bash
cd /home/chenhangwei/usv_workspace/src

# 1. 删除重复的快速参考（保留详细版本）
rm gs_gui/GRACEFUL_SHUTDOWN_QUICK_REF.md
rm gs_gui/GRACEFUL_SHUTDOWN_IMPLEMENTATION.md
rm gs_gui/LED_INFECTION_QUICK_START.md
rm gs_gui/LED_INFECTION_IMPLEMENTATION_SUMMARY.md
rm gs_gui/TEMPERATURE_HYSTERESIS_QUICK_REF.md
rm gs_gui/QUICK_REFERENCE_USV_INFO.md  # 如果与 USV_INFO_PANEL_GUIDE 重复
rm USV_STARTUP_DIFFERENCES_QUICK_REF.md

# 2. 删除 pytest 缓存
rm -rf gs_gui/.pytest_cache/

# 3. （可选）删除历史记录
rm gs_gui/BOOT_POSE_EXPLAINED.md
rm gs_gui/BOOT_POSE_MIGRATION.md
rm gs_gui/BOOT_POSE_REMOVAL_SUMMARY.md
rm gs_gui/BUGFIX_STEERING_BUTTON.md
rm gs_gui/INTEGRATION_COMPLETE.md
rm gs_gui/TEMPERATURE_DISPLAY_UPDATE.md
```

**节省**: 约 13-14 个文件

### 长期维护建议

1. **避免重复**: 不再创建 `*_QUICK_REF.md` 和 `*_IMPLEMENTATION.md` 等重复文档
2. **单一真相源**: 每个功能只保留一个主文档
3. **版本控制**: 使用 Git 历史记录追溯变更，不需要保留所有历史文档
4. **定期清理**: 每季度检查一次文档，删除过时内容

## ⚠️ 不要删除

以下文档**绝对不能删除**:

1. `.github/copilot-instructions.md` - AI 上下文
2. `gs_gui/MODULE_ARCHITECTURE.md` - 架构设计
3. `QUICK_START.md` - 快速入门
4. `gs_gui/REFACTOR_SUMMARY.md` - 重构记录
5. `usv_bringup/MAVROS_STARTUP_OPTIMIZATION.md` - 关键优化
6. `MAVLINK_COMMAND_TIMEOUT_GUIDE.md` - 故障排查

## 🔄 更新 .gitignore

建议添加到 `.gitignore`:

```gitignore
# Pytest 缓存
**/.pytest_cache/

# 临时文档（可选）
docs/archive/
*_QUICK_REF.md
*_IMPLEMENTATION.md
```

---

**总结**: 可以安全删除 **13-18 个重复或历史文档**，保留核心的 **20-25 个关键文档**。
