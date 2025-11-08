#!/bin/bash
# 测试 USV 集群启动器状态检测修复
# 
# 问题：usv_01 已连接但显示"检测中..."，usv_02/03 未连接也显示"检测中..."
# 修复：
# 1. 初始状态从"检测中..."改为"离线"
# 2. 窗口打开时立即执行首次检测（不等待 3 秒）
# 3. 增强状态变化日志输出

echo "=========================================="
echo "USV 集群启动器 - 状态检测修复测试"
echo "=========================================="
echo ""

# 进入工作空间
cd /home/chenhangwei/usv_workspace
source install/setup.bash

echo "📋 第 1 步：检查网络连通性"
echo "----------------------------------------"
for usv_id in usv_01 usv_02 usv_03; do
    case $usv_id in
        usv_01) ip="192.168.68.55" ;;
        usv_02) ip="192.168.68.54" ;;
        usv_03) ip="192.168.68.52" ;;
    esac
    
    echo -n "  [$usv_id] Ping $ip ... "
    if ping -c 1 -W 1 "$ip" &>/dev/null; then
        echo "✅ 在线"
    else
        echo "❌ 离线"
    fi
done
echo ""

echo "📋 第 2 步：检查 ROS 节点"
echo "----------------------------------------"
echo "  正在扫描 ROS 图..."
node_list=$(timeout 3 ros2 node list 2>/dev/null || echo "")

for usv_id in usv_01 usv_02 usv_03; do
    echo -n "  [$usv_id] ROS 节点检测: "
    count=$(echo "$node_list" | grep -c "/$usv_id/" || echo "0")
    if [ "$count" -gt 0 ]; then
        echo "✅ 检测到 $count 个节点"
    else
        echo "❌ 未检测到节点"
    fi
done
echo ""

echo "📋 第 3 步：预期状态"
echo "----------------------------------------"
echo "  [usv_01] 192.168.68.55 在线，无节点 → 应显示：🟡 在线"
echo "  [usv_02] 192.168.68.54 离线 → 应显示：⚫ 离线"
echo "  [usv_03] 192.168.68.52 离线 → 应显示：⚫ 离线"
echo ""

echo "✅ 测试环境检查完成"
echo ""
echo "🚀 现在请启动 GUI 并检查状态列："
echo "   方法 1: 在主界面点击 '菜单' → 'USV 集群启动'"
echo "   方法 2: 运行命令: ros2 run gs_gui main_gui_app"
echo ""
echo "📝 观察要点："
echo "   1. 窗口打开时，状态应立即从 '⚫ 离线' 更新"
echo "   2. usv_01 应快速显示 '🟡 在线'"
echo "   3. usv_02/03 应保持 '⚫ 离线'"
echo "   4. 日志中应看到状态变化记录（格式：usv_id: (首次) → status）"
echo ""
echo "=========================================="
