#!/bin/bash
# EKF Origin 生效验证脚本

echo "=========================================="
echo "EKF Origin 生效验证"
echo "=========================================="
echo ""

# 进入工作空间并 source 环境
cd ~/usv_workspace
source install/setup.bash

echo "1️⃣ 检查启动日志中的 EKF Origin 设置消息..."
echo "   查找: '✅ EKF Origin set successfully'"
echo ""

echo "2️⃣ 检查 Home Position (验证坐标系原点)..."
echo "   预期: latitude ≈ 22.518, longitude ≈ 113.901"
ros2 topic echo /usv_01/home_position/home --once 2>/dev/null | grep -A 3 "geo:" || echo "   ⚠️  Home Position topic 无数据"
echo ""

echo "3️⃣ 检查 Local Position 是否有效..."
echo "   预期: frame_id = 'map', 有数值输出"
ros2 topic echo /usv_01/local_position/pose --once 2>/dev/null | grep -A 10 "pose:" || echo "   ⚠️  Local Position topic 无数据"
echo ""

echo "4️⃣ 检查 USV 控制节点日志..."
echo "   查找关键消息:"
echo "   - '✅ Local Position 有效'"
echo "   - '✅ Home Position 已设置'"
echo "   - '🎯 EKF Origin 完全就绪'"
echo ""

echo "5️⃣ 检查 Global Position (GPS 坐标)..."
ros2 topic echo /usv_01/global_position/global --once 2>/dev/null | grep -E "(latitude|longitude|altitude):" || echo "   ⚠️  Global Position topic 无数据"
echo ""

echo "=========================================="
echo "验证完成！"
echo "=========================================="
echo ""
echo "✅ 如果看到以上所有数据，说明 EKF Origin 已生效"
echo "⚠️  如果有警告，请检查 MAVROS 和 GPS 状态"
