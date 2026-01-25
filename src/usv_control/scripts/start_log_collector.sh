#!/bin/bash
#
# USV 日志收集启动脚本
# 
# 用法:
#   ./start_log_collector.sh
#

echo "📊 启动 USV 导航日志收集..."
echo "   日志保存位置: ~/usv_logs/"
echo ""

# 检查日志目录
mkdir -p ~/usv_logs

# 启动日志收集节点
ros2 run usv_control log_collector

echo ""
echo "日志收集已停止"
