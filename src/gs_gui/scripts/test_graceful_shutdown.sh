#!/bin/bash

# 优雅关闭功能测试脚本
# 用于验证地面站关闭时是否正确发送关闭命令到USV

echo "=== 地面站优雅关闭功能测试 ==="
echo ""
echo "测试说明："
echo "1. 本脚本会监听USV的命令topic"
echo "2. 当地面站关闭时，应该看到三个命令："
echo "   - led_off"
echo "   - sound_stop"
echo "   - neck_stop"
echo ""

# 检查参数
if [ $# -eq 0 ]; then
    echo "用法: $0 <usv_namespace>"
    echo "示例: $0 usv_01"
    exit 1
fi

USV_NS=$1

echo "监听 USV: $USV_NS"
echo "按 Ctrl+C 停止监听"
echo ""
echo "=== LED命令监听 ==="
ros2 topic echo /$USV_NS/gs_led_command &
LED_PID=$!

echo "=== 声音命令监听 ==="
ros2 topic echo /$USV_NS/gs_sound_command &
SOUND_PID=$!

echo "=== 扭头命令监听 ==="
ros2 topic echo /$USV_NS/gs_action_command &
ACTION_PID=$!

echo ""
echo "现在请关闭地面站窗口..."
echo "按 Ctrl+C 结束监听"

# 等待用户中断
trap "kill $LED_PID $SOUND_PID $ACTION_PID 2>/dev/null; exit 0" INT

wait
