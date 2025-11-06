#!/bin/bash
# 低电量模式测试脚本
# 用于验证 LED 和声音节点的低电量功能是否正常启动和工作

echo "=========================================="
echo "低电量模式功能测试"
echo "=========================================="
echo ""

# 进入工作空间
cd /home/chenhangwei/usv_workspace

# Source 环境
source install/setup.bash 2>/dev/null

# 测试 USV 列表
USVS=("usv_01" "usv_02" "usv_03")

echo "📋 第 1 步：检查节点是否在线"
echo "----------------------------------------"
for usv in "${USVS[@]}"; do
    echo -n "  检查 /$usv/usv_led_node ... "
    if ros2 node list 2>/dev/null | grep -q "/$usv/usv_led_node"; then
        echo "✅ 在线"
    else
        echo "❌ 离线"
    fi
    
    echo -n "  检查 /$usv/usv_sound_node ... "
    if ros2 node list 2>/dev/null | grep -q "/$usv/usv_sound_node"; then
        echo "✅ 在线"
    else
        echo "❌ 离线"
    fi
done
echo ""

echo "📋 第 2 步：检查参数配置"
echo "----------------------------------------"
for usv in "${USVS[@]}"; do
    echo "  [$usv] LED 节点参数："
    low_bat=$(ros2 param get /$usv/usv_led_node low_battery_percentage 2>/dev/null | grep -oP '[\d.]+')
    recover_bat=$(ros2 param get /$usv/usv_led_node recover_battery_percentage 2>/dev/null | grep -oP '[\d.]+')
    echo "    low_battery_percentage: ${low_bat}%"
    echo "    recover_battery_percentage: ${recover_bat}%"
    
    echo "  [$usv] 声音节点参数："
    sound_low_bat=$(ros2 param get /$usv/usv_sound_node low_battery_percentage 2>/dev/null | grep -oP '[\d.]+')
    echo "    low_battery_percentage: ${sound_low_bat}%"
    echo ""
done

echo "📋 第 3 步：检查话题订阅"
echo "----------------------------------------"
for usv in "${USVS[@]}"; do
    echo "  [$usv] LED 节点订阅："
    ros2 node info /$usv/usv_led_node 2>/dev/null | grep -A 3 "Subscribers:" | grep -E "battery|usv_status" | sed 's/^/    /'
    
    echo "  [$usv] 声音节点订阅："
    ros2 node info /$usv/usv_sound_node 2>/dev/null | grep -A 3 "Subscribers:" | grep -E "battery|usv_status" | sed 's/^/    /'
    echo ""
done

echo "📋 第 4 步：检查电池状态话题"
echo "----------------------------------------"
for usv in "${USVS[@]}"; do
    echo -n "  检查 /$usv/battery 话题 ... "
    if ros2 topic list 2>/dev/null | grep -q "/$usv/battery"; then
        echo "✅ 存在"
        
        # 检查发布者和订阅者
        info=$(ros2 topic info /$usv/battery 2>/dev/null)
        pub_count=$(echo "$info" | grep "Publisher count:" | grep -oP '\d+')
        sub_count=$(echo "$info" | grep "Subscription count:" | grep -oP '\d+')
        echo "    发布者: ${pub_count}, 订阅者: ${sub_count}"
        
        # 检查话题频率
        echo -n "    检查话题频率 ... "
        hz_output=$(timeout 3 ros2 topic hz /$usv/battery 2>&1)
        if echo "$hz_output" | grep -q "average rate"; then
            rate=$(echo "$hz_output" | grep "average rate" | grep -oP '[\d.]+' | head -1)
            echo "${rate} Hz"
        else
            echo "⚠️ 未检测到消息发布"
        fi
    else
        echo "❌ 不存在"
    fi
    echo ""
done

echo "📋 第 5 步：检查实时电池数据（usv_01）"
echo "----------------------------------------"
echo "  正在监听 /usv_01/battery 消息 (3秒) ..."
timeout 3 ros2 topic echo /usv_01/battery 2>/dev/null | grep -E "voltage:|percentage:" | head -10

echo ""
echo "📋 第 6 步：检查 USV 状态消息（usv_01）"
echo "----------------------------------------"
echo "  正在监听 /usv_01/usv_status 消息 (3秒) ..."
timeout 3 ros2 topic echo /usv_01/usv_status 2>/dev/null | grep -E "battery_voltage:|low_voltage_mode:" | head -10

echo ""
echo "=========================================="
echo "✅ 测试完成"
echo "=========================================="
echo ""
echo "📝 总结："
echo "  1. 如果所有节点在线且参数正确，低电量模式已正常启动"
echo "  2. 节点正在订阅 battery 和 usv_status 话题"
echo "  3. 低电量判断基于飞控百分比（percentage 字段）"
echo "  4. 触发条件："
echo "     - LED: percentage < 10% → 红色呼吸灯"
echo "     - 声音: percentage < 10% → 特殊声音"
echo "  5. 恢复条件："
echo "     - LED: percentage > 15% → 恢复正常"
echo "     - 声音: percentage > 10% → 恢复正常"
echo ""
echo "⚠️ 注意事项："
echo "  - 如果 percentage = 0.0 或 -1.0，说明飞控未配置 BATT_CAPACITY"
echo "  - 必须在 QGroundControl 中设置 BATT_CAPACITY 参数"
echo "  - 详见: src/BATTERY_PERCENTAGE_GUIDE.md"
echo ""
