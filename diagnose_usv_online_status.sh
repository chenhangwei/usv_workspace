#!/bin/bash
# USV 在线状态诊断脚本
# 帮助排查为什么 USV ping 通了但显示离线

echo "=========================================="
echo "USV 在线状态诊断工具"
echo "=========================================="
echo ""

# 进入工作空间
cd /home/chenhangwei/usv_workspace
source install/setup.bash 2>/dev/null

# 检查的 USV 列表
USVS=("usv_01" "usv_02" "usv_03")
USV_IPS=("192.168.68.51" "192.168.68.52" "192.168.68.53")

echo "📋 第 1 步：检查本机 ROS_DOMAIN_ID"
echo "----------------------------------------"
echo "  当前 ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-未设置（使用默认值0）}"
echo ""

echo "📋 第 2 步：检查网络连通性"
echo "----------------------------------------"
for i in "${!USVS[@]}"; do
    usv="${USVS[$i]}"
    ip="${USV_IPS[$i]}"
    echo -n "  [$usv] Ping $ip ... "
    if ping -c 1 -W 1 "$ip" &>/dev/null; then
        echo "✅ 通"
    else
        echo "❌ 不通"
    fi
done
echo ""

echo "📋 第 3 步：检查 ROS 2 节点"
echo "----------------------------------------"
echo "  正在扫描 ROS 图..."
node_list=$(timeout 5 ros2 node list 2>/dev/null)

for usv in "${USVS[@]}"; do
    echo "  [$usv] ROS 节点检测:"
    count=$(echo "$node_list" | grep "/$usv/" | wc -l)
    if [ "$count" -gt 0 ]; then
        echo "    ✅ 检测到 $count 个节点"
        echo "$node_list" | grep "/$usv/" | head -3 | sed 's/^/      /'
        if [ "$count" -gt 3 ]; then
            echo "      ... (共 $count 个节点)"
        fi
    else
        echo "    ❌ 未检测到节点"
    fi
done
echo ""

echo "📋 第 4 步：检查 USV 状态话题"
echo "----------------------------------------"
for usv in "${USVS[@]}"; do
    echo "  [$usv] 状态话题检测:"
    
    # 检查话题是否存在
    if ros2 topic list 2>/dev/null | grep -q "/$usv/usv_status"; then
        echo "    ✅ /$usv/usv_status 话题存在"
        
        # 检查发布者
        info=$(ros2 topic info /$usv/usv_status 2>/dev/null)
        pub_count=$(echo "$info" | grep "Publisher count:" | grep -oP '\d+')
        sub_count=$(echo "$info" | grep "Subscription count:" | grep -oP '\d+')
        echo "      发布者: $pub_count, 订阅者: $sub_count"
        
        # 检查是否有消息发布
        echo -n "      检查消息发布 ... "
        if timeout 3 ros2 topic echo /$usv/usv_status --once &>/dev/null; then
            echo "✅ 有消息"
        else
            echo "❌ 无消息（3秒超时）"
        fi
    else
        echo "    ❌ /$usv/usv_status 话题不存在"
    fi
    echo ""
done

echo "📋 第 5 步：检查地面站节点"
echo "----------------------------------------"
if echo "$node_list" | grep -q "/main_gui_app"; then
    echo "  ✅ 地面站节点在线: /main_gui_app"
    
    # 检查是否订阅了 USV 状态
    echo ""
    echo "  地面站订阅的 USV 状态话题:"
    for usv in "${USVS[@]}"; do
        if ros2 topic info /$usv/usv_status 2>/dev/null | grep -q "main_gui_app"; then
            echo "    ✅ /$usv/usv_status"
        else
            echo "    ❌ /$usv/usv_status (未订阅)"
        fi
    done
else
    echo "  ❌ 地面站节点离线"
fi
echo ""

echo "📋 第 6 步：DDS 发现检测"
echo "----------------------------------------"
echo "  检查 DDS 多播是否工作..."
echo "  (如果地面站能看到 USV 节点，说明 DDS 发现正常)"
echo ""
for usv in "${USVS[@]}"; do
    count=$(echo "$node_list" | grep "/$usv/" | wc -l)
    if [ "$count" -gt 0 ]; then
        echo "  [$usv] ✅ DDS 发现正常（检测到 $count 个节点）"
    else
        echo "  [$usv] ❌ DDS 发现失败（可能是多播被阻止或 DOMAIN_ID 不匹配）"
    fi
done
echo ""

echo "=========================================="
echo "诊断完成"
echo "=========================================="
echo ""

echo "📝 判定逻辑说明："
echo "  在线条件："
echo "    1. 网络 ping 通 ✅"
echo "    2. ROS 节点在线 ✅"
echo "    3. usv_status 话题存在 ✅"
echo "    4. 有消息发布 ✅"
echo "    5. 地面站订阅成功 ✅"
echo ""
echo "  只有满足以上 5 个条件，USV 才会在地面站显示为在线"
echo "  缺少任何一个，都会显示离线或不在列表中"
echo ""

echo "🔧 常见问题排查："
echo "  1. 如果 ping 通但 ROS 节点不在线："
echo "     → 检查机载计算机的 USV launch 是否运行"
echo "     → ssh 到机载，执行: ros2 node list"
echo ""
echo "  2. 如果节点在线但话题无消息："
echo "     → 检查 usv_status_node 是否正常"
echo "     → 检查飞控连接（MAVROS）"
echo ""
echo "  3. 如果话题有消息但地面站未订阅："
echo "     → 检查 ROS_DOMAIN_ID 是否一致"
echo "     → 检查防火墙是否阻止多播"
echo "     → 尝试: export ROS_DOMAIN_ID=0 && 重启地面站"
echo ""
echo "  4. 如果地面站显示离线但实际在线："
echo "     → 等待 4-6 秒（系统需要连续检测确认）"
echo "     → 检查 /rosout 日志是否有错误"
echo ""
