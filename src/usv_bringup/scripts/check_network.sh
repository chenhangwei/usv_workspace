#!/bin/bash
# USV网络配置诊断脚本
# 用于检查MAVLink和DDS端口配置是否正确

echo "========================================="
echo "  USV 网络配置诊断工具"
echo "========================================="
echo ""

# 检查当前机器的IP地址
echo "📡 1. 当前网络接口和IP地址:"
echo "-------------------------------------------"
ip -4 addr show | grep -E "inet " | awk '{print $NF ": " $2}'
echo ""

# 检查预期的USV IP是否存在
echo "🔍 2. USV IP地址检查:"
echo "-------------------------------------------"
USV_IPS=("192.168.68.55:usv_01" "192.168.68.54:usv_02" "192.168.68.52:usv_03")
for entry in "${USV_IPS[@]}"; do
    IFS=':' read -r ip name <<< "$entry"
    if ip addr | grep -q "$ip"; then
        echo "✅ $name ($ip) - 已配置"
    else
        echo "❌ $name ($ip) - 未找到"
    fi
done
echo ""

# 检查MAVLink端口是否被占用
echo "🔌 3. MAVLink端口占用检查:"
echo "-------------------------------------------"
MAVLINK_PORTS=(14550 14551 14552 14553 14560 14570)
for port in "${MAVLINK_PORTS[@]}"; do
    if ss -ulnp | grep -q ":$port "; then
        echo "⚠️  UDP端口 $port - 已被占用"
        ss -ulnp | grep ":$port " | awk '{print "   进程: " $7}'
    else
        echo "✅ UDP端口 $port - 可用"
    fi
done
echo ""

# 检查DDS端口范围
echo "📦 4. DDS端口范围检查:"
echo "-------------------------------------------"
DDS_RANGES=("10150-10160:Domain_11(usv_01)" "10400-10410:Domain_12(usv_02)" "10650-10660:Domain_13(usv_03)" "32150-32160:Domain_99(GS)")
for entry in "${DDS_RANGES[@]}"; do
    IFS=':' read -r range name <<< "$entry"
    IFS='-' read -r start end <<< "$range"
    
    occupied=0
    for ((port=start; port<=end; port++)); do
        if ss -ulnp 2>/dev/null | grep -q ":$port "; then
            ((occupied++))
        fi
    done
    
    if [ $occupied -eq 0 ]; then
        echo "✅ $name ($range) - 所有端口可用"
    else
        echo "⚠️  $name ($range) - $occupied 个端口已占用"
    fi
done
echo ""

# 检查飞控串口
echo "🔗 5. 飞控串口检查:"
echo "-------------------------------------------"
if [ -e "/dev/ttyACM0" ]; then
    echo "✅ /dev/ttyACM0 - 存在"
    ls -l /dev/ttyACM0
    if groups | grep -q dialout; then
        echo "✅ 当前用户在 dialout 组中,有串口访问权限"
    else
        echo "⚠️  当前用户不在 dialout 组中,可能无法访问串口"
        echo "   修复命令: sudo usermod -aG dialout $USER"
        echo "   (需要重新登录生效)"
    fi
else
    echo "❌ /dev/ttyACM0 - 未找到"
    echo "   可能的原因:"
    echo "   - 飞控未连接"
    echo "   - USB线缆故障"
    echo "   - 驱动未安装"
fi
echo ""

# 检查到地面站的网络连通性
echo "🌐 6. 地面站连通性检查:"
echo "-------------------------------------------"
GCS_IP="192.168.68.50"
if ping -c 2 -W 1 $GCS_IP &>/dev/null; then
    echo "✅ 地面站 ($GCS_IP) - 可达"
    ping -c 3 $GCS_IP | tail -2
else
    echo "❌ 地面站 ($GCS_IP) - 不可达"
    echo "   可能的原因:"
    echo "   - 网络未连接"
    echo "   - IP地址配置错误"
    echo "   - 防火墙阻止ICMP"
fi
echo ""

# 检查DDS配置文件
echo "📄 7. DDS配置文件检查:"
echo "-------------------------------------------"
if [ -f "/etc/fastdds/usv_endpoints.xml" ]; then
    echo "✅ /etc/fastdds/usv_endpoints.xml - 存在"
    echo "   端口配置:"
    grep -E "<port>|<address>" /etc/fastdds/usv_endpoints.xml | sed 's/^/   /'
else
    echo "⚠️  /etc/fastdds/usv_endpoints.xml - 未找到"
fi
echo ""

# 检查ROS_DOMAIN_ID环境变量
echo "🌍 8. ROS环境变量检查:"
echo "-------------------------------------------"
if [ -n "$ROS_DOMAIN_ID" ]; then
    echo "✅ ROS_DOMAIN_ID = $ROS_DOMAIN_ID"
else
    echo "⚠️  ROS_DOMAIN_ID 未设置 (将使用默认值 0)"
fi

if [ -n "$FASTRTPS_DEFAULT_PROFILES_FILE" ]; then
    echo "✅ FASTRTPS_DEFAULT_PROFILES_FILE = $FASTRTPS_DEFAULT_PROFILES_FILE"
else
    echo "⚠️  FASTRTPS_DEFAULT_PROFILES_FILE 未设置"
    echo "   建议设置: export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastdds/usv_endpoints.xml"
fi
echo ""

# 推荐配置
echo "========================================="
echo "  📋 推荐配置命令"
echo "========================================="
echo ""
echo "根据当前诊断结果,建议执行以下命令:"
echo ""

# 判断当前是哪个USV
CURRENT_IP=$(ip -4 addr show | grep -oP '192\.168\.68\.\d+' | head -1)
case "$CURRENT_IP" in
    "192.168.68.55")
        echo "# 当前机器识别为: USV_01"
        echo "export ROS_DOMAIN_ID=11"
        echo "export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastdds/usv_endpoints.xml"
        echo "ros2 launch usv_bringup usv_launch.py namespace:=usv_01"
        ;;
    "192.168.68.54")
        echo "# 当前机器识别为: USV_02"
        echo "export ROS_DOMAIN_ID=12"
        echo "export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastdds/usv_endpoints.xml"
        echo "ros2 launch usv_bringup usv_launch.py namespace:=usv_02"
        ;;
    "192.168.68.52")
        echo "# 当前机器识别为: USV_03"
        echo "export ROS_DOMAIN_ID=13"
        echo "export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastdds/usv_endpoints.xml"
        echo "ros2 launch usv_bringup usv_launch.py namespace:=usv_03"
        ;;
    *)
        echo "# ⚠️ 未识别到标准USV IP地址 (当前: $CURRENT_IP)"
        echo "# 如果这是测试环境,可以使用以下通用配置:"
        echo "export ROS_DOMAIN_ID=11"
        echo ""
        echo "# 如果gcs_url绑定失败,请修改launch文件中的本机IP地址"
        echo "# 或使用 0.0.0.0 绑定所有接口:"
        echo "# gcs_url='udp://0.0.0.0:14551@192.168.68.50:14550'"
        ;;
esac
echo ""
echo "========================================="
