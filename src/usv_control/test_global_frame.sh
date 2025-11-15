#!/bin/bash
# 坐标转换节点 - 全局GPS坐标模式快速测试脚本

echo "====================================="
echo "坐标转换节点测试"
echo "====================================="
echo ""

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# 检查命名空间参数
USV_NS=${1:-usv_01}

echo -e "${BLUE}目标 USV: ${USV_NS}${NC}"
echo ""

# 1. 检查 coord_transform_node 配置
echo -e "${YELLOW}[1/6] 检查 coord_transform_node 配置${NC}"
echo "-------------------------------"

COORD_ENABLED=$(ros2 param get /${USV_NS}/coord_transform_node enable_coord_transform 2>/dev/null | grep -oP '(?<=value: ).*')
USE_GLOBAL_TARGET=$(ros2 param get /${USV_NS}/coord_transform_node use_global_position_target 2>/dev/null | grep -oP '(?<=value: ).*')

if [ -n "$COORD_ENABLED" ]; then
    if [ "$COORD_ENABLED" == "True" ]; then
        echo -e "${GREEN}✓ 坐标转换已启用${NC}"
        echo "  use_global_position_target: $USE_GLOBAL_TARGET"
        
        GPS_LAT=$(ros2 param get /${USV_NS}/coord_transform_node gps_origin_lat 2>/dev/null | grep -oP '[\d.]+')
        GPS_LON=$(ros2 param get /${USV_NS}/coord_transform_node gps_origin_lon 2>/dev/null | grep -oP '[\d.]+')
        GPS_ALT=$(ros2 param get /${USV_NS}/coord_transform_node gps_origin_alt 2>/dev/null | grep -oP '[-\d.]+')
        echo "  GPS 原点: ($GPS_LAT°, $GPS_LON°, ${GPS_ALT}m)"
    else
        echo -e "${BLUE}ℹ 坐标转换已禁用（使用局部坐标模式）${NC}"
    fi
else
    echo -e "${RED}✗ coord_transform_node 未运行${NC}"
fi
echo ""

# 2. 检查节点运行状态
echo -e "${YELLOW}[2/6] 检查节点运行状态${NC}"
echo "-------------------------------"

if ros2 node list 2>/dev/null | grep -q "${USV_NS}/coord_transform_node"; then
    echo -e "${GREEN}✓ coord_transform_node 正在运行${NC}"
else
    echo -e "${BLUE}ℹ coord_transform_node 未运行（可能使用局部坐标）${NC}"
fi

if ros2 node list 2>/dev/null | grep -q "${USV_NS}/usv_control_node"; then
    echo -e "${GREEN}✓ usv_control_node 正在运行${NC}"
else
    echo -e "${RED}✗ usv_control_node 未运行${NC}"
fi
echo ""

# 3. 检查话题
echo -e "${YELLOW}[3/6] 检查发布话题${NC}"
echo "-------------------------------"
if ros2 topic list 2>/dev/null | grep -q "/${USV_NS}/setpoint_raw/global"; then
    echo -e "${GREEN}✓ 全局坐标话题: /${USV_NS}/setpoint_raw/global${NC}"
fi

if ros2 topic list 2>/dev/null | grep -q "/${USV_NS}/setpoint_raw/local"; then
    echo -e "${GREEN}✓ 局部坐标话题: /${USV_NS}/setpoint_raw/local${NC}"
fi

if ros2 topic list 2>/dev/null | grep -q "/${USV_NS}/set_usv_target_position"; then
    echo -e "${GREEN}✓ 目标点输入话题: /${USV_NS}/set_usv_target_position${NC}"
fi
echo ""

# 4. 发送测试目标点
echo -e "${YELLOW}[4/6] 发送测试目标点${NC}"
echo "-------------------------------"
echo "发送目标: X=10m (东), Y=5m (北), Z=0m"
ros2 topic pub --once /${USV_NS}/set_usv_target_position geometry_msgs/msg/PoseStamped \
'{
  header: {frame_id: "map"},
  pose: {
    position: {x: 10.0, y: 5.0, z: 0.0}
  }
}' >/dev/null 2>&1

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ 目标点已发送${NC}"
else
    echo -e "✗ 发送失败"
fi
echo ""

# 5. 监听坐标转换输出
echo -e "${YELLOW}[5/6] 监听坐标转换 (5秒)${NC}"
echo "-------------------------------"
echo "等待 coord_transform_node 输出..."

timeout 5s ros2 topic echo /rosout 2>/dev/null | grep -m 1 "coord_transform" | grep "XYZ→GPS"

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ 检测到坐标转换输出${NC}"
else
    echo -e "${BLUE}ℹ 未检测到转换日志（可能无新目标点）${NC}"
fi
echo ""

# 6. 显示话题数据预览
echo -e "${YELLOW}[6/6] 话题数据预览${NC}"
echo "-------------------------------"

# 尝试监听全局坐标
echo "检查全局坐标话题..."
timeout 2s ros2 topic echo /${USV_NS}/setpoint_raw/global --once 2>/dev/null | head -15
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ 全局GPS坐标输出正常${NC}"
else
    # 尝试监听局部坐标
    echo "检查局部坐标话题..."
    timeout 2s ros2 topic echo /${USV_NS}/setpoint_raw/local --once 2>/dev/null | head -15
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ 局部坐标输出正常${NC}"
    else
        echo -e "${BLUE}ℹ 暂无坐标输出${NC}"
    fi
fi
echo ""

# 总结
echo "====================================="
echo -e "${GREEN}测试完成！${NC}"
echo "====================================="
echo ""
echo "📊 实时监控命令:"
echo "  coord_transform日志: ros2 topic echo /rosout | grep 'coord_transform'"
echo "  全局坐标输出: ros2 topic echo /${USV_NS}/setpoint_raw/global"
echo "  局部坐标输出: ros2 topic echo /${USV_NS}/setpoint_raw/local"
echo ""
echo "⚙️  切换坐标模式:"
echo "  编辑: usv_bringup/config/usv_params.yaml"
echo "  启用全局GPS: enable_coord_transform: true"
echo "  禁用全局GPS: enable_coord_transform: false"
echo ""
echo "📖 详细文档:"
echo "  架构说明: src/usv_control/COORDINATE_ARCHITECTURE.md"
echo "  测试指南: src/usv_control/TEST_GLOBAL_FRAME.md"
echo ""
