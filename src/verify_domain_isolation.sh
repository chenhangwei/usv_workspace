#!/bin/bash
# Domain 隔离架构验证脚本
# 用于验证地面站是否正确从配置文件加载 USV 列表

set -e

echo "========================================================"
echo "🔍 Domain 隔离架构验证脚本"
echo "========================================================"
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查函数
check_pass() {
    echo -e "${GREEN}✓${NC} $1"
}

check_fail() {
    echo -e "${RED}✗${NC} $1"
}

check_warn() {
    echo -e "${YELLOW}⚠${NC} $1"
}

# 1. 检查环境变量
echo "1️⃣  检查环境配置"
echo "---"

if [ -z "$ROS_DOMAIN_ID" ]; then
    check_fail "ROS_DOMAIN_ID 未设置"
    echo "   请运行: export ROS_DOMAIN_ID=99"
    exit 1
else
    if [ "$ROS_DOMAIN_ID" == "99" ]; then
        check_pass "ROS_DOMAIN_ID = $ROS_DOMAIN_ID (地面站)"
    else
        check_warn "ROS_DOMAIN_ID = $ROS_DOMAIN_ID (不是地面站 Domain)"
    fi
fi

# 2. 检查配置文件
echo ""
echo "2️⃣  检查配置文件"
echo "---"

FLEET_CONFIG_SRC="$HOME/usv_workspace/src/gs_bringup/config/usv_fleet.yaml"
FLEET_CONFIG_INSTALL="$HOME/usv_workspace/install/gs_bringup/share/gs_bringup/config/usv_fleet.yaml"

if [ -f "$FLEET_CONFIG_SRC" ]; then
    check_pass "配置文件存在: $FLEET_CONFIG_SRC"
    
    # 统计启用的 USV
    ENABLED_COUNT=$(grep -c "enabled: true" "$FLEET_CONFIG_SRC" || true)
    DISABLED_COUNT=$(grep -c "enabled: false" "$FLEET_CONFIG_SRC" || true)
    
    check_pass "已启用 USV: $ENABLED_COUNT"
    if [ $DISABLED_COUNT -gt 0 ]; then
        check_warn "已禁用 USV: $DISABLED_COUNT"
    fi
    
    # 列出已启用的 USV
    echo ""
    echo "   已启用的 USV:"
    while IFS= read -r line; do
        if [[ $line =~ ^[[:space:]]*([a-zA-Z0-9_]+):[[:space:]]*$ ]]; then
            USV_NAME="${BASH_REMATCH[1]}"
            CURRENT_USV="$USV_NAME"
        elif [[ $line =~ enabled:[[:space:]]*true ]]; then
            echo "      - $CURRENT_USV"
        fi
    done < "$FLEET_CONFIG_SRC"
else
    check_fail "配置文件不存在: $FLEET_CONFIG_SRC"
    exit 1
fi

# 3. 检查 Domain Bridge 配置
echo ""
echo "3️⃣  检查 Domain Bridge 配置"
echo "---"

BRIDGE_CONFIG="$HOME/domain_bridge/domain_bridge.yaml"

if [ -f "$BRIDGE_CONFIG" ]; then
    check_pass "Domain Bridge 配置存在"
    
    # 统计 Domain 和规则
    DOMAIN_COUNT=$(grep -c "^  - id:" "$BRIDGE_CONFIG" || true)
    RULE_COUNT=$(grep -c "^  - topic:" "$BRIDGE_CONFIG" || true)
    
    check_pass "已配置 Domain: $DOMAIN_COUNT"
    check_pass "转发规则: $RULE_COUNT 条"
    
    # 检查地面站 Domain (99)
    if grep -q "id: 99" "$BRIDGE_CONFIG"; then
        check_pass "地面站 Domain (99) 已配置"
    else
        check_fail "地面站 Domain (99) 未配置"
    fi
else
    check_warn "Domain Bridge 配置不存在: $BRIDGE_CONFIG"
    echo "   如果不使用 Domain Bridge，可以忽略此警告"
fi

# 4. 检查 Domain Bridge 运行状态
echo ""
echo "4️⃣  检查 Domain Bridge 运行状态"
echo "---"

if pgrep -f "domain_bridge" > /dev/null; then
    check_pass "Domain Bridge 正在运行"
else
    check_warn "Domain Bridge 未运行"
    echo "   启动方法: ros2 launch gs_bringup domain_bridge.launch.py"
fi

# 5. 检查地面站节点
echo ""
echo "5️⃣  检查地面站节点"
echo "---"

if pgrep -f "main_gui_app" > /dev/null; then
    check_pass "地面站节点正在运行"
    
    # 检查节点信息
    if command -v ros2 &> /dev/null; then
        NODE_INFO=$(ros2 node list 2>/dev/null | grep -c "main_gui_app" || true)
        if [ $NODE_INFO -gt 0 ]; then
            check_pass "地面站节点已注册到 ROS 图"
        fi
    fi
else
    check_warn "地面站节点未运行"
    echo "   启动方法: ros2 launch gs_bringup gs_launch.py"
fi

# 6. 检查话题列表
echo ""
echo "6️⃣  检查话题列表"
echo "---"

if command -v ros2 &> /dev/null; then
    TOPIC_COUNT=$(ros2 topic list 2>/dev/null | grep -c "usv_" || true)
    
    if [ $TOPIC_COUNT -gt 0 ]; then
        check_pass "发现 $TOPIC_COUNT 个 USV 相关话题"
        
        # 列出一些关键话题
        echo ""
        echo "   关键话题:"
        ros2 topic list 2>/dev/null | grep "usv_" | grep -E "(usv_state|set_usv_mode)" | head -6 | while read topic; do
            echo "      - $topic"
        done
    else
        check_warn "未发现 USV 话题"
        echo "   可能原因:"
        echo "      - USV 未启动"
        echo "      - Domain Bridge 未运行"
        echo "      - Domain ID 配置错误"
    fi
else
    check_warn "ros2 命令不可用，跳过话题检查"
fi

# 7. 总结
echo ""
echo "========================================================"
echo "📊 验证结果总结"
echo "========================================================"
echo ""

if [ "$ROS_DOMAIN_ID" == "99" ] && [ -f "$FLEET_CONFIG_SRC" ]; then
    echo -e "${GREEN}✓ 基础配置正确${NC}"
    echo ""
    echo "🚀 下一步:"
    echo "   1. 确保 Domain Bridge 正在运行"
    echo "      ./src/gs_bringup/scripts/domain_bridge.sh start"
    echo ""
    echo "   2. 启动地面站"
    echo "      ros2 launch gs_bringup gs_launch.py"
    echo ""
    echo "   3. 在各 USV 上启动节点"
    echo "      export ROS_DOMAIN_ID=11  # 根据 USV 调整"
    echo "      ros2 launch usv_bringup usv_launch.py"
else
    echo -e "${RED}✗ 配置存在问题，请检查上述错误${NC}"
    exit 1
fi

echo ""
echo "========================================================"
echo "📚 相关文档"
echo "========================================================"
echo "   - 架构说明: src/DOMAIN_ISOLATION_ARCHITECTURE.md"
echo "   - 迁移指南: src/DOMAIN_ISOLATION_MIGRATION.md"
echo "   - 部署指南: src/DOMAIN_BRIDGE_DEPLOYMENT.md"
echo ""
