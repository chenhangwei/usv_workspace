#!/bin/bash
# 参数管理系统 QGC 升级 - 快速验证脚本

echo "=========================================="
echo "USV 参数管理系统 QGC 升级 - 验证清单"
echo "=========================================="
echo ""

# 颜色定义
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查函数
check_file() {
    if [ -f "$1" ]; then
        echo -e "${GREEN}✅${NC} $2"
        return 0
    else
        echo -e "${RED}❌${NC} $2"
        return 1
    fi
}

check_success=0
check_total=0

echo "1. 核心代码文件检查"
echo "-------------------------------------------"

((check_total++))
if check_file "gs_gui/gs_gui/param_metadata.py" "param_metadata.py (元数据系统)"; then
    ((check_success++))
fi

((check_total++))
if check_file "gs_gui/gs_gui/param_validator.py" "param_validator.py (验证系统)"; then
    ((check_success++))
fi

((check_total++))
if check_file "gs_gui/gs_gui/param_manager.py" "param_manager.py (管理器)"; then
    ((check_success++))
fi

echo ""
echo "2. 文档文件检查"
echo "-------------------------------------------"

((check_total++))
if check_file "gs_gui/PARAM_QGC_UPGRADE_GUIDE.md" "升级指南"; then
    ((check_success++))
fi

((check_total++))
if check_file "gs_gui/PARAM_QGC_QUICK_START.md" "快速开始"; then
    ((check_success++))
fi

((check_total++))
if check_file "PARAM_QGC_ACHIEVEMENT_SUMMARY.md" "成果总结"; then
    ((check_success++))
fi

((check_total++))
if check_file "gs_gui/PARAM_DEMO_SCRIPT.md" "演示脚本"; then
    ((check_success++))
fi

((check_total++))
if check_file "PARAM_DELIVERY_PACKAGE.md" "交付清单"; then
    ((check_success++))
fi

echo ""
echo "3. 测试脚本检查"
echo "-------------------------------------------"

((check_total++))
if check_file "gs_gui/scripts/demo_param_qgc_features.py" "功能演示脚本"; then
    ((check_success++))
fi

((check_total++))
if check_file "gs_gui/scripts/param_diagnostic.py" "诊断脚本"; then
    ((check_success++))
fi

echo ""
echo "4. 功能验证"
echo "-------------------------------------------"

cd /home/chenhangwei/usv_workspace

echo -n "检查元数据加载... "
if python3 -c "from gs_gui.param_metadata import load_all_metadata; load_all_metadata(); print('OK')" 2>/dev/null | grep -q "OK"; then
    echo -e "${GREEN}✅${NC}"
    ((check_success++))
else
    echo -e "${RED}❌${NC}"
fi
((check_total++))

echo -n "检查参数验证器... "
if python3 -c "from gs_gui.param_validator import ParamValidator; print('OK')" 2>/dev/null | grep -q "OK"; then
    echo -e "${GREEN}✅${NC}"
    ((check_success++))
else
    echo -e "${RED}❌${NC}"
fi
((check_total++))

echo -n "检查参数管理器集成... "
if python3 -c "from gs_gui.param_manager import ParamManager; print('OK')" 2>/dev/null | grep -q "OK"; then
    echo -e "${GREEN}✅${NC}"
    ((check_success++))
else
    echo -e "${RED}❌${NC}"
fi
((check_total++))

echo ""
echo "5. 构建状态检查"
echo "-------------------------------------------"

echo -n "检查 gs_gui 包是否已构建... "
if [ -d "install/gs_gui" ]; then
    echo -e "${GREEN}✅${NC}"
    ((check_success++))
else
    echo -e "${RED}❌${NC} (需要运行: colcon build --packages-select gs_gui)"
fi
((check_total++))

echo ""
echo "=========================================="
echo "验证结果汇总"
echo "=========================================="

percentage=$((check_success * 100 / check_total))

echo "通过: $check_success / $check_total ($percentage%)"
echo ""

if [ $check_success -eq $check_total ]; then
    echo -e "${GREEN}✅ 所有检查通过！系统已就绪。${NC}"
    echo ""
    echo "下一步操作："
    echo "  1. 运行演示: python3 src/gs_gui/scripts/demo_param_qgc_features.py"
    echo "  2. 启动地面站: ros2 launch gs_bringup gs_launch.py"
    echo "  3. 查看文档: cat src/PARAM_QGC_UPGRADE_GUIDE.md"
    exit 0
elif [ $percentage -ge 80 ]; then
    echo -e "${YELLOW}⚠️  大部分检查通过，但有少量问题需要解决。${NC}"
    exit 1
else
    echo -e "${RED}❌ 检查失败较多，请检查文件完整性。${NC}"
    exit 2
fi
