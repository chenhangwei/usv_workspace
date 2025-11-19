#!/bin/bash
# USV项目代码质量检查脚本
# 自动检测项目中的常见问题

set -e

# 颜色定义
RED='\033[0;31m'
YELLOW='\033[1;33m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

WORKSPACE_ROOT="/home/chenhangwei/usv_workspace/src"

echo "========================================"
echo "  USV项目代码质量检查工具"
echo "========================================"
echo ""

# 统计变量
ISSUES_FOUND=0

# 检查1: 裸 except Exception
echo -e "${BLUE}[1/7] 检查裸 except Exception...${NC}"
COUNT=$(find "$WORKSPACE_ROOT" -name "*.py" -type f ! -path "*/test/*" ! -path "*/__pycache__/*" -exec grep -l "except.*:$" {} \; | wc -l)
if [ "$COUNT" -gt 0 ]; then
    echo -e "${YELLOW}⚠ 发现 $COUNT 个文件存在裸 except${NC}"
    echo "详细位置:"
    find "$WORKSPACE_ROOT" -name "*.py" -type f ! -path "*/test/*" ! -path "*/__pycache__/*" -exec grep -Hn "except.*:$" {} \; | head -10
    echo ""
    ISSUES_FOUND=$((ISSUES_FOUND + COUNT))
else
    echo -e "${GREEN}✓ 未发现裸 except${NC}"
fi
echo ""

# 检查2: print() 调试语句
echo -e "${BLUE}[2/7] 检查 print() 调试语句...${NC}"
COUNT=$(find "$WORKSPACE_ROOT" -name "*.py" -type f ! -path "*/test/*" ! -path "*/__pycache__/*" ! -path "*/scripts/*" -exec grep -l "^\s*print(" {} \; | wc -l)
if [ "$COUNT" -gt 0 ]; then
    echo -e "${YELLOW}⚠ 发现 $COUNT 个文件使用 print() 调试${NC}"
    echo "建议使用 self.get_logger().info/debug() 替代"
    find "$WORKSPACE_ROOT" -name "*.py" -type f ! -path "*/test/*" ! -path "*/__pycache__/*" ! -path "*/scripts/*" -exec grep -Hn "^\s*print(" {} \; | head -5
    echo ""
    ISSUES_FOUND=$((ISSUES_FOUND + COUNT))
else
    echo -e "${GREEN}✓ 未发现 print() 调试语句${NC}"
fi
echo ""

# 检查3: 未使用 SerialResourceManager 的串口
echo -e "${BLUE}[3/7] 检查串口资源管理...${NC}"
COUNT=$(find "$WORKSPACE_ROOT" -name "*.py" -type f ! -path "*/test/*" ! -path "*/__pycache__/*" -exec grep -l "serial.Serial(" {} \; | wc -l)
MANAGED=$(find "$WORKSPACE_ROOT" -name "*.py" -type f -exec grep -l "SerialResourceManager" {} \; | wc -l)
if [ "$COUNT" -gt "$MANAGED" ]; then
    echo -e "${YELLOW}⚠ 发现 $COUNT 个文件使用串口, 只有 $MANAGED 个使用 SerialResourceManager${NC}"
    echo "未管理的文件:"
    comm -23 \
        <(find "$WORKSPACE_ROOT" -name "*.py" -type f -exec grep -l "serial.Serial(" {} \; | sort) \
        <(find "$WORKSPACE_ROOT" -name "*.py" -type f -exec grep -l "SerialResourceManager" {} \; | sort) | head -5
    echo ""
    ISSUES_FOUND=$((ISSUES_FOUND + COUNT - MANAGED))
else
    echo -e "${GREEN}✓ 串口资源管理良好${NC}"
fi
echo ""

# 检查4: subprocess 未使用 ProcessTracker
echo -e "${BLUE}[4/7] 检查 subprocess 管理...${NC}"
COUNT=$(find "$WORKSPACE_ROOT" -name "*.py" -type f ! -path "*/test/*" ! -path "*/__pycache__/*" -exec grep -l "subprocess.Popen" {} \; | wc -l)
MANAGED=$(find "$WORKSPACE_ROOT" -name "*.py" -type f -exec grep -l "ProcessTracker" {} \; | wc -l)
if [ "$COUNT" -gt "$MANAGED" ]; then
    echo -e "${YELLOW}⚠ 发现 $COUNT 个文件使用 subprocess, 只有 $MANAGED 个使用 ProcessTracker${NC}"
    echo "未管理的文件:"
    comm -23 \
        <(find "$WORKSPACE_ROOT" -name "*.py" -type f -exec grep -l "subprocess.Popen" {} \; | sort) \
        <(find "$WORKSPACE_ROOT" -name "*.py" -type f -exec grep -l "ProcessTracker" {} \; | sort) | head -5
    echo ""
    ISSUES_FOUND=$((ISSUES_FOUND + COUNT - MANAGED))
else
    echo -e "${GREEN}✓ subprocess 管理良好${NC}"
fi
echo ""

# 检查5: GPS原点硬编码
echo -e "${BLUE}[5/7] 检查 GPS 原点硬编码...${NC}"
COUNT=$(find "$WORKSPACE_ROOT" -name "*.py" -type f -exec grep -l "22.5180977\|113.9007239" {} \; | wc -l)
if [ "$COUNT" -gt 2 ]; then  # 允许在2个文件中(配置和工具类)
    echo -e "${YELLOW}⚠ 发现 $COUNT 个文件硬编码 GPS 原点${NC}"
    echo "建议使用 ParamLoader.load_gps_origin() 统一加载"
    find "$WORKSPACE_ROOT" -name "*.py" -type f -exec grep -l "22.5180977\|113.9007239" {} \;
    echo ""
    ISSUES_FOUND=$((ISSUES_FOUND + COUNT - 2))
else
    echo -e "${GREEN}✓ GPS 原点配置集中${NC}"
fi
echo ""

# 检查6: 缺少 destroy_node() 的节点
echo -e "${BLUE}[6/7] 检查资源清理方法...${NC}"
NODES=$(find "$WORKSPACE_ROOT" -name "*_node.py" -type f | wc -l)
WITH_DESTROY=$(find "$WORKSPACE_ROOT" -name "*_node.py" -type f -exec grep -l "def destroy_node" {} \; | wc -l)
echo "节点总数: $NODES, 实现 destroy_node(): $WITH_DESTROY"
if [ "$WITH_DESTROY" -lt "$NODES" ]; then
    echo -e "${YELLOW}⚠ ${NODES}个节点中有$((NODES - WITH_DESTROY))个未实现 destroy_node()${NC}"
    echo "缺少清理的节点:"
    comm -23 \
        <(find "$WORKSPACE_ROOT" -name "*_node.py" -type f | sort) \
        <(find "$WORKSPACE_ROOT" -name "*_node.py" -type f -exec grep -l "def destroy_node" {} \; | sort) | head -5
    echo ""
    ISSUES_FOUND=$((ISSUES_FOUND + NODES - WITH_DESTROY))
else
    echo -e "${GREEN}✓ 所有节点实现资源清理${NC}"
fi
echo ""

# 检查7: 线程安全问题
echo -e "${BLUE}[7/7] 检查潜在的线程安全问题...${NC}"
UNSAFE_DICT=$(find "$WORKSPACE_ROOT/gs_gui" -name "*.py" -type f -exec grep -l "self\._.*= {}" {} \; | wc -l)
SAFE_DICT=$(find "$WORKSPACE_ROOT/gs_gui" -name "*.py" -type f -exec grep -l "ThreadSafeDict" {} \; | wc -l)
echo "gs_gui 中字典初始化: $UNSAFE_DICT, 使用 ThreadSafeDict: $SAFE_DICT"
if [ "$UNSAFE_DICT" -gt "$SAFE_DICT" ]; then
    echo -e "${YELLOW}⚠ 可能存在线程安全问题${NC}"
    echo "建议检查以下文件的共享状态:"
    find "$WORKSPACE_ROOT/gs_gui" -name "*.py" -type f -exec grep -Hn "self\._.*= {}" {} \; | head -5
    echo ""
fi
echo ""

# 生成报告
echo "========================================"
echo "  检查完成"
echo "========================================"
echo ""
echo -e "${BLUE}统计摘要:${NC}"
echo "- 发现问题数: $ISSUES_FOUND"
echo ""

if [ "$ISSUES_FOUND" -eq 0 ]; then
    echo -e "${GREEN}✓ 恭喜! 代码质量良好，未发现严重问题${NC}"
    exit 0
elif [ "$ISSUES_FOUND" -lt 10 ]; then
    echo -e "${YELLOW}⚠ 发现少量问题，建议尽快修复${NC}"
    exit 0
else
    echo -e "${RED}✗ 发现较多问题，建议系统性优化${NC}"
    echo ""
    echo "优化建议:"
    echo "1. 查看 OPTIMIZATION_GUIDE.md 了解详细方案"
    echo "2. 查看 QUICK_REFERENCE.md 快速参考"
    echo "3. 优先修复资源泄漏问题(串口、进程)"
    exit 1
fi
