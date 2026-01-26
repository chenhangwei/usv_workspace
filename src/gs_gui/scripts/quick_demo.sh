#!/bin/bash
# =============================================================================
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Implementation of quick demo.
#
# Author: chenhangwei
# Date: 2026-01-26
# =============================================================================
# USV 信息面板 - 快速演示脚本

echo "========================================"
echo "  USV 信息面板 - 快速演示"
echo "========================================"
echo ""

# 检查当前目录
if [ ! -f "gs_gui/usv_info_panel.py" ]; then
    echo "错误: 请在 gs_gui 包的根目录运行此脚本"
    echo "正确路径: /home/chenhangwei/usv_workspace/src/gs_gui"
    echo ""
    echo "使用方法:"
    echo "  cd /home/chenhangwei/usv_workspace/src/gs_gui"
    echo "  bash scripts/quick_demo.sh"
    exit 1
fi

echo "检测到的操作:"
echo "  1) 快速演示 (独立窗口)"
echo "  2) 运行单元测试"
echo "  3) 自动集成到主程序"
echo "  4) 查看文档"
echo ""
read -p "请选择操作 (1-4): " choice

case $choice in
    1)
        echo ""
        echo "启动快速演示..."
        echo "提示: 面板会每3秒自动切换状态，展示不同的 USV 信息"
        echo ""
        python3 scripts/demo_usv_info_panel.py
        ;;
    2)
        echo ""
        echo "运行单元测试..."
        python3 -m pytest test/test_usv_info_panel.py -v
        ;;
    3)
        echo ""
        echo "警告: 自动集成会修改以下文件:"
        echo "  - gs_gui/main_gui_app.py"
        echo "  - gs_gui/state_handler.py"
        echo "  - gs_gui/ui_utils.py"
        echo ""
        echo "原文件会被备份到 backup_YYYYMMDD_HHMMSS/ 目录"
        echo ""
        read -p "确认继续? (y/N): " confirm
        
        if [ "$confirm" = "y" ] || [ "$confirm" = "Y" ]; then
            echo ""
            echo "执行自动集成..."
            python3 scripts/integrate_usv_info_panel.py
            
            echo ""
            echo "集成完成！"
            echo ""
            echo "下一步操作:"
            echo "  1. 查看集成报告: cat INTEGRATION_REPORT.txt"
            echo "  2. 构建项目:"
            echo "     cd /home/chenhangwei/usv_workspace"
            echo "     colcon build --packages-select gs_gui"
            echo "     source install/setup.bash"
            echo "  3. 启动地面站:"
            echo "     ros2 launch gs_bringup gs_launch.py"
        else
            echo "已取消集成"
        fi
        ;;
    4)
        echo ""
        echo "可用文档:"
        echo "  - QUICK_REFERENCE_USV_INFO.md    (快速参考)"
        echo "  - USV_INFO_PANEL_README.md       (项目总结)"
        echo "  - USV_INFO_PANEL_GUIDE.md        (详细集成指南)"
        echo "  - INTEGRATION_COMPLETE.md        (完成总结)"
        echo ""
        read -p "打开哪个文档? (1-4, 或按回车跳过): " doc_choice
        
        case $doc_choice in
            1)
                cat QUICK_REFERENCE_USV_INFO.md | less
                ;;
            2)
                cat USV_INFO_PANEL_README.md | less
                ;;
            3)
                cat USV_INFO_PANEL_GUIDE.md | less
                ;;
            4)
                cat INTEGRATION_COMPLETE.md | less
                ;;
            *)
                echo "已取消"
                ;;
        esac
        ;;
    *)
        echo "无效选择"
        exit 1
        ;;
esac

echo ""
echo "========================================"
echo "  操作完成"
echo "========================================"
