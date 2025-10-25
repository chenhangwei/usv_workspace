#!/usr/bin/env python3
"""
USV 信息面板自动集成脚本

此脚本会自动完成以下操作：
1. 备份现有文件
2. 修改必要的 Python 文件
3. 更新 UI 连接逻辑
4. 生成集成报告

使用方法：
    cd /home/chenhangwei/usv_workspace/src/gs_gui
    python3 scripts/integrate_usv_info_panel.py

注意：运行前请确保已经提交当前代码到 git！
"""
import os
import sys
import shutil
import re
from datetime import datetime
from pathlib import Path


class UsvInfoPanelIntegrator:
    """USV 信息面板集成器"""
    
    def __init__(self, workspace_root):
        self.workspace_root = Path(workspace_root)
        self.gs_gui_root = self.workspace_root / 'gs_gui'
        self.backup_dir = self.workspace_root / f'backup_{datetime.now().strftime("%Y%m%d_%H%M%S")}'
        self.report = []
        
    def log(self, message, level="INFO"):
        """记录日志"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        log_message = f"[{timestamp}] [{level}] {message}"
        print(log_message)
        self.report.append(log_message)
    
    def backup_file(self, file_path):
        """备份文件"""
        try:
            rel_path = file_path.relative_to(self.workspace_root)
            backup_path = self.backup_dir / rel_path
            backup_path.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(file_path, backup_path)
            self.log(f"备份文件: {file_path}")
            return True
        except Exception as e:
            self.log(f"备份文件失败 {file_path}: {e}", "ERROR")
            return False
    
    def modify_main_gui_app(self):
        """修改 main_gui_app.py"""
        file_path = self.gs_gui_root / 'gs_gui' / 'main_gui_app.py'
        self.log(f"修改文件: {file_path}")
        
        if not self.backup_file(file_path):
            return False
        
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()
            
            # 添加导入
            if 'from gs_gui.usv_info_panel import UsvInfoPanel' not in content:
                import_line = 'from gs_gui.area_offset_dialog import AreaOffsetDialog'
                content = content.replace(
                    import_line,
                    f"{import_line}\nfrom gs_gui.usv_info_panel import UsvInfoPanel"
                )
                self.log("  ✓ 添加了 UsvInfoPanel 导入")
            
            # 在 __init__ 中添加面板创建代码
            init_marker = "# 在初始化最后刷新表格表头"
            if init_marker in content and 'self.usv_info_panel = UsvInfoPanel()' not in content:
                insert_code = """
        # 创建 USV 信息面板
        self.usv_info_panel = UsvInfoPanel()
        # 清除 groupBox_3 的原有内容
        while self.ui.groupBox_3.layout() and self.ui.groupBox_3.layout().count():
            item = self.ui.groupBox_3.layout().takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        # 将面板添加到 groupBox_3
        if self.ui.groupBox_3.layout():
            self.ui.groupBox_3.layout().addWidget(self.usv_info_panel)
        
        """
                content = content.replace(init_marker, insert_code + init_marker)
                self.log("  ✓ 添加了面板初始化代码")
            
            # 修改 StateHandler 初始化，传入 usv_info_panel
            state_handler_pattern = r'self\.state_handler = StateHandler\(\s*self\.table_manager,\s*self\.list_manager,\s*self\.ui_utils\.append_warning\s*\)'
            if re.search(state_handler_pattern, content):
                content = re.sub(
                    state_handler_pattern,
                    'self.state_handler = StateHandler(\n            self.table_manager,\n            self.list_manager,\n            self.ui_utils.append_warning,\n            self.usv_info_panel\n        )',
                    content
                )
                self.log("  ✓ 修改了 StateHandler 初始化")
            
            with open(file_path, 'w', encoding='utf-8') as f:
                f.write(content)
            
            self.log(f"✅ 成功修改 {file_path.name}")
            return True
        
        except Exception as e:
            self.log(f"修改 main_gui_app.py 失败: {e}", "ERROR")
            return False
    
    def modify_state_handler(self):
        """修改 state_handler.py"""
        file_path = self.gs_gui_root / 'gs_gui' / 'state_handler.py'
        self.log(f"修改文件: {file_path}")
        
        if not self.backup_file(file_path):
            return False
        
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()
            
            # 修改 __init__ 方法，添加 usv_info_panel 参数
            init_pattern = r'def __init__\(self, table_manager, list_manager, append_warning\):'
            if re.search(init_pattern, content):
                content = re.sub(
                    init_pattern,
                    'def __init__(self, table_manager, list_manager, append_warning, usv_info_panel=None):',
                    content
                )
                self.log("  ✓ 修改了 __init__ 方法签名")
                
                # 在 __init__ 中保存 usv_info_panel
                if 'self.usv_info_panel = usv_info_panel' not in content:
                    # 在 self.append_warning = append_warning 后添加
                    content = content.replace(
                        'self.append_warning = append_warning',
                        'self.append_warning = append_warning\n        self.usv_info_panel = usv_info_panel'
                    )
                    self.log("  ✓ 添加了 usv_info_panel 属性")
            
            # 在 _refresh_table 方法中添加面板更新逻辑
            refresh_method_pattern = r'def _refresh_table\(self\):.*?(?=\n    def |\Z)'
            refresh_method = re.search(refresh_method_pattern, content, re.DOTALL)
            
            if refresh_method and 'usv_info_panel.update_state' not in refresh_method.group():
                # 在方法末尾（try块内）添加更新逻辑
                insert_code = """
            # 更新选中 USV 的详细信息面板
            if self.usv_info_panel:
                try:
                    selected_ns = self._get_selected_namespace()
                    if selected_ns:
                        state = self.usv_states.get(selected_ns)
                        self.usv_info_panel.update_state(state)
                    else:
                        self.usv_info_panel.update_state(None)
                except Exception as e:
                    pass
"""
                # 找到最后一个 except Exception 前插入
                content = re.sub(
                    r'(def _refresh_table\(self\):.*?)(except Exception.*?\n)',
                    r'\1' + insert_code + r'\2',
                    content,
                    count=1,
                    flags=re.DOTALL
                )
                self.log("  ✓ 添加了面板更新逻辑")
            
            # 添加 _get_selected_namespace 辅助方法
            if '_get_selected_namespace' not in content:
                helper_method = '''
    def _get_selected_namespace(self):
        """获取当前选中的 USV namespace"""
        try:
            # 从 table_manager 的集群表格获取选中行
            table_view = self.table_manager.cluster_table_view
            selected_indexes = table_view.selectedIndexes()
            
            if not selected_indexes:
                return None
            
            selected_row = selected_indexes[0].row()
            model = table_view.model()
            if model is None:
                return None
            
            # 第一列是 namespace
            index = model.index(selected_row, 0)
            namespace = model.data(index)
            return namespace
        except Exception:
            return None
'''
                # 在类的末尾添加
                content = content.rstrip() + '\n' + helper_method + '\n'
                self.log("  ✓ 添加了 _get_selected_namespace 方法")
            
            with open(file_path, 'w', encoding='utf-8') as f:
                f.write(content)
            
            self.log(f"✅ 成功修改 {file_path.name}")
            return True
        
        except Exception as e:
            self.log(f"修改 state_handler.py 失败: {e}", "ERROR")
            return False
    
    def modify_ui_utils(self):
        """修改 ui_utils.py"""
        file_path = self.gs_gui_root / 'gs_gui' / 'ui_utils.py'
        self.log(f"修改文件: {file_path}")
        
        if not self.backup_file(file_path):
            return False
        
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()
            
            # 修改 update_selected_table_row 方法
            # 这个方法现在可以简化或移除，因为逻辑移到了 state_handler
            # 这里我们选择保留但添加注释说明
            
            if 'update_selected_table_row' in content:
                # 在方法前添加废弃注释
                content = content.replace(
                    'def update_selected_table_row(',
                    '''# NOTE: 此方法已部分废弃，USV 详细信息现在由 StateHandler 通过 UsvInfoPanel 更新
    # 保留此方法是为了向后兼容
    def update_selected_table_row('''
                )
                self.log("  ✓ 添加了废弃注释")
            
            with open(file_path, 'w', encoding='utf-8') as f:
                f.write(content)
            
            self.log(f"✅ 成功修改 {file_path.name}")
            return True
        
        except Exception as e:
            self.log(f"修改 ui_utils.py 失败: {e}", "ERROR")
            return False
    
    def generate_report(self):
        """生成集成报告"""
        report_path = self.gs_gui_root / 'INTEGRATION_REPORT.txt'
        self.log(f"生成集成报告: {report_path}")
        
        try:
            with open(report_path, 'w', encoding='utf-8') as f:
                f.write("=" * 70 + "\n")
                f.write("USV 信息面板集成报告\n")
                f.write("=" * 70 + "\n\n")
                
                f.write(f"备份目录: {self.backup_dir}\n\n")
                
                f.write("集成日志:\n")
                f.write("-" * 70 + "\n")
                for line in self.report:
                    f.write(line + "\n")
                f.write("-" * 70 + "\n\n")
                
                f.write("下一步操作:\n")
                f.write("1. 检查修改后的文件是否正确\n")
                f.write("2. 重新构建项目: colcon build --packages-select gs_gui\n")
                f.write("3. Source 环境: source install/setup.bash\n")
                f.write("4. 运行测试: python3 -m pytest test/test_usv_info_panel.py -v\n")
                f.write("5. 启动地面站: ros2 launch gs_bringup gs_launch.py\n\n")
                
                f.write("如果遇到问题，可以从备份目录恢复文件。\n")
            
            self.log(f"✅ 集成报告已生成")
            return True
        
        except Exception as e:
            self.log(f"生成报告失败: {e}", "ERROR")
            return False
    
    def run(self):
        """运行集成流程"""
        self.log("=" * 70)
        self.log("开始 USV 信息面板自动集成")
        self.log("=" * 70)
        
        # 检查文件存在
        required_files = [
            self.gs_gui_root / 'gs_gui' / 'main_gui_app.py',
            self.gs_gui_root / 'gs_gui' / 'state_handler.py',
            self.gs_gui_root / 'gs_gui' / 'ui_utils.py',
            self.gs_gui_root / 'gs_gui' / 'usv_info_panel.py'
        ]
        
        for file_path in required_files:
            if not file_path.exists():
                self.log(f"错误：文件不存在 {file_path}", "ERROR")
                return False
        
        self.log("✓ 所有必需文件存在")
        
        # 创建备份目录
        self.backup_dir.mkdir(parents=True, exist_ok=True)
        self.log(f"✓ 创建备份目录: {self.backup_dir}")
        
        # 执行修改
        success = True
        success &= self.modify_main_gui_app()
        success &= self.modify_state_handler()
        success &= self.modify_ui_utils()
        
        # 生成报告
        self.generate_report()
        
        if success:
            self.log("=" * 70)
            self.log("✅ 集成完成！")
            self.log("=" * 70)
            self.log("\n请执行以下命令完成最后步骤：")
            self.log("  cd /home/chenhangwei/usv_workspace")
            self.log("  colcon build --packages-select gs_gui")
            self.log("  source install/setup.bash")
            self.log("  ros2 launch gs_bringup gs_launch.py")
        else:
            self.log("=" * 70)
            self.log("❌ 集成过程中出现错误，请查看日志", "ERROR")
            self.log("=" * 70)
        
        return success


def main():
    """主函数"""
    if len(sys.argv) > 1:
        workspace_root = Path(sys.argv[1])
    else:
        # 默认路径
        workspace_root = Path('/home/chenhangwei/usv_workspace/src')
    
    if not workspace_root.exists():
        print(f"错误：工作空间路径不存在 {workspace_root}")
        return 1
    
    integrator = UsvInfoPanelIntegrator(workspace_root)
    success = integrator.run()
    
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
