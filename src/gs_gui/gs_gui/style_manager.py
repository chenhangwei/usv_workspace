"""
样式管理器 - 支持多种主题快速切换
提供现代化UI主题，增强视觉体验
"""
from PyQt5.QtWidgets import QWidget
import os


class StyleManager:
    """Qt样式表管理器"""
    
    # 预定义主题
    THEMES = {
        'modern_dark': 'modern_style.qss',  # 现代深色主题（默认）
        'light': None,  # 浅色主题（未来扩展）
        'classic': None,  # 经典主题（禁用样式表）
    }
    
    def __init__(self, widget: QWidget):
        """
        初始化样式管理器
        
        Args:
            widget: 要应用样式的Qt Widget（通常是MainWindow）
        """
        self.widget = widget
        self.current_theme = 'modern_dark'
        self._resource_dir = self._find_resource_dir()
    
    def _find_resource_dir(self) -> str:
        """查找资源目录路径（优先从源码目录查找）"""
        # 优先尝试开发模式：从源代码目录查找
        current_file = os.path.abspath(__file__)
        dev_resource_dir = os.path.normpath(os.path.join(os.path.dirname(current_file), '..', 'resource'))
        
        if os.path.exists(dev_resource_dir):
            return dev_resource_dir
        
        # 降级到安装模式（使用 share 目录）
        try:
            from ament_index_python.packages import get_package_share_directory
            share_dir = get_package_share_directory('gs_gui')
            install_resource_dir = os.path.join(share_dir, 'resource')
            if os.path.exists(install_resource_dir):
                return install_resource_dir
        except Exception:
            pass
        
        # 最后尝试 pkg_resources（旧方法，兼容性）
        try:
            import pkg_resources
            return pkg_resources.resource_filename('gs_gui', 'resource')
        except Exception:
            pass
        
        # 返回开发模式路径作为后备（即使不存在）
        return dev_resource_dir
    
    def load_theme(self, theme_name: str = 'modern_dark') -> bool:
        """
        加载指定主题
        
        Args:
            theme_name: 主题名称，可选值见 THEMES 字典
            
        Returns:
            bool: 是否成功加载
        """
        if theme_name not in self.THEMES:
            print(f"⚠ 未知主题: {theme_name}，使用默认主题")
            theme_name = 'modern_dark'
        
        qss_file = self.THEMES[theme_name]
        
        # classic主题：不使用样式表
        if qss_file is None:
            self.widget.setStyleSheet("")
            self.current_theme = theme_name
            print(f"✓ 已切换到主题: {theme_name} (无样式表)")
            return True
        
        # 加载QSS文件
        qss_path = os.path.join(self._resource_dir, qss_file)
        
        try:
            if os.path.exists(qss_path):
                with open(qss_path, 'r', encoding='utf-8') as f:
                    stylesheet = f.read()
                    self.widget.setStyleSheet(stylesheet)
                    self.current_theme = theme_name
                    print(f"✓ 已加载主题: {theme_name} ({qss_path})")
                    return True
            else:
                raise FileNotFoundError(f"样式表文件不存在: {qss_path}")
        except Exception as e:
            print(f"✗ 主题加载失败: {e}")
            # 使用后备样式
            self._apply_fallback_style()
            return False
    
    def _apply_fallback_style(self):
        """应用最小化后备样式（当QSS文件加载失败时）"""
        fallback_qss = """
            /* 最小化现代深色主题 */
            QMainWindow { background-color: #1e1e1e; }
            QWidget { color: #e0e0e0; background-color: #1e1e1e; }
            
            QGroupBox {
                border: 2px solid #3a3a3a;
                border-radius: 8px;
                margin-top: 12px;
                padding-top: 10px;
                background-color: #252525;
                color: #4fc3f7;
                font-weight: bold;
            }
            
            QPushButton {
                background-color: #424242;
                color: #e0e0e0;
                border: 1px solid #555555;
                border-radius: 5px;
                padding: 8px 16px;
                min-height: 28px;
            }
            QPushButton:hover { background-color: #4fc3f7; color: #000000; }
            
            QTableView {
                background-color: #2b2b2b;
                border: 1px solid #3a3a3a;
                selection-background-color: #1976d2;
            }
            
            QHeaderView::section {
                background-color: #333333;
                color: #4fc3f7;
                padding: 6px;
                border: 1px solid #3a3a3a;
                font-weight: bold;
            }
        """
        self.widget.setStyleSheet(fallback_qss)
        print("✓ 已应用后备样式")
    
    def toggle_theme(self):
        """在可用主题间切换（用于未来扩展）"""
        available_themes = list(self.THEMES.keys())
        current_index = available_themes.index(self.current_theme)
        next_index = (current_index + 1) % len(available_themes)
        next_theme = available_themes[next_index]
        self.load_theme(next_theme)
