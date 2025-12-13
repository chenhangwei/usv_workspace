"""
参数表格项委托

为不同类型的参数提供自定义编辑器：
- 整数参数：QSpinBox
- 浮点数参数：QDoubleSpinBox  
- 枚举参数：QComboBox
- 位掩码参数：自定义位掩码编辑器

类似 QGroundControl 的参数编辑体验
"""

from PyQt5.QtWidgets import (
    QStyledItemDelegate, QSpinBox, QDoubleSpinBox, QComboBox,
    QWidget, QStyleOptionViewItem, QMessageBox
)
from PyQt5.QtCore import Qt, QModelIndex
from PyQt5.QtGui import QColor, QPainter
from typing import Optional
from .param_manager import ParamInfo, ParamType
from .param_validator import ParamValidator
from .param_metadata import get_param_metadata


class ParamItemDelegate(QStyledItemDelegate):
    """
    参数表格项委托
    
    根据参数类型和元数据提供不同的编辑器：
    - 枚举参数：下拉列表
    - 整数参数：整数输入框（带范围限制）
    - 浮点数参数：浮点数输入框（带范围限制）
    """
    
    def __init__(self, param_manager, parent=None):
        super().__init__(parent)
        self.param_manager = param_manager
    
    def createEditor(self, parent: QWidget, option: QStyleOptionViewItem, 
                     index: QModelIndex) -> Optional[QWidget]:
        """
        创建编辑器
        
        根据参数类型创建合适的编辑器
        """
        # 只为值列创建编辑器
        if index.column() != 1:
            return super().createEditor(parent, option, index)
        
        # 获取参数信息
        param_name = index.model().index(index.row(), 0).data(Qt.DisplayRole)
        param = self.param_manager.get_param(param_name)
        
        if not param:
            return super().createEditor(parent, option, index)
        
        # 获取元数据
        meta = get_param_metadata(param_name)
        
        # 优先使用枚举值编辑器
        if meta and meta.values:
            return self._create_enum_editor(parent, param, meta)
        
        # 根据参数类型创建编辑器
        if param.param_type == ParamType.INTEGER:
            return self._create_int_editor(parent, param, meta)
        elif param.param_type == ParamType.REAL:
            return self._create_float_editor(parent, param, meta)
        else:
            # 默认编辑器
            return super().createEditor(parent, option, index)
    
    def _create_enum_editor(self, parent: QWidget, param: ParamInfo, meta) -> QComboBox:
        """创建枚举值下拉列表"""
        combo = QComboBox(parent)
        combo.setStyleSheet("""
            QComboBox {
                padding: 5px;
                border: 2px solid #3498db;
                border-radius: 3px;
                background-color: white;
                font-size: 14pt;
                min-height: 28px;
            }
            QComboBox::drop-down {
                border: none;
                width: 30px;
            }
            QComboBox::down-arrow {
                image: none;
                border-left: 5px solid transparent;
                border-right: 5px solid transparent;
                border-top: 8px solid #3498db;
                margin-right: 10px;
            }
            QComboBox QAbstractItemView {
                border: 2px solid #3498db;
                selection-background-color: #3498db;
                selection-color: white;
                font-size: 14pt;
            }
        """)
        
        # 添加枚举选项
        current_value = int(param.value)
        current_index = 0
        
        for i, (value, description) in enumerate(sorted(meta.values.items())):
            combo.addItem(f"{value}: {description}", value)
            if value == current_value:
                current_index = i
        
        # 设置当前值
        combo.setCurrentIndex(current_index)
        
        return combo
    
    def _create_int_editor(self, parent: QWidget, param: ParamInfo, meta) -> QSpinBox:
        """创建整数输入框"""
        spinbox = QSpinBox(parent)
        spinbox.setStyleSheet("""
            QSpinBox {
                padding: 5px;
                border: 2px solid #3498db;
                border-radius: 3px;
                background-color: white;
                font-size: 14pt;
                min-height: 28px;
            }
            QSpinBox::up-button, QSpinBox::down-button {
                width: 20px;
                border: none;
            }
            QSpinBox::up-arrow {
                image: none;
                border-left: 5px solid transparent;
                border-right: 5px solid transparent;
                border-bottom: 8px solid #3498db;
            }
            QSpinBox::down-arrow {
                image: none;
                border-left: 5px solid transparent;
                border-right: 5px solid transparent;
                border-top: 8px solid #3498db;
            }
        """)
        
        # 设置范围
        min_val = int(param.min_value) if param.min_value is not None else -2147483648
        max_val = int(param.max_value) if param.max_value is not None else 2147483647
        spinbox.setRange(min_val, max_val)
        
        # 设置步进值
        if meta and meta.increment:
            spinbox.setSingleStep(int(meta.increment))
        else:
            spinbox.setSingleStep(1)
        
        # 设置当前值
        spinbox.setValue(int(param.value))
        
        # 添加工具提示
        tooltip = self._build_tooltip(param, meta)
        if tooltip:
            spinbox.setToolTip(tooltip)
        
        return spinbox
    
    def _create_float_editor(self, parent: QWidget, param: ParamInfo, meta) -> QDoubleSpinBox:
        """创建浮点数输入框"""
        spinbox = QDoubleSpinBox(parent)
        spinbox.setStyleSheet("""
            QDoubleSpinBox {
                padding: 5px;
                border: 2px solid #3498db;
                border-radius: 3px;
                background-color: white;
                font-size: 14pt;
                min-height: 28px;
            }
            QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {
                width: 20px;
                border: none;
            }
            QDoubleSpinBox::up-arrow {
                image: none;
                border-left: 5px solid transparent;
                border-right: 5px solid transparent;
                border-bottom: 8px solid #3498db;
            }
            QDoubleSpinBox::down-arrow {
                image: none;
                border-left: 5px solid transparent;
                border-right: 5px solid transparent;
                border-top: 8px solid #3498db;
            }
        """)
        
        # 设置范围
        min_val = param.min_value if param.min_value is not None else -1e10
        max_val = param.max_value if param.max_value is not None else 1e10
        spinbox.setRange(min_val, max_val)
        
        # 设置精度（小数位数）
        spinbox.setDecimals(6)
        
        # 设置步进值
        if meta and meta.increment:
            spinbox.setSingleStep(meta.increment)
        else:
            # 自动计算合适的步进值
            value_range = max_val - min_val
            if value_range > 1000:
                spinbox.setSingleStep(10.0)
            elif value_range > 100:
                spinbox.setSingleStep(1.0)
            elif value_range > 10:
                spinbox.setSingleStep(0.1)
            else:
                spinbox.setSingleStep(0.01)
        
        # 设置当前值
        spinbox.setValue(param.value)
        
        # 添加工具提示
        tooltip = self._build_tooltip(param, meta)
        if tooltip:
            spinbox.setToolTip(tooltip)
        
        return spinbox
    
    def _build_tooltip(self, param: ParamInfo, meta) -> str:
        """构建工具提示文本"""
        lines = []
        
        # 参数名称
        lines.append(f"<b>{param.name}</b>")
        lines.append("")
        
        # 描述
        if meta and meta.description:
            lines.append(f"<b>描述：</b>{meta.description}")
            if meta.user_description:
                lines.append(f"<i>{meta.user_description}</i>")
            lines.append("")
        
        # 当前值和默认值
        lines.append(f"<b>当前值：</b>{param.value:.6g}")
        if meta and meta.default_value is not None:
            lines.append(f"<b>默认值：</b>{meta.default_value:.6g}")
        lines.append("")
        
        # 范围
        if param.min_value is not None or param.max_value is not None:
            min_str = f"{param.min_value:.6g}" if param.min_value is not None else "∞"
            max_str = f"{param.max_value:.6g}" if param.max_value is not None else "∞"
            lines.append(f"<b>范围：</b>{min_str} ~ {max_str}")
        
        # 单位
        if meta and meta.unit:
            lines.append(f"<b>单位：</b>{meta.unit}")
        
        # 步进值
        if meta and meta.increment:
            lines.append(f"<b>步进：</b>{meta.increment}")
        
        # 重启提示
        if meta and meta.reboot_required:
            lines.append("")
            lines.append("[!] <b>修改此参数需要重启飞控</b>")
        
        # 只读提示
        if meta and meta.read_only:
            lines.append("")
            lines.append("🔒 <b>此参数为只读</b>")
        
        return "<br>".join(lines)
    
    def setEditorData(self, editor: QWidget, index: QModelIndex):
        """
        设置编辑器数据
        
        从模型中读取数据并设置到编辑器
        """
        if isinstance(editor, QComboBox):
            # 枚举编辑器已在创建时设置
            pass
        elif isinstance(editor, (QSpinBox, QDoubleSpinBox)):
            # 数字编辑器已在创建时设置
            pass
        else:
            super().setEditorData(editor, index)
    
    def setModelData(self, editor: QWidget, model, index: QModelIndex):
        """
        将编辑器数据保存到模型
        
        验证数据合法性后保存
        """
        # 获取参数信息
        param_name = model.index(index.row(), 0).data(Qt.DisplayRole)
        param = self.param_manager.get_param(param_name)
        
        if not param:
            return
        
        # 从编辑器获取新值
        if isinstance(editor, QComboBox):
            new_value = float(editor.currentData())
        elif isinstance(editor, QSpinBox):
            new_value = float(editor.value())
        elif isinstance(editor, QDoubleSpinBox):
            new_value = editor.value()
        else:
            super().setModelData(editor, model, index)
            return
        
        # 验证新值
        valid, error_msg = ParamValidator.validate(param, new_value)
        
        if not valid:
            # 验证失败，显示错误
            QMessageBox.warning(
                editor.parentWidget(),
                "参数验证失败",
                f"参数 {param_name} 的值无效：\n\n{error_msg}\n\n"
                f"建议值：{ParamValidator.suggest_valid_value(param, new_value):.6g}"
            )
            return
        
        # 检查警告级别
        warning_level = ParamValidator.get_warning_level(param, new_value)
        
        if warning_level >= 2:
            # 显示警告
            warning_msg = ParamValidator.get_warning_message(param, new_value)
            
            reply = QMessageBox.warning(
                editor.parentWidget(),
                "参数修改警告",
                f"{warning_msg}\n\n是否继续修改？",
                QMessageBox.Yes | QMessageBox.No
            )
            
            if reply != QMessageBox.Yes:
                return
        
        # 保存到模型
        model.setData(index, f"{new_value:.6g}", Qt.DisplayRole)
    
    def paint(self, painter: QPainter, option: QStyleOptionViewItem, index: QModelIndex):
        """
        自定义绘制
        
        为修改的参数添加特殊标记
        """
        # 获取参数信息
        if index.column() == 1:  # 值列
            param_name = index.model().index(index.row(), 0).data(Qt.DisplayRole)
            param = self.param_manager.get_param(param_name)
            
            if param and param.is_modified:
                # 修改背景色
                painter.save()
                painter.fillRect(option.rect, QColor(255, 255, 200))
                painter.restore()
        
        # 默认绘制
        super().paint(painter, option, index)
