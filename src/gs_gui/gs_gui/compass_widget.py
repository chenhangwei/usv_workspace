"""
罗盘/航向指示器 Widget
类似 QGroundControl 的圆形航向显示
"""
from PyQt5.QtWidgets import QWidget
from PyQt5.QtCore import Qt, QPointF, QRectF
from PyQt5.QtGui import QPainter, QPen, QBrush, QColor, QFont, QPolygonF


class CompassWidget(QWidget):
    """
    罗盘航向指示器
    
    显示一个圆形罗盘，带有：
    - 基本方向标记（N, E, S, W）
    - 当前航向箭头
    - 航向数字显示
    """
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self._heading = 0.0  # 当前航向角度（0-360°，0°=北）
        self._target_heading = None  # 目标航向角度（可选）
        self._heading_error = None  # 航向误差（度数）
        self.setMinimumSize(120, 120)
        self.setSizePolicy(self.sizePolicy().horizontalPolicy(), 
                          self.sizePolicy().verticalPolicy())
    
    def set_heading(self, heading, target_heading=None, heading_error=None):
        """
        设置航向角度
        
        Args:
            heading: 当前航向角度（度数，0-360，0°=北，顺时针）
            target_heading: 目标航向角度（可选）
            heading_error: 航向误差（度数，可选）
        """
        self._heading = heading % 360.0
        self._target_heading = target_heading % 360.0 if target_heading is not None else None
        self._heading_error = heading_error
        self.update()  # 触发重绘
    
    def paintEvent(self, event):
        """绘制罗盘"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # 计算中心和半径
        width = self.width()
        height = self.height()
        size = min(width, height)
        center_x = width / 2
        center_y = height / 2
        radius = size / 2 - 10
        
        # 背景圆（外圈）
        painter.setPen(QPen(QColor("#34495e"), 2))
        painter.setBrush(QBrush(QColor("#ecf0f1")))
        painter.drawEllipse(QPointF(center_x, center_y), radius, radius)
        
        # 内圈（稍小）
        inner_radius = radius * 0.9
        painter.setPen(QPen(QColor("#95a5a6"), 1))
        painter.setBrush(QBrush(QColor("#ffffff")))
        painter.drawEllipse(QPointF(center_x, center_y), inner_radius, inner_radius)
        
        # 绘制刻度和方向标记
        painter.save()
        painter.translate(center_x, center_y)
        
        # 主要方向（N, E, S, W）
        directions = [
            (0, "N", QColor("#e74c3c")),    # 北：红色
            (90, "E", QColor("#3498db")),   # 东：蓝色
            (180, "S", QColor("#95a5a6")),  # 南：灰色
            (270, "W", QColor("#3498db"))   # 西：蓝色
        ]
        
        font = QFont("Arial", 10, QFont.Bold)
        painter.setFont(font)
        
        for angle, text, color in directions:
            painter.save()
            painter.rotate(angle)
            
            # 刻度线
            painter.setPen(QPen(color, 2))
            painter.drawLine(0, int(-inner_radius), 0, int(-inner_radius + 12))
            
            # 方向文字
            painter.rotate(-angle)  # 恢复方向，保持文字直立
            painter.setPen(QPen(color))
            text_rect = QRectF(-15, -inner_radius + 15, 30, 20)
            painter.drawText(text_rect, Qt.AlignCenter, text)
            
            painter.restore()
        
        # 次要刻度（每30度）
        for angle in range(0, 360, 30):
            if angle % 90 != 0:  # 跳过主方向
                painter.save()
                painter.rotate(angle)
                painter.setPen(QPen(QColor("#bdc3c7"), 1))
                painter.drawLine(0, int(-inner_radius), 0, int(-inner_radius + 6))
                painter.restore()
        
        painter.restore()
        
        # 绘制目标航向箭头（蓝色，如果存在）
        if self._target_heading is not None:
            painter.save()
            painter.translate(center_x, center_y)
            painter.rotate(self._target_heading)
            
            # 蓝色箭头（稍细一些，位于红色箭头下层）
            arrow_length = radius * 0.65
            arrow_width = 10
            target_arrow = QPolygonF([
                QPointF(0, -arrow_length),              # 箭头尖端
                QPointF(-arrow_width/2, -arrow_length/2),  # 左侧
                QPointF(0, -arrow_length/2 + 6),        # 中间凹陷
                QPointF(arrow_width/2, -arrow_length/2),   # 右侧
            ])
            
            painter.setPen(QPen(QColor("#2980b9"), 2))
            painter.setBrush(QBrush(QColor("#3498db")))  # 蓝色箭头
            painter.drawPolygon(target_arrow)
            
            painter.restore()
        
        # 绘制当前航向箭头（红色，在最上层）
        painter.save()
        painter.translate(center_x, center_y)
        painter.rotate(self._heading)
        
        # 红色箭头三角形
        arrow_length = radius * 0.7
        arrow_width = 12
        arrow = QPolygonF([
            QPointF(0, -arrow_length),              # 箭头尖端
            QPointF(-arrow_width/2, -arrow_length/2),  # 左侧
            QPointF(0, -arrow_length/2 + 8),        # 中间凹陷
            QPointF(arrow_width/2, -arrow_length/2),   # 右侧
        ])
        
        painter.setPen(QPen(QColor("#c0392b"), 2))
        painter.setBrush(QBrush(QColor("#e74c3c")))  # 红色箭头
        painter.drawPolygon(arrow)
        
        # 箭头尾部圆点
        painter.setBrush(QBrush(QColor("#34495e")))
        painter.drawEllipse(QPointF(0, 0), 5, 5)
        
        painter.restore()
        
        # 中心显示航向误差数字（如果有的话）
        painter.setPen(QPen(QColor("#2c3e50")))
        font_large = QFont("Arial", 14, QFont.Bold)
        painter.setFont(font_large)
        
        # 优先显示航向误差，如果没有则显示当前航向
        if self._heading_error is not None:
            # 显示航向误差，带正负号
            text = f"{self._heading_error:+.1f}°"
            text_color = QColor("#e74c3c") if abs(self._heading_error) > 10 else QColor("#27ae60")
        else:
            text = f"{self._heading:.1f}°"
            text_color = QColor("#2c3e50")
        
        text_rect = QRectF(center_x - 40, center_y + radius * 0.3, 80, 30)
        
        # 数字背景
        painter.setBrush(QBrush(QColor(255, 255, 255, 200)))
        painter.setPen(QPen(QColor("#95a5a6"), 1))
        bg_rect = text_rect.adjusted(-5, -2, 5, 2)
        painter.drawRoundedRect(bg_rect, 5, 5)
        
        # 数字文本
        painter.setPen(QPen(text_color))
        painter.drawText(text_rect, Qt.AlignCenter, text)
    
    def sizeHint(self):
        """建议尺寸"""
        from PyQt5.QtCore import QSize
        return QSize(150, 150)
    
    def minimumSizeHint(self):
        """最小尺寸"""
        from PyQt5.QtCore import QSize
        return QSize(120, 120)
