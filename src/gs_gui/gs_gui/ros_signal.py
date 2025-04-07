from PyQt5.QtCore import pyqtSignal, QObject
from sensor_msgs.msg import BatteryState
from common_interfaces.msg import UsvStatus
class ROSSignal(QObject):
    mode_command= pyqtSignal(str)  # 模式切换
    arm_command= pyqtSignal(str)  # 武装/解除武装

    receive_state_list= pyqtSignal(list)  # 无人船状态