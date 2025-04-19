from PyQt5.QtCore import pyqtSignal, QObject
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Point
from common_interfaces.msg import UsvStatus
class ROSSignal(QObject):
    arm_command= pyqtSignal(dict)  # 武装
    disarm_command= pyqtSignal(dict)  # 解除武装
    manaul_command= pyqtSignal(dict)  # manaul模式
    guided_command= pyqtSignal(dict)  # guided模式

    cluster_target_point_command= pyqtSignal(Point)     # 集群目标点
    cluster_target_velocity_command= pyqtSignal(float)  # 集群速度（油门大小）
    departed_target_point_command=pyqtSignal(Point)     # 离群目标点
    departed_target_velocity_command= pyqtSignal(float) # 离群速度（油门大小）

    receive_state_list= pyqtSignal(list)  # 无人船状态