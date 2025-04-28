from PyQt5.QtCore import pyqtSignal, QObject


class ROSSignal(QObject):
    arm_command= pyqtSignal(list)  # 武装
    disarm_command= pyqtSignal(list)  # 解除武装
    manual_command= pyqtSignal(list)  # manaul模式
    guided_command= pyqtSignal(list)  # guided模式

    cluster_target_point_command= pyqtSignal(list)     # 集群目标点
   
    departed_target_point_command=pyqtSignal(list)     # 离群目标点

    cluster_target_velocity_command= pyqtSignal(list)  # 集群目标速度

    departed_target_velocity_command= pyqtSignal(list)  # 离群目标速度
  

    receive_state_list= pyqtSignal(list)  # 无人船状态


    str_command=pyqtSignal(str)