# ros_signal.py
# 该文件定义了ROS信号类，用于在ROS节点和GUI界面之间传递信号

from PyQt5.QtCore import QObject, pyqtSignal


class ROSSignal(QObject):
    """
    ROS信号类，用于在ROS节点和GUI界面之间传递信号
    """
    # 定义各种信号
    receive_state_list = pyqtSignal(list)           # 接收状态列表信号，参数：状态列表
    manual_command = pyqtSignal(list)               # 手动命令信号，参数：命名空间列表
    guided_command = pyqtSignal(list)               # 切换到guided模式命令信号，参数：命名空间列表
    arm_command = pyqtSignal(list)                  # 解锁命令信号，参数：命名空间列表
    disarm_command = pyqtSignal(list)               # 加锁命令信号，参数：命名空间列表
    arco_command = pyqtSignal(str)                  # 切换到arco模式命令信号，参数：命令字符串
    steering_command = pyqtSignal(str)              # 切换到steering模式命令信号，参数：命令字符串
    cluster_target_point_command = pyqtSignal(list) # 集群目标点命令信号，参数：目标点列表
    departed_target_point_command = pyqtSignal(list)# 离群目标点命令信号，参数：目标点列表
    str_command = pyqtSignal(str)                   # 字符串命令信号，参数：命令字符串
    
    # 添加导航状态更新信号
    nav_status_update = pyqtSignal(str, str)        # 导航状态更新信号，参数：USV ID, 状态字符串

    # 模式常量
    MODE_MANUAL = "manual"
    MODE_GUIDED = "guided"
    MODE_ARCO = "arco"
    MODE_STEERING = "steering"