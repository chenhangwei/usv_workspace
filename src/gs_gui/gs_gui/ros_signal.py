from PyQt5.QtCore import pyqtSignal, QObject


class ROSSignal(QObject):
    """
    ROS信号类，用于GUI和ROS节点之间的通信
    
    该类定义了GUI和ROS节点之间通信所需的所有信号。
    信号按照功能分为几类：武装命令、模式切换、目标点设置、状态更新和通用命令。
    """

    # 模式常量定义
    MODE_MANUAL = "manual"          # 手动模式
    MODE_GUIDED = "guided"          # 制导模式
    MODE_ARCO = "arco"              # ARCO模式
    MODE_STEERING = "steering"      # 转向模式

    # 武装/解武装命令信号
    arm_command = pyqtSignal(list)      # 武装命令，参数：USV命名空间列表
    disarm_command = pyqtSignal(list)   # 解除武装命令，参数：USV命名空间列表

    # 模式切换命令信号
    manual_command = pyqtSignal(list)   # 手动模式命令，参数：USV命名空间列表
    guided_command = pyqtSignal(list)   # 制导模式命令，参数：USV命名空间列表
    arco_command = pyqtSignal(list)     # ARCO模式命令，参数：USV命名空间列表
    steering_command = pyqtSignal(list) # 转向模式命令，参数：USV命名空间列表

    # 目标点命令信号
    cluster_target_point_command = pyqtSignal(list)     # 集群目标点命令，参数：目标点列表
    departed_target_point_command = pyqtSignal(list)    # 离群目标点命令，参数：目标点列表

    # 状态更新信号
    receive_state_list = pyqtSignal(list)  # 接收USV状态列表，参数：USV状态字典列表
    cluster_progress_update = pyqtSignal(dict)  # 集群任务进度更新，参数：进度信息字典

    # 添加导航状态更新信号
    nav_status_update = pyqtSignal(str, str)        # 导航状态更新信号，参数：USV ID, 状态字符串
    navigation_feedback = pyqtSignal(str, object)   # 导航反馈信号，参数：USV ID, 反馈数据对象

    # 通用字符串命令信号
    str_command = pyqtSignal(str)          # 通用字符串命令，参数：命令字符串
    # 节点 -> GUI 的信息反馈
    node_info = pyqtSignal(str)            # 参数：信息字符串，供节点向 GUI 发送状态/反馈
    
    # 坐标系偏移量设置信号
    update_area_center = pyqtSignal(dict)  # 更新任务坐标系偏移量，参数：{'x': float, 'y': float, 'z': float}

