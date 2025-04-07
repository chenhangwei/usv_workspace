import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gs_gui.ros_signal import ROSSignal
from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import State
from common_interfaces.msg import UsvStatus
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class GroundStationNode(Node):
    def __init__(self, ros_signal):
        super().__init__('ground_station_node')
        self.ros_signal = ros_signal

   
        self.qos = QoSProfile(
            depth=10,  # 队列大小
            reliability=QoSReliabilityPolicy.BEST_EFFORT)

       
       
        # 创建发布器
        self.arm_publisher = self.create_publisher(String, 'usv_set_arming', 10)
        self.mode_publisher = self.create_publisher(String, 'usv_set_mode', 10)

        self.subscribers = {}  # 存储订阅器
        self.usv_status = {}  # 存储无人船的在线状态

        # 定时器：定期检查在线的无人船节点
        self.timer = self.create_timer(1.0, self.update_usv_namespaces)

    # 更新无人船命名空间
    def update_usv_namespaces(self):
        # 获取当前网络中的所有节点及其命名空间
        node_names_and_usv_namespaces = self.get_node_names_and_namespaces()
        # 获取当前的无人船命名空间
        current_usv_namespaces = [ns for _, ns in node_names_and_usv_namespaces if ns.startswith('/usv_')]
        #检查新增的无人船命名空间
        for namespace in current_usv_namespaces:
            if namespace not in self.subscribers:
                self.get_logger().info(f"发现新设备: {namespace}")
                # 创建订阅器
                self.create_usv_subscriber(namespace)
        # 检查断开的无人船命名空间        
        for namespace in list(self.subscribers.keys()):
            if namespace not in current_usv_namespaces:
                self.get_logger().info(f"设备离线: {namespace}")
                # 删除订阅器
                self.destroy_subscription(self.subscribers[namespace])
                del self.subscribers[namespace]
                del self.usv_status[namespace]

        # 获取当前在线的无人船列表     
        online_usv_list=self.get_online_usvs()
        # 向UI发布在线无人船列表
        self.ros_signal.receive_state_list.emit( online_usv_list)

    def create_usv_subscriber(self, namespace):
        # 创建订阅器
        state_subscriber = self.create_subscription(
           UsvStatus,
            f'{namespace}/usv_current_state',
            lambda msg, ns=namespace: self.state_callback(msg, ns),
            self.qos
        )
        # 将订阅器存储在字典中,记录状态
        self.subscribers[namespace] = {
            'state':state_subscriber,
        } 
           # 初始化无人船的状态
        self.usv_status[namespace] = {
            'connected': False,  # 假设初始状态为未连接
             }   

    def state_callback(self, msg, namespace):
        current_state = UsvStatus()
        current_state = msg
         # 如果 namespace 不在 usv_status 中，先初始化
        if namespace not in self.usv_status:
            self.usv_status[namespace] = {'connected': False}
         # 更新 connected 状态
            self.usv_status[namespace]['connected'] =current_state.connected           
    
    def get_online_usvs(self):
           # 返回当前在线的无人船列表
        online_usvs = [ns for ns, status in self.usv_status.items() if status['connected']]
        return online_usvs
        
    def  set_arming_callback(self, msg):
        """处理解锁/上锁命令"""
        if not isinstance(msg, String):
            self.get_logger().error('收到无效的解除武装消息类型')
            return
        # 发布解锁/上锁命令
        self.arm_publisher(msg.data)

    def set_mode_callback(self, msg):
        """处理模式切换命令"""
        if not isinstance(msg, String):
            self.get_logger().error('收到无效的模式消息类型')
            return
        # 发布模式切换命令
        self.mode_publisher(msg.data)

  