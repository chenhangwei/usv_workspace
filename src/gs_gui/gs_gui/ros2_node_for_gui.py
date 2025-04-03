import rclpy
from rclpy.node import Node
from common_interfaces.msg import UsvStatus
from std_msgs.msg import String
from gs_gui.ros_signal import ROSSignal
from sensor_msgs.msg import BatteryState
class GroundStationNode(Node):
    def __init__(self, ros_signal):
        super().__init__('ground_station_node')
        self.ros_signal = ros_signal

       
        self.ros_signal.send_command.connect(self.publish_command)  # Connect the signal to the slot
        
        self.subscribers = {}  # 存储订阅器
        self.usv_status = {}  # 存储无人船的在线状态

        # 定时器：定期检查在线的无人船节点
        self.timer = self.create_timer(2.0, self.update_usv_namespaces)

    def update_usv_namespaces(self):
        # 获取当前网络中的所有节点及其命名空间
        node_names_and_usv_namespaces = self.get_node_names_and_namespaces()
        # 获取当前的无人船命名空间
        current_usv_namespaces = [ns for _, ns in node_names_and_usv_namespaces if ns.startswith('/usv_')]
        #检查新增的无人船命名空间
        for namespace in current_usv_namespaces:
            if namespace not in self.subscribers:
                self.get_logger().info(f"New USV detected: {namespace}")
                # 创建订阅器
                self.create_usv_subscriber(namespace)
        # 检查断开的无人船命名空间        
        for namespace in list(self.subscribers.keys()):
            if namespace not in current_usv_namespaces:
                self.get_logger().info(f"USV disconnected: {namespace}")
                # 删除订阅器
                self.destroy_subscription(self.subscribers[namespace])
                del self.subscribers[namespace]
                del self.usv_status[namespace]

    def create_usv_subscriber(self, namespace):
        # 创建订阅器
        state_subscriber = self.create_subscription(
           UsvStatus,
            f'{namespace}/state',
            lambda msg, ns=namespace: self.state_callback(msg, ns),
            10
        )
        # 将订阅器存储在字典中,分别记录状态和电池状态
        self.subscribers[namespace] = {
            'state':state_subscriber,
        } 
           # 初始化无人船的状态
        self.usv_status[namespace] = {
            'connected': False,  # 假设初始状态为未连接
             }                        

    def state_callback(self, msg, namespace):
        self.usv_status[namespace] = msg.connected  # 更新在线状态
        self.get_logger().info(f"{namespace} State: {msg.mode}, connected: {msg.connected}")
    
    
    def get_online_usvs(self):
        # 返回当前在线的无人船列表
        pass
        

    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing command: {command}')