import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Float32
from gs_gui.ros_signal import ROSSignal
from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from common_interfaces.msg import UsvStatus
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class GroundStationNode(Node):
    def __init__(self,signal):
        super().__init__('groundstationnode')
        self.ros_signal=signal
        # 初始化 QoS 策略（根据需要调整）
        self.qos_a = QoSProfile(depth=10,reliability= QoSReliabilityPolicy.RELIABLE)
        
        # 初始化订阅器和发布器字典
        self.usv_state_subs = {}
        self.set_usv_target_position_pubs = {}
        self.set_usv_target_velocity_pubs = {}
        self.set_usv_mode_pubs={}
        self.set_usv_arming_pubs={}
        self.usv_states={}
   
        
        # 创建定时器，定期检查命名空间变化（例如每 5 秒）
        self.timer = self.create_timer(10.0, self.update_subscribers_and_publishers)
        
        # 立即执行一次初始化
        self.update_subscribers_and_publishers()

    def update_subscribers_and_publishers(self):
        # 获取当前命名空间列表
        namespaces = self.get_node_names_and_namespaces()
        current_ns_list = list(set([ns for _, ns in namespaces if ns.startswith('/usv_')]))


        # 如果命名空间未变化，快速退出
        if current_ns_list == self.last_ns_list:
            return
        
        self.last_ns_list = current_ns_list
        
        # 获取当前字典中的命名空间
        existing_ns = set(self.usv_state_subs.keys())
        
        # 找出新增和移除的命名空间
        new_ns = set(current_ns_list) - existing_ns  # 新增的命名空间
        removed_ns = existing_ns - set(current_ns_list)  # 已移除的命名空间
        
        # 为新增的命名空间创建订阅器和发布器
        for ns in new_ns:
            # 构造话题名
            topic_state = f"{ns}/usv_state"
            topic_position = f"{ns}/set_usv_target_position"
            topic_velocity = f"{ns}/set_usv_target_velocity"
            topic_mode=f'{ns}/set_usv_mode'
            topic_arming=f'{ns}/set_usv_arming'
            
            # 创建usv_state 订阅器
            self.usv_state_subs[ns] = self.create_subscription(
                UsvStatus,
                topic_state,
                lambda msg, ns=ns: self.usv_state_callback(msg, ns),
                self.qos_a
            )
            self.get_logger().info(f"新建订阅者 for {topic_state}")
            
            # 创建 set_usv_position 发布器
            self.set_usv_target_position_pubs[ns] = self.create_publisher(
                PoseStamped,
                topic_position,
                self.qos_a
            )
            self.get_logger().info(f"新建发布者 for {topic_position}")
            
            # 创建 set_usv_velocity 发布器
            self.set_usv_target_velocity_pubs[ns] = self.create_publisher(
                Float32,
                topic_velocity,
                self.qos_a
            )
            self.get_logger().info(f"新建发布者 for {topic_velocity}")

            #创建 set_usv_model 发布器
            self.set_usv_mode_pubs[ns]=self.create_publisher(
                String,
                topic_mode,
                self.qos_a
            )
            self.get_logger().info(f"新建发布者 for {topic_mode}")

            #创建 set_usv_arming 发布器
            self.set_usv_arming_pubs[ns]=self.create_publisher(
                String,
                topic_arming,
                self.qos_a
            )
            self.get_logger().info(f"新建发布者 for {topic_arming}")
        
        # 移除已不存在的命名空间的订阅器和发布器
        for ns in removed_ns:
            # 销毁订阅器
            if ns in self.usv_state_subs:
                self.destroy_subscription(self.usv_state_subs[ns])
                del self.usv_state_subs[ns]
                self.get_logger().info(f"已销毁订阅者： {ns}/usv_state")
            
            # 销毁 set_usv_target_position 发布器
            if ns in self.set_usv_target_position_pubs:
                self.destroy_publisher(self.set_usv_target_position_pubs[ns])
                del self.set_usv_target_position_pubs[ns]
                self.get_logger().info(f"已销毁发布者： {ns}/set_usv_target_position")
            
            # 销毁 set_usv_target_velocity 发布器
            if ns in self.set_usv_target_velocity_pubs:
                self.destroy_publisher(self.set_usv_target_velocity_pubs[ns])
                del self.set_usv_target_velocity_pubs[ns]
                self.get_logger().info(f"已销毁发布者： {ns}/set_usv_target_velocity")

             # 销毁 set_usv_model 发布器
            if ns in self.set_usv_mode_pubs:
                self.destroy_publisher(self.set_usv_mode_pubs[ns])
                del self.set_usv_mode_pubs[ns]
                self.get_logger().info(f"已销毁发布者： {ns}/set_usv_model")

             # 销毁 set_usv_arming 发布器
            if ns in self.set_usv_arming_pubs:
                self.destroy_publisher(self.set_usv_arming_pubs[ns])
                del self.set_usv_arming_pubs[ns]
                self.get_logger().info(f"已销毁发布者： {ns}/set_usv_arming")

    def usv_state_callback(self, msg, ns):
        if  isinstance(msg,UsvStatus):
            state_data={
                 'namespace': ns,
                'mode':msg.mode,
                'connected':msg.connected,
                'armed':msg.armed,
                'guided':msg.guided,
                'battery_voltage':msg.battery_voltage,
                'battery_prcentage':msg.battery_percentage,
                'power_supply_status':msg.power_supply_status,
                'position':msg.position,
                'velocity':msg.velocity,
                'yaw':msg.yaw,

            }
            self.usv_states[ns] = state_data
            self.ros_signal.receive_state_list.emit(list(self.usv_states.values()))

    def set_manaul_callback(self,msg):
        pass
    def set_guided_callback(self,msg):
        pass
    def set_arming_callback(self,msg):
        pass
    def set_disarming_callback(self,msg):
        pass


    #销毁
    def destroy_node(self):
        self.get_logger().info("Destroying node resources...")
        for ns in list(self.usv_state_subs.keys()):
            self.destroy_subscription(self.usv_state_subs[ns])
            del self.usv_state_subs[ns]
        for ns in list(self.set_usv_target_position_pubs.keys()):
            self.destroy_publisher(self.set_usv_target_position_pubs[ns])
            del self.set_usv_target_position_pubs[ns]
        for ns in list(self.set_usv_target_velocity_pubs.keys()):
            self.destroy_publisher(self.set_usv_target_velocity_pubs[ns])
            del self.set_usv_target_velocity_pubs[ns]
        for ns in list(self.set_usv_mode_pubs.keys()):
            self.destroy_publisher(self.set_usv_mode_pubs[ns])
            del self.set_usv_mode_pubs[ns]
        for ns in list(self.set_usv_arming_pubs.keys()):
            self.destroy_publisher(self.set_usv_arming_pubs[ns])
            del self.set_usv_arming_pubs[ns]

        super().destroy_node()




  