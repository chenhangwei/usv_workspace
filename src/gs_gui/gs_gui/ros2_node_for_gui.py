from math import dist
from scipy import cluster
from sympy import Quaternion, public
import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Float32
from gs_gui.ros_signal import ROSSignal
from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from common_interfaces.msg import UsvStatus
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.duration import Duration
import tf_transformations
from transitions import Machine
import time
import threading


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
        self.led_pubs={}
        self.sound_pubs={}
        self.usv_states={}
        self.last_ns_list = []  # 用于存储上一个命名空间列表
        self.is_runing=False #usv设备端是否运行
        self.run_step=0#运行步数
        self.usv_target_number=0 #完成的目标点数量
        self.send_target_loop_flag=False #发送目标点循环标志
   
        
        # 创建定时器，定期检查命名空间变化（例如每 2秒）
        self.timer = self.create_timer(2.0, self.update_subscribers_and_publishers)
        
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
            topic_led=f'{ns}/gs_led_command'
            topic_sound=f'{ns}/gs_sound_command'
            
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

            #创建 gs_led_command 发布器
            self.led_pubs[ns]=self.create_publisher(
                String,
                topic_led,
                self.qos_a
            )
            self.get_logger().info(f"新建发布者 for {topic_led}")

            #创建 gs_sound_command 发布器
            self.sound_pubs[ns]=self.create_publisher(
                String,
                topic_sound,
                self.qos_a
            )
            self.get_logger().info(f"新建发布者 for {topic_sound}")

        
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

             # 销毁 led_pubs 发布器
            if ns in self.led_pubs:
                self.destroy_publisher(self.led_pubs[ns])
                del self.led_pubs[ns]
                self.get_logger().info(f"已销毁发布者： {ns}/led")
            
             # 销毁 sound_pubs 发布器
            if ns in self.sound_pubs:
                self.destroy_publisher(self.sound_pubs[ns])
                del self.sound_pubs[ns]
                self.get_logger().info(f"已销毁发布者： {ns}/sound")



    def usv_state_callback(self, msg, ns):
        if  isinstance(msg,UsvStatus):
            state_data={
                'namespace':ns,
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
                'is_running':msg.is_runing,

            }
            self.usv_states[ns] = state_data
            self.ros_signal.receive_state_list.emit(list(self.usv_states.values()))

    def set_manaul_callback(self, msg):
        # 打印日志，表示接收到切换为手动模式的命令
        self.get_logger().info("接收到切换为Manaul模式")
        # 初始化 USV 列表
        usv_list = []
        usv_list = msg  # 将接收到的消息赋值给 USV 列表

        def publish_with_delay():
            # 遍历 USV 列表中的命名空间
            for ns in usv_list:
                self.get_logger().info(f"ns 类型: {type(ns)}, 内容: {ns}")
                # 检查命名空间是否存在于发布器字典中
                if ns in self.set_usv_mode_pubs:
                    # 创建一个 String 类型的消息
                    mode_msg = String()
                    mode_msg.data = "MANAUL"  # 设置消息内容为 "MANAUL"（手动模式）

                    # 使用对应命名空间的发布器发布模式切换消息
                    self.set_usv_mode_pubs[ns].publish(mode_msg)
                    # 打印日志，表示消息已发布到对应的话题
                    self.get_logger().info(f"发布到话题 {ns}/set_usv_mode")
                else:
                    # 如果命名空间不存在，打印警告日志
                    self.get_logger().warn(f"命名空间 {ns} 不存在，无法发布模式消息")
                time.sleep(0.1)  # 延时0.1秒，避免过快发布    
        threading.Thread(target=publish_with_delay,daemon=True).start()            
    def set_guided_callback(self,msg):
          # 打印日志，表示接收到切换为手动模式的命令
        self.get_logger().info("接收到切换为GUIDED模式")
        # 初始化 USV 列表
        usv_list = []
        usv_list = msg  # 将接收到的消息赋值给 USV 列表

        def publish_with_delay():
            # 遍历 USV 列表中的命名空间
            for ns in usv_list:
                self.get_logger().info(f"ns 类型: {type(ns)}, 内容: {ns}")
                # 检查命名空间是否存在于发布器字典中
                if ns in self.set_usv_mode_pubs:
                    # 创建一个 String 类型的消息
                    mode_msg = String()
                    mode_msg.data = "GUIDED"  # 设置消息内容为 "MANAUL"（手动模式）
                    # 使用对应命名空间的发布器发布模式切换消息
                    self.set_usv_mode_pubs[ns].publish(mode_msg)
                    # 打印日志，表示消息已发布到对应的话题
                    self.get_logger().info(f"发布到话题 {ns}/set_usv_mode")
                else:
                    # 如果命名空间不存在，打印警告日志
                    self.get_logger().warn(f"命名空间 {ns} 不存在，无法发布模式消息")

                time.sleep(0.1)  # 延时0.1秒，避免过快发布
        threading.Thread(target=publish_with_delay,daemon=True).start()            
    def set_arming_callback(self,msg):
        # 打印日志，表示接收到切换为手动模式的命令
        self.get_logger().info("接收到切换为Arming模式")
        # 初始化 USV 列表
        usv_list = []
        usv_list = msg

        def publish_with_delay():
            # 遍历 USV 列表中的命名空间
            for ns in usv_list:
                self.get_logger().info(f"ns 类型: {type(ns)}, 内容: {ns}")
                # 检查命名空间是否存在于发布器字典中
                if ns in self.set_usv_arming_pubs:
                    # 创建一个 String 类型的消息
                    arming_msg = String()
                    arming_msg.data = "ARMING"
                    # 设置消息内容为 "ARMING"（手动模式）
                    # 使用对应命名空间的发布器发布模式切换消息
                    self.set_usv_arming_pubs[ns].publish(arming_msg)
                    # 打印日志，表示消息已发布到对应的话题
                    self.get_logger().info(f"发布到话题 {ns}/set_usv_arming")
                else:
                    # 如果命名空间不存在，打印警告日志
                    self.get_logger().warn(f"命名空间 {ns} 不存在，无法发布模式消息")
                time.sleep(0.1)  # 延时0.1秒，避免过快发布    
        threading.Thread(target=publish_with_delay,daemon=True).start()    

    def set_disarming_callback(self,msg):
        # 打印日志，表示接收到切换为手动模式的命令
        self.get_logger().info("接收到切换为Disarming模式")
        # 初始化 USV 列表
        usv_list = []
        usv_list = msg
        # 将接收到的消息赋值给 USV 列表

        def publish_with_delay():
            # 遍历 USV 列表中的命名空间
            for ns in usv_list:
                self.get_logger().info(f"ns 类型: {type(ns)}, 内容: {ns}")
                # 检查命名空间是否存在于发布器字典中
                if ns in self.set_usv_arming_pubs:
                    # 创建一个 String 类型的消息
                    disarming_msg = String()
                    disarming_msg.data = "DISARMING"
                    # 设置消息内容为 "ARMING"（手动模式）
                    # 使用对应命名空间的发布器发布模式切换消息
                    self.set_usv_arming_pubs[ns].publish(disarming_msg)
                    # 打印日志，表示消息已发布到对应的话题
                    self.get_logger().info(f"发布到话题 {ns}/set_usv_arming")
                else:
                    # 如果命名空间不存在，打印警告日志
                    self.get_logger().warn(f"命名空间 {ns} 不存在，无法发布模式消息")
                time.sleep(0.1)  # 延时0.1秒，避免过快发布
        threading.Thread(target=publish_with_delay,daemon=True).start()
                
    
    def set_cluster_target_point_callback(self,msg):

        # 打印日志，表示接收到切换为手动模式的命令
        self.get_logger().info("接收到集群目标点")
        # 初始化 USV 列表
        temp_list = []
        temp_list = msg
        usv_id_set={} 
        usv_status_id_set={}
        self.send_target_loop_flag=True #发送目标点循环标志
        self.run_step=1
                   # 将接收到的消息赋值给 USV 列表
            # 遍历 USV 列表中的命名空间

        for ns in temp_list:     
            if isinstance(ns, dict): 
                usv_id_set = ns.get('usv_id', None)
                if  usv_id_set is None:
                    self.get_logger().warn("未找到usv_id,跳过该项")
                    continue
                else:
                    self.get_logger().warn("接收到的ns不是字典,跳过该项")   
                    continue

        # 检查命名空间是否存在于发布器字典中
        # 遍历 usv_states 字典，检查每个 USV 的状态
        for nn in self.usv_states:
            if isinstance(nn,dict):
                usv_status_id_set=nn.get('usv_id',None)
                if usv_status_id_set is None:
                    self.get_logger().warn("未找到usv_id,跳过该项")
                    continue
                else:
                    self.get_logger().warn("接收到的ns不是字典,跳过该项")   
                    continue
        # 检查 usv_status_id 是否与 usv_id 匹配
        if usv_id_set is not None and usv_status_id_set is not None:
            for n in usv_id_set:  
               for v in usv_status_id_set:
                    if n == v:
                        if  self.usv_states[v]['is_runing'] == False:
                            self.usv_target_number += 1



        while self.send_target_loop_flag:
       
            # 获取指定步骤下的 USV 列表
            cluster_usv_list=self.get_usvs_by_step( temp_list, self.run_step)  
            if cluster_usv_list is not None:
                self.get_logger().info(f"当前步数下的 USV 列表: {cluster_usv_list}")
            else:
                self.get_logger().warn("当前步数下的 USV 列表为空")
                self.send_target_loop_flag=False #发送目标点循环标志  
                self.run_step=0 

            def publish_with_delay():
                for ns in cluster_usv_list:     
                    if isinstance(ns, dict): 
                        usv_id= ns.get('usv_id', None)
                        # 检查命名空间是否存在于发布器字典中
                        if usv_id in self.set_usv_target_position_pubs:
                            # 创建一个 PoseStamped 类型的消息
                            target_point_msg = PoseStamped()
                            target_point_msg.header.frame_id = "map"
                            target_point_msg.header.stamp = self.get_clock().now().to_msg()
                            target_point_msg.pose.position.x = ns.get('position', {}).get('x', 0.0)
                            target_point_msg.pose.position.y = ns.get('position', {}).get('y', 0.0)
                            target_point_msg.pose.position.z = ns.get('position', {}).get('z', 0.0)

                            # 设置目标点的方向
                            yaw = ns.get('yaw', 0.0)

                            # 将yaw 转换为四元数
                            quaternion=tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
                            target_point_msg.pose.orientation.x = quaternion[0]
                            target_point_msg.pose.orientation.y = quaternion[1]
                            target_point_msg.pose.orientation.z = quaternion[2]
                            target_point_msg.pose.orientation.w = quaternion[3]

                            # 使用对应命名空间的发布器发布目标点消息
                            if  self.usv_target_number==len(cluster_usv_list):#判断是否所有的目标点已经到达目标位置
                                self.set_usv_target_position_pubs[usv_id].publish(target_point_msg)
                                self.run_step=+1
                                # 打印日志，表示消息已发布到对应的话题
                                self.get_logger().info(f"发布到话题 {ns}/set_usv_target_position")
                            else:
                                self.get_logger().info(f"目标点未到达，跳过发布到话题 {ns}/set_usv_target_position")
                    time.sleep(0.1)  # 延时0.1秒，避免过快发布
            threading.Thread(target=publish_with_delay,daemon=True).start()            
            
    def set_cluster_target_velocity_callback(self,msg):
        # 打印日志，表示接收到切换为手动模式的命令
        self.get_logger().info("接收到切换为Cluster目标的速度")

        # 初始化 USV 列表
        usv_list = []
        usv_list = msg
        # 将接收到的消息赋值给 USV 列表

        def publish_with_delay():
            # 遍历 USV 列表中的命名空间
            for ns in usv_list:
                self.get_logger().info(f"ns 类型: {type(ns)}, 内容: {ns}")
                # 检查命名空间是否存在于发布器字典中
                if ns in self.set_usv_target_velocity_pubs:
                    # 创建一个 Float32 类型的消息
                    target_velocity_msg = Float32()
                    target_velocity_msg.data = msg[ns]['velocity']
                    # 使用对应命名空间的发布器发布目标速度消息
                    self.set_usv_target_velocity_pubs[ns].publish(target_velocity_msg)
                    # 打印日志，表示消息已发布到对应的话题
                    self.get_logger().info(f"发布到话题 {ns}/set_usv_target_velocity")
                else:
                    # 如果命名空间不存在，打印警告日志
                    self.get_logger().warn(f"命名空间 {ns} 不存在，无法发布目标速度消息")
                time.sleep(0.1)  # 延时0.1秒，避免过快发布
        threading.Thread(target=publish_with_delay,daemon=True).start()


    def set_departed_target_point_callback(self,msg):

        # 打印日志，表示接收到切换为手动模式的命令
        self.get_logger().info("接收到切换为Cluster目标点")
        # 初始化 USV 列表
        usv_list = []
        usv_list = msg
        usv_id=''
        # 将接收到的消息赋值给 USV 列表

        def publish_with_delay():
            # 遍历 USV 列表中的命名空间
            for ns in usv_list:     
                    if isinstance(ns, dict): 
                        usv_id = ns.get('usv_id', None)
                        if usv_id is None:
                            self.get_logger().warn("未找到usv_id,跳过该项")
                            continue
                    else:
                        self.get_logger().warn("接收到的ns不是字典,跳过该项")   
                        continue

                    # 检查命名空间是否存在于发布器字典中

                    if usv_id in self.set_usv_target_position_pubs:
                        # 创建一个 PoseStamped 类型的消息
                        target_point_msg = PoseStamped()
                        target_point_msg.header.frame_id = "map"
                        target_point_msg.header.stamp = self.get_clock().now().to_msg()
                        target_point_msg.pose.position.x = ns.get('position', {}).get('x', 0.0)
                        target_point_msg.pose.position.y = ns.get('position', {}).get('y', 0.0)
                        target_point_msg.pose.position.z = ns.get('position', {}).get('z', 0.0)

                        # 设置目标点的方向
                        yaw = ns.get('yaw', 0.0)

                        # 将yaw 转换为四元数
                        quaternion=tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
                        target_point_msg.pose.orientation.x = quaternion[0]
                        target_point_msg.pose.orientation.y = quaternion[1]
                        target_point_msg.pose.orientation.z = quaternion[2]
                        target_point_msg.pose.orientation.w = quaternion[3]

                        # 使用对应命名空间的发布器发布目标点消息
                        self.set_usv_target_position_pubs[usv_id].publish(target_point_msg)
                        # 打印日志，表示消息已发布到对应的话题
                        self.get_logger().info(f"发布到话题 {ns}/set_usv_target_position")
                    time.sleep(0.1)  # 延时0.1秒，避免过快发布
        threading.Thread(target=publish_with_delay,daemon=True).start()        
    def set_departed_target_velocity_callback(self,msg):
        # 打印日志，表示接收到切换为手动模式的命令
        self.get_logger().info("接收到切换为Departed目标速度")
        # 初始化 USV 列表
        usv_list = []
        usv_list = msg
        # 将接收到的消息赋值给 USV 列表
        def publish_with_delay():
            # 遍历 USV 列表中的命名空间
            for ns in usv_list:
                self.get_logger().info(f"ns 类型: {type(ns)}, 内容: {ns}")
                # 检查命名空间是否存在于发布器字典中
                if ns in self.set_usv_target_velocity_pubs:
                    # 创建一个 Float32 类型的消息
                    target_velocity_msg = Float32()
                    target_velocity_msg.data = msg[ns]['velocity']
                    # 使用对应命名空间的发布器发布目标速度消息
                    self.set_usv_target_velocity_pubs[ns].publish(target_velocity_msg)
                    # 打印日志，表示消息已发布到对应的话题
                    self.get_logger().info(f"发布到话题 {ns}/set_usv_target_velocity")
                else:
                    # 如果命名空间不存在，打印警告日志
                    self.get_logger().warn(f"命名空间 {ns} 不存在，无法发布目标速度消息")
                time.sleep(0.1)
        threading.Thread(target=publish_with_delay,daemon=True).start()        

    # 获取指定步骤下的 USV 列表
    def get_usvs_by_step(self,cluster_usv_list, step):
        # 返回在指定步骤下的 USV 列表
        return [usv for usv in cluster_usv_list if usv.get('step') == step]
    

    def str_command_callback(self,msg):
            command_str=String()
            if not isinstance(msg,str):
                return
            command_str.data=msg

            def publish_with_delay():    
                try:
                    for ns in self.last_ns_list:
                        self.led_pubs[ns].publish(command_str)
                        self.get_logger().info(f'发布led或sound命令:{ns}-{command_str}')
                        time.sleep(5)  # 延时5秒
                except Exception as e:
                    self.get_logger().error(f'发布led或sound命令失败: {e}')

            threading.Thread(target=publish_with_delay,daemon=True).start()
            self.get_logger().info(f'已启动新线程发布命令')


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
        for ns in list(self.led_pubs.keys()):
            self.destroy_publisher(self.led_pubs[ns])
            del self.led_pubs[ns] 
        for ns in list(self.sound_pubs.keys()):
            self.destroy_publisher(self.sound_pubs[ns])
            del self.sound_pubs[ns]   

        super().destroy_node()




  