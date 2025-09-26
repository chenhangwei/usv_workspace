"""
无人船控制节点

该节点负责处理无人船的目标点控制逻辑。它订阅常规目标点和避障目标点，
根据避障标志决定使用哪个目标点，并将选定的目标点发布给飞控系统。
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, PositionTarget
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import math


class UsvControlNode(Node):
    """
    无人船控制节点类
    
    该节点实现目标点控制逻辑，处理常规目标点和避障目标点，
    根据避障标志决定使用哪个目标点，并将选定的目标点发布给飞控系统。
    """

    def __init__(self):
        """初始化无人船控制节点"""
        super().__init__('usv_control_node')

        # 声明参数
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('coordinate_frame', PositionTarget.FRAME_LOCAL_NED)
        
        # 获取参数值
        publish_rate_param = self.get_parameter('publish_rate').value
        publish_rate = 20.0 if publish_rate_param is None else float(publish_rate_param)
        self.frame_id = self.get_parameter('frame_id').value
        self.coordinate_frame = self.get_parameter('coordinate_frame').value
        
        # 创建 QoS 配置
        qos_best_effort = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        qos_reliable = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
    
        # 发布目标点到飞控
        self.target_point_pub = self.create_publisher(PositionTarget, 'setpoint_raw/local', qos_best_effort)   

        # 订阅当前状态
        self.state_sub = self.create_subscription(
            State, 'state', self.state_callback, qos_best_effort)
        
        # 订阅需要运行的目标点 (来自地面站)
        self.target_point_sub = self.create_subscription(
            PoseStamped, 'set_usv_target_position', self.set_target_point_callback, qos_best_effort)
        
        # 订阅避障目标点
        self.avoidance_target_point_sub = self.create_subscription(
            PositionTarget, 'avoidance_position', self.set_avoidance_target_position_callback, qos_best_effort)
        
        # 订阅避障标记
        self.avoidance_flag_sub = self.create_subscription(
            Bool, 'avoidance_flag', self.set_avoidance_flag_callback, qos_reliable)
        
        # 发送目标位置循环     
        self.publish_target_timer = self.create_timer(1.0/publish_rate, self.publish_target)
    
        # 初始化状态变量
        self.current_state = State()              # 当前状态
        self.current_target_position = PoseStamped()  # 常规目标点
        self.avoidance_position = PositionTarget()    # 避障目标点
        self.avoidance_flag = Bool(data=False)        # 避障标记，默认为False
        
        # 初始化消息对象和状态跟踪
        self.point_msg = PositionTarget()         # 目标点消息
        self.last_published_position = None       # 跟踪最后发布的坐标，避免重复发布
        
        # 日志记录
        self.get_logger().info(f'USV 控制节点已启动')
        self.get_logger().info(f'发布频率: {publish_rate} Hz')
        self.get_logger().info(f'坐标系: {self.frame_id}')

    def state_callback(self, msg):
        """
        状态回调函数
        
        Args:
            msg (State): 包含飞控状态信息的消息
        """
        if isinstance(msg, State):
            self.current_state = msg

    def set_target_point_callback(self, msg):
        """
        设置目标点回调函数
        
        Args:
            msg (PoseStamped): 包含目标点信息的消息
        """
        if not isinstance(msg, PoseStamped):
            self.get_logger().warn('收到无效的目标点消息类型')
            return      
        
        # 检查目标点坐标有效性
        if (msg.pose.position.x is None or msg.pose.position.y is None or 
            msg.pose.position.z is None):
            self.get_logger().warn('收到的目标点坐标无效')
            return
            
        old_position = self.current_target_position.pose.position
        new_position = msg.pose.position
        
        # 只有当目标点发生变化时才更新
        if (old_position.x != new_position.x or 
            old_position.y != new_position.y or 
            old_position.z != new_position.z):
            
            self.current_target_position = msg
            self.get_logger().info(f'更新常规目标点: ({new_position.x:.2f}, {new_position.y:.2f}, {new_position.z:.2f})')

    def set_avoidance_target_position_callback(self, msg):
        """
        设置避障目标点回调函数
        
        Args:
            msg (PositionTarget): 包含避障目标点信息的消息
        """
        if not isinstance(msg, PositionTarget):
            self.get_logger().warn('收到无效的避障目标点消息类型')
            return
            
        # 检查避障目标点坐标有效性
        if (msg.position.x is None or msg.position.y is None or 
            msg.position.z is None):
            self.get_logger().warn('收到的避障目标点坐标无效')
            return
            
        old_position = self.avoidance_position.position
        new_position = msg.position
        
        # 只有当避障目标点发生变化时才更新
        if (old_position.x != new_position.x or 
            old_position.y != new_position.y or 
            old_position.z != new_position.z):
            
            self.avoidance_position = msg
            self.get_logger().info(f'更新避障目标点: ({new_position.x:.2f}, {new_position.y:.2f}, {new_position.z:.2f})')

    def set_avoidance_flag_callback(self, msg):
        """
        设置避障标志回调函数
        
        Args:
            msg (Bool): 包含避障标志信息的消息
        """
        if not isinstance(msg, Bool):
            self.get_logger().warn('收到无效的避障标志消息类型')
            return
            
        # 只有当避障标志状态发生变化时才处理
        if self.avoidance_flag.data != msg.data:
            self.avoidance_flag = msg
            mode = "避障模式" if msg.data else "常规模式"
            self.get_logger().info(f'切换到: {mode}')

    def publish_target(self):
        """
        发布目标点函数
        
        根据避障标志决定使用常规目标点还是避障目标点，
        并将选定的目标点发布给飞控系统。
        """
        try:
            # 检查飞控是否已连接、已解锁且处于GUIDED模式
            if not self.current_state.connected:
                self.get_logger().debug('飞控未连接，等待连接...')
                return
                
            if not self.current_state.armed:
                self.get_logger().debug('飞控未解锁，等待解锁...')
                return
                
            if self.current_state.mode != "GUIDED":
                self.get_logger().debug(f'当前模式: {self.current_state.mode}，需要GUIDED模式')
                return  
            
            # 根据避障标志选择目标点
            if not self.avoidance_flag.data:    
                # 使用常规目标点 (PoseStamped转PositionTarget)
                px = self.current_target_position.pose.position.x
                py = self.current_target_position.pose.position.y
                pz = self.current_target_position.pose.position.z
                source = "常规"
            else:
                # 使用避障目标点
                px = self.avoidance_position.position.x
                py = self.avoidance_position.position.y
                pz = self.avoidance_position.position.z  
                source = "避障"
            
            # 检查目标点坐标是否有效
            if any(coord is None for coord in [px, py, pz]):
                self.get_logger().warn(f'{source}目标点坐标无效，忽略')
                return
                
            # 检查是否需要发布新目标点（避免重复发布相同位置）
            current_position = (round(px, 3), round(py, 3), round(pz, 3))
            if self.last_published_position == current_position:
                return  # 目标点未改变，跳过发布
                
            # 更新最后发布的坐标
            self.last_published_position = current_position
            
            # 构造并发布目标点消息
            self.point_msg.header.stamp = self.get_clock().now().to_msg()
            self.point_msg.header.frame_id = self.frame_id
            self.point_msg.coordinate_frame = self.coordinate_frame
            self.point_msg.type_mask = (
                PositionTarget.IGNORE_VX |
                PositionTarget.IGNORE_VY |
                PositionTarget.IGNORE_VZ |
                PositionTarget.IGNORE_AFX |
                PositionTarget.IGNORE_AFY |
                PositionTarget.IGNORE_AFZ |
                PositionTarget.FORCE |
                PositionTarget.IGNORE_YAW |
                PositionTarget.IGNORE_YAW_RATE
            )
            self.point_msg.position.x = px
            self.point_msg.position.y = py
            self.point_msg.position.z = pz  

            self.target_point_pub.publish(self.point_msg)
            
            # 记录成功发布的信息
            self.get_logger().debug(f'发布{source}目标点: ({px:.2f}, {py:.2f}, {pz:.2f})')

        except Exception as e:
            self.get_logger().error(f'发布目标点时发生异常: {str(e)}')


def main(args=None):
    """
    主函数
    
    初始化ROS 2节点并开始处理消息。
    
    Args:
        args: 命令行参数
    """
    rclpy.init(args=args)
    node = UsvControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()