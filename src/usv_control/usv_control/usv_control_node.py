"""
无人球控制节点

该节点负责处理无人球的目标点控制逻辑。它订阅常规目标点和避障目标点，
根据避障标志决定使用哪个目标点，并将选定的目标点发布给飞控系统。
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, PositionTarget
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

# 导入common_utils工具
from common_utils import ParamLoader, ParamValidator


class UsvControlNode(Node):
    """
    无人球控制节点类
    
    该节点实现目标点控制逻辑，处理常规目标点和避障目标点，
    根据避障标志决定使用哪个目标点，并将选定的目标点发布给飞控系统。
    """

    def __init__(self):
        """初始化无人球控制节点"""
        super().__init__('usv_control_node')
        
        # 创建参数加载器
        param_loader = ParamLoader(self)
        
        # 加载参数
        publish_rate = param_loader.load_param(
            'publish_rate',
            20.0,
            ParamValidator.frequency,
            '目标点发布频率(Hz)'
        )
        self.frame_id = param_loader.load_param(
            'frame_id',
            'map',
            ParamValidator.non_empty_string,
            '坐标系ID'
        )
        self.coordinate_frame = param_loader.load_param(
            'coordinate_frame',
            PositionTarget.FRAME_LOCAL_NED,
            lambda x: x in [1, 8],  # FRAME_BODY_NED=1, FRAME_LOCAL_NED=8
            '坐标框架类型'
        )
        
        # 创建 QoS 配置
        qos_best_effort = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        qos_reliable = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
    
        # 局部坐标模式: 发布 PositionTarget 到 setpoint_raw/local
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
        
        # 订阅清除目标点命令（任务停止时调用）
        self.clear_target_sub = self.create_subscription(
            Bool, 'clear_target', self.clear_target_callback, qos_reliable)
        
        # 局部控制模式：直接订阅 MAVROS 本地位置
        self.local_position_sub = self.create_subscription(
            PoseStamped, 'local_position/pose', self.local_position_callback, qos_best_effort)
        
        # 发送目标位置循环     
        self.publish_target_timer = self.create_timer(1.0/publish_rate, self.publish_target)
    
        # 初始化状态变量
        self.current_state = State()              # 当前状态
        self.current_target_position = PoseStamped()  # 常规目标点
        self.avoidance_position = PositionTarget()    # 避障目标点
        self.avoidance_flag = Bool(data=False)        # 避障标记，默认为False
        self.target_active = False                    # 目标点是否激活（任务运行中）
        self.local_position_valid = False             # 本地位置是否有效（验证 EKF Origin）
        self.ekf_origin_ready = False                 # EKF 原点就绪标志（LocalPos 有效）
        
        # 初始化消息对象和状态跟踪
        self.point_msg = PositionTarget()         # 目标点消息
        self.last_published_position = None       # 跟踪最后发布的坐标，避免重复发布
        
        # 日志记录
        self.get_logger().info(f'USV 控制节点已启动')
        self.get_logger().info(f'发布频率: {publish_rate} Hz')
        self.get_logger().info(f'坐标系: {self.frame_id}')
        
        self.get_logger().info('✅ 局部坐标控制已启用')
        self.get_logger().info('📤 发布话题: setpoint_raw/local (PositionTarget)')
        self.get_logger().info('📍 坐标系: FRAME_LOCAL_NED (相对EKF原点)')

    def state_callback(self, msg):
        """
        状态回调函数
        
        Args:
            msg (State): 包含飞控状态信息的消息
        """
        if isinstance(msg, State):
            self.current_state = msg


    
    def local_position_callback(self, msg):
        """
        本地位置回调函数（验证 EKF 原点是否生效）
        
        Args:
            msg (PoseStamped): 包含本地位置信息的消息
        """
        if isinstance(msg, PoseStamped):
            # 检查本地位置是否有效（不是全0或NaN）
            pos = msg.pose.position
            if not (pos.x == 0.0 and pos.y == 0.0 and pos.z == 0.0):
                if not self.local_position_valid:
                    self.local_position_valid = True
                    self.get_logger().info(
                        f'✅ Local Position 有效: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})'
                    )
                    self._check_ekf_origin_ready()
    
    def _check_ekf_origin_ready(self):
        """检查 EKF 原点是否完全就绪（LocalPos 有效）"""
        if self.local_position_valid and not self.ekf_origin_ready:
            self.ekf_origin_ready = True
            self.get_logger().info('🎯 EKF Origin 完全就绪，可以安全发布目标点！')

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
            self.target_active = True  # 激活目标点，开始发送 setpoint
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

    def clear_target_callback(self, msg):
        """
        清除目标点回调函数（任务停止时调用）
        
        Args:
            msg (Bool): True 表示清除目标点，停止发送 setpoint
        """
        if not isinstance(msg, Bool):
            self.get_logger().warn('收到无效的清除目标点消息类型')
            return
        
        if msg.data:
            # 清除目标点，停止发送 setpoint
            self.target_active = False
            self.current_target_position = PoseStamped()
            self.last_published_position = None
            self.get_logger().info('🛑 目标点已清除，停止发送 setpoint')
        else:
            self.get_logger().debug('收到清除目标点命令: False，忽略')

    def publish_target(self):
        """
        发布目标点函数
        
        发布 PositionTarget 到 setpoint_raw/local (局部坐标)
        注意：PX4 OFFBOARD 模式需要在解锁前就持续收到 setpoint (>2Hz)
        """
        try:
            # 检查飞控连接状态
            if not self.current_state.connected:
                self.get_logger().debug('飞控未连接，等待连接...')
                return
            
            # 检查目标点是否激活（任务运行中）
            if not self.target_active:
                self.get_logger().debug('目标点未激活，等待任务启动...')
                return
            
            # 注意：不再检查 armed 状态，因为 OFFBOARD 模式需要先有 setpoint 才能解锁
            # PX4 要求：进入 OFFBOARD 模式并解锁前，必须持续发送 setpoint
            
            if self.current_state.mode != "OFFBOARD":
                self.get_logger().debug(f'当前模式: {self.current_state.mode}，持续发布设定点以准备 OFFBOARD')
                # PX4 要求在切换 OFFBOARD 前必须有设定点流，因此这里不返回，继续发布

            
            # 局部控制模式需要 EKF 原点就绪
            if not self.ekf_origin_ready:
                if not self.local_position_valid:
                    self.get_logger().debug('⏳ 等待 Local Position 生效...')
                else:
                    self.get_logger().debug('⏳ EKF Origin 未完全就绪...')
                return
            
            # 根据避障标志选择目标点
            if not self.avoidance_flag.data:    
                # 使用常规目标点
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
            
            # 检查目标点是否有效（不是默认的 0,0,0）
            if px == 0.0 and py == 0.0 and pz == 0.0:
                self.get_logger().debug('目标点为默认值(0,0,0)，等待有效目标点...')
                return
                
            # 检查是否需要记录日志（目标点变化时）
            current_position = (round(px, 3), round(py, 3), round(pz, 3))
            if self.last_published_position != current_position:
                # 目标点变化，记录日志并更新
                self.last_published_position = current_position
                self.get_logger().info(f'📍 新目标点: ({px:.2f}, {py:.2f}, {pz:.2f})')
            
            # 注意：即使目标点相同也要持续发布，PX4 OFFBOARD 需要持续的 setpoint 流
            
            # ============================================================
            # 发布局部坐标模式: PositionTarget
            # ============================================================
            
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
            self.get_logger().debug(f'发布{source}目标点(局部): ({px:.2f}, {py:.2f}, {pz:.2f})')

        except Exception as e:
            self.get_logger().error(f'发布目标点时发生异常: {str(e)}')

    def destroy_node(self):
        """节点销毁时的资源清理"""
        if hasattr(self, 'publish_target_timer'):
            self.publish_target_timer.cancel()
        super().destroy_node()


def main(args=None):
    """
    主函数
    
    初始化ROS 2节点并开始处理消息。
    
    Args:
        args: 命令行参数
    """
    rclpy.init(args=args)
    node = UsvControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
