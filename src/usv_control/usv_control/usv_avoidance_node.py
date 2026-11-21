"""
无人球避障节点 - PX4 uXRCE-DDS 版本

该节点负责处理无人球的避障逻辑。通过订阅雷达数据、飞控状态、当前位置和目标位置，
当检测到障碍物时，自动调整目标点以避开障碍物，确保无人球安全运行。

话题映射：
- MAVROS /mavros/state -> /fmu/out/vehicle_status
- MAVROS /mavros/setpoint_raw/local -> /fmu/in/trajectory_setpoint
- MAVROS /mavros/local_position/pose -> /fmu/out/vehicle_local_position
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Bool

# PX4 消息类型
from px4_msgs.msg import (
    VehicleStatus,
    VehicleLocalPosition,
)

# 导入common_utils工具
from common_utils import ParamLoader, ParamValidator


class UsvAvoidancePx4Node(Node):
    """
    无人球避障节点类 - PX4 uXRCE-DDS 版本
    
    该节点实现基于超声波雷达的避障功能，当检测到障碍物时，
    自动调整无人球的目标位置以避开障碍物。
    """
    
    # PX4 导航状态枚举
    NAV_STATE_OFFBOARD = 14

    def __init__(self):
        """初始化无人球避障节点"""
        super().__init__('usv_avoidance_node')

        # =====================================================================
        # QoS 配置
        # =====================================================================
        qos_px4 = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        qos_reliable = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # =====================================================================
        # 参数加载
        # =====================================================================
        param_loader = ParamLoader(self)
        
        self.in_distance_value = param_loader.load_param(
            'in_distance_value',
            1.2,
            ParamValidator.positive,
            '避障触发距离(米)'
        )
        
        self.declare_parameter('avoid_offset', 2.0)
        self.declare_parameter('coordinate_system', 'NED')
        
        self.avoid_offset = self.get_parameter('avoid_offset').value
        self.coordinate_system = self.get_parameter('coordinate_system').value

        # =====================================================================
        # PX4 订阅器（通过 avoidance_position / avoidance_flag 交由 usv_control_node 统一执行）
        # =====================================================================
        self.status_sub = self.create_subscription(
            VehicleStatus,
            'fmu/out/vehicle_status_v1',
            self.vehicle_status_callback,
            qos_px4
        )
        
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            'fmu/out/vehicle_local_position',
            self.local_position_callback,
            qos_px4
        )

        # =====================================================================
        # 传感器订阅
        # =====================================================================
        self.radar_sub = self.create_subscription(
            Range,
            'ultrasonic_radar_range',
            self.radar_callback,
            qos_reliable
        )

        # =====================================================================
        # 内部目标点订阅（来自 usv_control_node）
        # =====================================================================
        self.target_sub = self.create_subscription(
            PoseStamped,
            'set_usv_target_position',
            self.target_callback,
            qos_reliable
        )

        # =====================================================================
        # 避障状态发布
        # =====================================================================
        self.avoidance_flag_pub = self.create_publisher(
            Bool,
            'avoidance_flag',
            qos_reliable
        )
        
        # 避障目标点发布（PoseStamped 格式，供 usv_control_node 使用）
        self.avoidance_position_pub = self.create_publisher(
            PoseStamped,
            'avoidance_position',
            qos_reliable
        )

        # =====================================================================
        # 定时器
        # =====================================================================
        self.avoidance_timer = self.create_timer(0.2, self.avoidance_run)

        # =====================================================================
        # 状态变量
        # =====================================================================
        self.current_status = None
        self.current_range = float('inf')
        self.current_position = Point()
        self.current_target = Point()
        self.obstacle_detected = False
        self.vehicle_armed = False
        self.nav_state = 0
        
        # 位置有效性
        self.position_valid = False

        self.get_logger().info('✅ USV 避障节点 (PX4 uXRCE-DDS) 已启动')
        self.get_logger().info(f'避障距离阈值: {self.in_distance_value} 米')
        self.get_logger().info(f'避障偏移量: {self.avoid_offset} 米')

    def vehicle_status_callback(self, msg: VehicleStatus):
        """PX4 飞控状态回调"""
        self.current_status = msg
        self.vehicle_armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self.nav_state = msg.nav_state

    def local_position_callback(self, msg: VehicleLocalPosition):
        """
        PX4 本地位置回调
        
        PX4 使用 NED 坐标系：
        - x: 北向
        - y: 东向
        - z: 下向（负值表示向上）
        """
        if msg.xy_valid and msg.z_valid:
            self.position_valid = True
            
            if self.coordinate_system == 'NED':
                # NED: x=北, y=东, z=下
                self.current_position.x = msg.x
                self.current_position.y = msg.y
                self.current_position.z = msg.z
            else:
                # 转换为 ENU: x=东, y=北, z=上
                self.current_position.x = msg.y
                self.current_position.y = msg.x
                self.current_position.z = -msg.z

    def radar_callback(self, msg: Range):
        """雷达数据回调"""
        if msg.range >= msg.min_range and msg.range <= msg.max_range:
            self.current_range = msg.range
        else:
            self.current_range = float('inf')

    def target_callback(self, msg: PoseStamped):
        """目标点回调"""
        self.current_target.x = msg.pose.position.x
        self.current_target.y = msg.pose.position.y
        self.current_target.z = msg.pose.position.z

    def avoidance_run(self):
        """
        避障主逻辑函数
        
        定期检查是否需要进行避障操作，并在必要时发布新的目标点。
        """
        try:
            # 获取最新的避障距离阈值参数
            self.in_distance_value = self.get_parameter("in_distance_value").value
            
            # 检查飞控状态：必须已连接、已解锁且处于 OFFBOARD 模式
            if self.current_status is None:
                return
            
            if not self.vehicle_armed:
                return
            
            # 只在 OFFBOARD 模式下执行避障
            if self.nav_state != self.NAV_STATE_OFFBOARD:
                return
            
            # 检测障碍物
            prev_detected = self.obstacle_detected
            self.obstacle_detected = self.current_range < self.in_distance_value
            
            # 状态变化时打印日志
            if self.obstacle_detected and not prev_detected:
                self.get_logger().warn(
                    f'🚨 检测到障碍物! 距离: {self.current_range:.2f}m < {self.in_distance_value:.2f}m'
                )
            elif not self.obstacle_detected and prev_detected:
                self.get_logger().info('✅ 障碍物已清除')

            # 如果检测到障碍物且位置信息有效，则计算避障目标点
            if self.obstacle_detected and self.position_valid:
                self._publish_avoidance_target()

            # 发布避障状态
            flag_msg = Bool()
            flag_msg.data = self.obstacle_detected
            self.avoidance_flag_pub.publish(flag_msg)

        except Exception as e:
            self.get_logger().error(f'避障程序运行异常: {str(e)}')

    def _publish_avoidance_target(self):
        """计算并发布避障目标点"""
        # 计算当前到目标的方向
        dx = self.current_target.x - self.current_position.x
        dy = self.current_target.y - self.current_position.y
        
        # 避免除零错误
        if dx == 0 and dy == 0:
            heading = 0.0
        else:
            heading = math.atan2(dy, dx)
        
        # 绕障：向右偏移（可根据雷达数据动态调整）
        avoid_x = self.current_position.x + self.avoid_offset * math.sin(heading)
        avoid_y = self.current_position.y - self.avoid_offset * math.cos(heading)
        avoid_z = self.current_target.z  # 保持目标高度

        # 发布 PoseStamped 供 usv_control_node 统一转换为 PX4 设定值（避免双发布冲突）
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = avoid_x
        pose_msg.pose.position.y = avoid_y
        pose_msg.pose.position.z = avoid_z
        pose_msg.pose.orientation.w = 1.0
        
        self.avoidance_position_pub.publish(pose_msg)
        
        self.get_logger().info(
            f'📍 避障目标点已发布: ({avoid_x:.2f}, {avoid_y:.2f}, {avoid_z:.2f})'
        )

    def destroy_node(self):
        """节点销毁时的资源清理"""
        if hasattr(self, 'avoidance_timer'):
            self.avoidance_timer.cancel()
        super().destroy_node()


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    node = UsvAvoidancePx4Node()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
