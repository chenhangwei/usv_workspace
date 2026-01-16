#!/usr/bin/env python3
"""
NavigateToPoint 导航节点 (基于话题通信)

该节点替代 Action Server,使用普通 ROS 2 话题实现导航功能。
更适合跨 Domain 通信场景,避免了 Domain Bridge 对 Action 转发的复杂性。

话题接口:
- 订阅: navigation_goal (NavigationGoal) - 接收导航目标
- 发布: navigation_feedback (NavigationFeedback) - 发送导航反馈
- 发布: navigation_result (NavigationResult) - 发送导航结果
- 发布: set_usv_target_position (PoseStamped) - 转发到控制节点

作者: Auto-generated
日期: 2025-11-19
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped
from common_interfaces.msg import NavigationGoal, NavigationFeedback, NavigationResult
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import math
from std_msgs.msg import Float32


class NavigateToPointNode(Node):
    """
    基于话题通信的导航节点
    
    功能:
    1. 接收导航目标(NavigationGoal)
    2. 定期发送反馈(NavigationFeedback)
    3. 到达或超时后发送结果(NavigationResult)
    4. 转发目标点到控制节点
    """

    def __init__(self):
        """初始化导航节点"""
        super().__init__('navigate_to_point_node')
        
        # 创建回调组
        self.callback_group = ReentrantCallbackGroup()
        
        # QoS 配置
        qos_best_effort = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        qos_reliable = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        # 订阅导航目标
        self.goal_sub = self.create_subscription(
            NavigationGoal,
            'navigation_goal',
            self.goal_callback,
            qos_reliable,
            callback_group=self.callback_group)
        
        # 发布导航反馈
        self.feedback_pub = self.create_publisher(
            NavigationFeedback,
            'navigation_feedback',
            qos_reliable)
        
        # 发布导航结果
        self.result_pub = self.create_publisher(
            NavigationResult,
            'navigation_result',
            qos_reliable)
        
        # 发布目标位置到控制节点
        self.target_pub = self.create_publisher(
            NavigationGoal,
            'set_usv_nav_goal',
            qos_reliable)

        # 订阅导航参数下发：到达阈值（米）
        # 说明：跨 Domain Bridge 时，参数服务不可用，改为话题下发
        self.nav_threshold_sub = self.create_subscription(
            Float32,
            'set_nav_arrival_threshold',
            self._nav_arrival_threshold_callback,
            qos_reliable,
            callback_group=self.callback_group)
        
        # 订阅当前位置 (使用 GPS 转换的统一坐标系)
        self.current_pose = None
        self.pose_sub = self.create_subscription(
            PoseStamped,
            'local_position/pose_from_gps',
            self.pose_callback,
            qos_best_effort,
            callback_group=self.callback_group)
        
        # 使用ParamLoader统一加载导航参数
        from common_utils import ParamLoader
        loader = ParamLoader(self)
        self.nav_arrival_threshold = loader.load_param('nav_arrival_threshold', 2.0)  # 到达阈值(米)
        self.nav_feedback_period = loader.load_param('nav_feedback_period', 0.5)      # 反馈周期(秒)
        self.distance_mode = loader.load_param(
            'distance_mode', '2d',
            validator=lambda x: x in ['2d', '3d'])  # 距离模式: 2d/3d

        # 重复目标去重：防止同一目标被重复发送导致任务状态机重置
        self.declare_parameter('dedup_goal_enabled', True)
        self.declare_parameter('dedup_goal_pos_epsilon', 0.01)
        self.declare_parameter('dedup_goal_yaw_epsilon_deg', 2.0)
        self.declare_parameter('dedup_goal_maneuver_param_epsilon', 1e-3)
        
        # 当前任务状态
        self.current_goal = None
        self.current_goal_id = None
        self.goal_start_time = None

        # 记录上一次被合并的重复goal_id，避免刷屏
        self._last_dedup_goal_id = None
        
        # 创建导航循环定时器
        self.nav_timer = self.create_timer(
            self.nav_feedback_period,
            self.navigation_loop,
            callback_group=self.callback_group)
        
        self.get_logger().info('NavigateToPoint 节点已启动 (话题模式)')
        self.get_logger().info(f'到达阈值: {self.nav_arrival_threshold}m')
        self.get_logger().info(f'距离模式: {self.distance_mode.upper()}')
        self.get_logger().info(f'反馈周期: {self.nav_feedback_period}s')

    def _nav_arrival_threshold_callback(self, msg: Float32):
        """运行时更新到达阈值（米）。"""
        try:
            value = float(msg.data)
        except Exception:
            self.get_logger().warn(f"收到非法 nav_arrival_threshold: {msg.data}")
            return

        if value <= 0.0:
            self.get_logger().warn(f"忽略 nav_arrival_threshold<=0: {value}")
            return

        old = getattr(self, 'nav_arrival_threshold', None)
        self.nav_arrival_threshold = value
        try:
            if old is None:
                self.get_logger().info(f"nav_arrival_threshold 已设置为 {value:.2f}m")
            else:
                self.get_logger().info(f"nav_arrival_threshold 更新: {float(old):.2f}m -> {value:.2f}m")
        except Exception:
            self.get_logger().info(f"nav_arrival_threshold 更新为 {value:.2f}m")
    
    def pose_callback(self, msg):
        """更新当前位置"""
        self.current_pose = msg
    
    def goal_callback(self, msg):
        """
        接收新的导航目标
        
        Args:
            msg (NavigationGoal): 导航目标消息
        """
        self.get_logger().info(
            f'📥 收到新目标 [ID={msg.goal_id}]: '
            f'({msg.target_pose.pose.position.x:.2f}, '
            f'{msg.target_pose.pose.position.y:.2f}, '
            f'{msg.target_pose.pose.position.z:.2f}), '
            f'超时={msg.timeout:.0f}s')

        # 如果与当前任务目标重复，则不覆盖 current_goal / 不重置计时
        if self._is_duplicate_goal(msg):
            if self._last_dedup_goal_id != msg.goal_id:
                self.get_logger().info(
                    f'♻️ 重复目标已合并: new_id={msg.goal_id} -> keep_id={self.current_goal_id}'
                )
                self._last_dedup_goal_id = msg.goal_id
            # 仅更新对外反馈的 goal_id，避免地面站等待“新ID”的反馈/结果
            self.current_goal_id = msg.goal_id
            return

        # 保存当前任务
        self.current_goal = msg
        self.current_goal_id = msg.goal_id
        self.goal_start_time = self.get_clock().now()
        self._last_dedup_goal_id = None

        # 立即转发目标到控制节点
        self.target_pub.publish(msg)
        self.get_logger().info('✓ 目标点已转发到控制节点')

    def _is_duplicate_goal(self, msg: NavigationGoal) -> bool:
        """判断新 goal 是否与当前 goal 等价（位置/航向/机动一致）。"""
        try:
            enabled_val = self.get_parameter('dedup_goal_enabled').value
            if enabled_val is None:
                return False
            if not bool(enabled_val):
                return False
        except Exception:
            return False

        if self.current_goal is None:
            return False

        def _param_float(name: str, default: float) -> float:
            try:
                v = self.get_parameter(name).value
                if v is None:
                    return default
                return float(v)
            except Exception:
                return default

        pos_eps = _param_float('dedup_goal_pos_epsilon', 0.01)
        yaw_eps_deg = _param_float('dedup_goal_yaw_epsilon_deg', 2.0)
        man_eps = _param_float('dedup_goal_maneuver_param_epsilon', 1e-3)

        pos_eps = max(0.0, pos_eps)
        yaw_eps = math.radians(max(0.0, yaw_eps_deg))
        man_eps = max(0.0, man_eps)

        a = self.current_goal
        b = msg

        ap = a.target_pose.pose.position
        bp = b.target_pose.pose.position
        dx = float(ap.x) - float(bp.x)
        dy = float(ap.y) - float(bp.y)
        dz = float(ap.z) - float(bp.z)
        if math.sqrt(dx * dx + dy * dy + dz * dz) > pos_eps:
            return False

        if bool(getattr(a, 'enable_yaw', False)) != bool(getattr(b, 'enable_yaw', False)):
            return False

        if bool(getattr(b, 'enable_yaw', False)):
            ay = self._yaw_from_quat(a.target_pose.pose.orientation)
            by = self._yaw_from_quat(b.target_pose.pose.orientation)
            if abs(self._wrap_pi(ay - by)) > yaw_eps:
                return False

        if int(getattr(a, 'maneuver_type', 0)) != int(getattr(b, 'maneuver_type', 0)):
            return False
        if abs(float(getattr(a, 'maneuver_param', 0.0)) - float(getattr(b, 'maneuver_param', 0.0))) > man_eps:
            return False

        # timeout 允许不同：如果你希望 timeout 不同也当新任务，可把下面打开
        # if abs(float(getattr(a, 'timeout', 0.0)) - float(getattr(b, 'timeout', 0.0))) > 1e-6:
        #     return False

        return True

    @staticmethod
    def _wrap_pi(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def _yaw_from_quat(q) -> float:
        """从四元数计算 yaw (rad)，与常见 ENU 公式一致。"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def navigation_loop(self):
        """
        导航主循环 - 定时检查进度并发送反馈
        
        该函数每隔固定周期执行一次,负责:
        1. 计算到目标点的距离和航向误差
        2. 发送导航反馈
        3. 检查是否到达或超时
        4. 发送导航结果
        """
        # 如果没有活动任务,跳过
        if self.current_goal is None:
            return
        
        # 如果还没收到当前位置,跳过
        if self.current_pose is None:
            return
        
        # 计算到目标点的距离
        distance = self._calculate_distance(
            self.current_goal.target_pose,
            self.current_pose)
        
        # 计算航向误差
        heading_error = self._calculate_heading_error(
            self.current_goal.target_pose,
            self.current_pose)
        
        # 估算剩余时间 (假设平均速度 0.5 m/s)
        estimated_time = distance / 0.5
        
        # 发布反馈
        feedback = NavigationFeedback()
        feedback.goal_id = self.current_goal_id
        feedback.distance_to_goal = distance
        feedback.heading_error = heading_error
        feedback.estimated_time = estimated_time
        feedback.timestamp = self.get_clock().now().to_msg()
        self.feedback_pub.publish(feedback)
        
        # 简化日志 - 只在距离变化较大时输出
        if not hasattr(self, '_last_distance') or abs(distance - self._last_distance) > 0.5:
            self.get_logger().info(
                f'导航中 [ID={self.current_goal_id}]: '
                f'距离={distance:.2f}m, 航向误差={heading_error:.1f}°')
            self._last_distance = distance
        
        # 检查是否到达目标点
        if distance < self.nav_arrival_threshold:
            self.get_logger().info(
                f'🎯 到达目标点! [ID={self.current_goal_id}] '
                f'最终距离={distance:.3f}m')
            
            # 发布成功结果
            result = NavigationResult()
            result.goal_id = self.current_goal_id
            result.success = True
            result.error_code = 0
            result.message = f'成功到达目标点,最终距离{distance:.3f}m'
            result.timestamp = self.get_clock().now().to_msg()
            self.result_pub.publish(result)
            
            # 清除当前任务
            self._clear_current_goal()
            return
        
        # 检查是否超时
        if self.goal_start_time is None:
            self.goal_start_time = self.get_clock().now()
        elapsed = (self.get_clock().now() - self.goal_start_time).nanoseconds / 1e9
        if elapsed > self.current_goal.timeout:
            self.get_logger().warn(
                f'⏱️ 导航超时! [ID={self.current_goal_id}] '
                f'耗时={elapsed:.1f}s, 剩余距离={distance:.2f}m')
            
            # 发布失败结果
            result = NavigationResult()
            result.goal_id = self.current_goal_id
            result.success = False
            result.error_code = 1  # 超时
            result.message = f'导航超时,耗时{elapsed:.1f}s,剩余距离{distance:.2f}m'
            result.timestamp = self.get_clock().now().to_msg()
            self.result_pub.publish(result)
            
            # 清除当前任务
            self._clear_current_goal()
            return
    
    def _clear_current_goal(self):
        """清除当前导航任务"""
        self.current_goal = None
        self.current_goal_id = None
        self.goal_start_time = None
        if hasattr(self, '_last_distance'):
            delattr(self, '_last_distance')
    
    def _calculate_distance(self, target_pose, current_pose):
        """
        计算到目标点的距离
        
        Args:
            target_pose (PoseStamped): 目标位置
            current_pose (PoseStamped): 当前位置
        
        Returns:
            float: 距离(米)
        """
        dx = target_pose.pose.position.x - current_pose.pose.position.x
        dy = target_pose.pose.position.y - current_pose.pose.position.y
        
        if self.distance_mode == '3d':
            dz = target_pose.pose.position.z - current_pose.pose.position.z
            return math.sqrt(dx*dx + dy*dy + dz*dz)
        else:
            return math.sqrt(dx*dx + dy*dy)
    
    def _calculate_heading_error(self, target_pose, current_pose):
        """
        计算航向误差
        
        Args:
            target_pose (PoseStamped): 目标位置
            current_pose (PoseStamped): 当前位置
        
        Returns:
            float: 航向误差(度)
        """
        # 计算期望航向
        dx = target_pose.pose.position.x - current_pose.pose.position.x
        dy = target_pose.pose.position.y - current_pose.pose.position.y
        desired_yaw = math.atan2(dy, dx)
        
        # 获取当前航向 (从四元数转换)
        from tf_transformations import euler_from_quaternion
        q = current_pose.pose.orientation
        _, _, current_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        # 计算误差并归一化到 [-180, 180]
        error = math.degrees(desired_yaw - current_yaw)
        while error > 180:
            error -= 360
        while error < -180:
            error += 360
        
        return error

    def destroy_node(self):
        """节点销毁时的资源清理"""
        if hasattr(self, 'nav_timer'):
            self.nav_timer.cancel()
        super().destroy_node()


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    node = NavigateToPointNode()
    
    # 使用多线程执行器支持并发
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
