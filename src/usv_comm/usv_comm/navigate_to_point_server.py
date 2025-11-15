#!/usr/bin/env python3
"""
NavigateToPoint Action服务器实现

该文件实现了NavigateToPoint Action服务器，用于接收导航目标点并控制USV移动到指定位置。
通过usv_control_node转发目标点，由其处理与飞控的通信。

作者: [待填写]
日期: [待填写]
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped
from common_interfaces.action import NavigateToPoint
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import math
import time


class NavigateToPointServer(Node):
    """
    NavigateToPoint Action服务器类
    
    该类实现了一个ROS2 Action服务器，用于接收导航目标点并控制USV移动到指定位置。
    通过MAVROS接口与飞控通信，发送位置控制指令。
    """

    def __init__(self):
        """
        初始化NavigateToPoint服务器节点
        """
        super().__init__('navigate_to_point_server')
        
        # 创建回调组，支持并发处理
        self.callback_group = ReentrantCallbackGroup()
        
        # 创建Action服务器
        self._action_server = ActionServer(
            self,
            NavigateToPoint,
            'navigate_to_point',
            self.execute_callback,
            callback_group=self.callback_group)
        
        # 创建QoS配置
        qos_best_effort = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        qos_reliable = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        # 创建订阅者，订阅当前姿态（使用 GPS 转换的统一坐标系）
        self.current_pose = None
        self.pose_sub = self.create_subscription(
            PoseStamped, 
            'local_position/pose_from_gps',  # 使用 GPS 转换坐标（A0 基站原点）
            self.pose_cb, 
            qos_best_effort,
            callback_group=self.callback_group)
        
        self.target_pub = self.create_publisher(
            PoseStamped, 
            'set_usv_target_position', 
            qos_reliable)
        
        
        # 导航参数
        self.declare_parameter('nav_arrival_threshold', 1.0)  # 到达目标点阈值(米)
        self.declare_parameter('nav_timeout_check_period', 1.0)  # 超时检查周期(秒)
        self.declare_parameter('nav_feedback_period', 0.5)  # 反馈发布周期(秒)
        
        self.nav_arrival_threshold = self.get_parameter('nav_arrival_threshold').get_parameter_value().double_value
        self.nav_timeout_check_period = self.get_parameter('nav_timeout_check_period').get_parameter_value().double_value
        self.nav_feedback_period = self.get_parameter('nav_feedback_period').get_parameter_value().double_value
        
        self.get_logger().info('NavigateToPoint服务器已启动')
        self.get_logger().info(f'到达目标点阈值: {self.nav_arrival_threshold} 米')

    def pose_cb(self, msg):
        """
        位置回调函数
        
        Args:
            msg (PoseStamped): 包含当前位置信息的消息
        """
        self.current_pose = msg

    def _calculate_heading_error(self, target_pose, current_pose):
        """
        计算航向误差（度）
        
        Args:
            target_pose (PoseStamped): 目标姿态
            current_pose (PoseStamped): 当前姿态
            
        Returns:
            float: 航向误差（度），范围 -180 ~ 180
        """
        from tf_transformations import euler_from_quaternion
        
        # 计算目标航向（yaw角）
        target_orientation = target_pose.pose.orientation
        target_euler = euler_from_quaternion([
            target_orientation.x,
            target_orientation.y,
            target_orientation.z,
            target_orientation.w
        ])
        target_yaw = target_euler[2]  # yaw 是第三个元素
        
        # 计算当前航向
        current_orientation = current_pose.pose.orientation
        current_euler = euler_from_quaternion([
            current_orientation.x,
            current_orientation.y,
            current_orientation.z,
            current_orientation.w
        ])
        current_yaw = current_euler[2]
        
        # 计算误差（弧度）
        error_rad = target_yaw - current_yaw
        
        # 归一化到 -π ~ π
        while error_rad > math.pi:
            error_rad -= 2 * math.pi
        while error_rad < -math.pi:
            error_rad += 2 * math.pi
        
        # 转换为度
        error_deg = math.degrees(error_rad)
        
        return error_deg

    async def execute_callback(self, goal_handle):
        """
        Action执行回调函数
        
        Args:
            goal_handle: Action目标句柄
            
        Returns:
            NavigateToPoint.Result: Action执行结果
        """
        # 🔍 调试日志：Action Server接收到目标点
        target_x = goal_handle.request.goal.pose.position.x
        target_y = goal_handle.request.goal.pose.position.y
        target_z = goal_handle.request.goal.pose.position.z
        
        self.get_logger().info(
            f"📥 [Action Server 接收] 导航目标点\n"
            f"  ├─ X坐标: {target_x:.3f} m\n"
            f"  ├─ Y坐标: {target_y:.3f} m\n"
            f"  ├─ Z坐标: {target_z:.3f} m\n"
            f"  └─ 超时: {goal_handle.request.timeout:.1f} s"
        )
        
        # 获取目标点和超时设置
        target_pose = goal_handle.request.goal
        timeout = goal_handle.request.timeout if goal_handle.request.timeout > 0 else 600.0
        start_time = time.time()
        
        # 记录上一次反馈发布时间
        last_feedback_time = start_time
        
        try:
            # 确保已获取当前位置
            while rclpy.ok() and self.current_pose is None:
                rclpy.spin_once(self, timeout_sec=0.1)
                # 检查是否超时
                if time.time() - start_time > timeout:
                    result = NavigateToPoint.Result()
                    result.success = False
                    result.error_code = 1  # 超时
                    result.message = '无法获取当前位置信息'
                    goal_handle.abort()
                    return result

            # 发布目标点到usv_control_node
            self._send_setpoint_to_control_node(target_pose)
            
            # 导航主循环
            while rclpy.ok():
                # 检查是否收到取消请求
                if goal_handle.is_cancel_requested:
                    result = NavigateToPoint.Result()
                    result.success = False
                    result.error_code = 4  # 被取消
                    result.message = '导航任务被取消'
                    goal_handle.canceled()
                    return result
                
                # 检查是否收到目标点
                if self.current_pose is None:
                    rclpy.spin_once(self, timeout_sec=0.1)
                    continue
                
                # 计算距离与航向
                dx = target_pose.pose.position.x - self.current_pose.pose.position.x
                dy = target_pose.pose.position.y - self.current_pose.pose.position.y
                dz = target_pose.pose.position.z - self.current_pose.pose.position.z
                distance_to_goal = math.sqrt(dx*dx + dy*dy + dz*dz)
                
                # 计算真实的航向误差
                heading_error = self._calculate_heading_error(target_pose, self.current_pose)
                
                # 发布反馈信息
                current_time = time.time()
                if current_time - last_feedback_time >= self.nav_feedback_period:
                    feedback = NavigateToPoint.Feedback()
                    feedback.distance_to_goal = distance_to_goal
                    feedback.heading_error = heading_error
                    # 简单的预计时间计算（实际应用中可能需要更复杂的计算）
                    feedback.estimated_time = distance_to_goal / 0.5  # 假设0.5m/s
                    goal_handle.publish_feedback(feedback)
                    last_feedback_time = current_time
                
                # 检查是否到达目标点
                if distance_to_goal < self.nav_arrival_threshold:
                    result = NavigateToPoint.Result()
                    result.success = True
                    result.error_code = 0
                    result.message = '成功到达目标点'
                    goal_handle.succeed()
                    return result
                
                # 检查是否超时
                if current_time - start_time > timeout:
                    result = NavigateToPoint.Result()
                    result.success = False
                    result.error_code = 1  # 超时
                    result.message = '导航超时未到达目标点'
                    goal_handle.abort()
                    return result
                
                # 继续发布目标点到控制节点
                self._send_setpoint_to_control_node(target_pose)
                
                # 等待下一个检查周期
                rclpy.spin_once(self, timeout_sec=self.nav_timeout_check_period)
                
        except Exception as e:
            self.get_logger().error(f'导航过程中发生错误: {e}')
            result = NavigateToPoint.Result()
            result.success = False
            result.error_code = 3  # 内部错误
            result.message = f'导航过程中发生错误: {str(e)}'
            goal_handle.abort()
            return result

    def _send_setpoint_to_control_node(self, target_pose):
        """
        发布目标点到控制节点
        
        Args:
            target_pose (PoseStamped): 目标点位姿
        """
        # 🔍 调试日志：转发到控制节点
        self.get_logger().info(
            f"📨 [Action Server 转发] → set_usv_target_position\n"
            f"  ├─ X: {target_pose.pose.position.x:.3f} m\n"
            f"  ├─ Y: {target_pose.pose.position.y:.3f} m\n"
            f"  └─ Z: {target_pose.pose.position.z:.3f} m"
        )
        
        # 发布目标点到usv_control_node
        self.target_pub.publish(target_pose)


def main(args=None):
    """
    主函数
    
    初始化ROS2并运行NavigateToPoint服务器节点
    """
    rclpy.init(args=args)
    
    # 创建服务器节点
    node = NavigateToPointServer()
    
    # 使用多线程执行器以支持并发处理
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        # 运行执行器
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('收到键盘中断信号，正在关闭...')
    finally:
        # 清理资源
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()