"""
导航处理模块

负责处理所有与 USV 导航相关的功能：
- 发送导航目标点
- 处理导航反馈和结果
- 验证目标位置
"""

import threading
import math
from typing import Optional, Dict, Any

from rclpy.node import Node
from common_interfaces.msg import NavigationGoal
from common_utils import ThreadSafeDict


class NavigationHandler:
    """导航处理器类"""
    
    def __init__(self, node: Node, usv_manager: Any, ros_signal: Any) -> None:
        """
        初始化导航处理器
        
        Args:
            node: ROS 节点实例
            usv_manager: USV 管理器实例
            ros_signal: ROS 信号对象
        """
        self.node = node
        self.usv_manager = usv_manager
        self.ros_signal = ros_signal
        self.logger = node.get_logger()
        
        # 导航状态管理
        self._next_goal_id = 1
        self._goal_id_lock = threading.Lock()
        self._goal_to_usv = ThreadSafeDict()  # 目标ID到USV的映射
        self._usv_nav_target_cache = ThreadSafeDict()  # 导航目标缓存
        self._send_locks = ThreadSafeDict()  # 发送锁
        
        # 安全范围参数
        self.MAX_DISTANCE = 500.0  # 最大水平距离 500m
        self.MAX_ALTITUDE = 10.0   # 最大高度 10m

    @property
    def nav_target_cache(self) -> ThreadSafeDict:
        """获取导航目标缓存"""
        return self._usv_nav_target_cache
    
    def validate_target_position(self, x: float, y: float, z: float) -> None:
        """
        验证目标点是否在安全范围内
        
        Args:
            x: 目标点X坐标
            y: 目标点Y坐标
            z: 目标点Z坐标
            
        Raises:
            ValueError: 如果目标点超出安全范围
        """
        # 计算2D距离
        distance_2d = math.sqrt(x**2 + y**2)
        
        # 检查水平距离
        if distance_2d > self.MAX_DISTANCE:
            raise ValueError(
                f"目标点距离过远: {distance_2d:.2f}m > {self.MAX_DISTANCE}m"
            )
        
        # 检查高度
        if abs(z) > self.MAX_ALTITUDE:
            raise ValueError(
                f"目标点高度异常: {abs(z):.2f}m > {self.MAX_ALTITUDE}m"
            )
    
    def _get_next_goal_id(self) -> int:
        """获取下一个目标ID"""
        with self._goal_id_lock:
            goal_id = self._next_goal_id
            self._next_goal_id += 1
            if self._next_goal_id > 65535:
                self._next_goal_id = 1
            return goal_id
    
    def send_nav_goal(
        self, 
        usv_id: str, 
        x: float, 
        y: float, 
        z: float = 0.0, 
        yaw: float = 0.0, 
        timeout: float = 300.0
    ) -> bool:
        """
        通过话题方式向指定USV发送导航目标点
        
        Args:
            usv_id: USV标识符
            x: 目标点X坐标
            y: 目标点Y坐标
            z: 目标点Z坐标
            yaw: 目标偏航角(弧度)
            timeout: 超时时间(秒)
        
        Returns:
            发送是否成功
        """
        # 验证目标点
        try:
            self.validate_target_position(x, y, z)
        except ValueError as e:
            self.logger.error(f"{usv_id} 导航目标验证失败: {e}")
            return False
        
        # 获取发送锁
        if usv_id not in self._send_locks:
            self._send_locks[usv_id] = threading.Lock()
        
        if not self._send_locks[usv_id].acquire(blocking=False):
            self.logger.warn(f"{usv_id} 正在发送导航目标，请稍后重试")
            return False
        
        try:
            # 获取发布器
            ns = f"/{usv_id}" if not usv_id.startswith('/') else usv_id
            publisher = self.usv_manager.nav_goal_pubs.get(usv_id)
            
            if publisher is None:
                self.logger.error(f"{usv_id} 导航发布器不存在")
                return False
            
            # 创建导航目标消息
            goal_msg = NavigationGoal()
            goal_msg.goal_id = self._get_next_goal_id()
            goal_msg.target_pose.header.stamp = self.node.get_clock().now().to_msg()
            goal_msg.target_pose.header.frame_id = 'map'
            goal_msg.target_pose.pose.position.x = float(x)
            goal_msg.target_pose.pose.position.y = float(y)
            goal_msg.target_pose.pose.position.z = float(z)
            
            # 设置目标偏航角
            goal_msg.target_pose.pose.orientation.w = math.cos(yaw / 2)
            goal_msg.target_pose.pose.orientation.z = math.sin(yaw / 2)
            
            goal_msg.timeout = float(timeout)
            
            # 记录目标映射
            self._goal_to_usv[goal_msg.goal_id] = usv_id
            
            # 缓存导航目标信息
            self._usv_nav_target_cache[usv_id] = {
                'goal_id': goal_msg.goal_id,
                'x': x, 'y': y, 'z': z,
                'yaw': yaw,
                'status': 'PENDING',
                'progress': 0.0,
                'start_time': self.node.get_clock().now().nanoseconds / 1e9
            }
            
            # 发布
            publisher.publish(goal_msg)
            self.logger.info(
                f"✓ {usv_id} 导航目标已发送: goal_id={goal_msg.goal_id}, "
                f"位置=({x:.2f}, {y:.2f}, {z:.2f})"
            )
            
            return True
            
        except Exception as e:
            self.logger.error(f"{usv_id} 发送导航目标失败: {e}")
            return False
        finally:
            self._send_locks[usv_id].release()
    
    def handle_feedback(self, msg, usv_id):
        """
        处理导航反馈
        
        Args:
            msg: NavigationFeedback 消息
            usv_id: USV 标识符
        """
        try:
            # 更新缓存
            if usv_id in self._usv_nav_target_cache:
                cache = self._usv_nav_target_cache[usv_id]
                cache['status'] = msg.status
                cache['progress'] = msg.progress
                cache['distance_remaining'] = msg.distance_remaining
                cache['eta'] = msg.eta
                
            # 通知GUI更新
            try:
                feedback_data = {
                    'usv_id': usv_id,
                    'goal_id': msg.goal_id,
                    'status': msg.status,
                    'progress': msg.progress,
                    'distance_remaining': msg.distance_remaining,
                    'eta': msg.eta
                }
                self.ros_signal.navigation_feedback.emit(feedback_data)
            except Exception:
                pass
                
        except Exception as e:
            self.logger.error(f"处理导航反馈失败: {e}")
    
    def handle_result(self, msg, usv_id):
        """
        处理导航结果
        
        Args:
            msg: NavigationResult 消息
            usv_id: USV 标识符
        """
        try:
            goal_id = msg.goal_id
            success = msg.success
            message = msg.message
            
            # 更新缓存
            if usv_id in self._usv_nav_target_cache:
                cache = self._usv_nav_target_cache[usv_id]
                cache['status'] = 'SUCCEEDED' if success else 'FAILED'
                cache['progress'] = 1.0 if success else cache.get('progress', 0.0)
                cache['result_message'] = message
            
            # 清理目标映射
            if goal_id in self._goal_to_usv:
                del self._goal_to_usv[goal_id]
            
            # 记录日志
            if success:
                self.logger.info(f"✓ {usv_id} 导航完成: {message}")
            else:
                self.logger.warn(f"✗ {usv_id} 导航失败: {message}")
            
            # 通知GUI
            try:
                result_data = {
                    'usv_id': usv_id,
                    'goal_id': goal_id,
                    'success': success,
                    'message': message
                }
                self.ros_signal.navigation_result.emit(result_data)
            except Exception:
                pass
                
        except Exception as e:
            self.logger.error(f"处理导航结果失败: {e}")
    
    def get_nav_target_cache(self, usv_id: Optional[str] = None) -> Optional[Dict[str, Any]]:
        """
        获取导航目标缓存
        
        Args:
            usv_id: 指定USV ID，为None时返回所有
            
        Returns:
            导航目标缓存
        """
        if usv_id:
            return self._usv_nav_target_cache.get(usv_id)
        return dict(self._usv_nav_target_cache)
    
    def cancel_navigation(self, usv_id: str) -> bool:
        """
        取消导航任务
        
        通过清除导航目标缓存并发送空目标点来取消当前导航。
        
        Args:
            usv_id: USV 标识符
            
        Returns:
            是否成功
        """
        try:
            # 清除导航目标缓存
            if usv_id in self._usv_nav_target_cache:
                cache = self._usv_nav_target_cache[usv_id]
                cache['status'] = 'CANCELLED'
                self.logger.info(f"{usv_id} 导航任务已取消 (goal_id={cache.get('goal_id', 'N/A')})")
                del self._usv_nav_target_cache[usv_id]
            
            # 清除目标ID映射
            goal_ids_to_remove = [
                gid for gid, uid in list(self._goal_to_usv.items()) 
                if uid == usv_id
            ]
            for gid in goal_ids_to_remove:
                del self._goal_to_usv[gid]
            
            self.logger.info(f"✓ {usv_id} 取消导航请求已处理")
            return True
            
        except Exception as e:
            self.logger.error(f"{usv_id} 取消导航失败: {e}")
            return False
