#!/usr/bin/env python3
"""
USV优雅关闭服务节点

提供ROS 2服务接口，允许地面站远程优雅关闭USV节点
支持：
1. 全部关闭：停止所有USV节点
2. 选择性关闭：停止指定的节点
3. 延迟关闭：给节点时间保存状态

作者：自动生成
日期：2025-11-20
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from common_interfaces.srv import ShutdownControl
import subprocess
import time
import os
import signal
import psutil
from typing import List, Optional


class ShutdownServiceNode(Node):
    """
    USV优雅关闭服务节点
    
    提供两个服务：
    1. /shutdown_all - 关闭所有USV节点（Trigger）
    2. /shutdown_selective - 选择性关闭指定节点（ShutdownControl）
    """
    
    def __init__(self):
        super().__init__('shutdown_service')
        
        # 获取命名空间（用于识别当前USV）
        self.namespace = self.get_namespace()
        self.get_logger().info(f'优雅关闭服务已启动 (命名空间: {self.namespace})')
        
        # 服务1：关闭所有节点
        self.shutdown_all_srv = self.create_service(
            Trigger,
            'shutdown_all',
            self.shutdown_all_callback
        )
        
        # 服务2：选择性关闭
        # 注意：ShutdownControl是自定义服务，需要在common_interfaces中定义
        # 如果不存在，可以先使用Trigger，或者创建该服务类型
        try:
            self.shutdown_selective_srv = self.create_service(
                ShutdownControl,
                'shutdown_selective',
                self.shutdown_selective_callback
            )
        except Exception as e:
            self.get_logger().warn(f'无法创建选择性关闭服务: {e}')
            self.shutdown_selective_srv = None
        
        self.get_logger().info('✅ 优雅关闭服务就绪')
        self.get_logger().info(f'   - 服务: {self.namespace}/shutdown_all')
        if self.shutdown_selective_srv:
            self.get_logger().info(f'   - 服务: {self.namespace}/shutdown_selective')
    
    def shutdown_all_callback(self, request, response):
        """
        关闭所有USV节点的回调
        
        策略：
        1. 记录请求
        2. 发送SIGTERM信号给所有ROS节点
        3. 等待节点优雅退出（最多5秒）
        4. 如果仍有节点，发送SIGKILL强制终止
        """
        self.get_logger().info('🛑 收到全部关闭请求')
        
        try:
            # 获取当前进程的PID（shutdown_service自己）
            my_pid = os.getpid()
            
            # 查找所有ROS节点进程
            ros_pids = self._find_ros_processes(exclude_pids=[my_pid])
            
            if not ros_pids:
                response.success = True
                response.message = '没有需要关闭的节点'
                self.get_logger().info('✅ 没有其他节点运行')
                return response
            
            self.get_logger().info(f'找到 {len(ros_pids)} 个ROS节点进程')
            
            # 第一阶段：温和终止（SIGTERM）
            self.get_logger().info('📤 发送SIGTERM信号...')
            for pid in ros_pids:
                try:
                    os.kill(pid, signal.SIGTERM)
                    self.get_logger().info(f'   发送SIGTERM到PID {pid}')
                except ProcessLookupError:
                    pass  # 进程已经不存在
                except Exception as e:
                    self.get_logger().warn(f'   无法向PID {pid}发送信号: {e}')
            
            # 等待节点退出（最多5秒）
            self.get_logger().info('⏳ 等待节点优雅退出（最多5秒）...')
            wait_time = 0
            while wait_time < 5.0:
                time.sleep(0.5)
                wait_time += 0.5
                
                # 检查是否还有进程存活
                alive_pids = [pid for pid in ros_pids if self._is_process_alive(pid)]
                if not alive_pids:
                    self.get_logger().info('✅ 所有节点已优雅退出')
                    break
                
                self.get_logger().info(f'   还有 {len(alive_pids)} 个进程存活...')
            
            # 第二阶段：强制终止（SIGKILL）
            alive_pids = [pid for pid in ros_pids if self._is_process_alive(pid)]
            if alive_pids:
                self.get_logger().warn(f'⚠️  {len(alive_pids)} 个进程未响应SIGTERM，发送SIGKILL')
                for pid in alive_pids:
                    try:
                        os.kill(pid, signal.SIGKILL)
                        self.get_logger().info(f'   发送SIGKILL到PID {pid}')
                    except ProcessLookupError:
                        pass
                    except Exception as e:
                        self.get_logger().error(f'   无法强制终止PID {pid}: {e}')
                
                time.sleep(1.0)  # 等待SIGKILL生效
            
            # 验证是否全部关闭
            final_alive = [pid for pid in ros_pids if self._is_process_alive(pid)]
            if final_alive:
                response.success = False
                response.message = f'部分节点关闭失败 (剩余{len(final_alive)}个)'
                self.get_logger().error(f'❌ {len(final_alive)} 个进程仍在运行: {final_alive}')
            else:
                response.success = True
                response.message = f'已成功关闭 {len(ros_pids)} 个节点'
                self.get_logger().info(f'✅ 已成功关闭所有节点 (共{len(ros_pids)}个)')
            
            # 延迟关闭自己（给时间返回响应）
            self.get_logger().info('🔚 准备关闭自己（2秒后）')
            self.create_timer(2.0, self._shutdown_self)
            
            return response
        
        except Exception as e:
            self.get_logger().error(f'❌ 关闭失败: {e}')
            response.success = False
            response.message = f'关闭失败: {str(e)}'
            return response
    
    def shutdown_selective_callback(self, request, response):
        """
        选择性关闭指定节点的回调
        
        request.node_names: 要关闭的节点名称列表（可以是完整名称或部分匹配）
        """
        self.get_logger().info(f'🎯 收到选择性关闭请求: {request.node_names}')
        
        try:
            # 查找匹配的进程
            target_pids = self._find_matching_processes(request.node_names)
            
            if not target_pids:
                response.success = False
                response.message = '未找到匹配的节点进程'
                self.get_logger().warn('⚠️  未找到匹配的节点')
                return response
            
            self.get_logger().info(f'找到 {len(target_pids)} 个匹配进程')
            
            # 温和终止
            for pid in target_pids:
                try:
                    os.kill(pid, signal.SIGTERM)
                    self.get_logger().info(f'   发送SIGTERM到PID {pid}')
                except Exception as e:
                    self.get_logger().warn(f'   无法向PID {pid}发送信号: {e}')
            
            # 等待退出
            time.sleep(2.0)
            
            # 检查结果
            alive = [pid for pid in target_pids if self._is_process_alive(pid)]
            if alive:
                # 强制终止
                self.get_logger().warn(f'⚠️  强制终止 {len(alive)} 个未响应的进程')
                for pid in alive:
                    try:
                        os.kill(pid, signal.SIGKILL)
                    except:
                        pass
                time.sleep(0.5)
            
            response.success = True
            response.message = f'已关闭 {len(target_pids)} 个节点'
            self.get_logger().info(f'✅ 已关闭目标节点 (共{len(target_pids)}个)')
            
            return response
        
        except Exception as e:
            self.get_logger().error(f'❌ 选择性关闭失败: {e}')
            response.success = False
            response.message = f'关闭失败: {str(e)}'
            return response
    
    def _find_ros_processes(self, exclude_pids: List[int] = None) -> List[int]:
        """
        查找所有ROS节点进程
        
        Args:
            exclude_pids: 要排除的PID列表
        
        Returns:
            ROS进程的PID列表
        """
        exclude_pids = exclude_pids or []
        ros_pids = []
        
        try:
            # 方法1：使用psutil查找（更可靠）
            for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                try:
                    if proc.info['pid'] in exclude_pids:
                        continue
                    
                    cmdline = proc.info['cmdline']
                    if not cmdline:
                        continue
                    
                    cmdline_str = ' '.join(cmdline)
                    
                    # 匹配ROS节点特征
                    # 包括：ros2 run, __node, mavros, usv_*, 但排除shutdown_service自己
                    if any(keyword in cmdline_str for keyword in [
                        'ros2 run',
                        'ros2 launch',
                        '__node:=',
                        'mavros_node',
                        'usv_status_node',
                        'usv_control_node',
                        'usv_command_node',
                        'gps_to_local_node',
                        'coord_transform_node',
                        'navigate_to_point_node',
                    ]) and 'shutdown_service' not in cmdline_str:
                        ros_pids.append(proc.info['pid'])
                        self.get_logger().debug(f'   找到ROS进程: PID={proc.info["pid"]}, cmd={cmdline_str[:80]}...')
                
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue
        
        except Exception as e:
            self.get_logger().error(f'查找ROS进程失败: {e}')
            
            # 方法2：回退到pgrep命令
            try:
                result = subprocess.run(
                    ['pgrep', '-f', 'ros2'],
                    capture_output=True,
                    text=True,
                    timeout=5
                )
                if result.returncode == 0:
                    for line in result.stdout.strip().split('\n'):
                        if line:
                            pid = int(line)
                            if pid not in exclude_pids:
                                ros_pids.append(pid)
            except Exception as e2:
                self.get_logger().error(f'pgrep回退方案也失败: {e2}')
        
        return ros_pids
    
    def _find_matching_processes(self, node_names: List[str]) -> List[int]:
        """
        查找匹配指定节点名的进程
        
        Args:
            node_names: 节点名称列表（支持部分匹配）
        
        Returns:
            匹配的进程PID列表
        """
        matching_pids = []
        
        try:
            for proc in psutil.process_iter(['pid', 'cmdline']):
                try:
                    cmdline = proc.info['cmdline']
                    if not cmdline:
                        continue
                    
                    cmdline_str = ' '.join(cmdline)
                    
                    # 检查是否匹配任一节点名
                    for node_name in node_names:
                        if node_name in cmdline_str:
                            matching_pids.append(proc.info['pid'])
                            self.get_logger().info(f'   匹配: PID={proc.info["pid"]}, 节点={node_name}')
                            break
                
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue
        
        except Exception as e:
            self.get_logger().error(f'查找匹配进程失败: {e}')
        
        return matching_pids
    
    def _is_process_alive(self, pid: int) -> bool:
        """检查进程是否存活"""
        try:
            os.kill(pid, 0)  # 信号0只检查，不终止
            return True
        except OSError:
            return False
    
    def _shutdown_self(self):
        """关闭自己"""
        self.get_logger().info('👋 关闭shutdown_service节点')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    
    node = ShutdownServiceNode()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
