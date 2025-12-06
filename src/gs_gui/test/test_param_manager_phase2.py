#!/usr/bin/env python3
"""
参数管理器 Phase 2 测试脚本

测试基于 topic 订阅的异步参数加载功能
"""

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import Param
from mavros_msgs.srv import ParamPull
import time


class ParamTestPublisher(Node):
    """
    测试用参数发布器
    
    模拟 MAVROS param 插件的行为，发布参数到 topic
    """
    
    def __init__(self):
        super().__init__('param_test_publisher')
        
        # 创建参数发布器
        self.param_pub = self.create_publisher(
            Param,
            '/usv_01/param/param_value',
            10
        )
        
        # 创建 ParamPull 服务
        self.pull_service = self.create_service(
            ParamPull,
            '/usv_01/param/pull',
            self.handle_param_pull
        )
        
        self.get_logger().info("参数测试发布器已启动")
    
    def handle_param_pull(self, request, response):
        """处理 ParamPull 服务请求"""
        self.get_logger().info("收到 ParamPull 请求，开始发布测试参数...")
        
        # 模拟参数列表
        test_params = [
            ('ARMING_CHECK', 1, True),
            ('ARMING_VOLT_MIN', 10.5, False),
            ('GPS_TYPE', 1, True),
            ('COMPASS_USE', 1, True),
            ('BATT_CAPACITY', 5000, True),
            ('FRAME_TYPE', 2, True),
            ('SYSID_THISMAV', 1, True),
            ('COMPASS_DEC', 0.0, False),
            ('WPNAV_SPEED', 500.0, False),
            ('RTL_ALT', 1500, True),
        ]
        
        # 异步发布参数（模拟 MAVROS 逐个发送）
        def publish_params():
            for i, (name, value, is_int) in enumerate(test_params):
                # 等待一小段时间模拟真实情况
                time.sleep(0.1)
                
                # 创建并发布参数消息
                msg = Param()
                msg.header.frame_id = name  # 参数名在 frame_id
                
                if is_int:
                    msg.value.integer = int(value)
                    msg.value.real = 0.0
                else:
                    msg.value.integer = 0
                    msg.value.real = float(value)
                
                self.param_pub.publish(msg)
                self.get_logger().info(f"发布参数 {i+1}/{len(test_params)}: {name} = {value}")
            
            self.get_logger().info(f"完成发布 {len(test_params)} 个测试参数")
        
        # 在后台线程发布
        import threading
        thread = threading.Thread(target=publish_params, daemon=True)
        thread.start()
        
        # 立即返回响应
        response.success = True
        response.param_received = len(test_params)
        
        return response


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    node = ParamTestPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
