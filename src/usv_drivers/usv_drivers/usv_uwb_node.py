"""
无人球UWB定位节点

该节点负责读取UWB定位数据并通过PX4原生话题发布。
支持多种UWB定位设备，通过串口通信获取数据。

PX4 话题：/fmu/in/vehicle_visual_odometry
消息类型：px4_msgs/msg/VehicleOdometry
"""

import rclpy 
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry
import math

# 导入common_utils工具
from common_utils import SerialResourceManager, ParamLoader, ParamValidator


class UsvUwbNode(Node):
    """
    无人球UWB定位节点类
    
    该节点实现UWB定位数据读取和发布功能，通过串口与UWB定位设备通信，
    将读取到的数据转换为PoseStamped消息格式并发布。
    """

    def __init__(self):
        """初始化无人球UWB定位节点"""
        super().__init__('usv_uwb_node')
        
        # 创建参数加载器
        param_loader = ParamLoader(self)
        
        # 加载串口参数
        port = param_loader.load_param(
            'uwb_port',
            '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5787006321-if00',
            ParamValidator.non_empty_string,
            'UWB串口路径'
        )
        baudrate = param_loader.load_param(
            'uwb_baudrate',
            115200,
            lambda x: x in [9600, 19200, 38400, 57600, 115200],
            'UWB波特率'
        )
        timeout = param_loader.load_param(
            'uwb_timeout',
            1.0,
            ParamValidator.positive,
            '串口超时(秒)'
        )
        
        # 创建串口资源管理器
        self.serial_manager = SerialResourceManager(self.get_logger())
        
        # 打开串口
        if not self.serial_manager.open(port, baudrate, timeout):
            self.get_logger().error('初始化UWB串口失败，节点退出')
            raise RuntimeError(f'Failed to open serial port {port}')
        
        # 创建 PX4 原生话题发布器
        # QoS 配置：与 PX4 uXRCE-DDS Agent 兼容
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            # PX4 输入话题通常为 VOLATILE（避免 latched 行为导致兼容性问题）
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.vio_pub = self.create_publisher(
            VehicleOdometry, 
            'fmu/in/vehicle_visual_odometry', 
            qos_profile
        )
        self.get_logger().info("发布到 PX4 原生话题: fmu/in/vehicle_visual_odometry")

        # =====================================================================
        # 坐标系/标定参数（默认假设：LinkTrack 输出为 ENU（x=East,y=North,z=Up））
        # 最终发布给 PX4 的 VehicleOdometry.pose_frame=NED（x=North,y=East,z=Down）
        # =====================================================================
        self.declare_parameter('swap_xy', False)           # 交换 UWB x/y
        self.declare_parameter('invert_x', False)          # UWB x 取反
        self.declare_parameter('invert_y', False)          # UWB y 取反
        self.declare_parameter('invert_z', False)          # UWB z 取反（z-up 场景一般不需要）
        self.declare_parameter('yaw_offset_deg', 0.0)      # 围绕 +Z(up) 的平面旋转（度，右手系，逆时针为正）

        # NED 偏移（米）：用于把 UWB 世界原点平移到 PX4 本地原点
        self.declare_parameter('offset_n', 0.0)
        self.declare_parameter('offset_e', 0.0)
        self.declare_parameter('offset_d', 0.0)

        def _param_bool(name: str, default: bool) -> bool:
            v = self.get_parameter(name).value
            return bool(v) if v is not None else default

        def _param_float(name: str, default: float) -> float:
            v = self.get_parameter(name).value
            return float(v) if v is not None else default

        self.swap_xy = _param_bool('swap_xy', False)
        self.invert_x = _param_bool('invert_x', False)
        self.invert_y = _param_bool('invert_y', False)
        self.invert_z = _param_bool('invert_z', False)
        self.yaw_offset_rad = math.radians(_param_float('yaw_offset_deg', 0.0))
        self.offset_n = _param_float('offset_n', 0.0)
        self.offset_e = _param_float('offset_e', 0.0)
        self.offset_d = _param_float('offset_d', 0.0)
         
        # 20 Hz 定时器
        self.timer = self.create_timer(0.05, self.read_and_publish)
        
        self.buffer = bytearray()

    def read_and_publish(self):
        """
        读取串口数据并发布 PoseStamped 消息
        
        解析 NLink_LinkTrack_Tag_Frame0 协议 (128 bytes)
        """
        # 检查串口是否打开
        if not self.serial_manager.is_open:
            return

        sp = self.serial_manager.serial_port
        if sp is None:
            return
        
        try:
            # 读取所有可用数据
            if sp.in_waiting > 0:
                data = sp.read(sp.in_waiting)
                self.buffer.extend(data)
            
            # 处理缓冲区
            while len(self.buffer) >= 128:
                # 查找帧头 0x55
                header_idx = self.buffer.find(b'\x55')
                
                if header_idx == -1:
                    # 没有找到帧头，清空缓冲区
                    self.buffer.clear()
                    break
                
                # 丢弃帧头之前的数据
                if header_idx > 0:
                    del self.buffer[:header_idx]
                
                # 检查是否有足够的数据
                if len(self.buffer) < 128:
                    break
                
                # 检查功能字 (0x01)
                if self.buffer[1] != 0x01:
                    # 帧头错误，丢弃第一个字节继续查找
                    del self.buffer[0]
                    continue
                
                # 校验和检查 (Sum of bytes 0-126)
                frame = self.buffer[:128]
                checksum = sum(frame[:-1]) & 0xFF
                if checksum != frame[127]:
                    self.get_logger().warn(f"校验和错误: calc {checksum} != recv {frame[127]}")
                    del self.buffer[0]
                    continue
                
                # 解析并发布
                self.parse_and_publish_frame(frame)
                
                # 移除已处理的帧
                del self.buffer[:128]
                
        except Exception as e:
            self.get_logger().error(f'读取UWB数据出错: {e}')

    def parse_and_publish_frame(self, frame):
        """解析一帧数据并发布 VehicleOdometry 消息"""
        try:
            # 解析位置 (int24 * 1000) -> meters
            # x: bytes 4-6, y: bytes 7-9, z: bytes 10-12
            pos_x = self.parse_int24(frame[4:7]) / 1000.0
            pos_y = self.parse_int24(frame[7:10]) / 1000.0
            pos_z = self.parse_int24(frame[10:13]) / 1000.0

            # 1) 按配置做基础修正（交换/取反）
            x_u, y_u, z_u = pos_x, pos_y, pos_z
            if self.swap_xy:
                x_u, y_u = y_u, x_u
            if self.invert_x:
                x_u = -x_u
            if self.invert_y:
                y_u = -y_u
            if self.invert_z:
                z_u = -z_u

            # 2) 平面旋转（绕 +Z/up）。默认把 LinkTrack 的水平轴对齐到 ENU。
            #    旋转后仍视为 ENU：x=east, y=north, z=up
            if self.yaw_offset_rad != 0.0:
                c = math.cos(self.yaw_offset_rad)
                s = math.sin(self.yaw_offset_rad)
                x_e = c * x_u - s * y_u
                y_n = s * x_u + c * y_u
            else:
                x_e = x_u
                y_n = y_u
            z_up = z_u
            
            # 创建 VehicleOdometry 消息
            msg = VehicleOdometry()
            
            # 时间戳（微秒）
            now = self.get_clock().now()
            msg.timestamp = int(now.nanoseconds / 1000)
            msg.timestamp_sample = msg.timestamp
            
            # 坐标系设置
            # POSE_FRAME_NED = 1: NED 地球固定坐标系
            msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
            msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
            
            # 3) ENU(z-up) -> PX4 NED(z-down)
            # ENU: x=east, y=north, z=up
            # NED: x=north, y=east,  z=down
            n = y_n
            e = x_e
            d = -z_up

            # 4) NED 偏移（用于对齐原点）
            n += self.offset_n
            e += self.offset_e
            d += self.offset_d

            msg.position[0] = float(n)
            msg.position[1] = float(e)
            msg.position[2] = float(d)
            
            # 四元数设置为 NaN（表示无效/未知，让 EKF 使用 IMU 姿态）
            msg.q[0] = float('nan')
            msg.q[1] = float('nan')
            msg.q[2] = float('nan')
            msg.q[3] = float('nan')
            
            # 速度设置为 NaN（UWB 不提供速度）
            msg.velocity[0] = float('nan')
            msg.velocity[1] = float('nan')
            msg.velocity[2] = float('nan')
            
            # 角速度设置为 NaN
            msg.angular_velocity[0] = float('nan')
            msg.angular_velocity[1] = float('nan')
            msg.angular_velocity[2] = float('nan')
            
            # 位置方差（根据 UWB 精度设置，单位：m²）
            # 典型 UWB 精度约 10cm，方差 = 0.1² = 0.01
            msg.position_variance[0] = 0.01
            msg.position_variance[1] = 0.01
            msg.position_variance[2] = 0.04  # Z 轴精度稍差
            
            # 姿态方差设置为较大值（表示不可信）
            msg.orientation_variance[0] = 1.0
            msg.orientation_variance[1] = 1.0
            msg.orientation_variance[2] = 1.0
            
            # 速度方差设置为 NaN
            msg.velocity_variance[0] = float('nan')
            msg.velocity_variance[1] = float('nan')
            msg.velocity_variance[2] = float('nan')
            
            # 质量指标
            msg.quality = 0  # 默认质量
            
            self.vio_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"解析帧错误: {e}")

    def parse_int24(self, data):
        """解析3字节整数 (Little-endian)"""
        val = data[0] | (data[1] << 8) | (data[2] << 16)
        # 符号扩展
        if val & 0x800000:
            val -= 0x1000000
        return val
    
    def destroy_node(self):
        """节点销毁时关闭串口"""
        self.serial_manager.close()
        super().destroy_node()


def main(args=None):
    """
    主函数
    
    初始化ROS 2节点并开始处理消息。
    
    Args:
        args: 命令行参数
    """
    rclpy.init(args=args)
    node = UsvUwbNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()