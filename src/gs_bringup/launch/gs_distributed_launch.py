"""
ROS 2 分布式启动文件 - 地面站端
该文件实现从地面站通过 SSH 远程启动多艘 USV 的机载节点

特性：
1. 使用 ROS 2 原生的 ExecuteProcess 实现 SSH 远程启动
2. 从 usv_fleet.yaml 读取 USV 配置（IP、用户名、工作空间等）
3. 支持并行/串行启动多艘 USV
4. 自动管理 SSH 连接和进程生命周期
5. 可选的输出转发和错误处理

使用方法：
    ros2 launch gs_bringup gs_distributed_launch.py
    
    # 可选参数：
    ros2 launch gs_bringup gs_distributed_launch.py \
        fleet_config:=/path/to/usv_fleet.yaml \
        use_local_simulation:=true
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    LogInfo,
    EmitEvent,
    TimerAction,
)
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.events import Shutdown
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    FindExecutable,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml
import os


def load_fleet_config(config_file):
    """
    加载 USV 集群配置文件
    
    Args:
        config_file: usv_fleet.yaml 的路径
        
    Returns:
        dict: 包含 USV 配置的字典
    """
    try:
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
        return config
    except Exception as e:
        print(f"错误：无法加载集群配置文件 {config_file}: {e}")
        return None


def create_ssh_launch_command(usv_config):
    """
    创建通过 SSH 远程启动 USV 节点的命令
    
    Args:
        usv_config: 单个 USV 的配置字典
        
    Returns:
        list: SSH 命令参数列表
    """
    hostname = usv_config['hostname']
    username = usv_config['username']
    workspace = usv_config['workspace']
    namespace = usv_config['namespace']
    fcu_url = str(usv_config.get('fcu_url', '') or '')
    platform_mode = str(usv_config.get('platform_mode', '3d')).strip().lower() or '3d'

    # 从 fcu_url 推导串口参数（仅支持 serial:///dev/xxx:baudrate 形式；否则保持 usv_launch.py 默认值）
    use_ethernet_arg = None
    serial_port_arg = None
    baudrate_arg = None
    if fcu_url.startswith('serial://'):
        try:
            # serial:///dev/ttyACM0:921600
            raw = fcu_url[len('serial://'):]
            if raw.startswith('/'):
                raw = raw[1:]  # 去掉多余的一个 '/'
            # raw 现在形如 '/dev/ttyACM0:921600' 或 'dev/ttyACM0:921600'
            if ':' in raw:
                port, baud = raw.rsplit(':', 1)
                if not port.startswith('/'):
                    port = '/' + port
                serial_port_arg = port
                baudrate_arg = baud
                use_ethernet_arg = 'false'
        except Exception:
            pass
    
    # 构造远程执行的命令
    # 1. Source ROS 2 环境
    # 2. Source 工作空间
    # 3. 启动 USV launch 文件
    # 注意：使用 'source /opt/ros/*/setup.bash' 或直接指定 ROS 版本
    # 这里使用通配符自动匹配安装的 ROS 版本
    remote_cmd = (
        f"bash -c '"
        f"source /opt/ros/*/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash; "
        f"source {workspace}/install/setup.bash; "
        f"ros2 launch usv_bringup usv_launch.py "
        f"namespace:={namespace} "
        f"platform_mode:={platform_mode}"
    )

    if use_ethernet_arg is not None:
        remote_cmd += f" use_ethernet:={use_ethernet_arg}"
    if serial_port_arg is not None:
        remote_cmd += f" serial_port:={serial_port_arg}"
    if baudrate_arg is not None:
        remote_cmd += f" baudrate:={baudrate_arg}"
    
    # 关闭引号
    remote_cmd += "'"
    
    # SSH 命令格式
    # -o StrictHostKeyChecking=no: 跳过主机密钥检查
    # -o ConnectTimeout=10: 连接超时
    # -t: 强制分配伪终端（某些程序需要）
    ssh_cmd = [
        'ssh',
        '-o', 'StrictHostKeyChecking=no',
        '-o', 'ConnectTimeout=10',
        '-t',
        f'{username}@{hostname}',
        remote_cmd
    ]
    
    return ssh_cmd


def generate_launch_description():
    """
    生成分布式启动描述
    
    流程：
    1. 加载 USV 集群配置
    2. 为每个启用的 USV 创建 SSH 远程启动进程
    3. 启动地面站 GUI 节点
    4. 注册进程事件处理器（启动/退出日志）
    """
    
    # =============================================================================
    # 参数声明
    # =============================================================================
    
    # 集群配置文件路径
    fleet_config_arg = DeclareLaunchArgument(
        'fleet_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('gs_bringup'),
            'config',
            'usv_fleet.yaml'
        ]),
        description='USV 集群配置文件路径'
    )
    
    # 地面站参数文件
    gs_param_arg = DeclareLaunchArgument(
        'gs_param_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('gs_bringup'),
            'config',
            'gs_params.yaml'
        ]),
        description='地面站参数文件'
    )
    
    # 是否使用本地模拟（不通过 SSH，用于单机测试）
    use_local_sim_arg = DeclareLaunchArgument(
        'use_local_simulation',
        default_value='false',
        description='是否使用本地模拟模式（不启动远程 USV）'
    )
    
    # 是否启动地面站 GUI（从 GUI 菜单调用时设为 false）
    launch_gui_arg = DeclareLaunchArgument(
        'launch_gui',
        default_value='true',
        description='是否启动地面站 GUI 节点（从 GUI 菜单调用时应设为 false）'
    )
    
    fleet_config_file = LaunchConfiguration('fleet_config')
    gs_param_file = LaunchConfiguration('gs_param_file')
    use_local_sim = LaunchConfiguration('use_local_simulation')
    launch_gui = LaunchConfiguration('launch_gui')
    
    # =============================================================================
    # 加载集群配置
    # =============================================================================
    
    # 获取配置文件的默认路径（用于加载配置）
    # 使用 Python 查找包路径，而非 Launch substitution
    from ament_index_python.packages import get_package_share_directory
    try:
        gs_bringup_share = get_package_share_directory('gs_bringup')
        default_fleet_config = os.path.join(gs_bringup_share, 'config', 'usv_fleet.yaml')
    except Exception as e:
        print(f"警告：无法找到 gs_bringup 包: {e}")
        default_fleet_config = None
    
    # 尝试加载配置
    if default_fleet_config and os.path.exists(default_fleet_config):
        fleet_config = load_fleet_config(default_fleet_config)
    else:
        fleet_config = None
    
    if fleet_config is None:
        print("警告：未找到集群配置文件，将仅启动地面站")
        usv_list = []
        launch_options = {}
    else:
        usv_list = fleet_config.get('usv_fleet', {})
        launch_options = fleet_config.get('launch_options', {})
    
    # =============================================================================
    # 远程 USV 启动进程
    # =============================================================================
    
    usv_processes = []
    launch_delay = launch_options.get('launch_delay', 2.0)
    show_remote_output = launch_options.get('show_remote_output', True)
    
    current_delay = 0.0  # 累计延迟时间
    
    for usv_id, usv_config in usv_list.items():
        # 检查是否启用该 USV
        if not usv_config.get('enabled', False):
            print(f"跳过 {usv_id}（未启用）")
            continue
        
        # 创建 SSH 启动命令
        ssh_cmd = create_ssh_launch_command(usv_config)
        
        # 创建远程启动进程
        base_process = ExecuteProcess(
            cmd=ssh_cmd,
            name=f'{usv_id}_remote_launch',
            output='screen' if show_remote_output else 'log',
            shell=False,
            # 进程描述（用于日志）
            additional_env={'ROS_DOMAIN_ID': str(launch_options.get('ros_domain_id', 0))},
        )
        
        # 注册进程启动事件（注意：必须在 TimerAction 包装前注册）
        start_event = RegisterEventHandler(
            OnProcessStart(
                target_action=base_process,
                on_start=[
                    LogInfo(msg=f"正在启动 {usv_id} @ {usv_config['hostname']}...")
                ]
            )
        )
        
        # 注册进程退出事件
        exit_event = RegisterEventHandler(
            OnProcessExit(
                target_action=base_process,
                on_exit=[
                    LogInfo(msg=f"{usv_id} 进程已退出")
                ]
            )
        )
        
        # 添加启动延迟（在注册事件后再包装）
        if current_delay > 0:
            delayed_process = TimerAction(
                period=current_delay,
                actions=[base_process]
            )
            usv_processes.extend([start_event, exit_event, delayed_process])
        else:
            usv_processes.extend([start_event, exit_event, base_process])
        
        current_delay += launch_delay
    
    # =============================================================================
    # 启动描述
    # =============================================================================
    
    launch_items = [
        # 参数声明
        fleet_config_arg,
        gs_param_arg,
        use_local_sim_arg,
        launch_gui_arg,  # 新增参数
        
        # 启动说明
        LogInfo(msg="========================================"),
        LogInfo(msg="ROS 2 分布式启动 - USV 集群系统"),
        LogInfo(msg=f"已启用 {len(usv_processes)//3} 艘 USV"),
        LogInfo(msg="========================================"),
    ]
    
    # 条件启动地面站节点（仅当 launch_gui=true 时）
    # 注意：从 GUI 菜单调用时应设置 launch_gui:=false
    from launch.conditions import IfCondition
    
    ground_station_node_conditional = Node(
        package='gs_gui',
        executable='main_gui_app',
        name='main_gui_app',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            gs_param_file
        ],
        condition=IfCondition(launch_gui)
    )
    
    launch_items.append(ground_station_node_conditional)
    
    # 添加远程 USV 启动进程
    launch_items.extend(usv_processes)
    
    return LaunchDescription(launch_items)
