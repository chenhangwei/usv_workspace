"""
单载具独立仿真 + RL 启动文件 (usv_sitl_agent_launch.py)

参考 `usv_rl` 分支单艇独立启动习惯，每个终端仅独立启动**单个载具**：
只需指定命名空间 `namespace:=usv_XX`、载具类型 `vehicle_type:=...`、可选 RL 模型 `rl_model:=...`，
系统即自动完成：
1. 自动从 `usv_XX` 推导唯一 PX4 实例编号 (`instance_id`) 与无碰撞默认出生坐标（也可通过 `spawn_x/y/z` 覆盖）
2. 自动检测 `MicroXRCEAgent` 与 `Gazebo (gz sim)` 是否已在运行：
   - 首台启动时自动拉起 `MicroXRCEAgent` 和 `Gazebo` 世界；
   - 后续任意终端启动 `usv_02`、`usv_03`... 时自动附着同一 `Gazebo` 世界并复用共享 `MicroXRCEAgent`，无需手动传 `start_agent:=false` 或 `standalone_world:=true`
3. PX4 自动加载对应机架 (`4025` 差速船 / `4024` 无人球 / `4022` 仿生飞艇 / `4023` 仿生海豚) 与 Gazebo 物理模型
4. 自动加载匹配的 RL 策略模型（差速船默认加载 `usv_rl` 分支同款 `usv_rl_policy.pt`，也支持 `rl_model:=xxx.pt` 自定义指定）

典型用法（每个终端独立跑一条）：
  # 终端 1：启动差速无人船 usv_01（自动匹配 PX4 4025 usv_boat + usv_rl_policy.pt）
  ros2 launch usv_sim sitl_rl_launch.py namespace:=usv_01 vehicle_type:=usv_diff

  # 终端 2：启动第 2 艘差速船或异构无人球 usv_02（自动附着同一世界、错开出生点并加载对应机架与 RL 模型）
  ros2 launch usv_sim sitl_rl_launch.py namespace:=usv_02 vehicle_type:=flying_sphere rl_model:=usv_rl_policy.pt

  # 终端 3：启动仿生飞艇 usv_03
  ros2 launch usv_sim sitl_rl_launch.py namespace:=usv_03 vehicle_type:=bionic_airship
"""

import os
import re
import subprocess
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


# 机架类型别名归一化：方便中英文/简写输入
VEHICLE_TYPE_ALIASES = {
    'usv_diff': 'usv_diff',
    'usv': 'usv_diff',
    'boat': 'usv_diff',
    'diff_boat': 'usv_diff',
    'rover_differential': 'usv_diff',
    'flying_sphere': 'flying_sphere',
    'sphere': 'flying_sphere',
    'uav': 'flying_sphere',
    'bionic_airship': 'bionic_airship',
    'airship': 'bionic_airship',
    'bionic_dolphin': 'bionic_dolphin',
    'dolphin': 'bionic_dolphin',
}

# 机架档案表：统一定义各机架的 PX4 Autostart、Gazebo 模型、维度、默认到点阈值、默认高度与对应 RL 模型
VEHICLE_PROFILES = {
    'usv_diff': {
        'autostart': '4025',
        'sim_model': 'usv_boat',
        'platform_mode': '2d',
        'use_6dof': 'false',
        'reach_xy': '0.50',
        'reach_z': '0.80',
        'default_z': 0.0,
        'rl_model': 'usv_rl_policy.pt',
        'rl_cruise_speed': '0.60',
    },
    'bionic_airship': {
        'autostart': '4022',
        'sim_model': 'bionic_airship',
        'platform_mode': '3d',
        'use_6dof': 'true',
        'reach_xy': '4.00',
        'reach_z': '1.50',
        'default_z': 0.5,
        'rl_model': 'bionic_airship_rl_policy.pt',
        'rl_cruise_speed': '1.20',
    },
    'bionic_dolphin': {
        'autostart': '4023',
        'sim_model': 'bionic_dolphin',
        'platform_mode': '3d',
        'use_6dof': 'true',
        'reach_xy': '1.50',
        'reach_z': '0.80',
        'default_z': 0.2,
        'rl_model': 'bionic_dolphin_rl_policy.pt',
        'rl_cruise_speed': '1.00',
    },
    'flying_sphere': {
        'autostart': '4024',
        'sim_model': 'flying_sphere',
        'platform_mode': '3d',
        'use_6dof': 'true',
        'reach_xy': '0.80',
        'reach_z': '0.50',
        'default_z': 0.2,
        'rl_model': 'flying_sphere_rl_policy.pt',
        'rl_cruise_speed': '0.80',
    },
}


def _resolve_namespace_and_instance(namespace_raw: str, instance_raw: str) -> tuple[str, int]:
    """兼容 usv_rl 启动习惯：支持传 namespace:=usv_03 或 instance:=2 双向自动推导。"""
    ns = namespace_raw.strip()
    inst = instance_raw.strip()
    if ns and ns != '__auto__':
        m = re.search(r'(\d+)$', ns)
        if m:
            num = max(1, int(m.group(1)))
            if inst and inst != '__auto__':
                return ns, max(1, int(inst))
            return ns, num
        if inst and inst != '__auto__':
            val = int(inst)
            return ns, (val + 1 if val == 0 else max(1, val))
        return ns, 1

    # 未传 namespace 时，按 usv_rl 风格从 instance 推导 (0 -> usv_01, 2 -> usv_03)
    if inst and inst != '__auto__':
        val = int(inst)
        sysid = val + 1 if val >= 0 else 1
        return f'usv_{sysid:02d}', sysid
    return 'usv_01', 1


def _default_spawn_pose(inst_id: int, default_z: float) -> tuple[str, str, str]:
    """当未显式指定 spawn_x/y/z 时，按实例编号自动生成不重叠的网格出生点（ENU，间距 3.0m）。"""
    idx = max(0, inst_id - 1)
    col = idx % 5
    row = idx // 5
    # col: 0 -> 0.0, 1 -> 3.0, 2 -> -3.0, 3 -> 6.0, 4 -> -6.0
    if col == 0:
        y = 0.0
    elif col % 2 == 1:
        y = 3.0 * ((col + 1) // 2)
    else:
        y = -3.0 * (col // 2)
    x = 0.0 if row == 0 else -3.5 * row
    return f'{x:.2f}', f'{y:.2f}', f'{default_z:.2f}'


def _is_process_running(pattern: str) -> bool:
    """检查当前主机是否已有匹配的仿真支撑进程在运行。"""
    try:
        res = subprocess.run(
            ['pgrep', '-f', pattern],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        return res.returncode == 0
    except Exception:
        return False


def _resolve_model_path(rl_model_arg: str, default_model_filename: str, share_dir: str) -> str:
    """解析 RL 策略模型路径：
    1) 若用户显式传了绝对/相对路径、模型名或 models/ 下的文件名，优先解析；
    2) 若为 __auto__，先查找该机架专属模型 (<vehicle_type>_rl_policy.pt)，不存在则回退至通用 usv_rl_policy.pt。
    """
    models_dir = os.path.join(share_dir, 'models')
    candidate = rl_model_arg.strip()
    if candidate and candidate != '__auto__':
        expanded = os.path.expanduser(candidate)
        if os.path.isfile(expanded):
            return os.path.abspath(expanded)
        in_share = os.path.join(models_dir, candidate)
        if os.path.isfile(in_share):
            return in_share
        if not candidate.endswith('.pt'):
            in_share_pt = os.path.join(models_dir, f'{candidate}.pt')
            if os.path.isfile(in_share_pt):
                return in_share_pt
        return os.path.abspath(expanded)

    specific = os.path.join(models_dir, default_model_filename)
    if os.path.isfile(specific):
        return specific
    return os.path.join(models_dir, 'usv_rl_policy.pt')


def _setup_agent(context, *args, **kwargs):
    ns_raw = LaunchConfiguration('namespace').perform(context)
    inst_id_raw = LaunchConfiguration('instance_id').perform(context).strip()
    inst_alias_raw = LaunchConfiguration('instance').perform(context).strip()
    explicit_inst = inst_alias_raw if (inst_alias_raw and inst_alias_raw != '__auto__') else inst_id_raw
    namespace, inst_id = _resolve_namespace_and_instance(ns_raw, explicit_inst)

    vtype_raw = LaunchConfiguration('vehicle_type').perform(context).strip().lower()
    vehicle_type = VEHICLE_TYPE_ALIASES.get(vtype_raw, vtype_raw)
    if vehicle_type not in VEHICLE_PROFILES:
        raise RuntimeError(
            f"不支持的 vehicle_type='{vtype_raw}'。可选类型: {', '.join(sorted(VEHICLE_PROFILES.keys()))}"
        )
    profile = VEHICLE_PROFILES[vehicle_type]

    nav_mode = LaunchConfiguration('nav_mode').perform(context).strip().lower()
    rl_alias_arg = LaunchConfiguration('rl_model').perform(context).strip()
    rl_policy_arg = LaunchConfiguration('rl_policy_model').perform(context).strip()
    chosen_rl_arg = rl_alias_arg if (rl_alias_arg and rl_alias_arg != '__auto__') else rl_policy_arg

    rl_device = LaunchConfiguration('rl_policy_device').perform(context).strip()
    rl_rate = LaunchConfiguration('rl_publish_rate').perform(context).strip()
    rl_cruise_arg = LaunchConfiguration('rl_cruise_speed').perform(context).strip()
    rl_encounter = LaunchConfiguration('rl_encounter_type').perform(context).strip()

    px4_dir = LaunchConfiguration('px4_dir').perform(context).strip()
    xrce_agent_bin = LaunchConfiguration('xrce_agent_bin').perform(context).strip()
    agent_port = LaunchConfiguration('agent_port').perform(context).strip()

    start_agent_raw = LaunchConfiguration('start_agent').perform(context).strip().lower()
    if start_agent_raw == '__auto__':
        start_agent = not _is_process_running(f'MicroXRCEAgent.*-p *{agent_port}')
    else:
        start_agent = start_agent_raw in ('1', 'true', 'yes', 'on')

    headless_raw = LaunchConfiguration('headless').perform(context).strip().lower()
    gui_raw = LaunchConfiguration('gui').perform(context).strip().lower()
    if headless_raw != '__auto__':
        headless = headless_raw in ('1', 'true', 'yes', 'on')
    elif gui_raw != '__auto__':
        headless = gui_raw not in ('1', 'true', 'yes', 'on')
    else:
        headless = False

    standalone_raw = LaunchConfiguration('standalone_world').perform(context).strip().lower()
    if standalone_raw == '__auto__':
        gz_running = _is_process_running(r'gz sim.*-s')
        px4_running = _is_process_running(r'build/px4_sitl_default/bin/px4')
        if gz_running and not px4_running:
            # 清理上一轮异常退出遗留的孤立 Gazebo 世界（含旧实体模型如 flying_sphere_1 等）
            subprocess.run(['pkill', '-f', r'gz sim'], check=False)
            import time as _time
            _time.sleep(1.0)
            standalone_world = False
        else:
            standalone_world = gz_running
    else:
        standalone_world = standalone_raw in ('1', 'true', 'yes', 'on')

    gz_world = LaunchConfiguration('gz_world').perform(context).strip()

    auto_x, auto_y, auto_z = _default_spawn_pose(inst_id, float(profile['default_z']))
    spawn_x_raw = LaunchConfiguration('spawn_x').perform(context).strip()
    spawn_y_raw = LaunchConfiguration('spawn_y').perform(context).strip()
    spawn_z_raw = LaunchConfiguration('spawn_z').perform(context).strip()
    x_alias_raw = LaunchConfiguration('x').perform(context).strip()
    y_alias_raw = LaunchConfiguration('y').perform(context).strip()
    z_alias_raw = LaunchConfiguration('z').perform(context).strip()
    yaw_alias_raw = LaunchConfiguration('yaw').perform(context).strip()
    eff_x = x_alias_raw if (x_alias_raw and x_alias_raw != '__auto__') else spawn_x_raw
    eff_y = y_alias_raw if (y_alias_raw and y_alias_raw != '__auto__') else spawn_y_raw
    eff_z = z_alias_raw if (z_alias_raw and z_alias_raw != '__auto__') else spawn_z_raw
    spawn_x = auto_x if (not eff_x or eff_x == '__auto__') else eff_x
    spawn_y = auto_y if (not eff_y or eff_y == '__auto__') else eff_y
    spawn_z = auto_z if (not eff_z or eff_z == '__auto__') else eff_z
    spawn_yaw_cfg = LaunchConfiguration('spawn_yaw').perform(context).strip()
    spawn_yaw = yaw_alias_raw if (yaw_alias_raw and yaw_alias_raw != '__auto__') else spawn_yaw_cfg

    platform_mode = LaunchConfiguration('platform_mode').perform(context).strip()
    if not platform_mode or platform_mode == '__auto__':
        platform_mode = profile['platform_mode']

    use_6dof = LaunchConfiguration('use_setpoint_6dof').perform(context).strip()
    if not use_6dof or use_6dof == '__auto__':
        use_6dof = profile['use_6dof']

    reach_xy = LaunchConfiguration('reach_xy_threshold').perform(context).strip()
    if not reach_xy or reach_xy == '__auto__':
        reach_xy = profile['reach_xy']

    reach_z = LaunchConfiguration('reach_z_threshold').perform(context).strip()
    if not reach_z or reach_z == '__auto__':
        reach_z = profile['reach_z']

    rl_cruise = rl_cruise_arg if (rl_cruise_arg and rl_cruise_arg != '__auto__') else profile['rl_cruise_speed']

    usv_control_share = FindPackageShare('usv_control').perform(context)
    resolved_rl_model = _resolve_model_path(chosen_rl_arg, profile['rl_model'], usv_control_share)
    rl_enabled_str = 'true' if nav_mode == 'rl' else 'false'

    px4_bin = os.path.join(px4_dir, 'build', 'px4_sitl_default', 'bin', 'px4')
    gz_env = os.path.join(px4_dir, 'build', 'px4_sitl_default', 'rootfs', 'gz_env.sh')
    romfs_dir = os.path.join(px4_dir, 'build', 'px4_sitl_default', 'etc')
    inst_dir = f'/tmp/hetero_swarm_sitl/px4_{namespace}'
    domain_id = LaunchConfiguration('domain_id').perform(context).strip()
    pose_str = f'{spawn_x},{spawn_y},{spawn_z},0,0,{spawn_yaw}'

    env_parts = [
        f'PX4_SYS_AUTOSTART={profile["autostart"]}',
        f'PX4_SIM_MODEL={profile["sim_model"]}',
        f'PX4_GZ_MODEL_POSE="{pose_str}"',
        f'PX4_GZ_WORLD={gz_world}',
        f'PX4_UXRCE_DDS_NS={namespace}',
        f'UXRCE_DDS_DOM_ID={domain_id}',
        f'UXRCE_DDS_PRT={agent_port}',
    ]
    if headless:
        env_parts.append('HEADLESS=1')
    if standalone_world:
        env_parts.append('PX4_GZ_STANDALONE=1')

    # PX4 -i N 会将 MAV_SYS_ID 设为 N+1，因此 usv_01(inst_id=1) 对应 px4 -i 0 (MAV_SYS_ID=1)
    px4_instance = max(0, int(inst_id) - 1)
    px4_cmd = (
        f'if [ -f "{gz_env}" ]; then source "{gz_env}"; fi; '
        f'rm -rf "{inst_dir}" && mkdir -p "{inst_dir}" && cd "{inst_dir}" && '
        f'{" ".join(env_parts)} '
        f'"{px4_bin}" -i {px4_instance} -d "{romfs_dir}" < <(tail -f /dev/null)'
    )

    actions = [
        LogInfo(
            msg=(
                f"\n{'=' * 68}\n"
                f"🚀 单载具独立仿真启动: {namespace} (PX4 instance={px4_instance}, MAV_SYS_ID={px4_instance + 1})\n"
                f"   载具类型 : {vehicle_type} -> PX4 Autostart={profile['autostart']} / 模型={profile['sim_model']}\n"
                f"   控制维度 : platform_mode={platform_mode}, nav_mode={nav_mode}\n"
                f"   世界附着 : {'复用已有 Gazebo 世界' if standalone_world else '新建 Gazebo 世界'} | "
                f"MicroXRCEAgent: {'本进程拉起' if start_agent else f'复用已有端口 {agent_port}'}\n"
                f"   出生坐标 : ENU ({spawn_x}, {spawn_y}, {spawn_z}) yaw={spawn_yaw}\n"
                f"   RL 模型  : {resolved_rl_model if nav_mode == 'rl' else '未启用 (position 模式)'}\n"
                f"{'=' * 68}"
            )
        )
    ]

    if start_agent:
        actions.append(
            ExecuteProcess(
                cmd=[xrce_agent_bin, 'udp4', '-p', agent_port],
                output='screen',
                name=f'micro_xrce_agent_{namespace}',
            )
        )

    actions.append(
        ExecuteProcess(
            cmd=['bash', '-c', px4_cmd],
            output='screen',
            name=f'px4_{namespace}',
        )
    )

    # 通过独立子进程启动该载具的 ROS 2 控制 + 通信 + RL 推理节点
    usv_stack = ExecuteProcess(
        cmd=[
            'ros2', 'launch', 'usv_bringup', 'usv_launch.py',
            f'namespace:={namespace}',
            'group_id:=GS',
            'use_zenoh:=false',
            'use_ethernet:=false',
            'use_simulation:=true',
            'start_agent:=false',
            'use_hardware_drivers:=false',
            'use_uwb:=false',
            'coordinate_system:=ENU',
            f'spawn_offset_x:={spawn_x}',
            f'spawn_offset_y:={spawn_y}',
            f'spawn_offset_z:={spawn_z}',
            f'platform_mode:={platform_mode}',
            f'use_setpoint_6dof:={use_6dof}',
            f'vehicle_type:={vehicle_type}',
            f'reach_xy_threshold:={reach_xy}',
            f'reach_z_threshold:={reach_z}',
            f'nav_mode:={nav_mode}',
            f'rl_policy_enabled:={rl_enabled_str}',
            f'rl_policy_model:={resolved_rl_model}',
            f'rl_policy_device:={rl_device}',
            f'rl_publish_rate:={rl_rate}',
            f'rl_cruise_speed:={rl_cruise}',
            f'rl_encounter_type:={rl_encounter}',
        ],
        output='screen',
        name=f'usv_stack_{namespace}',
    )

    actions.append(TimerAction(period=2.5, actions=[usv_stack]))
    return actions


def generate_launch_description():
    domain_id_arg = DeclareLaunchArgument('domain_id', default_value='99', description='ROS_DOMAIN_ID')
    namespace_arg = DeclareLaunchArgument('namespace', default_value='__auto__', description='载具命名空间 (如 usv_01, usv_02; 未传则按 instance 自动推导)')
    instance_id_arg = DeclareLaunchArgument('instance_id', default_value='__auto__', description='PX4 实例号 (__auto__ 表示从 usv_N 自动提取)')
    instance_alias_arg = DeclareLaunchArgument('instance', default_value='__auto__', description='兼容 usv_rl 风格 instance 参数 (如 0->usv_01, 2->usv_03)')
    vehicle_type_arg = DeclareLaunchArgument(
        'vehicle_type',
        default_value='usv_diff',
        description='载具类型: usv_diff (差速无人船) | flying_sphere (无人球) | bionic_airship (仿生飞艇) | bionic_dolphin (仿生海豚)',
    )
    nav_mode_arg = DeclareLaunchArgument(
        'nav_mode',
        default_value='rl',
        description="导航模式: 'rl'（端到端 RL 控制）或 'position'（位置航点控制）",
    )
    platform_mode_arg = DeclareLaunchArgument('platform_mode', default_value='__auto__', description='2d / 3d (__auto__ 按机架自动选择)')
    use_6dof_arg = DeclareLaunchArgument('use_setpoint_6dof', default_value='__auto__', description='true / false (__auto__ 按机架自动选择)')
    reach_xy_arg = DeclareLaunchArgument('reach_xy_threshold', default_value='__auto__', description='水平到点阈值 (m)')
    reach_z_arg = DeclareLaunchArgument('reach_z_threshold', default_value='__auto__', description='垂向到点阈值 (m)')

    rl_model_alias_arg = DeclareLaunchArgument(
        'rl_model',
        default_value='__auto__',
        description='RL 模型路径或文件名 (__auto__ 按载具类型自动匹配，例如差速船自动加载 usv_rl_policy.pt)',
    )
    rl_model_arg = DeclareLaunchArgument('rl_policy_model', default_value='__auto__', description='RL 策略模型路径 (与 rl_model 等效)')
    rl_device_arg = DeclareLaunchArgument('rl_policy_device', default_value='cpu', description='RL 推理设备 (cpu / cuda)')
    rl_rate_arg = DeclareLaunchArgument('rl_publish_rate', default_value='10.0', description='RL 推理频率 (Hz)')
    rl_cruise_arg = DeclareLaunchArgument('rl_cruise_speed', default_value='__auto__', description='RL 巡航速度 (m/s)')
    rl_encounter_arg = DeclareLaunchArgument('rl_encounter_type', default_value='auto', description='RL 会遇模式')

    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='__auto__', description='出生点 ENU X (m) (__auto__ 按实例号自动错位防碰撞)')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='__auto__', description='出生点 ENU Y (m) (__auto__ 按实例号自动错位防碰撞)')
    spawn_z_arg = DeclareLaunchArgument('spawn_z', default_value='__auto__', description='出生点 ENU Z (m) (__auto__ 按机架默认高度)')
    spawn_yaw_arg = DeclareLaunchArgument('spawn_yaw', default_value='0.0', description='出生点偏航角 (rad)')
    x_alias_arg = DeclareLaunchArgument('x', default_value='__auto__', description='spawn_x 简写别名')
    y_alias_arg = DeclareLaunchArgument('y', default_value='__auto__', description='spawn_y 简写别名')
    z_alias_arg = DeclareLaunchArgument('z', default_value='__auto__', description='spawn_z 简写别名')
    yaw_alias_arg = DeclareLaunchArgument('yaw', default_value='__auto__', description='spawn_yaw 简写别名')

    px4_dir_arg = DeclareLaunchArgument(
        'px4_dir',
        default_value=os.path.expanduser('~/PX4-Autopilot'),
        description='PX4-Autopilot 源码目录',
    )
    xrce_agent_arg = DeclareLaunchArgument(
        'xrce_agent_bin',
        default_value=os.path.expanduser('~/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent'),
        description='MicroXRCEAgent 可执行文件路径',
    )
    agent_port_arg = DeclareLaunchArgument('agent_port', default_value='8888', description='MicroXRCEAgent UDP 端口')
    start_agent_arg = DeclareLaunchArgument(
        'start_agent',
        default_value='__auto__',
        description='是否启动 MicroXRCEAgent (__auto__ 检测端口未占用时自动启动，已占用则自动复用)',
    )
    headless_arg = DeclareLaunchArgument('headless', default_value='__auto__', description='Gazebo 无头模式 (true/false)')
    gui_arg = DeclareLaunchArgument('gui', default_value='__auto__', description='兼容 usv_rl 风格 gui:=true/false 参数')
    standalone_world_arg = DeclareLaunchArgument(
        'standalone_world',
        default_value='__auto__',
        description='Gazebo 附着模式 (__auto__ 检测已有 gz sim 时自动附着，首台自动建世界)',
    )
    gz_world_arg = DeclareLaunchArgument('gz_world', default_value='default', description='Gazebo 世界名称')

    set_domain_id = SetEnvironmentVariable(name='ROS_DOMAIN_ID', value=LaunchConfiguration('domain_id'))

    return LaunchDescription([
        domain_id_arg,
        namespace_arg,
        instance_id_arg,
        instance_alias_arg,
        vehicle_type_arg,
        nav_mode_arg,
        platform_mode_arg,
        use_6dof_arg,
        reach_xy_arg,
        reach_z_arg,
        rl_model_alias_arg,
        rl_model_arg,
        rl_device_arg,
        rl_rate_arg,
        rl_cruise_arg,
        rl_encounter_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        spawn_yaw_arg,
        x_alias_arg,
        y_alias_arg,
        z_alias_arg,
        yaw_alias_arg,
        px4_dir_arg,
        xrce_agent_arg,
        agent_port_arg,
        start_agent_arg,
        headless_arg,
        gui_arg,
        standalone_world_arg,
        gz_world_arg,
        set_domain_id,
        OpaqueFunction(function=_setup_agent),
    ])
