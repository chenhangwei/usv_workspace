#!/usr/bin/env python3
"""
独立测试 USV 状态检测逻辑
模拟 usv_fleet_launcher_optimized.py 中的状态检测过程
"""
import subprocess
import yaml
import os
from concurrent.futures import ThreadPoolExecutor, as_completed

def check_host_online(hostname):
    """检查主机是否在线"""
    if not hostname:
        return False
    try:
        result = subprocess.run(
            ['ping', '-c', '1', '-W', '1', '-q', hostname],
            capture_output=True,
            timeout=2
        )
        return result.returncode == 0
    except Exception:
        return False

def main():
    print("=" * 60)
    print("USV 状态检测逻辑测试")
    print("=" * 60)
    
    # 加载配置
    workspace = os.path.expanduser('~/usv_workspace')
    config_file = os.path.join(
        workspace,
        'install/gs_bringup/share/gs_bringup/config/usv_fleet.yaml'
    )
    
    try:
        with open(config_file, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
            fleet_config = config.get('usv_fleet', {})
    except Exception as e:
        print(f"❌ 加载配置失败: {e}")
        return
    
    print(f"\n✅ 加载配置: {len(fleet_config)} 艘 USV")
    
    # 步骤 1: 获取 ROS 节点
    print("\n📋 步骤 1: 检测 ROS 节点")
    try:
        result = subprocess.run(
            ['ros2', 'node', 'list'],
            capture_output=True,
            text=True,
            timeout=2
        )
        online_nodes = result.stdout.strip().split('\n') if result.returncode == 0 else []
        print(f"  检测到 {len(online_nodes)} 个节点")
        for node in online_nodes:
            print(f"    - {node}")
    except Exception as e:
        print(f"  ❌ ROS 节点检测失败: {e}")
        online_nodes = []
    
    # 步骤 2: 并行 ping 检测
    print("\n📋 步骤 2: 并行 Ping 检测")
    host_status = {}
    
    with ThreadPoolExecutor(max_workers=10) as executor:
        futures = {}
        for usv_id, cfg in fleet_config.items():
            if not cfg.get('enabled', False):
                continue
            hostname = cfg.get('hostname', '')
            if hostname and hostname not in host_status:
                future = executor.submit(check_host_online, hostname)
                futures[future] = (usv_id, hostname)
        
        for future in as_completed(futures):
            usv_id, hostname = futures[future]
            try:
                is_online = future.result()
                host_status[hostname] = is_online
                status_str = "✅ 在线" if is_online else "❌ 离线"
                print(f"  [{usv_id}] {hostname}: {status_str}")
            except Exception as e:
                print(f"  [{usv_id}] {hostname}: ⚠️ ping 失败 ({e})")
                host_status[hostname] = False
    
    # 步骤 3: 判断状态
    print("\n📋 步骤 3: 状态判断")
    status_updates = {}
    
    for usv_id, cfg in fleet_config.items():
        if not cfg.get('enabled', False):
            continue
        
        namespace = f"/{usv_id}"
        hostname = cfg.get('hostname', '')
        
        # 检查节点
        has_nodes = any(namespace in node for node in online_nodes)
        
        # 检查主机
        is_host_online = host_status.get(hostname, False)
        
        # 状态判断
        if has_nodes:
            new_status = 'running'
        elif is_host_online:
            new_status = 'online'
        else:
            new_status = 'offline'
        
        status_updates[usv_id] = new_status
        
        # 显示详细信息
        print(f"  [{usv_id}]")
        print(f"    主机: {hostname} ({'在线' if is_host_online else '离线'})")
        print(f"    节点: {'有' if has_nodes else '无'}")
        print(f"    状态: {new_status}")
    
    # 总结
    print("\n" + "=" * 60)
    print("📊 状态总结")
    print("=" * 60)
    status_icons = {
        'offline': '⚫ 离线',
        'online': '🟡 在线',
        'running': '🟢 运行中'
    }
    for usv_id, status in status_updates.items():
        print(f"  {usv_id}: {status_icons.get(status, status)}")
    
    print("\n✅ 测试完成")

if __name__ == '__main__':
    main()
