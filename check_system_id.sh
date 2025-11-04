#!/bin/bash
# USV 系统 ID 诊断脚本
# 检查 MAVROS 配置的系统 ID 是否与飞控实际 ID 匹配

echo "=========================================="
echo "USV 系统 ID 诊断工具"
echo "=========================================="
echo ""

# 检查在线的 USV
echo "📡 检测在线的 USV..."
online_usvs=$(ros2 node list | grep -oP 'usv_\d+' | sort -u)

if [ -z "$online_usvs" ]; then
    echo "❌ 未检测到在线的 USV 节点"
    exit 1
fi

echo "✅ 在线的 USV: $online_usvs"
echo ""

# 检查每个 USV 的系统 ID
for usv in $online_usvs; do
    echo "=========================================="
    echo "🔍 检查 /$usv"
    echo "=========================================="
    
    # 检查连接状态
    echo "1️⃣ 连接状态:"
    connected=$(ros2 topic echo /$usv/state --once 2>/dev/null | grep "connected:" | awk '{print $2}')
    if [ "$connected" == "true" ]; then
        echo "   ✅ 已连接"
    else
        echo "   ❌ 未连接"
        continue
    fi
    
    # 检查 MAVROS 配置的系统 ID
    echo ""
    echo "2️⃣ MAVROS 配置:"
    
    # 自身系统 ID
    system_id=$(ros2 param get /$usv/mavros system_id 2>/dev/null | grep -oP '\d+')
    echo "   - system_id (MAVROS自身): $system_id"
    
    # 目标系统 ID
    target_system_id=$(ros2 param get /$usv/mavros target_system_id 2>/dev/null | grep -oP '\d+')
    echo "   - target_system_id (目标飞控): $target_system_id"
    
    # 目标组件 ID
    target_component_id=$(ros2 param get /$usv/mavros target_component_id 2>/dev/null | grep -oP '\d+')
    echo "   - target_component_id: $target_component_id"
    
    # 检查飞控实际的系统状态
    echo ""
    echo "3️⃣ 飞控实际状态:"
    
    # 从 state topic 获取系统状态
    system_status=$(ros2 topic echo /$usv/state --once 2>/dev/null | grep "system_status:" | awk '{print $2}')
    echo "   - 系统状态码: $system_status"
    
    # 尝试获取飞控的 SYSID 参数（通过 MAVROS）
    echo ""
    echo "4️⃣ 尝试读取飞控参数 SYSID_THISMAV:"
    # 注意：这需要 param 插件，如果白名单未包含可能失败
    fcu_sysid=$(timeout 3 ros2 service call /$usv/param/get mavros_msgs/srv/ParamGet "{param_id: 'SYSID_THISMAV'}" 2>/dev/null | grep "integer:" | awk '{print $2}')
    
    if [ -n "$fcu_sysid" ]; then
        echo "   ✅ 飞控 SYSID_THISMAV = $fcu_sysid"
    else
        echo "   ⚠️  无法读取（param 插件未加载或超时）"
    fi
    
    # 分析匹配情况
    echo ""
    echo "5️⃣ 匹配性分析:"
    
    if [ "$system_id" == "$target_system_id" ]; then
        echo "   ✅ system_id == target_system_id = $system_id"
    else
        echo "   ⚠️  system_id ($system_id) != target_system_id ($target_system_id)"
    fi
    
    if [ -n "$fcu_sysid" ]; then
        if [ "$target_system_id" == "$fcu_sysid" ]; then
            echo "   ✅ target_system_id ($target_system_id) == 飞控 SYSID ($fcu_sysid)"
        else
            echo "   ❌ target_system_id ($target_system_id) != 飞控 SYSID ($fcu_sysid)"
            echo "   🔧 建议: 修改启动参数 tgt_system=$fcu_sysid"
        fi
    fi
    
    echo ""
done

echo "=========================================="
echo "✅ 诊断完成"
echo "=========================================="
echo ""
echo "💡 提示:"
echo "   - system_id: MAVROS 自身的 MAVLink 系统 ID"
echo "   - target_system_id: MAVROS 要通信的目标飞控系统 ID"
echo "   - 正常情况下 system_id == target_system_id"
echo "   - 如果不匹配，会导致命令超时和通信失败"
echo ""
