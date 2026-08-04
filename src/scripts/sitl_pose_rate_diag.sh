#!/bin/bash
# SITL 位姿遥测频率诊断 + 提速工具
#
# 背景: 2026-06-30 部署排查发现 SITL 里 local_position/pose_from_gps 实际只有
# ~0.127Hz (每 ~7.9s 一帧), 而策略/MPC 以 10Hz 闭环 -> 在过期位姿上控制 ->
# 兜圈/走反/不到点。根因 = MAVLink 流速率 × ArduPilot 实时率(RTF) 两个乘数都偏低。
#
# 本脚本:
#   diag  : 测量每艘船的关键遥测话题实际频率 + 估算 RTF (sim 时间 vs wall 时间)
#   boost : 通过 MAVROS set_message_interval 把位置/姿态流拉到目标频率
#
# 用法:
#   bash sitl_pose_rate_diag.sh diag  "usv_01 usv_02 usv_03"
#   bash sitl_pose_rate_diag.sh boost "usv_01 usv_02 usv_03" 20
#
set -o pipefail
MODE="${1:-diag}"
NAMESPACES="${2:-usv_01 usv_02 usv_03}"
RATE_HZ="${3:-20}"

# ArduPilot/MAVLink 消息 ID
MSG_GLOBAL_POSITION_INT=33
MSG_LOCAL_POSITION_NED=32
MSG_ATTITUDE=30
MSG_GPS_RAW_INT=24
MSG_SYSTEM_TIME=2

measure_hz() {
  local topic="$1"; local secs="${2:-6}"
  timeout "$((secs+3))" ros2 topic hz "$topic" --window 50 2>/dev/null \
    | grep -m1 'average rate' | awk '{print $3}'
}

diag() {
  echo "===== SITL 遥测频率诊断 (window ~6s) ====="
  for ns in $NAMESPACES; do
    echo "--- /$ns ---"
    for t in global_position/global local_position/pose local_position/pose_from_gps imu/data; do
      hz=$(measure_hz "/$ns/$t" 6)
      printf "  %-32s %s Hz\n" "$t" "${hz:-NO_DATA}"
    done
    # RTF 估算: MAVROS time_reference 携带 FCU(sim)时间, 对比 wall 时间推进比
    echo "  估算 RTF (sim秒/真实秒, ~5s 采样)..."
    python3 - "$ns" <<'PY'
import subprocess, sys, time, re
ns = sys.argv[1]
def grab():
    try:
        out = subprocess.run(['ros2','topic','echo','--once',f'/{ns}/time_reference'],
                             capture_output=True,text=True,timeout=8).stdout
        m = re.search(r'sec:\s*(\d+)\s*\n\s*nanosec:\s*(\d+)', out)
        # time_reference.time_ref is the FCU/sim time
        mm = re.findall(r'sec:\s*(\d+)\s*\n\s*nanosec:\s*(\d+)', out)
        if len(mm) >= 2:
            s,nsec = mm[1]
        else:
            s,nsec = mm[0]
        return float(s)+float(nsec)*1e-9
    except Exception as e:
        return None
t0=grab(); w0=time.monotonic()
time.sleep(5.0)
t1=grab(); w1=time.monotonic()
if t0 and t1:
    rtf=(t1-t0)/(w1-w0)
    print(f"    RTF≈{rtf:.3f}  (1.0=实时; <0.5 说明 SITL 跑不动, 这是位姿低频的主因)")
else:
    print("    time_reference 不可用, 改用 sim_vehicle 控制台的 'RTF' 行查看")
PY
  done
  echo
  echo "判读:"
  echo "  - pose_from_gps / global_position 远低于 ~5Hz => 闭环在过期位姿上控制(危险)。"
  echo "  - 若 RTF≈1 但话题频率低 => 流速率受限, 跑: bash $0 boost \"$NAMESPACES\" 20"
  echo "  - 若 RTF<<1 (如 0.13) => 机器跑不动 SITL, 见脚本末尾 RTF 处理办法。"
}

boost() {
  # 本固件无 SRn_* 参数, 只能运行时请求消息间隔。
  # 首选 MAVROS cmd/command(CommandLong, MAV_CMD_SET_MESSAGE_INTERVAL=511), 它一定存在;
  # 回退 set_message_interval 服务(部分 mavros 版本提供)。
  local interval_us
  interval_us=$(python3 -c "print(int(1000000/${RATE_HZ}))")
  echo "===== 请求 MAVLink 消息间隔 -> ${RATE_HZ}Hz (interval=${interval_us}us) ====="
  for ns in $NAMESPACES; do
    # 解析命令服务名 (mavros 在 /$ns 下直接挂, 或带 /mavros 段)
    csvc="/$ns/cmd/command"
    ros2 service type "$csvc" >/dev/null 2>&1 || csvc="/$ns/mavros/cmd/command"
    echo "--- $ns via $csvc ---"
    for mid in $MSG_GLOBAL_POSITION_INT $MSG_LOCAL_POSITION_NED $MSG_ATTITUDE $MSG_GPS_RAW_INT $MSG_SYSTEM_TIME; do
      ros2 service call "$csvc" mavros_msgs/srv/CommandLong \
        "{broadcast: false, command: 511, confirmation: 0, param1: ${mid}.0, param2: ${interval_us}.0, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0}" \
        >/dev/null 2>&1 \
        && echo "  msg $mid -> ${RATE_HZ}Hz OK" \
        || echo "  msg $mid 失败 (检查 mavros 是否就绪/服务名)"
    done
  done
  echo "提速后重新诊断: bash $0 diag \"$NAMESPACES\""
}

case "$MODE" in
  diag)  diag ;;
  boost) boost ;;
  *) echo "用法: bash $0 {diag|boost} \"usv_01 usv_02 usv_03\" [rate_hz]"; exit 1 ;;
esac
