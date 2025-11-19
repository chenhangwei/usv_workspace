# 无人船网络端口配置指南

## 📋 问题诊断报告

### ⚠️ 发现的关键问题

#### 1. MAVLink端口冲突 (严重!)
**问题**: 所有USV使用相同的地面站MAVLink端口 `14550`
**后果**: 
- 多个USV同时向同一端口发送数据会导致消息覆盖
- 地面站无法区分不同USV的MAVLink消息
- QGroundControl可能只显示最后收到消息的USV数据

#### 2. DDS端口配置错误 (需修正)
**问题**: DDS XML配置使用了错误的端口号
- 当前配置: `7411, 7412, 7413, 7499` (错误!)
- ROS 2 DDS端口计算公式: `7400 + (Domain_ID × 250) + 偏移量`

**正确的端口范围**:
- Domain 11: `10150 - 10399` (起始端口10150)
- Domain 12: `10400 - 10649` (起始端口10400)
- Domain 13: `10650 - 10899` (起始端口10650)
- Domain 99: `32150 - 32399` (起始端口32150)

---

## ✅ 修复后的完整配置

### 1. MAVLink通信端口 (飞控 ↔ 地面站)

#### 架构设计
```
┌─────────────────────────────────────────────────────────────┐
│                    地面站 (192.168.68.50)                    │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │ QGC:14550 ←─┼──│ QGC:14560 ←─┼──│ QGC:14570 ←─┼─     │
│  └──────────────┘  └──────────────┘  └──────────────┘      │
└─────────┬──────────────────┬──────────────────┬─────────────┘
          │                  │                  │
          │ MAVLink          │ MAVLink          │ MAVLink
          │                  │                  │
┌─────────▼─────┐  ┌─────────▼─────┐  ┌─────────▼─────┐
│   USV_01      │  │   USV_02      │  │   USV_03      │
│ 192.168.68.55 │  │ 192.168.68.54 │  │ 192.168.68.52 │
│   Port:14551  │  │   Port:14552  │  │   Port:14553  │
└───────────────┘  └───────────────┘  └───────────────┘
```

#### usv_launch.py 配置

**USV_01** (`namespace='usv_01'`, IP: 192.168.68.55):
```python
gcs_url_arg = DeclareLaunchArgument(
    'gcs_url',
    default_value='udp://14551@192.168.68.50:14550',  # 本机14551 → 地面站14550
    description='地面站MAVLink通信地址'
)

# MAVROS节点
mavros_node = Node(
    parameters=[{
        'fcu_url': 'udp://192.168.10.1:14550@192.168.10.2:14550',  # 飞控连接
        'gcs_url': gcs_url,
        'system_id': 101,        # MAVROS系统ID
        'target_system_id': 1,   # 飞控系统ID
    }]
)
```

**USV_02** (`namespace='usv_02'`, IP: 192.168.68.54):
```python
gcs_url_arg = DeclareLaunchArgument(
    'gcs_url',
    default_value='udp://14552@192.168.68.50:14560',  # 本机14552 → 地面站14560
    description='地面站MAVLink通信地址'
)

# MAVROS节点
mavros_node = Node(
    parameters=[{
        'fcu_url': 'udp://192.168.10.1:14550@192.168.10.2:14550',  # 飞控连接
        'gcs_url': gcs_url,
        'system_id': 102,        # MAVROS系统ID (不同于usv_01!)
        'target_system_id': 2,   # 飞控系统ID (不同于usv_01!)
    }]
)
```

**USV_03** (`namespace='usv_03'`, IP: 192.168.68.52):
```python
gcs_url_arg = DeclareLaunchArgument(
    'gcs_url',
    default_value='udp://14553@192.168.68.50:14570',  # 本机14553 → 地面站14570
    description='地面站MAVLink通信地址'
)

# MAVROS节点
mavros_node = Node(
    parameters=[{
        'fcu_url': 'udp://192.168.10.1:14550@192.168.10.2:14550',  # 飞控连接
        'gcs_url': gcs_url,
        'system_id': 103,        # MAVROS系统ID (不同于usv_01/02!)
        'target_system_id': 3,   # 飞控系统ID (不同于usv_01/02!)
    }]
)
```

#### 地面站QGroundControl配置
需要配置3个UDP连接监听不同端口:
1. **Tools → Application Settings → Comm Links → Add**
   - Name: `USV_01`
   - Type: `UDP`
   - Listening Port: `14550`
   
2. **Add 第二个连接**
   - Name: `USV_02`
   - Type: `UDP`
   - Listening Port: `14560`
   
3. **Add 第三个连接**
   - Name: `USV_03`
   - Type: `UDP`
   - Listening Port: `14570`

---

### 2. ROS 2 DDS通信端口 (ROS节点间通信)

#### 正确的DDS配置文件

**文件位置**: `~/domain_bridge/dds_profile.xml`

```xml
<?xml version="1.0" encoding="UTF-8"?>
<dds>
  <!-- 
    ROS 2 DDS 端口自动分配规则:
    基础端口 = 7400 + (Domain_ID × 250)
    每个Domain使用连续的250个端口(0-249偏移量)
    
    重要端口偏移:
    - +0:   元流量单播发现端口 (metatraffic unicast)
    - +1:   元流量多播发现端口 (metatraffic multicast)  
    - +10:  用户流量单播数据端口 (user traffic unicast)
    - +11:  用户流量多播数据端口 (user traffic multicast)
  -->

  <!-- USV 01 - Domain 11 -->
  <participant profile_name="usv_01">
    <rtps>
      <builtin>
        <metatrafficUnicastLocatorList>
          <locator>
            <udpv4>
              <address>192.168.68.55</address>
              <port>10150</port> <!-- 7400 + 11×250 = 10150 -->
            </udpv4>
          </locator>
        </metatrafficUnicastLocatorList>
      </builtin>
      <defaultUnicastLocatorList>
        <locator>
          <udpv4>
            <address>192.168.68.55</address>
            <port>10160</port> <!-- 基础端口+10 用于用户数据 -->
          </udpv4>
        </locator>
      </defaultUnicastLocatorList>
    </rtps>
  </participant>

  <!-- USV 02 - Domain 12 -->
  <participant profile_name="usv_02">
    <rtps>
      <builtin>
        <metatrafficUnicastLocatorList>
          <locator>
            <udpv4>
              <address>192.168.68.54</address>
              <port>10400</port> <!-- 7400 + 12×250 = 10400 -->
            </udpv4>
          </locator>
        </metatrafficUnicastLocatorList>
      </builtin>
      <defaultUnicastLocatorList>
        <locator>
          <udpv4>
            <address>192.168.68.54</address>
            <port>10410</port> <!-- 基础端口+10 用于用户数据 -->
          </udpv4>
        </locator>
      </defaultUnicastLocatorList>
    </rtps>
  </participant>

  <!-- USV 03 - Domain 13 -->
  <participant profile_name="usv_03">
    <rtps>
      <builtin>
        <metatrafficUnicastLocatorList>
          <locator>
            <udpv4>
              <address>192.168.68.52</address>
              <port>10650</port> <!-- 7400 + 13×250 = 10650 -->
            </udpv4>
          </locator>
        </metatrafficUnicastLocatorList>
      </builtin>
      <defaultUnicastLocatorList>
        <locator>
          <udpv4>
            <address>192.168.68.52</address>
            <port>10660</port> <!-- 基础端口+10 用于用户数据 -->
          </udpv4>
        </locator>
      </defaultUnicastLocatorList>
    </rtps>
  </participant>

  <!-- Ground Station - Domain 99 -->
  <participant profile_name="ground_station">
    <rtps>
      <builtin>
        <metatrafficUnicastLocatorList>
          <locator>
            <udpv4>
              <address>192.168.68.50</address>
              <port>32150</port> <!-- 7400 + 99×250 = 32150 -->
            </udpv4>
          </locator>
        </metatrafficUnicastLocatorList>
      </builtin>
      <defaultUnicastLocatorList>
        <locator>
          <udpv4>
            <address>192.168.68.50</address>
            <port>32160</port> <!-- 基础端口+10 用于用户数据 -->
          </udpv4>
        </locator>
      </defaultUnicastLocatorList>
    </rtps>
  </participant>
</dds>
```

#### 使用DDS配置文件

**在USV端**:
```bash
# 设置环境变量指向配置文件
export FASTRTPS_DEFAULT_PROFILES_FILE=~/domain_bridge/dds_profile.xml
export ROS_DOMAIN_ID=11  # usv_01使用Domain 11

# 启动USV
ros2 launch usv_bringup usv_launch.py namespace:=usv_01
```

**在地面站端**:
```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=~/domain_bridge/dds_profile.xml
export ROS_DOMAIN_ID=99  # 地面站使用Domain 99

# 启动地面站
ros2 launch gs_bringup gs_launch.py
```

---

## 🔧 实施步骤

### Step 1: 备份当前配置
```bash
cd ~/usv_workspace/src
cp usv_bringup/launch/usv_launch.py usv_bringup/launch/usv_launch.py.backup
```

### Step 2: 更新usv_01配置 (已完成✅)
当前文件已更新MAVLink端口为 `14551 → 14550`

### Step 3: 创建usv_02和usv_03的配置文件
```bash
# 复制usv_01的配置作为模板
cd ~/usv_workspace/src/usv_bringup/launch
cp usv_launch.py usv_02_launch.py
cp usv_launch.py usv_03_launch.py

# 然后分别修改:
# usv_02_launch.py:
#   - namespace='usv_02'
#   - gcs_url='udp://14552@192.168.68.50:14560'
#   - system_id=102, target_system_id=2
#
# usv_03_launch.py:
#   - namespace='usv_03'
#   - gcs_url='udp://14553@192.168.68.50:14570'
#   - system_id=103, target_system_id=3
```

### Step 4: 更新DDS配置文件
```bash
# 创建新的DDS配置文件
nano ~/domain_bridge/dds_profile.xml
# (粘贴上面的正确配置)
```

### Step 5: 更新启动脚本
在每个USV的启动脚本中添加:
```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=~/domain_bridge/dds_profile.xml
export ROS_DOMAIN_ID=11  # usv_01用11, usv_02用12, usv_03用13
```

### Step 6: 配置QGroundControl
按照上述"地面站QGroundControl配置"章节设置3个UDP连接。

### Step 7: 测试验证
```bash
# 在USV_01上
ros2 launch usv_bringup usv_launch.py namespace:=usv_01

# 在地面站上监控MAVLink消息
sudo tcpdump -i any -n udp port 14550 -A  # 应该看到来自192.168.68.55的消息
sudo tcpdump -i any -n udp port 14560 -A  # 应该看到来自192.168.68.54的消息
sudo tcpdump -i any -n udp port 14570 -A  # 应该看到来自192.168.68.52的消息

# 验证ROS 2通信
ros2 topic list  # 地面站应该能看到所有USV的话题(通过domain_bridge)
```

---

## 📊 端口分配总结表

| 设备 | IP地址 | MAVLink本机端口 | MAVLink地面站端口 | DDS Domain | DDS端口范围 |
|------|--------|----------------|------------------|-----------|------------|
| USV_01 | 192.168.68.55 | 14551 | 14550 | 11 | 10150-10399 |
| USV_02 | 192.168.68.54 | 14552 | 14560 | 12 | 10400-10649 |
| USV_03 | 192.168.68.52 | 14553 | 14570 | 13 | 10650-10899 |
| 地面站 | 192.168.68.50 | - | 14550/14560/14570 | 99 | 32150-32399 |

---

## ⚠️ 注意事项

### 1. 防火墙配置
确保所有端口在防火墙中开放:
```bash
# 在USV上
sudo ufw allow 14551/udp  # MAVLink
sudo ufw allow 10150:10399/udp  # DDS (usv_01)

# 在地面站上
sudo ufw allow 14550/udp  # MAVLink from usv_01
sudo ufw allow 14560/udp  # MAVLink from usv_02
sudo ufw allow 14570/udp  # MAVLink from usv_03
sudo ufw allow 32150:32399/udp  # DDS
```

### 2. 网络延迟监控
多USV场景建议监控网络质量:
```bash
# 持续ping监控
ping -i 0.2 192.168.68.55  # 监控到usv_01的延迟
```

### 3. 带宽考虑
- 每个USV的MAVLink流量约 10-50 KB/s
- 每个USV的ROS 2 DDS流量约 100-500 KB/s
- 3个USV总带宽需求: 约 1-2 Mbps

### 4. 故障排查
如果某个USV无法连接:
```bash
# 检查MAVLink连接
nc -u 192.168.68.50 14550 < /dev/null  # 测试UDP端口可达性

# 检查DDS通信
ROS_DOMAIN_ID=11 ros2 topic list  # 在地面站上列出Domain 11的话题
```

---

## 📚 参考资料

1. **MAVLink协议**: https://mavlink.io/en/
2. **MAVROS文档**: http://wiki.ros.org/mavros
3. **ROS 2 DDS端口配置**: https://docs.ros.org/en/humble/Concepts/About-Domain-ID.html
4. **Fast-DDS配置**: https://fast-dds.docs.eprosima.com/

---

**更新时间**: 2025-11-18  
**维护者**: @chenhangwei  
**版本**: v2.0 (修复MAVLink端口冲突和DDS端口错误)
