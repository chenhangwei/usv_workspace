# USV 机载计算机重启功能配置指南

**日期**: 2025-11-07  
**问题**: 通过地面站集群启动器无法远程重启 USV 机载计算机  
**原因**: `sudo reboot` 命令需要密码，SSH 远程执行时无法交互输入

---

## 🔍 问题诊断

### 测试 SSH 连接
```bash
# 在地面站执行
ssh chenhangwei@192.168.68.54 'echo "连接正常"'
```

✅ **预期输出**: `连接正常`  
❌ **如果失败**: 检查网络连接和 SSH 配置

### 测试 sudo 权限
```bash
# 在地面站执行
ssh chenhangwei@192.168.68.54 'sudo -n reboot 2>&1'
```

❌ **当前输出**: `sudo: a password is required`  
✅ **预期输出**: （无输出，直接重启）

---

## 💡 解决方案（推荐）

### 方法 1：配置免密 sudo（推荐）

在**每艘 USV 机载计算机**上执行以下步骤：

#### Step 1: SSH 登录到机载计算机
```bash
# 从地面站登录到 usv_02
ssh chenhangwei@192.168.68.54
```

#### Step 2: 配置免密 sudo
```bash
# 编辑 sudoers 配置（使用 visudo 确保语法安全）
sudo visudo

# 在文件末尾添加以下行（将 chenhangwei 替换为实际用户名）
chenhangwei ALL=(ALL) NOPASSWD: /sbin/reboot
```

**重要提示**：
- 使用 `visudo` 而不是直接编辑 `/etc/sudoers`，因为它会检查语法错误
- 保存并退出：按 `Ctrl+X`，然后按 `Y`，最后按 `Enter`

#### Step 3: 验证配置
```bash
# 在 USV 机载计算机上测试（不需要密码）
sudo -n reboot

# 或从地面站远程测试
ssh chenhangwei@192.168.68.54 'sudo -n reboot'
```

✅ **预期结果**: 机载计算机立即开始重启（SSH 连接断开）

---

### 方法 2：批量配置脚本

如果有多艘 USV，可以使用脚本批量配置：

```bash
#!/bin/bash
# batch_sudo_setup.sh - 批量配置 USV 免密 sudo

# USV 列表（根据 usv_fleet.yaml 配置）
USV_LIST=(
    "chenhangwei@192.168.68.55"  # usv_01
    "chenhangwei@192.168.68.54"  # usv_02
    "chenhangwei@192.168.68.52"  # usv_03
)

# 要添加的 sudoers 规则
SUDO_RULE="chenhangwei ALL=(ALL) NOPASSWD: /sbin/reboot"

for USV in "${USV_LIST[@]}"; do
    echo "配置 $USV..."
    
    # 检查是否已有该规则
    ssh -o StrictHostKeyChecking=no "$USV" \
        "sudo grep -q 'NOPASSWD: /sbin/reboot' /etc/sudoers || \
         echo '$SUDO_RULE' | sudo EDITOR='tee -a' visudo"
    
    if [ $? -eq 0 ]; then
        echo "✅ $USV 配置成功"
    else
        echo "❌ $USV 配置失败"
    fi
done

echo "配置完成！"
```

**使用方法**：
```bash
# 保存脚本
vim batch_sudo_setup.sh

# 添加执行权限
chmod +x batch_sudo_setup.sh

# 运行脚本
./batch_sudo_setup.sh
```

---

## 🧪 验证配置

### 1. 从地面站远程重启测试
```bash
# 测试 usv_01
ssh chenhangwei@192.168.68.55 'sudo reboot'

# 测试 usv_02
ssh chenhangwei@192.168.68.54 'sudo reboot'

# 测试 usv_03
ssh chenhangwei@192.168.68.52 'sudo reboot'
```

### 2. 通过集群启动器测试
1. 启动地面站 GUI
2. 打开 `USV 集群启动器`
3. 选择 `usv_02`
4. 点击 `🔄` 按钮（单个重启）
5. 确认对话框
6. 观察日志输出

**预期日志**：
```
[OK] 已向 usv_02 (192.168.68.54) 发送 SSH 重启命令
[OK] 已向 usv_02 发送重启命令，系统将在 30-60 秒后重新上线
```

---

## 🛠️ 故障排查

### 问题 1: `sudo: a password is required`

**原因**: sudoers 配置未生效  
**解决**:
1. 检查 sudoers 文件语法：`sudo visudo -c`
2. 确认规则在文件末尾，且用户名正确
3. 重新登录 SSH 会话

### 问题 2: `sudo: /sbin/reboot: command not found`

**原因**: reboot 路径不对  
**解决**:
```bash
# 查找 reboot 命令实际路径
which reboot

# 可能的路径：
# /sbin/reboot
# /usr/sbin/reboot
# /bin/reboot

# 修改 sudoers 规则为实际路径
```

### 问题 3: SSH 连接超时

**原因**: 网络不通或防火墙阻止  
**解决**:
```bash
# 检查网络连通性
ping 192.168.68.54

# 检查 SSH 端口
nc -zv 192.168.68.54 22

# 检查防火墙（在 USV 上执行）
sudo ufw status
```

### 问题 4: 权限被拒绝

**原因**: SELinux 或 AppArmor 限制  
**解决**:
```bash
# 检查 SELinux 状态（在 USV 上执行）
getenforce

# 如果是 Enforcing，临时禁用测试
sudo setenforce 0

# 检查 AppArmor
sudo aa-status
```

---

## 📋 配置检查清单

配置完成后，逐项检查：

- [ ] SSH 免密登录已配置（`ssh-copy-id`）
- [ ] sudoers 规则已添加（`chenhangwei ALL=(ALL) NOPASSWD: /sbin/reboot`）
- [ ] sudoers 语法检查通过（`sudo visudo -c`）
- [ ] 手动 SSH 重启测试成功
- [ ] 地面站 GUI 重启功能测试成功
- [ ] 所有 USV 均已配置（usv_01, usv_02, usv_03）

---

## 🔐 安全建议

1. **最小权限原则**: 只授予 `reboot` 命令免密权限，不要用 `ALL`
   ```bash
   # ❌ 不推荐（过于宽松）
   chenhangwei ALL=(ALL) NOPASSWD: ALL
   
   # ✅ 推荐（仅 reboot）
   chenhangwei ALL=(ALL) NOPASSWD: /sbin/reboot
   ```

2. **限制来源 IP**（可选）:
   ```bash
   # 只允许从地面站 IP 执行
   chenhangwei 192.168.68.53=(ALL) NOPASSWD: /sbin/reboot
   ```

3. **审计日志**: 启用 sudo 日志记录
   ```bash
   # 在 /etc/sudoers 中添加
   Defaults logfile=/var/log/sudo.log
   ```

---

## 🎯 代码说明

当前实现（`ground_station_node.py`）使用以下逻辑：

```python
ssh_cmd = [
    'ssh',
    '-o', 'StrictHostKeyChecking=no',
    '-o', 'ConnectTimeout=5',
    f'{username}@{hostname}',
    'systemctl reboot || sudo reboot'  # 先尝试 systemctl，失败则用 sudo
]
```

**重启流程**：
1. 尝试 `systemctl reboot`（某些系统无需 sudo）
2. 如果失败，回退到 `sudo reboot`（需要本文档的配置）
3. 如果仍失败，回退到 MAVLink 命令（可能不被支持）

---

## 📚 相关文档

- **集群配置**: `gs_bringup/config/usv_fleet.yaml`
- **分布式启动**: `DISTRIBUTED_LAUNCH_GUIDE.md`
- **重启功能**: `USV_FLEET_LAUNCHER_REBOOT.md`

---

**配置完成后，请重启地面站 GUI 以应用更改。**
