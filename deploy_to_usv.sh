#!/bin/bash
# USV 代码同步脚本
# 用法: ./deploy_to_usv.sh <USV_IP>
# 示例: ./deploy_to_usv.sh 192.168.68.52

USV_IP=$1
USV_USER="chenhangwei"
WORKSPACE_DIR="usv_workspace"

if [ -z "$USV_IP" ]; then
    echo "❌ 错误: 请指定 USV 的 IP 地址"
    echo "用法: $0 <USV_IP>"
    exit 1
fi

echo "🚀正在将代码同步到 $USV_USER@$USV_IP..."

# 使用 rsync 同步 src 目录 (排除构建产物和 git 文件)
rsync -avz --progress \
    --exclude 'build' \
    --exclude 'install' \
    --exclude 'log' \
    --exclude '.git' \
    --exclude '.vscode' \
    --exclude '__pycache__' \
    --exclude '*.pyc' \
    src/ $USV_USER@$USV_IP:~/$WORKSPACE_DIR/src/

if [ $? -ne 0 ]; then
    echo "❌ 同步失败，请检查网络连接或 SSH 配置"
    exit 1
fi

echo "✅ 代码同步完成."
echo "🔄 正在远程机器上执行 colcon build..."

# 远程执行编译
ssh $USV_USER@$USV_IP "source /opt/ros/humble/setup.bash && cd ~/$WORKSPACE_DIR && colcon build --symlink-install"

if [ $? -eq 0 ]; then
    echo "🎉 $USV_IP 部署并编译成功！请重启该 USV 的节点以生效。"
else
    echo "❌ 远程编译失败"
    exit 1
fi
