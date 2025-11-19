#!/bin/bash
# 重新编译 navigate_to_point_server 节点

echo "🔨 重新编译 usv_comm 包..."
cd /home/chenhangwei/usv_workspace

# 清理旧的编译文件
rm -rf build/usv_comm install/usv_comm

# 编译 usv_comm 包
colcon build --packages-select usv_comm --symlink-install

echo "✅ 编译完成！"
echo ""
echo "📝 下一步操作："
echo "1. source install/setup.bash"
echo "2. 重新启动 usv_launch.py"
echo ""
echo "🔍 验证节点是否可用："
echo "   ros2 pkg executables usv_comm | grep navigate"
