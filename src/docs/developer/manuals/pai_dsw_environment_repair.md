# PAI-DSW Environment Repair

这份手册面向当前这台 PAI-DSW 实例的实际状态，不是通用假设文档。

本次实测结果：

1. 系统是 Ubuntu 24.04.1 LTS
2. 当前用户是 `root`
3. `apt-get` 和 `sudo` 可用
4. GPU 可用，Torch 能识别 `NVIDIA A10`
5. 当前缺失 `/opt/ros/jazzy/setup.bash`
6. 当前缺失 `ros2`、`colcon`、`rclpy`
7. 当前缺失 `gymnasium`、`stable_baselines3`

因此，这台实例的阻塞点不是 GPU 或 NAS，而是 ROS 2 运行时没有装。

## 1. 先看当前状态

运行预检：

```bash
cd /mnt/workspace/usv_workspace
python3 src/usv_rl/usv_rl/preflight_check.py \
  --output-json /mnt/data/evals/usv_rl/preflight.json || true
```

如果结果仍是：

1. `ros_runtime_ready = false`
2. `mappo_train_ready = false`

就说明实例还不能直接跑当前仓库的 ROS 2 训练链。

## 2. 修复方案

对于当前这台机器，推荐优先顺序是：

1. 如果你能换镜像：直接换到带 ROS 2 Jazzy 的 Ubuntu 24.04 镜像
2. 如果你不想换镜像：运行仓库里的自举脚本补环境

自举脚本路径：

[/scripts/pai_dsw_bootstrap_ros_jazzy.sh](scripts/pai_dsw_bootstrap_ros_jazzy.sh)

执行方式：

```bash
cd /mnt/workspace/usv_workspace
bash scripts/pai_dsw_bootstrap_ros_jazzy.sh
```

这个脚本会做下面几件事：

1. 配置 ROS 2 apt 源
2. 安装 `ros-jazzy-ros-base`
3. 安装 `python3-colcon-common-extensions`
4. 安装 `python3-rosdep`
5. 先把 Jazzy 需要的 rosdep/rosdistro 最小索引下载到本地，再用 `file://` 本地缓存执行 `rosdep update`
6. 通过带 `--break-system-packages` 的 `pip` 安装 `casadi`、`gymnasium` 和 `stable-baselines3`

如果脚本在 `rosdep update` 阶段仍然超时，不代表 ROS 2 安装失败。当前这台 DSW 已经实测能把 ROS 2 Jazzy 主体装上，失败点主要是 rosdep 索引拉取。现在脚本默认会先从清华 rosdistro 镜像下载 Jazzy 需要的最小索引到本地，再让 rosdep 读取本地 `file://` 缓存；如果你后续想切别的镜像，也可以在执行前指定环境变量。例如：

```bash
export ROSDEP_SOURCE_BASE=https://mirrors.tuna.tsinghua.edu.cn/rosdistro
export LOCAL_ROSDEP_CACHE_DIR=/tmp/pai_dsw_rosdep
```

此时可以先继续后续步骤，再在网络稳定时单独执行：

```bash
source /opt/ros/jazzy/setup.bash
bash scripts/pai_dsw_bootstrap_ros_jazzy.sh
rosdep install --from-paths src --ignore-src -r -y
```

## 3. 自举完成后的标准流程

```bash
source /opt/ros/jazzy/setup.bash
cd /mnt/workspace/usv_workspace
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to usv_rl usv_sim --symlink-install
source install/setup.bash
python3 src/usv_rl/usv_rl/preflight_check.py \
  --output-json /mnt/data/evals/usv_rl/preflight_after_bootstrap.json || true
```

期望看到：

1. `ros_runtime_ready = true`
2. `mappo_train_ready = true`

`ppo_optional_ready` 为 `true` 则表示 PPO 相关依赖也已齐全。

## 4. 修复成功后的第一条验证命令

如果预检通过，先不要直接长训练，先做一个最小冒烟：

```bash
cd /mnt/workspace/usv_workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run usv_rl evaluate_mappo_residual \
  --policy zero \
  --num-agents 5 \
  --episodes 1 \
  --steps-per-episode 20 \
  --scenario five_usv_dense_head_on \
  --output-json /mnt/data/evals/usv_rl/mappo_zero_smoke.json
```

这条通过后，再进入正式训练。

## 5. 不建议的做法

1. 不要先装 `gymnasium` 和 `stable-baselines3` 就开始训练，缺 ROS 2 时依然跑不起来
2. 不要把 ROS 2、colcon、rclpy 的缺失当成 Python 虚拟环境问题，它是系统级依赖缺失
3. 不要把模型、日志、checkpoint 写回源码目录，继续使用 `/mnt/data`

## 6. 当前结论

这台 DSW 实例具备继续使用的价值，因为它已经有：

1. Ubuntu 24.04
2. root 权限
3. apt 可用
4. A10 GPU 可用
5. NAS 目录可写

所以当前最务实的路线不是重建仓库，而是先把 ROS 2 Jazzy 和相关工具补齐。