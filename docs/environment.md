# ROS 2 Jazzy 可复现环境

## 已验证环境

2026-08-29 在 x86_64 Ubuntu 24.04 上使用 `pixi.lock` 完成全新安装，并通过 `pixi install --locked` 和 `pixi run setup` 复验。

| 组件 | 实际版本 |
|---|---|
| ROS | ROS 2 Jazzy（RoboStack） |
| Python | 3.12.14 |
| GCC/G++ | 14.4.0 |
| CMake | 3.31.8 |
| PCL | 1.15.1 |
| Eigen | 5.0.1 |
| NumPy | 2.5.2 |
| rosbags | 0.11.5 |
| Pixi | 0.76.0（宿主工具，不由项目锁定） |
| direnv | 2.32.1（宿主工具，不由项目锁定） |

版本以提交的 `pixi.lock` 为准；上表用于快速排错。运行 `pixi run check` 可以查看当前机器的实际解析结果。

## 环境如何锁定

- `pixi.toml` 声明 ROS、C++ 构建链和 Python 工具；
- `pixi.lock` 固定所有直接和间接依赖；
- `.envrc` 监听两个文件，并用 `pixi shell-hook --locked` 激活；
- `scripts/check_environment.py` 检查 Jazzy、RViz、rosbag2、PCL、Eigen、编译器和 Python 包。

`.pixi/` 是每台机器的本地环境，不提交 Git。锁文件必须提交。

## ROS1 bag 与 ROS 2 的边界

官方 `data.bag` 是 ROS1 bag，且点云为 `livox_ros_driver2/CustomMsg`。本项目用 `rosbags` 直接只读解析，因此审计、截取和 RTK 导出不需要 ROS1。

原版 FAST-LIO2 主要面向 ROS1，因此项目已选择 ROS 2 移植版并固定到提交 `2fffc570a25d0df172720bac034fbdb6a13d2162`；`scripts/fetch_fast_lio.py` 获取并核对该版本。ROS1 bag 先由 `rosbags` 无损转换成 ROS2 MCAP，项目内的消息接口只复现 bag 所需的 Livox 消息定义。完整 652 秒运行已经通过帧数门禁；外参仍以 MID360 上游初值和在线细化为候选，而不是宣称有官方标定值。

RoboStack 环境与系统 `/opt/ros/*` 不应混合 source。进入本仓库后直接使用 Pixi 环境中的 `ros2`、`colcon` 和 RViz。
