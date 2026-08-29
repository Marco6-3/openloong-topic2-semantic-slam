# 第三方依赖与许可记录

本项目不提交第三方源码或模型权重；构建脚本按固定提交获取依赖。

| 依赖 | 来源与固定版本 | 用途 | 许可说明 |
|---|---|---|---|
| FAST_LIO_ROS2 | `Ericsii/FAST_LIO_ROS2@2fffc570a25d0df172720bac034fbdb6a13d2162` | LiDAR–IMU 里程计和配准点云 | 仓库根 `LICENSE` 为 GPL-2.0，`package.xml` 标为 BSD，存在上游元数据不一致；本项目按更严格的 GPL-2.0 记录，且初赛提交包不含其源码 |
| ikd-Tree | 上述仓库子模块 `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4` | FAST-LIO 增量 KD-tree | 随上游固定子模块获取，不单独分发 |
| ROS 2 Jazzy / PCL / Eigen / NumPy / rosbags | 具体构建见 `pixi.lock` | 运行环境、点云和 bag 处理 | 各自上游许可；锁文件保存精确来源和版本 |

仓库内的 `livox_ros_driver2` 只定义与官方 bag 匹配的离线消息接口，不包含 Livox SDK、设备驱动或厂商二进制文件。当前没有下载或使用任何语义模型权重。
