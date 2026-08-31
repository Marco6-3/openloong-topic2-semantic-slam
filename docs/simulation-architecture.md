# 复赛实时语义 SLAM 架构

## 目标和边界

复赛链路以官方 `house.world`、相机、VLP-16 和车辆模型为输入，同时运行定位、建图和语义分析。官方 `/state_estimation` 是根据控制指令积分出的仿真里程计，只作为 `odom` 层先验和离线误差对照，不作为最终 SLAM 位姿；官方 `/registered_scan` 同样不作为 SLAM 输入。

ROS Noetic 与 Gazebo Classic 放在 `simulation/` 的独立 Pixi 项目和 `data/simulation/catkin_ws` 工作区中。它不写 `/opt/ros`、`~/.gazebo/models`、`~/.bashrc`、`~/.zshrc`，也不改变仓库原有 ROS 2 Jazzy 环境。

## 数据流

```text
Gazebo VLP-16 (/velodyne_points, 5 Hz)
  ├─> VLP16 水平切片 (/scan)
  │    └─> slam_toolbox ─> map→odom、/map、/slam_pose、SLAM 轨迹
  └─> 按 map→sensor 位姿累积 ─> /semantic/geometry_map

Gazebo RGB (/camera/image + /camera/camera_info, 15 Hz)
  └─> 与 /velodyne_points 做近似时间同步并执行显式时间差门禁
       └─> 轻量实例分割（模型自带类别）
            ├─> /semantic/image（检测框、掩膜、类别、置信度）
            └─> 通过 TF 将雷达点变换到相机 frame 后投影关联
                 ├─> /semantic/cloud（当前帧带 label/confidence/rgb 的点）
                 ├─> /semantic/map（体素融合后的三维语义地图）
                 ├─> /semantic/markers（类别与三维位置）
                 └─> /semantic/status（时差、TF frame、耗时和体素容量统计）
```

## 坐标系与时间同步

```text
map ──slam_toolbox──> odom ──车辆里程计──> sensor
                                            ├─> velodyne
                                            └─> camera
```

官方包把车体、相机和雷达生成成三个独立 Gazebo 模型，而且原始实现让雷达不随车辆偏航，同时发布 `map→sensor`。准备脚本只在被 Git 忽略的本地副本上应用补丁：雷达随车体完整旋转，里程计改发 `odom→sensor`，为 `slam_toolbox` 留出 `map→odom`。仓库不篡改下载的 ZIP，SHA-256 会在解压前校验。

官方 VLP-16 xacro 默认选择 CPU ray，但源码包同时提供 GPU ray 分支和插件。本方案启用其 GPU 分支，仍保持官方 16 线、350 水平采样和 5 Hz 配置；在录屏开发机上可显著减少 Gazebo 射线计算造成的实时因子下降。它只优化仿真传感器生成，不改变 SLAM 或语义算法，也不代表 RK3588 部署依赖独立显卡。

语义节点使用 `ApproximateTimeSynchronizer` 同步图像、相机内参和点云，默认队列为 20、同步窗口为 80 ms，并再次检查图像/内参与点云的实际时间差。`/semantic/status` 发布每个已处理帧的 `image_cloud_delta_ms` 与 `camera_info_cloud_delta_ms`，运行验收要求两者 P95 不超过 50 ms。

投影不再在算法中写死相机与雷达的高度差。节点按点云时间戳查询 `camera←velodyne` TF，将所有点变换到相机相关 frame，再使用 `CameraInfo.K` 投影。官方 Gazebo `camera` frame 使用前向 `+X`、左向 `+Y`、上向 `+Z`，因此 `camera_frame_convention=auto` 对普通 `camera` frame 使用 body 轴转换；frame 名包含 `optical` 时自动按 REP-103 optical-frame 的右向、下向、前向坐标投影。

## 算法选择

- **定位与二维结构地图**：`slam_toolbox`，输入由 VLP-16 多条近水平扫描线归并得到的 `LaserScan`。它执行扫描匹配和位姿图优化，适合官方室内平面运动场景。
- **三维地图**：按 SLAM 的 `map→sensor` 变换累积 VLP-16 点云，并做 0.10 m 固定分辨率体素融合；上限 20 万体素、2 秒发布一次，避免长时间运行时大点云序列化拖慢实时链路。达到容量上限后，已存在体素仍会用新观测刷新，只有从未出现的新体素被拒绝；拒绝计数写入 `/semantic/status`，防止地图在满容量后整体“冻结”。不会把 `/registered_scan` 的真值变换当算法结果。
- **语义模型**：YOLOv8n-seg，类别完全来自预训练模型自身，不依赖赛事提供类别表。模型约 340 万参数；开发机以 ONNX/OpenCV DNN 运行，导出的固定 320 输入便于后续转 RKNN。在 RK3588 上需要再做 RKNN INT8 转换和真板性能验收，不能仅凭模型大小声称已经通过真板测试。
- **动态物体**：`person` 等可移动实例保留在实时语义层和 marker 中，但默认不融合进静态几何地图，降低行人拖影。

## 可视化与录屏

RViz 同时显示 `/map`、SLAM 轨迹、当前雷达帧、三维几何地图、彩色语义地图、类别 marker 和语义叠加图像。终端持续输出帧率、图像—点云时差、推理耗时、检测类别、关联点数、地图体素数和容量拒绝计数。Gazebo 窗口用于证明传感器数据来自官方仿真场景；最终由用户把 Gazebo、RViz 和日志终端同时录入画面。

自动演示路线按完整圆%��交替转向，形成有界的双圆/“8”字轨迹；它会反复经过可观测区域并触发回环，不会像固定时长的左右蛇形路线那样逐步漂出场景。

## 验收门禁

1. 官方五个话题存在且频率符合配置，相机约 15 Hz、雷达约 5 Hz。
2. TF 树只有一个 `map→odom→sensor` 链，且 `camera←velodyne` 可查询，不存在重复发布者或断链。
3. 图像、相机内参与点云同步时差 P95 不超过 50 ms，`sync_rejected` 为 0。
4. 车辆运动后 `/map` 和 SLAM 轨迹持续更新，三维地图体素数增长直至容量门限；达到门限后抽查已存在体素仍持续更新。
5. `/semantic/image`、`/semantic/cloud`、`/semantic/map`、`/semantic/markers` 和 `/semantic/status` 均持续发布。
6. 语义点在 RViz 中与被检测物体空间位置一致；抽查前、左、右视野，排除轴向、镜像或外参错误。
7. 连续运行至少 10 分钟，无节点退出、TF 外推错误或地图无界增长；记录实时因子、CPU/GPU、语义帧率与容量拒绝计数。

代码层单元测试覆盖 body/optical 两套投影约定以及“达到容量后仍刷新旧体素”的回归场景。ROS/Gazebo 时序、TF 和 10 分钟持续运行仍必须在开发机上执行 `./scripts/simulation/sim.sh validate` 与长时回归后再合并到稳定分支。
