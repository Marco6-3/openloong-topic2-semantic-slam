# 官方资料与已确认事实

本页只记录从赛事官网、官方对象存储文件及官方仿真包中核实的信息。技术推测放在其他文档中。

## 官方入口

- 赛题页面：<https://www.openloong.org.cn/cn/contests/master/j2daue>
- 初赛数据包：<https://opensource-dataset.obs.cn-east-3.myhuaweicloud.com/openloong-contest-resources/master-1/data.bag>
- 数据说明：<https://openloong.obs.cn-east-3.myhuaweicloud.com/public/static/contests/master-1/dataset-guide.md>
- 轨迹样例：<https://opensource-dataset.obs.cn-east-3.myhuaweicloud.com/openloong-contest-resources/master-1/path.yaml>
- 仿真说明：<https://openloong.obs.cn-east-3.myhuaweicloud.com/public/static/contests/master-1/simulation-environment.md>
- 仿真模型：<https://openloong.obs.cn-east-3.myhuaweicloud.com/public/static/contests/master-1/gazebo_models.zip>
- 车辆模型源码包：<https://openloong.obs.cn-east-3.myhuaweicloud.com/public/static/contests/master-1/src.zip>
- 答疑论坛：<https://forum.openloong.org.cn/forum.php?mod=forumdisplay&fid=18>

## 赛程

| 阶段 | 时间 | 说明 |
|---|---|---|
| 报名组队和初赛提交 | 7月29日—9月13日 | 初赛最多提交10次，以最终版本评分 |
| 初赛评审 | 9月14日—9月16日 | 前30%晋级 |
| 晋级名单 | 9月17日 | 官网公布 |
| 复赛 | 9月18日—10月8日 | 最多提交5次 |
| 复赛评审 | 10月9日—10月12日 | 前8个或前5%晋级 |
| 决赛名单 | 10月13日 | 官网公布 |
| 线下总决赛 | 暂定1月初 | 任务验证及颁奖 |

赛程可能变化，应持续关注官网和答疑群。

## 初赛数据

官方数据说明列出以下 ROS 话题：

| 传感器 | 话题 | 说明 |
|---|---|---|
| MID360 雷达 | `/livox/lidar` | 激光雷达点云 |
| MID360 IMU | `/livox/imu` | 惯性测量 |
| RTK 定位 | `/qx/evk` | `sensor_msgs/NavSatFix` |
| RTK 速度 | `/qx/evk/velocity` | `geometry_msgs/TwistStamped` |
| RTK 航向 | `/qx/evk/heading` | `std_msgs/Float64` |

数据包 HTTP 元数据显示：

- `Content-Length`: 2,532,226,415 bytes，约 2.36 GiB；
- 支持 Range 请求；
- 最后修改时间：2026-07-23。

官方轨迹样例：

- 顶层字段为 `path`；
- 每点包含 `altitude`、`latitude`、`longitude`、`stamp`；
- 样例共 5342 点；
- 覆盖约 542.49 秒，平均约 9.85 Hz；
- 文件末尾包含 `total_points: 5342`。

## RTK质量信息

官方说明给出了 `NavSatFix.status.status` 的映射：

| 原始质量 | NavSatStatus | 含义 |
|---|---:|---|
| 0 | -1 | 不可用 |
| 1 | 0 | 单点定位 |
| 2 | 1 | 伪距差分/SBAS |
| 4 | 2 | RTK固定解 |
| 5 | 2 | RTK浮点解 |
| 6 | 0 | 惯导定位 |

官方还按定位质量给出了 ENU 位置协方差近似值，因此融合时不应把所有 RTK 点等权处理。

## 提交要求

主赛题页面要求：

1. 整个采集场景的 PCD 点云与语义标注信息；
2. 车辆行驶的完整轨迹 YAML；
3. 包含 RViz 等可视化界面的算法全过程录像；
4. ZIP 发送至 `open@openloong.org.cn`，抄送 `luqinghua@openloong.net`；
5. 压缩包原则上不超过 500 MB。

邮件主题和压缩包格式：

```text
大师赛第一期赛题2——队伍名称——队长姓名——队长手机号
```

官方数据说明另行注明：无需提交源码。

## 仿真包核实结果

官方仿真说明使用 ROS1、Gazebo 和 `catkin_make`。源码包中可确认：

- 相机 `/camera/image`：320×180，15 Hz；
- 相机信息 `/camera/camera_info`；
- VLP-16 点云 `/velodyne_points`：5 Hz；
- 车辆估计状态 `/state_estimation`；
- 速度控制输入 `/cmd_vel`；
- RViz 配置随车辆模型包提供。

仿真环境包含家具、小物件和行人，可用于复赛阶段的语义关联、动态物体过滤和导航测试。

2026-08-29 重新下载并校验的资源指纹：

| 资源 | 大小 | Last-Modified | SHA-256 |
|---|---:|---|---|
| `src.zip` | 315,398 bytes | 2026-07-23 | `0b518cf584967c664aa45554c6031fff3c9074ed479cecc74d1aebe8a6cc9d99` |
| `gazebo_models.zip` | 47,887,303 bytes | 2026-08-11 | `676f2c0034e31a7dc1024086b56c9bd71690c46c68448a39c0af18df54fdabe0` |

两个 ZIP 均通过完整性检查和路径穿越检查。准备脚本保留下载归档不变，仅在被 Git 忽略的 catkin 工作区副本应用补丁。

源码核实还发现官方包不能直接作为 SLAM 坐标链使用：车辆节点发布 `map→sensor`，VLP-16 模型不会随车辆偏航，`house.world` 引用了模型 ZIP 中不存在的 `hokuyo` 网格，顶层 `src/CMakeLists.txt` 为空，且自定义 VLP-16 插件目录没有自动进入 Gazebo 搜索路径。这些不是赛事算法要求，而是仿真集成问题；具体隔离修正见[复赛仿真实时语义 SLAM 架构](simulation-architecture.md)。
