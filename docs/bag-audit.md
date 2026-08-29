# 初赛 `data.bag` 审计

审计日期：2026-08-29

本页记录对官方原始数据的只读审计结果。原始 bag、短样例和生成的 JSON 均被 `.gitignore` 排除，不随仓库分发。

## 结论先行

- 数据是 ROS1 bag v2.0，共 156,525 条消息，时长 652.431410753 秒。
- `/livox/lidar` 不是 `sensor_msgs/PointCloud2`，而是 `livox_ros_driver2/CustomMsg`（`rosbags` 内部规范名为 `livox_ros_driver2/msg/CustomMsg`）。FAST-LIO2 应使用 Livox CustomMsg 输入路径。
- CustomPoint 明确含纳秒单位的 `uint32 offset_time`，每帧覆盖约 99.07—101.25 ms；50 个均匀样本都不是全零，足以提供点级运动畸变补偿所需的相对时间。点在数组中的 `offset_time` 并非全局单调，适配器不得假设数组顺序就是严格时间顺序。
- 实际 bag 只有官方列出的 LiDAR、IMU、3 个 RTK 话题及 `/rosout`，没有相机、里程计、`/tf`、`/tf_static` 或 `/clock`。
- LiDAR 与 IMU 的 `frame_id` 都是 `livox_frame`；RTK 位置与速度的 `frame_id` 都是 `wgs84`；heading 是无 Header 的 `std_msgs/Float64`。
- bag 内没有 LiDAR/IMU 到 RTK 的刚体外参，也没有能恢复此外参的 TF。当前只能确认同名 Livox 坐标系，不能从 bag 推断 `livox_frame` 与 RTK 天线/车体的平移和朝向；建图前须向组委会索取或现场标定。
- RTK 三个话题按索引近似同步：velocity 相对 NavSatFix 的 bag 时间中位差为 0.078 ms，heading 为 0.136 ms。LiDAR 在一帧最后一点之后约 1.54—5.01 ms 写入 bag。
- 生成并回读验证了从原始起点后 30 秒开始、持续约 60 秒的开发样例，共 14,400 条消息；样例文件不提交 Git，可用仓库脚本一条命令重建。

## 文件身份

| 项目 | 值 |
|---|---|
| 来源 | 官方对象存储 `master-1/data.bag` |
| 下载时间 | 2026-08-29（北京时间） |
| 文件大小 | 2,532,226,415 bytes |
| SHA-256 | `35ec4e14c13aeceb238f0ac1edd171a255d7615b1a1bc6204f79679b29156ae2` |
| bag 起止时间 | 399.889542176 s—1052.320952929 s |
| bag 时长 | 652.431410753 s |
| 消息总数 | 156,525 |

这些时间不是 Unix 纪元时间。bag 中也没有 `/clock`，因此只能按 bag 内的统一时间轴解释，不能据此恢复实际采集日期。

## 话题、类型和频率

类型栏采用 `rosbags` 的 ROS2 风格规范名；对应 ROS1 工具会显示例如 `sensor_msgs/Imu`、`livox_ros_driver2/CustomMsg`。

| 话题 | 类型 | 数量 | 实测频率 | 周期间隔中位数 | 最大间隔 | `frame_id` |
|---|---|---:|---:|---:|---:|---|
| `/livox/imu` | `sensor_msgs/msg/Imu` | 130,435 | 200.000 Hz | 4.984 ms | 10.611 ms | `livox_frame` |
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | 6,522 | 10.000 Hz | 99.978 ms | 103.036 ms | `livox_frame` |
| `/qx/evk` | `sensor_msgs/msg/NavSatFix` | 6,521 | 10.000 Hz | 99.990 ms | 186.982 ms | `wgs84` |
| `/qx/evk/velocity` | `geometry_msgs/msg/TwistStamped` | 6,521 | 10.000 Hz | 99.990 ms | 186.938 ms | `wgs84` |
| `/qx/evk/heading` | `std_msgs/msg/Float64` | 6,521 | 10.000 Hz | 99.991 ms | 187.012 ms | 无 Header |
| `/rosout` | `rosgraph_msgs/msg/Log` | 5 | 不适用 | 0.021 ms | 0.051 ms | 空 |

频率按各话题首末消息计算。RTK 偶有约 187 ms 的长间隔，融合端应使用原始时间戳，不要假设严格 10 Hz 等间隔。

## Livox 点云字段与时间

`CustomMsg`：

```text
std_msgs/Header header
uint64 timebase
uint32 point_num
uint8 lidar_id
uint8[3] rsvd
livox_ros_driver2/CustomPoint[] points
```

`CustomPoint`：

```text
uint32 offset_time   # 相对 timebase，单位 ns
float32 x
float32 y
float32 z
uint8 reflectivity
uint8 tag
uint8 line
```

50 帧均匀抽样结果：

- 每帧 19,872—20,064 点，`point_num == len(points)`；
- `offset_time` 最小值均为 0 ns；
- 最大值为 99,074,253—101,247,277 ns；
- `header.stamp == timebase`；
- bag 写入时间比 `header.stamp` 晚 101.07—105.28 ms；
- bag 写入时间比本帧最后一点晚 1.54—5.01 ms；
- 每帧有 123—142 次负向时间跳变，因此不能用数组下标代替 `offset_time`。

结论：点级相对时间存在且量级合理，FAST-LIO2 的 Livox CustomMsg 预处理路径可用于下一阶段；接入时仍需用真实小样例检查驱动版本与字段定义完全一致。

## RTK 质量快照

`/qx/evk` 的 6,521 条消息中：

| `NavSatStatus.status` | 数量 | 处理建议 |
|---:|---:|---|
| -1 | 3 | 丢弃；对应零经纬高 |
| 0 | 783 | 低质量，降权或按规则过滤 |
| 2 | 5,735 | 官方映射可能同时包含 RTK fixed/float，仍结合协方差加权 |

出现的 ENU 协方差对角线只有四组：`(0,0,0)`、`(0.0004,0.0004,0.0016)`、`(0.09,0.09,0.25)`、`(9,9,25)`。不要仅凭 `status == 2` 把 fixed 与 float 当成同一精度。

## 时间与坐标系边界

- IMU、LiDAR Header 都使用 `livox_frame`；这说明消息声明了同一坐标系，但不能证明传感器物理原点完全重合。
- NavSatFix 与 TwistStamped Header 使用 `wgs84`。严格说 `frame_id` 通常应描述笛卡尔坐标系，而 WGS84 是地理参考；后续 ENU 转换必须明确原点、轴向和高度基准。
- heading 无 Header，只能按邻近 bag 时间与同索引 RTK 消息关联。尚未确认 heading 的零方向、正方向和角度单位；数值范围为 0.12—359.9，推测单位是度，但在官方答复前不能当作已确认事实。
- 缺少 `/tf`、`/tf_static` 和标定参数，因此不能完成 `wgs84/enu -> vehicle -> livox_frame` 的完整变换链。

## 复现审计

本机环境为 Ubuntu 24.04、Python 3.12.3；没有安装 ROS1/ROS2 命令行工具，因此使用可跨平台只读解析 ROS1 bag 的 `rosbags 0.11.5`。脚本不会发布话题或修改原始 bag。

```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt

mkdir -p data/raw data/intermediate data/samples
curl -L --fail --retry 5 --continue-at - \
  --output data/raw/data.bag \
  https://opensource-dataset.obs.cn-east-3.myhuaweicloud.com/openloong-contest-resources/master-1/data.bag
sha256sum data/raw/data.bag

.venv/bin/python scripts/audit_bag.py \
  data/raw/data.bag --samples 50 --sha256 \
  --output data/intermediate/bag-audit.json

.venv/bin/python scripts/slice_bag.py \
  data/raw/data.bag data/samples/data_30s_60s.bag \
  --start 30 --duration 60
```

样例回读结果：

| 项目 | 值 |
|---|---|
| 文件大小 | 232,980,936 bytes |
| SHA-256 | `257322984639810a5f9a24ddccdef713219a730bdd44de510b5635bc8c072f31` |
| 实际时长 | 59.995529857 秒 |
| 消息数 | 14,400 |
| 组成 | IMU 12,000；LiDAR 600；三个 RTK 话题各 600 |

截取脚本按原始序列化消息复制，不改变消息时间戳。输出路径必须不存在，避免误覆盖已有样例。

## ROS1 播放与 RViz

在装有 ROS1、`livox_ros_driver2` 消息包的环境中：

```bash
source /opt/ros/noetic/setup.bash
source /path/to/livox_ros_driver2/devel/setup.bash
roscore
rosparam set use_sim_time true
rosbag info data/samples/data_30s_60s.bag
rosbag play --clock data/samples/data_30s_60s.bag
```

`CustomMsg` 不能直接作为 RViz 的 PointCloud2 显示。下一位应先接入 FAST-LIO2/Livox 转换节点，再在 RViz 中显示转换后的点云与轨迹。本次主机没有 ROS1，以上 ROS 原生播放和 RViz 显示命令尚未在本机执行；bag 结构、反序列化和样例回读已经实际验证。

## 下一位可以直接开始的工作

1. 建立 ROS1 Noetic + `livox_ros_driver2` + FAST-LIO2 环境。
2. 将 LiDAR 话题设为 `/livox/lidar`、IMU 设为 `/livox/imu`，选择 Livox CustomMsg 预处理。
3. 先用 `data_30s_60s.bag` 编译和调参，再运行 652 秒完整 bag。
4. 外参未提供时不要把猜测参数固化到 `config/`；向组委会索取 LiDAR/IMU/RTK 安装关系。
5. RTK 融合必须过滤 3 条 `status=-1` 消息，并结合协方差区分质量。
