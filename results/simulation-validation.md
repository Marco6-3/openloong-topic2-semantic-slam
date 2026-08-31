# 复赛实时仿真候选验收

## 验收对象

- 官方 `house.world`、相机和 VLP-16 模型；
- GPU ray VLP-16，16 线、350 水平采样、5 Hz；
- `slam_toolbox` 实时定位与二维占据栅格；
- YOLOv8n-seg 固定 320 输入 ONNX，由模型自身 COCO 预训练能力给出类别；
- 掩膜与 VLP-16 投影关联、当前帧语义点云、三维几何/语义体素地图和 marker；
- 有界“8”字自动路线。

验证命令：

```bash
./scripts/simulation/sim.sh headless
./scripts/simulation/sim.sh validate 60 \
  data/outputs/simulation/runtime-validation-gpu.json
```

## 60 秒量化结果

| 指标 | 结果 |
|---|---:|
| 自动门禁 | PASS |
| 仿真实时因子 | 0.999 |
| 相机 | 15.000 Hz |
| VLP-16 / LaserScan | 5.000 / 5.000 Hz |
| 仿真里程计 | 200.000 Hz |
| SLAM 地图 / 轨迹 | 0.999 / 4.933 Hz |
| 语义图像、点云、状态 | 约 5.000 Hz |
| 三维几何/语义地图 | 0.477 Hz |
| 车辆累计行驶 | 20.951 m |
| SLAM 轨迹 | 389 个位姿 |
| 地图 | 456×358，0.05 m/格 |
| 语义推理中位数 / P95 | 75.7 / 99.6 ms |
| 语义总链路 P95 | 223.3 ms |

验收期间模型输出了 `bed`、`chair`、`couch`、`cup`、`dining table`、`person`、`tv` 等自身类别。仿真贴图也产生了 `airplane`、`traffic light` 等低置信或语境不合理判断；这些是通用预训练模型在合成纹理上的能力边界，不应伪装成赛事提供的真值类别。

TF 门禁全部通过：`map→odom`、`odom→sensor`、`sensor→velodyne`、`sensor→camera` 只有一条完整链。动态类别仍发布在当前帧点云和 marker，但不进入持久三维地图。

机器可读的完整采样结果保存在被 Git 忽略的 `data/outputs/simulation/runtime-validation-gpu.json`。开发机使用 RTX 4090 加速 Gazebo ray 传感器；语义 ONNX 仍以 OpenCV DNN CPU 后端计时，因此上述数值不能替代 RK3588 真板 RKNN INT8 验收。

## 10 分钟持续运行

同一进程不重启持续运行超过 10 分钟后，再执行一次完整 60 秒门禁，仍为 PASS：RTF 0.999，雷达/Scan 5.000 Hz，语义状态 4.870 Hz，SLAM 地图 0.999 Hz，语义推理 P95 92.4 ms、总链路 P95 336.8 ms，TF 四段继续完整。此时轨迹已累计 3,809 个位姿，二维地图为 429×357 格。

三维几何地图达到配置的 20 万体素上限后保持有界，节点没有退出；动态 `person` 等继续出现在当前帧点云和 marker 中，但不会推动持久地图增长。启动阶段出现过一次等待 SLAM TF 的节流提示，TF 树建立后没有形成持续错误。机器可读结果为 `data/outputs/simulation/runtime-validation-after-10min.json`。
