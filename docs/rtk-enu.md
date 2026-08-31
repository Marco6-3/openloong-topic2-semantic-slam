# RTK 转 ENU 与质量过滤

## 已完成结果

`scripts/export_rtk_enu.py` 从 `/qx/evk` 读取 `sensor_msgs/NavSatFix`，用 WGS84 椭球先转换到 ECEF，再以首个通过过滤的点为原点旋转到 East/North/Up。

在完整 `data/data.bag` 上执行：

```bash
pixi run rtk-enu
```

实际结果：

| 项目 | 数值 |
|---|---:|
| 输入 RTK 点 | 6,521 |
| 保留点 | 5,735 |
| 过滤点 | 786 |
| 水平范围 East | -65.450—137.512 m |
| 水平范围 North | -0.553—352.981 m |
| 高程范围 Up | -3.068—3.210 m |

默认原点为 `(31.12115284°, 121.60327063°, 6.2988 m)`。生成文件位于 `data/intermediate/rtk_enu.csv` 和 `data/intermediate/rtk_enu.json`，均不提交 Git。

## 默认过滤规则

- 经纬高必须有限且经纬度范围合法；
- `(0, 0)` 位置无效；
- `NavSatStatus.status >= 0`；
- 协方差不能是 UNKNOWN，水平协方差必须为正；
- 水平标准差不超过 0.5 m。

完整 bag 中 3 个零坐标点被过滤，另有 783 个点因水平标准差为 3 m 被过滤。保留的 5,735 个点状态均为 2，水平标准差为 0.02 m。CSV 同时保存 `horizontal_weight = 1 / max(var_e, var_n)`，便于后续图优化按测量精度加权。

阈值和原点都可显式覆盖：

```bash
python scripts/export_rtk_enu.py data/data.bag data/intermediate/custom_enu.csv \
  --max-horizontal-sigma 1.0 \
  --origin 31.12115284 121.60327063 6.2988 \
  --metadata data/intermediate/custom_enu.json
```

当前只完成 RTK 自身的坐标转换和质量处理。RTK 与 LiDAR/IMU 轨迹的刚体对齐仍受安装外参缺失影响，不能从 bag 中臆造。
