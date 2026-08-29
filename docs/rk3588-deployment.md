# RK3588 部署可行性与验收门禁

## 当前结论

当前仓库版本**不能直接在 RK3588 上运行**。现有语义环境锁定 `linux-64`、PyTorch GPU 和 CUDA 12.9，而 RK3588 是 ARM64 + RKNPU，不提供 CUDA。4.9 MiB 权重和 124 万参数说明模型本身较小，不等于板端软件链路已经完成。

Rockchip 官方 RKNN-Toolkit2 支持 RK3588，并要求先在 PC 上把模型转换成 RKNN，再通过板端 Python Lite API 或 C/C++ Runtime 推理。官方 ONNX 算子表包含本模型主干需要的 `Conv`、`ConvTranspose`、`Gather/GatherElements`、`ReduceSum`、`Sqrt`、`Softmax`、`Reshape` 和 `Transpose`；但官方 PyTorch 算子表把 `aten::gather`、`aten::sqrt` 列为不支持。因此应导出固定形状 ONNX 后转换，不能假定当前 PyTorch 模块可被 RKNN 直接接收。

参考：

- <https://github.com/airockchip/rknn-toolkit2>
- <https://github.com/airockchip/rknn-toolkit2/blob/master/doc/RKNNToolKit2_OP_Support-2.3.2.md>
- <https://github.com/airockchip/rknn_model_zoo>

## 实测边界

当前 x86 主机上，45,056 点固定输入的结果如下：

| 路径 | 设备 | 时间 |
|---|---|---:|
| KNN/层级输入构造 | x86 CPU | 全量实际运行平均 122.61 ms |
| 模型前向 | RTX 4090 | 平均 14.26 ms，P95 28.43 ms |
| 模型前向 | x86 CPU，4线程 | 单次合成输入 1.798 s |

CPU 数字不是 RK3588 性能，但足以说明不能只把当前脚本复制到板上并指望维持每秒一次的语义更新；NPU 加速和预处理优化是必要工作。

## 推荐拆分

```text
MID360/配准点
      │
      ▼
ARM CPU C++：半径过滤、6 cm 去重、KNN、四层采样索引
      │  固定形状 coords/features/neighbor/pool/interp（int32 索引）
      ▼
RK3588 NPU：RandLA-Net 网络主干（FP16，验证后再尝试 INT8）
      │
      ▼
ARM CPU C++：softmax/标签映射、ENU 投影、多帧概率融合
```

KNN 和 cKDTree 属于输入构造，不应塞进 RKNN 图。板端建议用 C++ 的 nanoflann、PCL KD-tree 或经过基准测试的等价实现，避免依赖当前 SciPy/Python 环境。网络输入固定为 45,056 点以及四层预先计算的邻接和插值索引。

## 实施顺序

1. 为网络主干增加固定形状 ONNX 导出器，把 KNN/采样索引作为模型输入。
2. 在 x86 上用 ONNX Runtime 与当前 PyTorch 输出做逐点概率和标签对齐。
3. 用 RKNN-Toolkit2 2.3.2 先转换 FP16，确认所有算子实际落在 NPU；不要仅凭“转换成功”判断正确。
4. 在 RK3588 上用 RKNN Runtime C API 运行，CPU 侧完成 KNN 和地图融合。
5. FP16 通过后再评估 INT8；注意 attentive pooling 和 softmax 的量化误差。
6. 连续运行至少 10 分钟，记录预处理、NPU、后处理 P50/P95、RSS、温度和是否降频。

## 内部验收门禁

- ARM64 构建不依赖 CUDA，部署包不包含完整桌面 ROS/PyTorch 环境。
- 固定验证集上，FP16 RKNN 相对 PyTorch 的逐点 top-1 一致率至少 99%，类别概率平均绝对误差不超过 0.01。
- CPU 预处理 + NPU 前向 + 后处理的 P95 不超过 900 ms，以维持当前每 10 帧、约 1 Hz 的语义更新节奏。
- 连续 10 分钟无崩溃、内存持续增长或热降频导致的门禁失败。
- 报告板卡型号、内存、系统镜像、RKNPU 驱动、RKNN Runtime 和 Toolkit2 的精确版本。

这些是项目内部工程门禁，不是组委会公布指标。没有目标 RK3588 开发板及其系统镜像时，只能完成导出和转换侧工作，不能把“算子表看起来支持”写成“已经支持 RK3588”。
