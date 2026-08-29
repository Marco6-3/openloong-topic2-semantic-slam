# 手动录制 SLAM 与语义分析

## 结论

旧的 `rviz_accelerated_follow.webm` 只展示几何 SLAM，不作为最终提交录像。最终录像应把 RViz 与语义终端同时录入：RViz 显示轨迹、原始点云和按模型类别着色的 `SemanticCloud`；终端显示模型、语义帧数、位置匹配覆盖、平均置信度和主要类别。

以下命令都在仓库根目录执行。先生成一次 RViz 配置：

```bash
pixi run python scripts/prepare_semantic_rviz.py \
  --output data/outputs/final/manual_semantic_recording.rviz
```

## 录制前启动三个窗口

终端一放在屏幕右侧并保持可见，启动语义发布和分析日志：

```bash
pixi run python scripts/publish_semantic_cloud.py --log-every 50
```

看到“等待 `/cloud_registered`”后继续。终端二启动跟随视角所需 TF：

```bash
pixi run python scripts/odom_to_tf.py
```

终端三启动 RViz：

```bash
pixi run ros2 run rviz2 rviz2 \
  -d data/outputs/final/manual_semantic_recording.rviz \
  --ros-args -p use_sim_time:=true
```

在 RViz 左侧确认 `CloudRegistered`、`Odometry`、`Path` 和 `SemanticCloud (RandLA-Net RGB)` 均已勾选，`SemanticCloud` 状态为 `OK`、颜色转换器为 `RGB8`。建议 RViz 占屏幕约 70%，终端一占约 30%。

## 开始整屏录制并回放

先启动 OBS、系统录屏或其他整屏录制工具，输出 WebM/VP8 到：

```text
data/outputs/final/rviz_manual_slam_semantic.webm
```

开始录屏后立即在终端四执行全量 6.2 倍速回放：

```bash
pixi run ros2 bag play data/outputs/final/rviz_replay \
  --clock --rate 6.2
```

回放约 105 秒。结束后立即停止录屏，使成片保留在 120 秒以内；再用 `Ctrl+C` 关闭前三个进程。

## 校验与重新打包

```bash
pixi run python scripts/validate_video.py \
  data/outputs/final/rviz_manual_slam_semantic.webm \
  --output data/outputs/final/manual_semantic_video_validation.json

pixi run python scripts/package_submission.py \
  --private-rename --requirements-confirmed \
  --output data/outputs/submission_ready_to_rename.zip
```

打包脚本默认只接受上述人工录制路径；文件不存在时会拒绝生成新提交包。最终仍需由队长本地修改 ZIP 文件名和邮件主题。
