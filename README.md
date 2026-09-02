# OpenLoong 大师赛第一期·赛题二协作仓库

本仓库用于协作完成 OpenLoong 大师系列赛第一期赛题二：**具身感知与导航——非结构化场景下的在线语义地图构建与目标关联**。

初赛基线已冻结；当前在官方 Gazebo 场景中准备复赛实时链路：车辆自动运动时同步完成 SLAM 定位与建图、轻量模型语义分析、二维/三维语义位置标注和可视化。

## 当前状态

- 已报名赛题二。
- 初赛截止时间：**2026-09-13 23:59（北京时间）**。
- 官方数据包约 2.36 GiB，已在本地完成校验和只读审计，但不纳入仓库。
- 已核实公开数据说明、轨迹样例及仿真包结构。
- 已确认 LiDAR 为 Livox CustomMsg 且含点级时间戳；bag 内无相机、里程计、TF 或外参，详见 [bag 审计](docs/bag-audit.md)。
- 已锁定 ROS 2 Jazzy + Pixi 开发环境，并用 direnv 在进入目录时自动按 `pixi.lock` 激活。
- 已将高质量 RTK 点转换为本地 ENU 轨迹，详见 [RTK 转换说明](docs/rtk-enu.md)。
- 已用全部 652 秒数据跑通 ROS2 FAST-LIO2、RTK 轨迹融合、逐帧地图重建和 RandLA-Net 语义融合；最终得到 6,518 个轨迹点和 1,524,881 个地图点，详见 [全量候选结果](results/final-run.md)。
- 组委会已确认不提供固定类别列表，由模型自身能力定义语义且模型需适合边缘设备。当前模型权重 4.9 MiB、124 万参数，全量语义位置覆盖率 99.89%；模型体积符合轻量方向，但当前 CUDA 实现尚未在 RK3588 真板验证。
- 已完成同时包含 RViz 彩色语义点云和语义分析终端的人工录屏；交付版为 2560×1374、30 fps、115.43 秒的 VP8 WebM。视频、PCD、bag 和 ZIP 均留在被 Git 忽略的 `data/outputs/`。
- 已在独立 Pixi 环境中跑通官方 ROS Noetic + Gazebo Classic 场景、`slam_toolbox`、YOLOv8n-seg ONNX、相机/雷达语义投影、三维体素地图和自动驾驶路线；不会修改系统 ROS、`~/.gazebo/models` 或本仓库原有 ROS 2 环境。

## 第一次配置环境

适用于 x86_64 Ubuntu 24.04。先安装 [Pixi](https://pixi.prefix.dev/latest/installation/) 和 [direnv](https://direnv.net/docs/installation.html)，再按当前使用的 shell 配置 hook。

Bash 用户执行：

```bash
curl -fsSL https://pixi.sh/install.sh | sh
sudo apt-get update && sudo apt-get install -y direnv
grep -qxF 'eval "$(direnv hook bash)"' ~/.bashrc || printf '\neval "$(direnv hook bash)"\n' >> ~/.bashrc
exec bash
```

Zsh 用户完整执行：

```zsh
curl -fsSL https://pixi.sh/install.sh | sh
sudo apt-get update && sudo apt-get install -y direnv
grep -qxF 'eval "$(direnv hook zsh)"' ~/.zshrc || printf '\neval "$(direnv hook zsh)"\n' >> ~/.zshrc
exec zsh
```

Pixi 安装脚本会把 `~/.pixi/bin` 写入 shell 启动配置；重新启动 shell 后可用 `pixi --version` 和 `direnv version` 检查安装结果。

在仓库根目录依次执行：

```bash
pixi install --locked
pixi run setup
direnv allow
```

完成。以后只要 `cd` 进入本仓库，`.envrc` 就会执行 `pixi shell-hook --locked`：锁文件和 `pixi.toml` 不一致时会直接报错，不会自动更新依赖。

常用命令：

```bash
pixi run check      # 检查 ROS、PCL、Eigen、编译器等版本
pixi run test       # 运行测试
pixi run audit-bag  # 审计 data/data.bag
pixi run rtk-enu    # 导出 RTK 的 ENU 轨迹
pixi run build      # colcon 构建 src/
pixi run convert-bag # ROS1 bag 转换并核对为 ROS2 MCAP
pixi run baseline   # 2x 完整运行 FAST-LIO2，并执行丢帧门禁
pixi run fuse       # LIO + RTK 融合并导出官方格式 path.yaml
pixi run rebuild-map # 逐帧全局校正后重建 ENU 地图
pixi install -e semantic-gpu --locked # 单独安装 GPU 语义环境
pixi run -e semantic-gpu semantic # 下载固定权重、逐帧推理并做多帧融合
pixi run -e semantic-gpu semantic-test # 验证网络结构与固定权重兼容性
pixi run semantic-fallback # 仅在模型不可用时生成几何语义兜底
pixi run validate-output # 校验最终 PCD 和 YAML
pixi run retime-video # 重建使用 Header 时间戳的可视化 bag
pixi run record-video # 6x 录制跟随机体的 RViz 视频（需要 DISPLAY）
pixi run validate-video # 校验编码、分辨率和 <=2 分钟门禁
```

比赛数据默认放在 `data/data.bag`，不会进入 Git。若放在其他位置，可以直接给脚本传路径，例如：

```bash
python scripts/export_rtk_enu.py /path/to/data.bag data/intermediate/rtk_enu.csv
```

需要有意升级依赖时，修改 `pixi.toml` 后执行 `pixi lock`，检查 `pixi.lock` 的变化，再运行 `pixi install --locked`。

## 初赛人工录屏：四个终端同时运行

下面不是四选一。先按编号依次启动，最后四个进程保持同时运行；录屏画面同时包含 RViz 和终端一的语义日志。所有命令都在仓库根目录执行。

终端一（语义发布与日志）：

```bash
pixi run python scripts/publish_semantic_cloud.py --log-every 50
```

终端二（跟随视角 TF）：

```bash
pixi run python scripts/odom_to_tf.py
```

终端三（RViz）：

```bash
pixi run ros2 run rviz2 rviz2 \
  -d data/outputs/final/manual_semantic_recording.rviz \
  --ros-args -p use_sim_time:=true
```

开始整屏录制后，终端四启动回放：

```bash
pixi run ros2 bag play data/outputs/final/rviz_replay \
  --clock --rate 6.2
```

完整窗口布局、RViz 检查项和视频校验见[初赛手动录屏说明](docs/manual-semantic-recording.md)。

## 复赛仿真实时语义 SLAM

复赛环境完全位于 `simulation/.pixi` 与被 Git 忽略的 `data/simulation/catkin_ws`。准备过程会按固定 SHA-256 下载官方资源，只给本地副本打坐标系、缺失模型和实时性补丁，并启用官方源码自带的 VLP-16 GPU ray 分支；不执行 `apt install`，也不写 shell 配置或系统 ROS。

首次准备：

```bash
./scripts/simulation/sim.sh prepare
./scripts/simulation/sim.sh model
```

录屏时使用两个终端，按顺序启动后保持同时运行：

终端一（Gazebo、自动路线、SLAM、语义节点和 RViz）：

```bash
./scripts/simulation/sim.sh demo
```

等 Gazebo、RViz 已显示数据后，终端二启动联合状态面板，并保持在录屏画面内：

```bash
./scripts/simulation/sim.sh monitor
```

状态面板每秒显示仿真实时因子、SLAM 轨迹位姿数、地图尺寸、模型判断的类别与位置关联点、推理耗时及通过多帧一致性门禁的语义地图体素数。因此录像可同时证明 SLAM 算法在运行以及语义分析数据在实时更新。持久语义地图会过滤单帧瞬态误检，并拒绝相机—雷达时间差超过 120 ms 的关联。录屏候选还启用了官方场景高精度类别策略、真实 TF 投影、掩膜边界与遮挡深度过滤；最终首圈真值位置误差中位数 0.170 m、P95 0.626 m，详见[复赛仿真验收](results/simulation-validation.md)。

开始录屏前把 Gazebo、RViz 和终端二平铺到同一桌面：Gazebo 用于证明官方仿真场景与车辆运动，RViz 保留 `SLAM Map`、`SLAM Trajectory`、`Semantic Map`、`3D Semantic Labels` 和 `Semantic Camera`，终端二必须露出完整的一行实时状态。Gazebo 首次打开可能占满屏幕，直接取消最大化并手动缩放即可。建议先录制 60–90 秒；结束时**先停止并保存录屏**，再依次在终端二、终端一按 `Ctrl+C`，避免把 Gazebo Classic 的退出日志录入成片。

开发验收可在仿真运行时从第三个终端执行：

```bash
./scripts/simulation/sim.sh validate
```

该命令连续观察 60 秒，检查话题频率、TF 树、车辆位移、地图/轨迹增长、语义输出、推理耗时和实时因子。架构、官方包修正原因与验收边界见[复赛仿真架构](docs/simulation-architecture.md)。

## 快速入口

- [TODO 与负责人分工](TODO.md)
- [官方资料与已确认事实](docs/official-resources.md)
- [初赛 bag 审计与复现命令](docs/bag-audit.md)
- [ROS 2 Jazzy 环境与边界](docs/environment.md)
- [RTK 转 ENU 与质量过滤](docs/rtk-enu.md)
- [技术路线](docs/technical-plan.md)
- [已验证的分层架构](docs/architecture.md)
- [全量候选运行报告](results/final-run.md)
- [复赛实时仿真候选验收](results/simulation-validation.md)
- [静止、急转弯、RTK间断与动态边界检查](results/robustness-check.md)
- [RK3588 部署可行性与验收门禁](docs/rk3588-deployment.md)
- [手动录制 SLAM 与语义分析](docs/manual-semantic-recording.md)
- [复赛仿真实时语义 SLAM 架构](docs/simulation-architecture.md)
- [第三方依赖与许可](docs/third-party.md)
- [待确认问题与规则冲突](docs/open-questions.md)
- [数据目录说明](data/README.md)
- [协作约定](CONTRIBUTING.md)

## 推荐工作流

1. 认领 `TODO.md` 中的任务，并在任务后填写负责人。
2. 从 `main` 创建短生命周期分支，例如 `feat/bag-audit`。
3. 提交中记录运行环境、命令、输入和验证结果。
4. 通过 Pull Request 合并，避免直接把未经验证的参数写入主分支。
5. 大文件、数据集、模型权重、生成地图和视频不得直接提交到 Git。

## 仓库规划

```text
.
├─ README.md
├─ pixi.toml / pixi.lock / .envrc
├─ TODO.md
├─ CONTRIBUTING.md
├─ docs/
│  ├─ official-resources.md
│  ├─ technical-plan.md
│  └─ open-questions.md
├─ data/
│  └─ README.md
├─ config/              # 后续存放最终可复现参数
├─ scripts/             # 数据审计、转换、环境检查和导出脚本
├─ tests/               # 可重复的自动化测试
├─ src/                 # 自研 ROS 2 包或节点
└─ results/             # 仅保留小型指标和说明，不存大文件
```

## 重要边界

- 官方规则说明比赛数据不得用于商业用途。
- 数据包、模型权重、PCD、bag、视频和压缩包不进入 Git 历史。
- 当前技术路线是团队工程方案，不等同于组委会指定方案。
- 语义类别已按组委会答复由轻量模型定义；PCD 字段和具体评分指标仍未公开，因此同时提交字段说明与模型运行指标。

## 生成提交包

正式交付只能由掌握真实身份信息的队长在本地执行。每次打包前重新核对官方页面、群通知和答疑；不要让 Codex 猜测、保存或代填队长姓名与手机号。推荐使用私密模式生成内部不含身份信息的包，再由队长仅在本地重命名 ZIP，并把邮件主题设成相同名称：

先按[手动录屏说明](docs/manual-semantic-recording.md)生成 `data/outputs/final/rviz_manual_slam_semantic.webm`；缺少该文件时，默认打包命令会拒绝继续。

```bash
pixi run python scripts/package_submission.py \
  --private-rename --requirements-confirmed \
  --output data/outputs/submission_ready_to_rename.zip
```

也可以由队长完全在本地一次性生成最终名称：

```bash
pixi run python scripts/package_submission.py \
  --team '队伍名称' --leader '队长姓名' --phone '队长手机号' \
  --requirements-confirmed \
  --output data/outputs/大师赛第一期赛题2——队伍名称——队长姓名——队长手机号.zip
```

脚本会再次校验两个 PCD、轨迹和视频，写入 SHA-256 清单并检查 500 MiB 上限，但不会自动发送邮件。
