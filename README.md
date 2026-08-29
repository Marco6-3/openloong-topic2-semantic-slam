# OpenLoong 大师赛第一期·赛题二协作仓库

本仓库用于协作完成 OpenLoong 大师系列赛第一期赛题二：**具身感知与导航——非结构化场景下的在线语义地图构建与目标关联**。

当前目标是先完成一个稳定、可复现、可提交的初赛基线：使用 MID360 激光雷达、IMU 和 RTK 构建稠密点云地图，输出完整轨迹与语义标注，并录制 RViz 运行视频。在此基础上再加入全局漂移修正、动态物体过滤和语义增强。

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
- 已验证同步语义点云可在 RViz 按类别着色；旧的纯 RViz 视频不再作为最终稿，等待队长按[手动录屏说明](docs/manual-semantic-recording.md)同时录入 RViz 与语义分析终端。视频、PCD、bag 和 ZIP 均留在被 Git 忽略的 `data/outputs/`。

## 第一次配置环境

适用于 x86_64 Ubuntu 24.04。先安装 [Pixi](https://pixi.prefix.dev/latest/installation/) 和 [direnv](https://direnv.net/docs/installation.html)，再按当前使用的 shell 配置 hook。

Bash 用户执行：

```bash
curl -fsSL https://pixi.sh/install.sh | sh
sudo apt-get update && sudo apt-get install -y direnv
grep -qxF 'eval "$(direnv hook bash)"' ~/.bashrc || printf '\neval "$(direnv hook bash)"\n' >> ~/.bashrc
exec bash
```

Zsh 用户把后两行换成：

```zsh
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

## 快速入口

- [TODO 与负责人分工](TODO.md)
- [官方资料与已确认事实](docs/official-resources.md)
- [初赛 bag 审计与复现命令](docs/bag-audit.md)
- [ROS 2 Jazzy 环境与边界](docs/environment.md)
- [RTK 转 ENU 与质量过滤](docs/rtk-enu.md)
- [技术路线](docs/technical-plan.md)
- [已验证的分层架构](docs/architecture.md)
- [全量候选运行报告](results/final-run.md)
- [静止、急转弯、RTK间断与动态边界检查](results/robustness-check.md)
- [RK3588 部署可行性与验收门禁](docs/rk3588-deployment.md)
- [手动录制 SLAM 与语义分析](docs/manual-semantic-recording.md)
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
