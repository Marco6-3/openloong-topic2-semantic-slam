# OpenLoong 大师赛第一期·赛题二协作仓库

本仓库用于协作完成 OpenLoong 大师系列赛第一期赛题二：**具身感知与导航——非结构化场景下的在线语义地图构建与目标关联**。

当前目标是先完成一个稳定、可复现、可提交的初赛基线：使用 MID360 激光雷达、IMU 和 RTK 构建稠密点云地图，输出完整轨迹与语义标注，并录制 RViz 运行视频。在此基础上再加入全局漂移修正、动态物体过滤和语义增强。

## 当前状态

- 已报名赛题二。
- 初赛截止时间：**2026-09-13 23:59（北京时间）**。
- 官方数据包约 2.36 GiB，已在本地完成校验和只读审计，但不纳入仓库。
- 已核实公开数据说明、轨迹样例及仿真包结构。
- 已确认 LiDAR 为 Livox CustomMsg 且含点级时间戳；bag 内无相机、里程计、TF 或外参，详见 [bag 审计](docs/bag-audit.md)。

## 快速入口

- [TODO 与负责人分工](TODO.md)
- [官方资料与已确认事实](docs/official-resources.md)
- [初赛 bag 审计与复现命令](docs/bag-audit.md)
- [技术路线](docs/technical-plan.md)
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
├─ TODO.md
├─ CONTRIBUTING.md
├─ docs/
│  ├─ official-resources.md
│  ├─ technical-plan.md
│  └─ open-questions.md
├─ data/
│  └─ README.md
├─ config/              # 后续存放最终可复现参数
├─ scripts/             # 后续存放数据审计、转换和导出脚本
├─ src/                 # 后续存放自研 ROS 包或节点
└─ results/             # 仅保留小型指标和说明，不存大文件
```

## 重要边界

- 官方规则说明比赛数据不得用于商业用途。
- 数据包、模型权重、PCD、bag、视频和压缩包不进入 Git 历史。
- 当前技术路线是团队工程方案，不等同于组委会指定方案。
- 语义类别、标注格式、坐标系和具体评分指标尚未公开，必须得到组委会答复后再锁定最终输出格式。
