# 数据目录说明

本目录只保存数据来源、校验值和获取说明，不保存比赛原始数据。

## 官方数据

- 文件：`data.bag`
- 地址：<https://opensource-dataset.obs.cn-east-3.myhuaweicloud.com/openloong-contest-resources/master-1/data.bag>
- 服务器报告大小：2,532,226,415 bytes

下载后建议在本地执行：

```bash
sha256sum data.bag
rosbag info data.bag
```

将以下信息写入 `docs/bag-audit.md`：

- SHA-256；
- 实际文件大小；
- ROS bag版本和时长；
- 全部话题、消息类型、消息数和频率；
- frame_id；
- 点云字段；
- 首尾时间戳；
- 是否存在损坏消息或时间跳变。

## 本地目录建议

```text
data/
├─ raw/          # 原始bag，只读保存
├─ samples/      # 30—60秒开发样例
├─ intermediate/ # 转换、缓存和临时结果
└─ outputs/      # PCD、轨迹、视频和提交包
```

这些目录均已由 `.gitignore` 排除。不要使用比赛数据进行商业用途，也不要公开重新分发。
