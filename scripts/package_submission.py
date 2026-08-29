#!/usr/bin/env python3
"""校验并打包初赛 PCD、语义、轨迹和 RViz 视频；不发送邮件。"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import tempfile
import zipfile
from datetime import datetime, timezone
from pathlib import Path

if __package__:
    from .validate_path import validate as validate_path
    from .validate_pcd import validate as validate_pcd
    from .validate_video import validate as validate_video
else:
    from validate_path import validate as validate_path
    from validate_pcd import validate as validate_pcd
    from validate_video import validate as validate_video


PROJECT_ROOT = Path(__file__).resolve().parents[1]
MAX_ARCHIVE_BYTES = 500 * 1024 * 1024


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--team", help="队伍名称")
    parser.add_argument("--leader", help="队长姓名")
    parser.add_argument("--phone", help="队长手机号")
    parser.add_argument(
        "--private-rename",
        action="store_true",
        help="生成不含身份信息的待重命名包；队长随后在本地修改 ZIP 名和邮件主题",
    )
    parser.add_argument("--geometry", type=Path, default=Path("data/outputs/final/map_geometry_enu.pcd"))
    parser.add_argument("--semantic", type=Path, default=Path("data/outputs/final/map_semantic.pcd"))
    parser.add_argument("--path", type=Path, default=Path("data/outputs/fused/path.yaml"))
    parser.add_argument(
        "--video", type=Path, default=Path("data/outputs/final/rviz_accelerated_follow.webm")
    )
    parser.add_argument("--labels", type=Path, default=Path("config/semantic_labels.yaml"))
    parser.add_argument(
        "--semantic-metrics",
        type=Path,
        default=Path("data/outputs/final/semantic_metrics.json"),
    )
    parser.add_argument(
        "--requirements-confirmed",
        action="store_true",
        help="队长已在本次打包前重新核对官方提交要求",
    )
    parser.add_argument("--output", type=Path, required=True, help="输出 ZIP（必须不存在）")
    return parser.parse_args()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def validate_identity(value: str, name: str) -> str:
    cleaned = value.strip()
    if not cleaned or any(character in cleaned for character in "/\\\n\r\t"):
        raise SystemExit(f"{name} 为空或含非法字符")
    return cleaned


def resolve_identity(args: argparse.Namespace) -> tuple[str, dict[str, str]]:
    identity_values = (args.team, args.leader, args.phone)
    if args.private_rename:
        if any(value is not None for value in identity_values):
            raise SystemExit("--private-rename 不得同时传入队伍名称、队长姓名或手机号")
        return "大师赛第一期赛题2_交付内容", {
            "identity_status": "intentionally omitted for private local rename",
            "archive_and_email_naming_template": (
                "大师赛第一期赛题2——队伍名称——队长姓名——队长手机号"
            ),
        }
    if any(value is None for value in identity_values):
        raise SystemExit("非私密模式必须同时传入 --team、--leader 和 --phone")
    team = validate_identity(args.team, "队伍名称")
    leader = validate_identity(args.leader, "队长姓名")
    phone = validate_identity(args.phone, "手机号")
    if not re.fullmatch(r"[+0-9 -]{6,24}", phone):
        raise SystemExit("手机号格式无效")
    subject = f"大师赛第一期赛题2——{team}——{leader}——{phone}"
    return subject, {
        "email_subject": subject,
        "team": team,
        "leader": leader,
    }


def main() -> int:
    args = parse_args()
    if not args.requirements_confirmed:
        raise SystemExit(
            "拒绝生成交付包：请由掌握真实身份信息的队长重新核对官方要求后，"
            "显式传入 --requirements-confirmed"
        )
    root_name, identity_manifest = resolve_identity(args)
    archive = args.output.expanduser().resolve()
    if archive.exists() or archive.suffix.lower() != ".zip":
        raise SystemExit("输出必须是不存在的 .zip 文件")
    sources = {
        "map_geometry.pcd": args.geometry.expanduser().resolve(),
        "map_semantic.pcd": args.semantic.expanduser().resolve(),
        "path.yaml": args.path.expanduser().resolve(),
        "rviz_demo.webm": args.video.expanduser().resolve(),
        "semantic_labels.yaml": args.labels.expanduser().resolve(),
        "semantic_metrics.json": args.semantic_metrics.expanduser().resolve(),
    }
    missing = [str(path) for path in sources.values() if not path.is_file()]
    if missing:
        raise SystemExit(f"缺少交付文件：{missing}")
    geometry_report = validate_pcd(sources["map_geometry.pcd"])
    semantic_report = validate_pcd(sources["map_semantic.pcd"], ("label", "confidence", "rgb"))
    path_report = validate_path(sources["path.yaml"])
    video_report = validate_video(sources["rviz_demo.webm"])
    for report, packaged_name in (
        (geometry_report, "map_geometry.pcd"),
        (semantic_report, "map_semantic.pcd"),
        (path_report, "path.yaml"),
        (video_report, "rviz_demo.webm"),
    ):
        report["path"] = packaged_name

    archive.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="openloong-submission-") as temporary:
        root = Path(temporary) / root_name
        root.mkdir()
        files = {}
        for name, source in sources.items():
            destination = root / name
            shutil.copy2(source, destination)
            files[name] = {"size_bytes": destination.stat().st_size, "sha256": sha256(destination)}
        manifest = {
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "requirements_confirmed_by_user": True,
            **identity_manifest,
            "files": files,
            "validation": {
                "geometry": geometry_report,
                "semantic": semantic_report,
                "path": path_report,
                "video": video_report,
            },
            "coordinate_frame": "map_geometry.pcd and map_semantic.pcd use local ENU; origin is recorded in project RTK metadata",
            "semantic_status": "RandLA-Net model-defined classes with per-point confidence and multi-frame ENU fusion",
        }
        (root / "submission_manifest.json").write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        with zipfile.ZipFile(archive, "x", compression=zipfile.ZIP_DEFLATED, compresslevel=6) as bundle:
            for item in sorted(root.rglob("*")):
                if item.is_file():
                    bundle.write(item, item.relative_to(root.parent))
    if archive.stat().st_size > MAX_ARCHIVE_BYTES:
        raise SystemExit(
            f"压缩包 {archive.stat().st_size / 1024**2:.1f} MiB 超过 500 MiB；文件保留供调整，禁止发送"
        )
    print(f"提交包已生成：{archive} ({archive.stat().st_size / 1024**2:.1f} MiB)")
    if args.private_rename:
        print("身份信息：未写入压缩包；请由队长本地重命名 ZIP 并设置相同邮件主题")
        print("命名模板：大师赛第一期赛题2——队伍名称——队长姓名——队长手机号")
    else:
        print(f"邮件主题：{root_name}")
    print("收件人：open@openloong.org.cn；抄送：luqinghua@openloong.net")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
