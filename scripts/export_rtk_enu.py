#!/usr/bin/env python3
"""从 ROS bag 导出经过质量过滤的 WGS84 RTK 轨迹到本地 ENU CSV。"""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

import numpy as np
from rosbags.highlevel import AnyReader


WGS84_A = 6_378_137.0
WGS84_F = 1.0 / 298.257_223_563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


@dataclass(frozen=True)
class Fix:
    bag_time_ns: int
    header_time_ns: int | None
    latitude: float
    longitude: float
    altitude: float
    status: int
    covariance_type: int
    covariance_e: float
    covariance_n: float
    covariance_u: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="ROS1 .bag 文件或 ROS2 bag 目录")
    parser.add_argument("output", type=Path, help="输出 ENU CSV")
    parser.add_argument("--metadata", type=Path, help="可选的处理摘要 JSON")
    parser.add_argument("--topic", default="/qx/evk", help="NavSatFix 话题")
    parser.add_argument("--min-status", type=int, default=0, help="最低 NavSatStatus.status")
    parser.add_argument(
        "--max-horizontal-sigma",
        type=float,
        default=0.5,
        help="保留点的最大水平标准差（米）；未知协方差不通过",
    )
    parser.add_argument(
        "--origin",
        type=float,
        nargs=3,
        metavar=("LAT", "LON", "ALT"),
        help="ENU 原点；默认使用第一个通过过滤的 RTK 点",
    )
    return parser.parse_args()


def stamp_ns(stamp: Any) -> int | None:
    if stamp is None:
        return None
    if hasattr(stamp, "sec") and hasattr(stamp, "nanosec"):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    if hasattr(stamp, "secs") and hasattr(stamp, "nsecs"):
        return int(stamp.secs) * 1_000_000_000 + int(stamp.nsecs)
    return None


def geodetic_to_ecef(latitude: float, longitude: float, altitude: float) -> np.ndarray:
    """将 WGS84 纬经高转换为 ECEF XYZ（米）。"""
    lat = math.radians(latitude)
    lon = math.radians(longitude)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    radius = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    return np.asarray(
        [
            (radius + altitude) * cos_lat * math.cos(lon),
            (radius + altitude) * cos_lat * math.sin(lon),
            (radius * (1.0 - WGS84_E2) + altitude) * sin_lat,
        ],
        dtype=np.float64,
    )


def ecef_to_enu(
    ecef: np.ndarray, origin_ecef: np.ndarray, origin_latitude: float, origin_longitude: float
) -> np.ndarray:
    """将 ECEF XYZ 转换为给定 WGS84 原点处的 East/North/Up。"""
    lat = math.radians(origin_latitude)
    lon = math.radians(origin_longitude)
    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    sin_lon, cos_lon = math.sin(lon), math.cos(lon)
    rotation = np.asarray(
        [
            [-sin_lon, cos_lon, 0.0],
            [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
            [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat],
        ],
        dtype=np.float64,
    )
    return rotation @ (ecef - origin_ecef)


def enu_to_geodetic(
    enu: np.ndarray, origin_latitude: float, origin_longitude: float, origin_altitude: float
) -> tuple[float, float, float]:
    """将局部 ENU 米制坐标转换回 WGS84 纬经高。"""
    lat = math.radians(origin_latitude)
    lon = math.radians(origin_longitude)
    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    sin_lon, cos_lon = math.sin(lon), math.cos(lon)
    rotation = np.asarray(
        [
            [-sin_lon, cos_lon, 0.0],
            [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
            [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat],
        ],
        dtype=np.float64,
    )
    ecef = geodetic_to_ecef(origin_latitude, origin_longitude, origin_altitude) + rotation.T @ enu
    x, y, z = (float(value) for value in ecef)
    longitude = math.atan2(y, x)
    horizontal = math.hypot(x, y)
    latitude = math.atan2(z, horizontal * (1.0 - WGS84_E2))
    altitude = 0.0
    for _ in range(10):
        sin_value = math.sin(latitude)
        radius = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_value * sin_value)
        altitude = horizontal / math.cos(latitude) - radius
        updated = math.atan2(
            z, horizontal * (1.0 - WGS84_E2 * radius / (radius + altitude))
        )
        if abs(updated - latitude) < 1e-14:
            latitude = updated
            break
        latitude = updated
    return math.degrees(latitude), math.degrees(longitude), altitude


def rejection_reason(fix: Fix, min_status: int, max_horizontal_sigma: float) -> str | None:
    values = (fix.latitude, fix.longitude, fix.altitude)
    if not all(math.isfinite(value) for value in values):
        return "non_finite_position"
    if fix.latitude == 0.0 and fix.longitude == 0.0:
        return "zero_position"
    if not (-90.0 <= fix.latitude <= 90.0 and -180.0 <= fix.longitude <= 180.0):
        return "position_out_of_range"
    if fix.status < min_status:
        return "status_below_minimum"
    variances = (fix.covariance_e, fix.covariance_n)
    if fix.covariance_type == 0 or not all(math.isfinite(value) and value > 0 for value in variances):
        return "unknown_covariance"
    if math.sqrt(max(variances)) > max_horizontal_sigma:
        return "horizontal_sigma_too_large"
    return None


def iter_fixes(reader: AnyReader, topic: str) -> Iterable[Fix]:
    connections = [connection for connection in reader.connections if connection.topic == topic]
    if not connections:
        available = ", ".join(sorted({connection.topic for connection in reader.connections}))
        raise SystemExit(f"未找到话题 {topic}；可用话题：{available}")
    for connection, timestamp, rawdata in reader.messages(connections=connections):
        msg = reader.deserialize(rawdata, connection.msgtype)
        covariance = msg.position_covariance
        yield Fix(
            bag_time_ns=int(timestamp),
            header_time_ns=stamp_ns(getattr(msg.header, "stamp", None)),
            latitude=float(msg.latitude),
            longitude=float(msg.longitude),
            altitude=float(msg.altitude),
            status=int(msg.status.status),
            covariance_type=int(msg.position_covariance_type),
            covariance_e=float(covariance[0]),
            covariance_n=float(covariance[4]),
            covariance_u=float(covariance[8]),
        )


def main() -> int:
    args = parse_args()
    source = args.input.expanduser().resolve()
    target = args.output.expanduser().resolve()
    metadata_path = args.metadata.expanduser().resolve() if args.metadata else None
    if not source.exists():
        raise SystemExit(f"输入不存在：{source}")
    if args.max_horizontal_sigma <= 0:
        raise SystemExit("--max-horizontal-sigma 必须大于 0")

    with AnyReader([source]) as reader:
        fixes = list(iter_fixes(reader, args.topic))

    rejected: Counter[str] = Counter()
    accepted: list[Fix] = []
    for fix in fixes:
        reason = rejection_reason(fix, args.min_status, args.max_horizontal_sigma)
        if reason:
            rejected[reason] += 1
        else:
            accepted.append(fix)
    if not accepted:
        raise SystemExit("没有 RTK 点通过质量过滤，请检查阈值或输入数据")

    origin = tuple(args.origin) if args.origin else (
        accepted[0].latitude,
        accepted[0].longitude,
        accepted[0].altitude,
    )
    origin_ecef = geodetic_to_ecef(*origin)
    rows: list[dict[str, int | float | None]] = []
    for fix in accepted:
        enu = ecef_to_enu(
            geodetic_to_ecef(fix.latitude, fix.longitude, fix.altitude),
            origin_ecef,
            origin[0],
            origin[1],
        )
        horizontal_variance = max(fix.covariance_e, fix.covariance_n)
        rows.append(
            {
                "bag_time_ns": fix.bag_time_ns,
                "header_time_ns": fix.header_time_ns,
                "latitude_deg": fix.latitude,
                "longitude_deg": fix.longitude,
                "altitude_m": fix.altitude,
                "east_m": float(enu[0]),
                "north_m": float(enu[1]),
                "up_m": float(enu[2]),
                "status": fix.status,
                "covariance_type": fix.covariance_type,
                "sigma_e_m": math.sqrt(fix.covariance_e),
                "sigma_n_m": math.sqrt(fix.covariance_n),
                "sigma_u_m": math.sqrt(fix.covariance_u) if fix.covariance_u >= 0 else math.nan,
                "horizontal_weight": 1.0 / horizontal_variance,
            }
        )

    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    enu_array = np.asarray([[row["east_m"], row["north_m"], row["up_m"]] for row in rows])
    summary = {
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "input": str(source),
        "topic": args.topic,
        "filter": {
            "min_status": args.min_status,
            "max_horizontal_sigma_m": args.max_horizontal_sigma,
            "unknown_covariance_is_rejected": True,
        },
        "origin_wgs84": {"latitude": origin[0], "longitude": origin[1], "altitude": origin[2]},
        "input_count": len(fixes),
        "accepted_count": len(accepted),
        "rejected_count": len(fixes) - len(accepted),
        "rejected_by_reason": dict(sorted(rejected.items())),
        "status_counts_accepted": dict(sorted(Counter(fix.status for fix in accepted).items())),
        "bounds_enu_m": {
            axis: {"min": float(enu_array[:, index].min()), "max": float(enu_array[:, index].max())}
            for index, axis in enumerate(("east", "north", "up"))
        },
        "first_fix": asdict(accepted[0]),
        "output": str(target),
    }
    if metadata_path:
        metadata_path.parent.mkdir(parents=True, exist_ok=True)
        metadata_path.write_text(
            json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )

    print(
        f"已导出 {len(accepted)}/{len(fixes)} 个 RTK 点到 {target}；"
        f"过滤 {len(fixes) - len(accepted)} 个点。"
    )
    if metadata_path:
        print(f"处理摘要：{metadata_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
