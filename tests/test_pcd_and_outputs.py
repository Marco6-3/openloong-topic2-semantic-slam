from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml

from scripts.create_semantic_map import classify_geometry
from scripts.pcd_io import read_pcd, write_pcd
from scripts.validate_path import validate as validate_path
from scripts.validate_pcd import validate as validate_pcd
from scripts.validate_video import parse_discovery


def test_binary_pcd_round_trip_and_validation(tmp_path: Path) -> None:
    points = np.asarray(
        [(1.0, 2.0, 3.0, 4.0), (-1.0, 0.0, 2.5, 8.0)],
        dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4")],
    )
    path = tmp_path / "map.pcd"
    write_pcd(path, points)
    metadata, recovered = read_pcd(path)
    assert metadata.points == 2
    np.testing.assert_array_equal(recovered, points)
    report = validate_pcd(path)
    assert report["points"] == 2


def test_path_validation(tmp_path: Path) -> None:
    path = tmp_path / "path.yaml"
    path.write_text(
        yaml.safe_dump(
            {
                "path": [
                    {"altitude": 1.0, "latitude": 31.0, "longitude": 121.0, "stamp": 10.0},
                    {"altitude": 1.1, "latitude": 31.1, "longitude": 121.1, "stamp": 10.1},
                ],
                "total_points": 2,
            }
        ),
        encoding="utf-8",
    )
    report = validate_path(path)
    assert report["total_points"] == 2


def test_geometry_classifier_uses_local_height() -> None:
    xyz = np.asarray(
        [[0.1, 0.1, 0.0], [0.2, 0.2, 0.1], [0.3, 0.3, 1.0], [0.4, 0.4, 2.1]]
    )
    labels, confidence = classify_geometry(xyz, 1.0, 0.25, 1.8, 2)
    np.testing.assert_array_equal(labels, [1, 1, 2, 3])
    assert np.all(confidence > 0)


def test_parse_video_discovery() -> None:
    report = parse_discovery(
        """Properties:
  Duration: 0:01:53.160737115
    video #1: VP8
      Width: 1453
      Height: 846
      Frame rate: 30/1
"""
    )
    assert abs(float(report["duration_seconds"]) - 113.160737115) < 1e-9
    assert report["codec"] == "VP8"
    assert report["frame_rate"] == "30/1"
