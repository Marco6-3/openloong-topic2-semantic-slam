from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
import yaml

from scripts.create_semantic_map import classify_geometry
from scripts.package_submission import main as package_submission
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


def test_semantic_pcd_validation_checks_confidence_and_labels(tmp_path: Path) -> None:
    points = np.asarray(
        [(1.0, 2.0, 3.0, 1, 0.25, 0), (2.0, 3.0, 4.0, 9, 0.75, 0)],
        dtype=[
            ("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("label", "<u2"),
            ("confidence", "<f4"), ("rgb", "<u4"),
        ],
    )
    path = tmp_path / "semantic.pcd"
    write_pcd(path, points)
    report = validate_pcd(path, ("label", "confidence", "rgb"))
    assert report["label_counts"] == {"1": 1, "9": 1}
    assert report["confidence"] == {"min": 0.25, "max": 0.75, "mean": 0.5}


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


def test_submission_package_requires_fresh_requirements_confirmation(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setattr(
        "sys.argv",
        [
            "package_submission.py",
            "--team",
            "占位队名",
            "--leader",
            "占位姓名",
            "--phone",
            "13800000000",
            "--output",
            str(tmp_path / "submission.zip"),
        ],
    )
    with pytest.raises(SystemExit, match="--requirements-confirmed"):
        package_submission()
