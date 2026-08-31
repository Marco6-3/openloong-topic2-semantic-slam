from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = (
    ROOT
    / "simulation"
    / "src"
    / "openloong_semantic_slam"
    / "nodes"
    / "semantic_mapper_core.py"
)
SPEC = importlib.util.spec_from_file_location("semantic_mapper_core", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
semantic_core = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(semantic_core)


def test_body_frame_projection_uses_tf_transformed_coordinates() -> None:
    xyz_camera = np.asarray(
        [
            [1.0, 0.0, 0.0],
            [1.0, -0.10, -0.20],
            [-1.0, 0.0, 0.0],
        ],
        dtype=np.float32,
    )
    indices, u, v, convention = semantic_core.project_points(
        xyz_camera,
        [100.0, 0.0, 50.0, 0.0, 100.0, 30.0, 0.0, 0.0, 1.0],
        (80, 100, 3),
        frame_id="camera",
        convention="auto",
    )
    assert convention == "body"
    np.testing.assert_array_equal(indices, [0, 1])
    np.testing.assert_array_equal(u, [50, 60])
    np.testing.assert_array_equal(v, [30, 50])


def test_optical_frame_projection_is_selected_automatically() -> None:
    indices, u, v, convention = semantic_core.project_points(
        np.asarray([[0.10, 0.20, 1.0]], dtype=np.float32),
        [100.0, 0.0, 50.0, 0.0, 100.0, 30.0, 0.0, 0.0, 1.0],
        (80, 100),
        frame_id="/camera_color_optical_frame",
    )
    assert convention == "optical"
    np.testing.assert_array_equal(indices, [0])
    np.testing.assert_array_equal(u, [60])
    np.testing.assert_array_equal(v, [50])
    assert semantic_core.normalize_frame_id("/camera") == "camera"


def test_geometry_voxel_refreshes_existing_key_after_capacity_is_reached() -> None:
    store = {"old": (1,)}
    assert semantic_core.update_geometry_voxel(store, "old", (2,), 1) == "updated"
    assert store["old"] == (2,)
    assert semantic_core.update_geometry_voxel(store, "new", (3,), 1) == "rejected"
    assert "new" not in store


def test_semantic_voxel_refreshes_existing_key_after_capacity_is_reached() -> None:
    store = {"old": (None, None, 1, 0.40)}
    assert (
        semantic_core.update_semantic_voxel(store, "old", (None, None, 1, 0.80), 1)
        == "updated"
    )
    assert store["old"][3] == pytest.approx(0.80)
    assert (
        semantic_core.update_semantic_voxel(store, "old", (None, None, 1, 0.20), 1)
        == "unchanged"
    )
    assert store["old"][3] == pytest.approx(0.80)
    assert (
        semantic_core.update_semantic_voxel(store, "new", (None, None, 2, 0.90), 1)
        == "rejected"
    )


def test_invalid_camera_convention_is_rejected() -> None:
    with pytest.raises(ValueError, match="camera frame convention"):
        semantic_core.resolve_camera_convention("camera", "sideways")
