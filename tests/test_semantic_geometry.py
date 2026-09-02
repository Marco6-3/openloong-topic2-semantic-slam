import importlib.util
from pathlib import Path

import numpy as np


MODULE_PATH = (
    Path(__file__).parents[1]
    / "simulation/src/openloong_semantic_slam/nodes/semantic_geometry.py"
)
SPEC = importlib.util.spec_from_file_location("semantic_geometry", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def detection(mask, probability=None, score=0.9, class_id=0):
    return {
        "class_id": class_id,
        "score": score,
        "mask": mask,
        "mask_probability": probability,
        "rgb": (255, 0, 0),
    }


def test_depth_discontinuity_rejects_background_behind_an_instance_mask():
    mask = np.zeros((21, 21), dtype=bool)
    mask[8:13, 8:13] = True
    probability = mask.astype(np.float32)
    # Both returns project to the same pixel, but only the 2 m foreground
    # surface belongs to the detected object.
    xyz = np.asarray([[2.0, 0.0, 0.0], [6.0, 0.0, 0.0]], dtype=np.float32)
    labels, confidence, _colors, _instances, visible, _dynamic, stats = MODULE.associate_semantics(
        xyz,
        [detection(mask, probability)],
        (10.0, 10.0, 10.0, 10.0),
        mask.shape,
        mask_erosion_pixels=1,
        depth_gap_m=0.75,
        depth_gap_ratio=0.1,
        depth_min_support=1,
    )
    assert visible.tolist() == [True, True]
    assert labels.tolist() == [1, 0]
    assert confidence[0] > 0.8
    assert stats["depth_rejected"] == 1


def test_mask_erosion_rejects_boundary_bleed():
    mask = np.zeros((21, 21), dtype=bool)
    mask[9:12, 9:12] = True
    # u=9 lies on the left boundary and disappears after one-pixel erosion.
    xyz = np.asarray([[2.0, 0.2, 0.0]], dtype=np.float32)
    labels, _confidence, _colors, _instances, _visible, _dynamic, stats = MODULE.associate_semantics(
        xyz,
        [detection(mask)],
        (10.0, 10.0, 10.0, 10.0),
        mask.shape,
        mask_erosion_pixels=1,
    )
    assert labels.tolist() == [0]
    assert stats["boundary_rejected"] == 1


def test_mask_probability_resolves_overlapping_instances_per_point():
    mask = np.zeros((21, 21), dtype=bool)
    mask[8:13, 8:13] = True
    low = np.full(mask.shape, 0.55, dtype=np.float32)
    high = np.full(mask.shape, 0.90, dtype=np.float32)
    xyz = np.asarray([[2.0, 0.0, 0.0]], dtype=np.float32)
    labels, confidence, *_ = MODULE.associate_semantics(
        xyz,
        [detection(mask, low, score=0.9, class_id=0), detection(mask, high, score=0.8, class_id=1)],
        (10.0, 10.0, 10.0, 10.0),
        mask.shape,
        mask_erosion_pixels=0,
    )
    assert labels.tolist() == [2]
    assert np.isclose(confidence[0], 0.72)


def test_dynamic_mask_excludes_boundary_points_from_static_geometry():
    mask = np.zeros((21, 21), dtype=bool)
    mask[9:12, 9:12] = True
    item = detection(mask)
    item["dynamic"] = True
    xyz = np.asarray([[2.0, 0.2, 0.0]], dtype=np.float32)
    labels, _confidence, _colors, _instances, _visible, dynamic, _stats = (
        MODULE.associate_semantics(
            xyz,
            [item],
            (10.0, 10.0, 10.0, 10.0),
            mask.shape,
            mask_erosion_pixels=1,
        )
    )
    assert labels.tolist() == [0]
    assert dynamic.tolist() == [True]
