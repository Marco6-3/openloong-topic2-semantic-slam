import importlib.util
import sys
from pathlib import Path

import numpy as np


MODULE_PATH = (
    Path(__file__).parents[1]
    / "simulation/src/openloong_semantic_slam/nodes/semantic_fusion.py"
)
SPEC = importlib.util.spec_from_file_location("semantic_fusion", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)
TemporalVoxelFusion = MODULE.TemporalVoxelFusion


def frame(label, confidence=0.7, duplicate_points=1):
    xyz = np.repeat([[0.04, 0.04, 0.04]], duplicate_points, axis=0).astype(np.float32)
    labels = np.full(duplicate_points, label, dtype=np.uint16)
    scores = np.full(duplicate_points, confidence, dtype=np.float32)
    colors = np.repeat([[10 * label, 20, 30]], duplicate_points, axis=0).astype(np.uint8)
    return xyz, labels, scores, colors


def test_transient_voxel_is_not_published_until_seen_in_two_frames():
    fusion = TemporalVoxelFusion(0.1, 100, min_observations=2)
    stats = fusion.update(*frame(1, duplicate_points=20))
    assert stats["candidate_voxels"] == 1
    assert stats["stable_voxels"] == 0
    assert fusion.snapshot()[0].shape == (0, 3)

    stats = fusion.update(*frame(1))
    assert stats["stable_voxels"] == 1
    assert fusion.snapshot()[2].tolist() == [1]


def test_consistent_multiframe_evidence_beats_single_high_confidence_outlier():
    fusion = TemporalVoxelFusion(0.1, 100, min_observations=2, evidence_decay=1.0)
    fusion.update(*frame(2, confidence=0.99))
    for _ in range(3):
        fusion.update(*frame(1, confidence=0.70))
    _, _, labels, confidence = fusion.snapshot()
    assert labels.tolist() == [1]
    assert confidence[0] > 0.55


def test_dynamic_labels_are_excluded_from_persistent_candidates():
    fusion = TemporalVoxelFusion(0.1, 100)
    stats = fusion.update(*frame(3), excluded_labels={3})
    assert stats["candidate_voxels"] == 0


def test_visible_misses_remove_a_previously_stable_false_positive():
    fusion = TemporalVoxelFusion(
        0.1,
        100,
        min_observations=2,
        negative_evidence_decay=0.5,
        max_consecutive_misses=3,
    )
    positive = frame(1)
    fusion.update(*positive, observed_mask=np.asarray([True]))
    fusion.update(*positive, observed_mask=np.asarray([True]))
    assert fusion.stable_count() == 1

    xyz, _labels, _scores, colors = positive
    empty_labels = np.zeros(1, dtype=np.uint16)
    empty_scores = np.zeros(1, dtype=np.float32)
    for _ in range(3):
        stats = fusion.update(
            xyz, empty_labels, empty_scores, colors, observed_mask=np.asarray([True])
        )
    assert stats["removed_voxels"] == 1
    assert stats["candidate_voxels"] == 0
    assert fusion.stable_count() == 0


def test_voxels_outside_camera_view_are_not_treated_as_negative_evidence():
    fusion = TemporalVoxelFusion(0.1, 100, min_observations=2)
    positive = frame(1)
    fusion.update(*positive, observed_mask=np.asarray([True]))
    fusion.update(*positive, observed_mask=np.asarray([True]))
    for _ in range(20):
        fusion.update(*positive, observed_mask=np.asarray([False]))
    assert fusion.stable_count() == 1
