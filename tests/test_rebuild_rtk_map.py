from __future__ import annotations

import numpy as np

from scripts.rebuild_rtk_map import aggregate_frame, merge_voxels


def test_voxel_aggregation_tracks_frames_and_centroids() -> None:
    first = np.asarray([[0.01, 0.01, 0.01, 1.0], [0.09, 0.09, 0.09, 3.0]])
    second = np.asarray([[0.02, 0.02, 0.02, 5.0], [1.0, 0.0, 0.0, 7.0]])
    merged = merge_voxels([aggregate_frame(first, 0.1), aggregate_frame(second, 0.1)])
    keys, sums, counts, observations = merged
    origin = np.flatnonzero(np.all(keys == [0, 0, 0], axis=1))[0]
    np.testing.assert_allclose(sums[origin] / counts[origin], [0.04, 0.04, 0.04, 3.0])
    assert counts[origin] == 3
    assert observations[origin] == 2
