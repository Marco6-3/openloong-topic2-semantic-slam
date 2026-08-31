from __future__ import annotations

import math

import numpy as np

from scripts.fuse_trajectory import robust_planar_alignment, smooth, yaw_multiply


def test_robust_planar_alignment_handles_outlier() -> None:
    source = np.asarray([[x, y] for x in range(5) for y in range(4)], dtype=float)
    yaw = math.radians(27.0)
    expected_rotation = np.asarray(
        [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
    )
    expected_translation = np.asarray([12.0, -3.5])
    target = source @ expected_rotation.T + expected_translation
    target[-1] += [50.0, -40.0]

    rotation, translation, _ = robust_planar_alignment(source, target)

    np.testing.assert_allclose(rotation, expected_rotation, atol=0.02)
    np.testing.assert_allclose(translation, expected_translation, atol=0.1)


def test_smooth_preserves_length_and_constant_values() -> None:
    values = np.full((9, 3), [1.0, 2.0, 3.0])
    result = smooth(values, 5)
    assert result.shape == values.shape
    np.testing.assert_allclose(result, values)


def test_yaw_multiply_rotates_identity_quaternion() -> None:
    result = yaw_multiply(np.asarray([[0.0, 0.0, 0.0, 1.0]]), math.pi / 2)
    np.testing.assert_allclose(result, [[0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)]])
