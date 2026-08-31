from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np
import pytest


HAS_TORCH = importlib.util.find_spec("torch") is not None
pytestmark = pytest.mark.skipif(not HAS_TORCH, reason="默认环境不安装 PyTorch")
if HAS_TORCH:
    import torch

    from scripts.create_model_semantic_map import (
        aggregate_prediction,
        merge_predictions,
        quaternion_matrix,
    )
    from scripts.randlanet_model import NUM_CLASSES, load_pretrained, prepare_cloud


def test_quaternion_matrix_rotates_body_to_world() -> None:
    identity = quaternion_matrix(0, 0, 0, 1)
    np.testing.assert_allclose(identity, np.eye(3))
    rotation = quaternion_matrix(0, 0, np.sqrt(0.5), np.sqrt(0.5))
    np.testing.assert_allclose(np.asarray([[1.0, 0.0, 0.0]]) @ rotation.T, [[0.0, 1.0, 0.0]], atol=1e-7)


def test_prediction_fusion_averages_each_window_once() -> None:
    points = np.asarray([[0.01, 0.01, 0.01], [0.02, 0.02, 0.02], [1.0, 0.0, 0.0]])
    probabilities = np.zeros((3, NUM_CLASSES), dtype=np.float32)
    probabilities[:, 0] = [1.0, 0.0, 0.25]
    probabilities[:, 1] = [0.0, 1.0, 0.75]
    first = aggregate_prediction(points, probabilities, 0.2)
    merged = merge_predictions([first, first])
    np.testing.assert_allclose(merged[1] / merged[2][:, None], first[1])
    np.testing.assert_array_equal(merged[2], [2, 2])


def test_prepare_cloud_builds_expected_hierarchy() -> None:
    rng = np.random.default_rng(3)
    points = rng.normal(size=(300, 3)).astype(np.float32)
    prepared = prepare_cloud(points, torch.device("cpu"), num_points=256, seed=3)
    assert prepared.inputs["features"].shape == (1, 256, 3)
    assert [tensor.shape[1] for tensor in prepared.inputs["coords"]] == [256, 64, 16, 4]
    assert all(tensor.shape[-1] == 16 for tensor in prepared.inputs["neighbor_indices"])


def test_pinned_checkpoint_loads_when_available() -> None:
    weights = Path("data/models/randlanet_semantickitti_202201071330utc.pth")
    if not weights.is_file():
        pytest.skip("模型权重不进入 Git，CI 未下载时跳过")
    model = load_pretrained(weights, torch.device("cpu"))
    assert sum(parameter.numel() for parameter in model.parameters()) == 1_242_307
