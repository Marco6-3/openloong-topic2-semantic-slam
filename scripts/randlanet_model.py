#!/usr/bin/env python3
"""RandLA-Net SemanticKITTI 推理模型与确定性点云层级构造。

网络结构按 RandLA-Net 论文及 Open3D-ML 的 MIT 许可实现重写，参数命名保持与
Open3D-ML 发布的 SemanticKITTI 检查点兼容。这里只包含推理所需组件。上游版权
归 Open3D (www.open3d.org) 所有，完整许可见 third_party/licenses/Open3D-ML-LICENSE。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
import torch
from torch import nn
import torch.nn.functional as functional


NUM_CLASSES = 19
NUM_POINTS = 45_056
K_NEIGHBORS = 16
SUB_SAMPLING_RATIOS = (4, 4, 4, 4)
DIM_OUTPUT = (16, 64, 128, 256)


class SharedMLP(nn.Module):
    def __init__(
        self,
        in_channels: int,
        out_channels: int,
        *,
        bn: bool = True,
        activation_fn: nn.Module | None = None,
        transpose: bool = False,
    ) -> None:
        convolution = nn.ConvTranspose2d if transpose else nn.Conv2d
        super().__init__()
        self.conv = convolution(in_channels, out_channels, kernel_size=1)
        self.batch_norm = (
            nn.BatchNorm2d(out_channels, eps=1e-6, momentum=0.01) if bn else None
        )
        self.activation_fn = activation_fn

    def forward(self, inputs: torch.Tensor) -> torch.Tensor:
        outputs = self.conv(inputs)
        if self.batch_norm is not None:
            outputs = self.batch_norm(outputs)
        if self.activation_fn is not None:
            outputs = self.activation_fn(outputs)
        return outputs


class LocalSpatialEncoding(nn.Module):
    def __init__(self, d_in: int, d_out: int, num_neighbors: int, *, encode_pos: bool) -> None:
        super().__init__()
        self.num_neighbors = num_neighbors
        self.mlp = SharedMLP(d_in, d_out, activation_fn=nn.LeakyReLU(0.2))
        self.encode_pos = encode_pos

    @staticmethod
    def gather_neighbor(coords: torch.Tensor, neighbor_indices: torch.Tensor) -> torch.Tensor:
        batch_size, num_points, num_neighbors = neighbor_indices.size()
        dimensions = coords.shape[2]
        extended_indices = neighbor_indices.unsqueeze(1).expand(
            batch_size, dimensions, num_points, num_neighbors
        )
        extended_coords = coords.transpose(-2, -1).unsqueeze(-1).expand(
            batch_size, dimensions, num_points, num_neighbors
        )
        return torch.gather(extended_coords, 2, extended_indices)

    def forward(
        self,
        coords: torch.Tensor,
        features: torch.Tensor,
        neighbor_indices: torch.Tensor,
        relative_features: torch.Tensor | None = None,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        batch_size, num_points, num_neighbors = neighbor_indices.size()
        if self.encode_pos:
            neighbor_xyz = self.gather_neighbor(coords, neighbor_indices)
            xyz = coords.transpose(-2, -1).unsqueeze(-1).expand(
                batch_size, 3, num_points, num_neighbors
            )
            relative_xyz = xyz - neighbor_xyz
            relative_distance = torch.sqrt(torch.sum(relative_xyz**2, dim=1, keepdim=True))
            relative_features = torch.cat(
                [relative_distance, relative_xyz, xyz, neighbor_xyz], dim=1
            )
        elif relative_features is None:
            raise ValueError("LocalSpatialEncoding 第二遍需要第一遍的相对特征")
        encoded = self.mlp(relative_features)
        neighbor_features = self.gather_neighbor(
            features.transpose(1, 2).squeeze(3), neighbor_indices
        )
        return torch.cat([neighbor_features, encoded], dim=1), encoded


class AttentivePooling(nn.Module):
    def __init__(self, in_channels: int, out_channels: int) -> None:
        super().__init__()
        self.score_fn = nn.Sequential(nn.Linear(in_channels, in_channels), nn.Softmax(dim=-2))
        self.mlp = SharedMLP(in_channels, out_channels, activation_fn=nn.LeakyReLU(0.2))

    def forward(self, features: torch.Tensor) -> torch.Tensor:
        scores = self.score_fn(features.permute(0, 2, 3, 1)).permute(0, 3, 1, 2)
        pooled = torch.sum(scores * features, dim=-1, keepdim=True)
        return self.mlp(pooled)


class LocalFeatureAggregation(nn.Module):
    def __init__(self, d_in: int, d_out: int, num_neighbors: int) -> None:
        super().__init__()
        self.mlp1 = SharedMLP(d_in, d_out // 2, activation_fn=nn.LeakyReLU(0.2))
        self.lse1 = LocalSpatialEncoding(10, d_out // 2, num_neighbors, encode_pos=True)
        self.pool1 = AttentivePooling(d_out, d_out // 2)
        self.lse2 = LocalSpatialEncoding(
            d_out // 2, d_out // 2, num_neighbors, encode_pos=False
        )
        self.pool2 = AttentivePooling(d_out, d_out)
        self.mlp2 = SharedMLP(d_out, 2 * d_out)
        self.shortcut = SharedMLP(d_in, 2 * d_out)
        self.lrelu = nn.LeakyReLU()

    def forward(
        self, coords: torch.Tensor, features: torch.Tensor, neighbor_indices: torch.Tensor
    ) -> torch.Tensor:
        aggregated = self.mlp1(features)
        aggregated, relative = self.lse1(coords, aggregated, neighbor_indices)
        aggregated = self.pool1(aggregated)
        aggregated, _ = self.lse2(coords, aggregated, neighbor_indices, relative)
        aggregated = self.pool2(aggregated)
        return self.lrelu(self.mlp2(aggregated) + self.shortcut(features))


class RandLANet(nn.Module):
    def __init__(self, device: torch.device) -> None:
        super().__init__()
        self.device = device
        self.fc0 = nn.Linear(3, 8)
        self.bn0 = nn.BatchNorm2d(8, eps=1e-6, momentum=0.01)
        self.encoder = nn.ModuleList()
        d_in = 8
        for d_out in DIM_OUTPUT:
            self.encoder.append(LocalFeatureAggregation(d_in, d_out, K_NEIGHBORS))
            d_in = 2 * d_out
        self.mlp = SharedMLP(d_in, d_in, activation_fn=nn.LeakyReLU(0.2))
        self.decoder = nn.ModuleList()
        decoder_in = [768, 384, 160, 64]
        decoder_out = [256, 128, 32, 32]
        for in_channels, out_channels in zip(decoder_in, decoder_out, strict=True):
            self.decoder.append(
                SharedMLP(
                    in_channels,
                    out_channels,
                    bn=True,
                    activation_fn=nn.LeakyReLU(0.2),
                    transpose=True,
                )
            )
        self.fc1 = nn.Sequential(
            SharedMLP(2 * DIM_OUTPUT[0], 64, activation_fn=nn.LeakyReLU(0.2)),
            SharedMLP(64, 32, activation_fn=nn.LeakyReLU(0.2)),
            nn.Dropout(0.5),
            SharedMLP(32, NUM_CLASSES, bn=False),
        )

    @staticmethod
    def random_sample(features: torch.Tensor, pool_indices: torch.Tensor) -> torch.Tensor:
        batch_size, channels = features.shape[:2]
        pool_indices = pool_indices.reshape(batch_size, -1)
        sampled = torch.gather(
            features.squeeze(-1),
            2,
            pool_indices.unsqueeze(1).expand(-1, channels, -1),
        )
        sampled = sampled.reshape(batch_size, channels, -1, K_NEIGHBORS)
        return sampled.max(dim=3, keepdim=True)[0]

    @staticmethod
    def nearest_interpolation(features: torch.Tensor, interp_indices: torch.Tensor) -> torch.Tensor:
        batch_size, channels = features.shape[:2]
        interp_indices = interp_indices.reshape(batch_size, -1)
        interpolated = torch.gather(
            features.squeeze(-1),
            2,
            interp_indices.unsqueeze(1).expand(-1, channels, -1),
        )
        return interpolated.unsqueeze(3)

    def forward(self, inputs: dict[str, list[torch.Tensor] | torch.Tensor]) -> torch.Tensor:
        features = self.fc0(inputs["features"])  # type: ignore[index]
        features = nn.LeakyReLU(0.2)(
            self.bn0(features.transpose(-2, -1).unsqueeze(-1))
        )
        encoder_features: list[torch.Tensor] = []
        coords = inputs["coords"]  # type: ignore[assignment]
        neighbor_indices = inputs["neighbor_indices"]  # type: ignore[assignment]
        subsampling_indices = inputs["subsampling_indices"]  # type: ignore[assignment]
        interpolation_indices = inputs["interpolation_indices"]  # type: ignore[assignment]
        for layer_index in range(len(self.encoder)):
            encoded = self.encoder[layer_index](
                coords[layer_index], features, neighbor_indices[layer_index]  # type: ignore[index]
            )
            features = self.random_sample(encoded, subsampling_indices[layer_index])  # type: ignore[index]
            if layer_index == 0:
                encoder_features.append(encoded)
            encoder_features.append(features)
        features = self.mlp(encoder_features[-1])
        for layer_index in range(len(self.decoder)):
            interpolated = self.nearest_interpolation(
                features, interpolation_indices[-layer_index - 1]  # type: ignore[index]
            )
            features = self.decoder[layer_index](
                torch.cat([encoder_features[-layer_index - 2], interpolated], dim=1)
            )
        return self.fc1(features).squeeze(-1).transpose(1, 2)


@dataclass(frozen=True)
class PreparedCloud:
    inputs: dict[str, list[torch.Tensor] | torch.Tensor]
    selected_indices: np.ndarray


def _knn(tree_points: np.ndarray, query_points: np.ndarray, k: int) -> np.ndarray:
    actual_k = min(k, len(tree_points))
    indices = cKDTree(tree_points).query(query_points, k=actual_k, workers=-1)[1]
    if actual_k == 1:
        indices = np.asarray(indices)[:, None]
    if actual_k < k:
        indices = np.pad(indices, ((0, 0), (0, k - actual_k)), mode="edge")
    return np.asarray(indices, dtype=np.int64)


def prepare_cloud(
    points: np.ndarray,
    device: torch.device,
    *,
    num_points: int = NUM_POINTS,
    seed: int = 0,
) -> PreparedCloud:
    """抽样并构建 RandLA-Net 的 4 层 KNN/随机下采样输入。"""
    points = np.asarray(points, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) == 0:
        raise ValueError("输入点云必须为非空 Nx3 数组")
    if not np.all(np.isfinite(points)):
        raise ValueError("输入点云包含非有限值")
    if num_points < 256:
        raise ValueError("num_points 必须至少为 256")
    rng = np.random.default_rng(seed)
    if len(points) >= num_points:
        selected = rng.choice(len(points), size=num_points, replace=False)
    else:
        extra = rng.choice(len(points), size=num_points - len(points), replace=True)
        selected = np.concatenate([np.arange(len(points), dtype=np.int64), extra])
        rng.shuffle(selected)
    selected = np.asarray(selected, dtype=np.int64)
    sampled = points[selected].copy()
    sampled[:, :2] -= sampled[:, :2].mean(axis=0, keepdims=True)

    coordinates: list[np.ndarray] = []
    neighbors: list[np.ndarray] = []
    pools: list[np.ndarray] = []
    interpolation: list[np.ndarray] = []
    current = sampled
    for ratio in SUB_SAMPLING_RATIOS:
        neighbor = _knn(current, current, K_NEIGHBORS)
        sub_count = max(1, len(current) // ratio)
        sub_points = current[:sub_count]
        pool = neighbor[:sub_count]
        up = _knn(sub_points, current, 1)
        coordinates.append(current)
        neighbors.append(neighbor)
        pools.append(pool)
        interpolation.append(up)
        current = sub_points

    def tensors(values: list[np.ndarray]) -> list[torch.Tensor]:
        return [torch.from_numpy(value).unsqueeze(0).to(device) for value in values]

    return PreparedCloud(
        inputs={
            "coords": tensors(coordinates),
            "neighbor_indices": tensors(neighbors),
            "subsampling_indices": tensors(pools),
            "interpolation_indices": tensors(interpolation),
            "features": torch.from_numpy(sampled).unsqueeze(0).to(device),
        },
        selected_indices=selected,
    )


def load_pretrained(weights: Path, device: torch.device) -> RandLANet:
    checkpoint = torch.load(weights, map_location="cpu", weights_only=False)
    state = checkpoint.get("model_state_dict", checkpoint)
    model = RandLANet(device)
    model.load_state_dict(state, strict=True)
    return model.to(device).eval()


@torch.inference_mode()
def predict_probabilities(model: RandLANet, prepared: PreparedCloud) -> np.ndarray:
    logits = model(prepared.inputs)
    return functional.softmax(logits, dim=-1).squeeze(0).cpu().numpy().astype(np.float32)
