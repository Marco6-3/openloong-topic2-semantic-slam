#!/usr/bin/env python3
"""Pure helpers shared by the ROS semantic mapper and unit tests."""

from collections.abc import MutableMapping, Sequence
from typing import Any, Literal

import numpy as np


CameraConvention = Literal["auto", "body", "optical"]
UpdateResult = Literal["inserted", "updated", "unchanged", "rejected"]


def normalize_frame_id(frame_id: str) -> str:
    """Return a tf2-compatible frame id without a leading slash."""
    return str(frame_id).strip().lstrip("/")


def resolve_camera_convention(frame_id: str, requested: str = "auto") -> Literal["body", "optical"]:
    """Resolve whether points use a body frame or ROS optical-frame axes."""
    convention = str(requested).strip().lower()
    if convention not in {"auto", "body", "optical"}:
        raise ValueError(
            f"camera frame convention must be auto, body or optical, got {requested!r}"
        )
    if convention == "auto":
        return "optical" if "optical" in normalize_frame_id(frame_id).lower() else "body"
    return convention


def project_points(
    xyz_camera: np.ndarray,
    intrinsic: Sequence[float],
    image_shape: Sequence[int],
    *,
    frame_id: str = "",
    convention: CameraConvention = "auto",
    min_depth: float = 0.10,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, Literal["body", "optical"]]:
    """Project 3-D points in a camera-related frame into image pixels.

    ``body`` follows the simulator convention: +X forward, +Y left, +Z up.
    ``optical`` follows REP-103 camera optical axes: +X right, +Y down, +Z forward.
    """
    xyz = np.asarray(xyz_camera, dtype=np.float32)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError(f"xyz_camera must have shape (N, 3), got {xyz.shape}")
    if len(intrinsic) != 9:
        raise ValueError("camera intrinsic matrix must contain 9 values")
    if len(image_shape) < 2:
        raise ValueError("image_shape must contain height and width")
    if min_depth <= 0.0:
        raise ValueError("min_depth must be positive")

    matrix = np.asarray(intrinsic, dtype=np.float64).reshape(3, 3)
    fx, fy = float(matrix[0, 0]), float(matrix[1, 1])
    cx, cy = float(matrix[0, 2]), float(matrix[1, 2])
    if not np.all(np.isfinite([fx, fy, cx, cy])) or fx <= 0.0 or fy <= 0.0:
        raise ValueError("camera intrinsics must contain finite positive fx and fy")

    resolved = resolve_camera_convention(frame_id, convention)
    if resolved == "optical":
        horizontal = xyz[:, 0]
        vertical = xyz[:, 1]
        depth = xyz[:, 2]
    else:
        horizontal = -xyz[:, 1]
        vertical = -xyz[:, 2]
        depth = xyz[:, 0]

    finite = np.all(np.isfinite(xyz), axis=1) & np.isfinite(depth) & (depth > min_depth)
    candidate_indices = np.flatnonzero(finite)
    if candidate_indices.size == 0:
        empty = np.empty(0, dtype=np.int32)
        return empty, empty.copy(), empty.copy(), resolved

    candidate_depth = depth[candidate_indices]
    u_float = cx + fx * horizontal[candidate_indices] / candidate_depth
    v_float = cy + fy * vertical[candidate_indices] / candidate_depth
    height, width = int(image_shape[0]), int(image_shape[1])
    inside = (
        np.isfinite(u_float)
        & np.isfinite(v_float)
        & (u_float >= -0.5)
        & (u_float < width - 0.5)
        & (v_float >= -0.5)
        & (v_float < height - 0.5)
    )
    point_indices = candidate_indices[inside].astype(np.int32, copy=False)
    u = np.rint(u_float[inside]).astype(np.int32)
    v = np.rint(v_float[inside]).astype(np.int32)
    return point_indices, u, v, resolved


def update_geometry_voxel(
    store: MutableMapping[Any, Any], key: Any, value: Any, max_voxels: int
) -> UpdateResult:
    """Insert a new geometry voxel or refresh an existing one at capacity."""
    if max_voxels <= 0:
        raise ValueError("max_voxels must be positive")
    existed = key in store
    if not existed and len(store) >= max_voxels:
        return "rejected"
    store[key] = value
    return "updated" if existed else "inserted"


def update_semantic_voxel(
    store: MutableMapping[Any, Any],
    key: Any,
    value: Any,
    max_voxels: int,
    *,
    confidence_index: int = 3,
) -> UpdateResult:
    """Keep the highest-confidence semantic observation while respecting capacity."""
    if max_voxels <= 0:
        raise ValueError("max_voxels must be positive")
    previous = store.get(key)
    if previous is None:
        if len(store) >= max_voxels:
            return "rejected"
        store[key] = value
        return "inserted"

    if float(value[confidence_index]) >= float(previous[confidence_index]):
        store[key] = value
        return "updated"
    return "unchanged"
