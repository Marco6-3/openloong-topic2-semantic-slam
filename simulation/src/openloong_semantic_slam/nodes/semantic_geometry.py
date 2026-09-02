#!/usr/bin/env python3
"""Pure geometry helpers for camera--LiDAR semantic association."""

from __future__ import annotations

import cv2
import numpy as np


def quaternion_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = x * x + y * y + z * z + w * w
    if norm < 1e-12:
        return np.eye(3, dtype=np.float32)
    scale = 2.0 / norm
    return np.asarray(
        [
            [1.0 - scale * (y * y + z * z), scale * (x * y - z * w), scale * (x * z + y * w)],
            [scale * (x * y + z * w), 1.0 - scale * (x * x + z * z), scale * (y * z - x * w)],
            [scale * (x * z - y * w), scale * (y * z + x * w), 1.0 - scale * (x * x + y * y)],
        ],
        dtype=np.float32,
    )


def transform_points(xyz: np.ndarray, transform) -> np.ndarray:
    """Apply a geometry_msgs TransformStamped-compatible transform."""
    q = transform.transform.rotation
    rotation = quaternion_matrix(q.x, q.y, q.z, q.w)
    t = transform.transform.translation
    translation = np.asarray((t.x, t.y, t.z), dtype=np.float32)
    return xyz @ rotation.T + translation


def erode_mask(mask: np.ndarray, pixels: int) -> np.ndarray:
    if pixels <= 0:
        return np.asarray(mask, dtype=bool)
    size = pixels * 2 + 1
    kernel = np.ones((size, size), dtype=np.uint8)
    return cv2.erode(np.asarray(mask, dtype=np.uint8), kernel, iterations=1).astype(bool)


def nearest_depth_cluster(
    depths: np.ndarray,
    absolute_gap_m: float,
    relative_gap: float,
    minimum_support: int,
) -> np.ndarray:
    """Keep the nearest supported surface before a clear depth discontinuity."""
    keep = np.ones(depths.shape[0], dtype=bool)
    if depths.shape[0] < minimum_support * 2:
        return keep
    order = np.argsort(depths)
    ordered = depths[order]
    gaps = np.diff(ordered)
    for split in range(minimum_support, ordered.shape[0] - minimum_support + 1):
        threshold = max(absolute_gap_m, relative_gap * float(ordered[split - 1]))
        if gaps[split - 1] > threshold:
            keep[:] = False
            keep[order[:split]] = True
            break
    return keep


def associate_semantics(
    xyz_camera: np.ndarray,
    detections: list[dict],
    intrinsics: tuple[float, float, float, float],
    image_shape: tuple[int, ...],
    *,
    mask_erosion_pixels: int = 1,
    depth_gap_m: float = 0.75,
    depth_gap_ratio: float = 0.12,
    depth_min_support: int = 3,
    minimum_point_confidence: float = 0.16,
) -> tuple[
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    dict[str, int],
]:
    """Project points and associate only mask-interior, depth-consistent returns."""
    point_count = xyz_camera.shape[0]
    labels = np.zeros(point_count, dtype=np.uint16)
    confidence = np.zeros(point_count, dtype=np.float32)
    colors = np.full((point_count, 3), 150, dtype=np.uint8)
    instance_ids = np.full(point_count, -1, dtype=np.int16)
    visible = np.zeros(point_count, dtype=bool)
    dynamic_exclusion = np.zeros(point_count, dtype=bool)
    if point_count == 0:
        return (
            labels,
            confidence,
            colors,
            instance_ids,
            visible,
            dynamic_exclusion,
            {"projected": 0},
        )

    fx, fy, cx, cy = intrinsics
    forward = xyz_camera[:, 0]
    candidates = np.nonzero(forward > 0.10)[0]
    u = np.rint(cx - fx * xyz_camera[candidates, 1] / forward[candidates]).astype(np.int32)
    v = np.rint(cy - fy * xyz_camera[candidates, 2] / forward[candidates]).astype(np.int32)
    height, width = image_shape[:2]
    inside = (u >= 0) & (u < width) & (v >= 0) & (v < height)
    point_indices = candidates[inside]
    u, v = u[inside], v[inside]
    visible[point_indices] = True

    raw_matches = 0
    eroded_matches = 0
    depth_rejected = 0
    low_confidence_rejected = 0
    for detection_index, detection in enumerate(detections):
        raw_mask = np.asarray(detection["mask"], dtype=bool)
        raw_matched = raw_mask[v, u]
        raw_matches += int(np.count_nonzero(raw_matched))
        if detection.get("dynamic", False) and np.any(raw_matched):
            raw_indices = point_indices[raw_matched]
            raw_depth_keep = nearest_depth_cluster(
                forward[raw_indices], depth_gap_m, depth_gap_ratio, depth_min_support
            )
            dynamic_exclusion[raw_indices[raw_depth_keep]] = True
        safe_mask = erode_mask(raw_mask, mask_erosion_pixels)
        matched = safe_mask[v, u]
        matched_indices = point_indices[matched]
        if matched_indices.size == 0:
            continue
        eroded_matches += int(matched_indices.size)

        depths = forward[matched_indices]
        depth_keep = nearest_depth_cluster(
            depths, depth_gap_m, depth_gap_ratio, depth_min_support
        )
        depth_rejected += int(np.count_nonzero(~depth_keep))
        matched_indices = matched_indices[depth_keep]
        matched_u, matched_v = u[matched][depth_keep], v[matched][depth_keep]
        if matched_indices.size == 0:
            continue

        mask_probability = detection.get("mask_probability")
        if mask_probability is None:
            pixel_probability = np.ones(matched_indices.size, dtype=np.float32)
        else:
            pixel_probability = np.asarray(mask_probability, dtype=np.float32)[matched_v, matched_u]
        point_score = float(detection["score"]) * pixel_probability
        score_keep = point_score >= minimum_point_confidence
        low_confidence_rejected += int(np.count_nonzero(~score_keep))
        matched_indices = matched_indices[score_keep]
        point_score = point_score[score_keep]
        if matched_indices.size == 0:
            continue

        # Overlapping instances compete per point using mask-aware confidence.
        better = point_score > confidence[matched_indices]
        matched_indices = matched_indices[better]
        point_score = point_score[better]
        labels[matched_indices] = int(detection["class_id"]) + 1
        confidence[matched_indices] = point_score
        colors[matched_indices] = np.asarray(detection["rgb"], dtype=np.uint8)
        instance_ids[matched_indices] = detection_index

    return labels, confidence, colors, instance_ids, visible, dynamic_exclusion, {
        "projected": int(point_indices.size),
        "raw_mask_matches": raw_matches,
        "eroded_mask_matches": eroded_matches,
        "boundary_rejected": raw_matches - eroded_matches,
        "depth_rejected": depth_rejected,
        "low_confidence_rejected": low_confidence_rejected,
        "associated": int(np.count_nonzero(labels)),
    }
