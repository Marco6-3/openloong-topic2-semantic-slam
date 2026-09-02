#!/usr/bin/env python3
"""Bounded temporal evidence fusion for the real-time semantic voxel map.

This module deliberately has no ROS dependency so the fusion policy can be
regression-tested without starting Gazebo.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class VoxelEvidence:
    position: np.ndarray
    scores: dict[int, float] = field(default_factory=dict)
    colors: dict[int, tuple[int, int, int]] = field(default_factory=dict)
    observations: int = 0
    stable: bool = False


class TemporalVoxelFusion:
    """Fuse at most one observation per voxel and label from every frame."""

    def __init__(
        self,
        voxel_size: float,
        max_voxels: int,
        min_observations: int = 2,
        min_consensus: float = 0.55,
        evidence_decay: float = 0.95,
    ) -> None:
        if voxel_size <= 0.0:
            raise ValueError("voxel_size must be positive")
        if max_voxels <= 0:
            raise ValueError("max_voxels must be positive")
        if min_observations < 1:
            raise ValueError("min_observations must be at least one")
        if not 0.0 <= min_consensus <= 1.0:
            raise ValueError("min_consensus must be within [0, 1]")
        if not 0.0 < evidence_decay <= 1.0:
            raise ValueError("evidence_decay must be within (0, 1]")
        self.voxel_size = float(voxel_size)
        self.max_voxels = int(max_voxels)
        self.min_observations = int(min_observations)
        self.min_consensus = float(min_consensus)
        self.evidence_decay = float(evidence_decay)
        self.voxels: dict[tuple[int, int, int], VoxelEvidence] = {}
        self._stable_count = 0

    def update(
        self,
        xyz: np.ndarray,
        labels: np.ndarray,
        confidence: np.ndarray,
        colors: np.ndarray,
        excluded_labels: set[int] | None = None,
    ) -> dict[str, int]:
        """Add a frame, collapsing duplicate point evidence inside each voxel."""
        excluded = excluded_labels or set()
        frame_observations: dict[
            tuple[tuple[int, int, int], int], tuple[np.ndarray, tuple[int, int, int], float]
        ] = {}
        keys = np.floor(xyz / self.voxel_size).astype(np.int32)
        for index, key_array in enumerate(keys):
            label = int(labels[index])
            if label <= 0 or label in excluded:
                continue
            key = tuple(int(value) for value in key_array)
            observation_key = (key, label)
            score = float(confidence[index])
            previous = frame_observations.get(observation_key)
            # Multiple LiDAR returns in one frame are spatial support, not
            # independent temporal votes. Keep only their strongest evidence.
            if previous is None or score > previous[2]:
                frame_observations[observation_key] = (
                    xyz[index].copy(),
                    tuple(int(value) for value in colors[index]),
                    score,
                )

        observed_voxels: set[tuple[int, int, int]] = set()
        accepted = 0
        for (key, label), (position, color, score) in frame_observations.items():
            evidence = self.voxels.get(key)
            if evidence is None:
                if len(self.voxels) >= self.max_voxels:
                    continue
                evidence = VoxelEvidence(position=position)
                self.voxels[key] = evidence
            if key not in observed_voxels:
                for existing_label in list(evidence.scores):
                    evidence.scores[existing_label] *= self.evidence_decay
                    if evidence.scores[existing_label] < 1e-4:
                        del evidence.scores[existing_label]
                        evidence.colors.pop(existing_label, None)
                evidence.observations += 1
                evidence.position = evidence.position * 0.8 + position * 0.2
                observed_voxels.add(key)
            evidence.scores[label] = evidence.scores.get(label, 0.0) + score
            evidence.colors[label] = color
            accepted += 1
        for key in observed_voxels:
            evidence = self.voxels[key]
            stable = self.winner(evidence) is not None
            if stable != evidence.stable:
                self._stable_count += 1 if stable else -1
                evidence.stable = stable
        return {
            "frame_voxels": len(observed_voxels),
            "frame_label_votes": accepted,
            "candidate_voxels": len(self.voxels),
            "stable_voxels": self.stable_count(),
        }

    def winner(self, evidence: VoxelEvidence) -> tuple[int, float] | None:
        if evidence.observations < self.min_observations or not evidence.scores:
            return None
        label, score = max(evidence.scores.items(), key=lambda item: item[1])
        total = sum(evidence.scores.values())
        consensus = score / total if total > 0.0 else 0.0
        if consensus < self.min_consensus:
            return None
        return label, consensus

    def stable_count(self) -> int:
        return self._stable_count

    def snapshot(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        positions = []
        colors = []
        labels = []
        confidence = []
        for evidence in self.voxels.values():
            winner = self.winner(evidence)
            if winner is None:
                continue
            label, consensus = winner
            positions.append(evidence.position)
            colors.append(evidence.colors[label])
            labels.append(label)
            confidence.append(consensus)
        return (
            np.asarray(positions, dtype=np.float32).reshape(-1, 3),
            np.asarray(colors, dtype=np.uint8).reshape(-1, 3),
            np.asarray(labels, dtype=np.uint16),
            np.asarray(confidence, dtype=np.float32),
        )
