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
    consecutive_misses: int = 0
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
        negative_evidence_decay: float = 0.65,
        max_consecutive_misses: int = 5,
        minimum_winner_score: float = 0.55,
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
        if not 0.0 < negative_evidence_decay <= 1.0:
            raise ValueError("negative_evidence_decay must be within (0, 1]")
        if max_consecutive_misses < 1:
            raise ValueError("max_consecutive_misses must be positive")
        if minimum_winner_score < 0.0:
            raise ValueError("minimum_winner_score must be non-negative")
        self.voxel_size = float(voxel_size)
        self.max_voxels = int(max_voxels)
        self.min_observations = int(min_observations)
        self.min_consensus = float(min_consensus)
        self.evidence_decay = float(evidence_decay)
        self.negative_evidence_decay = float(negative_evidence_decay)
        self.max_consecutive_misses = int(max_consecutive_misses)
        self.minimum_winner_score = float(minimum_winner_score)
        self.voxels: dict[tuple[int, int, int], VoxelEvidence] = {}
        self._stable_count = 0

    def update(
        self,
        xyz: np.ndarray,
        labels: np.ndarray,
        confidence: np.ndarray,
        colors: np.ndarray,
        excluded_labels: set[int] | None = None,
        observed_mask: np.ndarray | None = None,
    ) -> dict[str, int]:
        """Add a frame, including negative evidence for visible unlabeled voxels."""
        excluded = excluded_labels or set()
        if observed_mask is None:
            observed_mask = labels > 0
        observed_mask = np.asarray(observed_mask, dtype=bool)
        if observed_mask.shape != labels.shape:
            raise ValueError("observed_mask must have the same shape as labels")
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

        visible_voxels = {
            tuple(int(value) for value in key_array) for key_array in keys[observed_mask]
        }
        positive_voxels = {key for key, _label in frame_observations}
        removed = 0
        negative_updates = 0
        for key in visible_voxels:
            evidence = self.voxels.get(key)
            if evidence is None:
                continue
            for existing_label in list(evidence.scores):
                evidence.scores[existing_label] *= self.evidence_decay
                if key not in positive_voxels:
                    evidence.scores[existing_label] *= self.negative_evidence_decay
                if evidence.scores[existing_label] < 1e-4:
                    del evidence.scores[existing_label]
                    evidence.colors.pop(existing_label, None)
            if key in positive_voxels:
                evidence.consecutive_misses = 0
            else:
                evidence.consecutive_misses += 1
                negative_updates += 1
                if evidence.consecutive_misses >= self.max_consecutive_misses:
                    if evidence.stable:
                        self._stable_count -= 1
                    del self.voxels[key]
                    removed += 1

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
                evidence.observations += 1
                evidence.consecutive_misses = 0
                evidence.position = evidence.position * 0.8 + position * 0.2
                observed_voxels.add(key)
            evidence.scores[label] = evidence.scores.get(label, 0.0) + score
            evidence.colors[label] = color
            accepted += 1
        for key in visible_voxels | observed_voxels:
            evidence = self.voxels.get(key)
            if evidence is None:
                continue
            stable = self.winner(evidence) is not None
            if stable != evidence.stable:
                self._stable_count += 1 if stable else -1
                evidence.stable = stable
        return {
            "frame_voxels": len(observed_voxels),
            "frame_label_votes": accepted,
            "visible_voxels": len(visible_voxels),
            "negative_updates": negative_updates,
            "removed_voxels": removed,
            "candidate_voxels": len(self.voxels),
            "stable_voxels": self.stable_count(),
        }

    def winner(self, evidence: VoxelEvidence) -> tuple[int, float] | None:
        if evidence.observations < self.min_observations or not evidence.scores:
            return None
        label, score = max(evidence.scores.items(), key=lambda item: item[1])
        if score < self.minimum_winner_score:
            return None
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
