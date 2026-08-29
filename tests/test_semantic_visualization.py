from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("rclpy", reason="语义可视化节点只在 ROS 环境测试")
sensor_messages = pytest.importorskip("sensor_msgs.msg", reason="语义可视化节点需要 ROS 消息")
PointCloud2 = sensor_messages.PointCloud2

from scripts.publish_semantic_cloud import SemanticProjector, semantic_message
from scripts.record_rviz_video import semantic_cloud_display


def test_semantic_projection_uses_enu_correction_and_radius() -> None:
    projector = SemanticProjector(
        semantic_xyz=np.asarray([[11.0, 22.0, 3.0], [50.0, 50.0, 0.0]]),
        labels=np.asarray([13, 9]),
        confidence=np.asarray([0.8, 0.9]),
        rgb=np.asarray([0xFFC800, 0xFF00FF]),
        trajectory_stamps=np.asarray([100]),
        corrections=np.asarray([[1.0, 2.0, 3.0]]),
        rotation_xy=np.eye(2),
        translation=np.asarray([10.0, 20.0, 0.0]),
        nearest_radius=0.2,
    )
    labels, confidence, rgb, matched, stamp_error = projector.project(
        np.asarray([[0.0, 0.0, 0.0], [100.0, 100.0, 100.0]]), 105
    )
    np.testing.assert_array_equal(labels, [13, 0])
    np.testing.assert_allclose(confidence, [0.8, 0.0])
    np.testing.assert_array_equal(rgb, [0xFFC800, 0x808080])
    np.testing.assert_array_equal(matched, [True, False])
    assert stamp_error == 5


def test_semantic_message_exposes_rviz_rgb_and_analysis_fields() -> None:
    source = PointCloud2()
    source.header.frame_id = "camera_init"
    message = semantic_message(
        source,
        np.asarray([[1.0, 2.0, 3.0]]),
        np.asarray([15], dtype=np.uint16),
        np.asarray([0.75], dtype=np.float32),
        np.asarray([0x00AF00], dtype=np.uint32),
    )
    assert message.header.frame_id == "camera_init"
    assert message.width == 1
    assert message.point_step == 24
    assert [(field.name, field.offset) for field in message.fields] == [
        ("x", 0),
        ("y", 4),
        ("z", 8),
        ("rgb", 12),
        ("label", 16),
        ("confidence", 20),
    ]
    assert len(message.data) == 24


def test_semantic_rviz_display_uses_rgb_topic() -> None:
    display = semantic_cloud_display()
    assert display["Name"] == "SemanticCloud (RandLA-Net RGB)"
    assert display["Color Transformer"] == "RGB8"
    assert display["Topic"]["Value"] == "/semantic_cloud"
