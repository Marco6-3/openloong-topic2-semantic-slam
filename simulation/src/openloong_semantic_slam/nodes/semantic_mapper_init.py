#!/usr/bin/env python3
"""Initialization and ROS wiring for the synchronized semantic mapper."""

import message_filters
import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

from semantic_inference import YoloSegmentation
from semantic_mapper_core import normalize_frame_id


class SemanticMapperInitialization:
    def __init__(self) -> None:
        model_path = str(rospy.get_param("~model_path"))
        labels_path = str(rospy.get_param("~labels_path"))
        self.detector = YoloSegmentation(
            model_path,
            labels_path,
            int(rospy.get_param("~input_size", 320)),
            float(rospy.get_param("~confidence", 0.25)),
            float(rospy.get_param("~iou", 0.45)),
        )
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.map_frame = normalize_frame_id(str(rospy.get_param("~map_frame", "map")))
        self.voxel_size = float(rospy.get_param("~voxel_size", 0.10))
        self.max_voxels = int(rospy.get_param("~max_voxels", 200000))
        self.map_publish_period = float(rospy.get_param("~map_publish_period", 2.0))
        self.sync_queue_size = int(rospy.get_param("~sync_queue_size", 20))
        self.sync_slop = float(rospy.get_param("~sync_slop", 0.08))
        self.max_sync_delta = float(rospy.get_param("~max_sync_delta", self.sync_slop))
        self.tf_timeout = float(rospy.get_param("~tf_timeout", 0.20))
        self.camera_frame_convention = str(rospy.get_param("~camera_frame_convention", "auto"))
        self.min_camera_depth = float(rospy.get_param("~min_camera_depth", 0.10))
        self.dynamic_names = set(
            rospy.get_param(
                "~dynamic_classes", ["person", "bicycle", "car", "motorcycle", "bus", "truck"]
            )
        )
        if self.voxel_size <= 0.0:
            raise ValueError("voxel_size must be positive")
        if self.max_voxels <= 0:
            raise ValueError("max_voxels must be positive")
        if self.map_publish_period <= 0.0:
            raise ValueError("map_publish_period must be positive")
        if self.sync_queue_size < 2:
            raise ValueError("sync_queue_size must be at least 2")
        if self.sync_slop <= 0.0 or self.max_sync_delta <= 0.0:
            raise ValueError("sync_slop and max_sync_delta must be positive")
        if self.tf_timeout <= 0.0:
            raise ValueError("tf_timeout must be positive")
        if self.min_camera_depth <= 0.0:
            raise ValueError("min_camera_depth must be positive")

        self.geometry_voxels: dict[tuple[int, int, int], tuple[np.ndarray, tuple[int, int, int]]] = {}
        self.semantic_voxels: dict[tuple[int, int, int], tuple[np.ndarray, tuple[int, int, int], int, float]] = {}
        self.frame_count = 0
        self.last_map_publish = rospy.Time(0)
        self.sync_rejected = 0
        self.geometry_capacity_rejections = 0
        self.semantic_capacity_rejections = 0

        self.image_pub = rospy.Publisher("/semantic/image", Image, queue_size=1)
        self.cloud_pub = rospy.Publisher("/semantic/cloud", PointCloud2, queue_size=1)
        self.geometry_pub = rospy.Publisher("/semantic/geometry_map", PointCloud2, queue_size=1, latch=True)
        self.map_pub = rospy.Publisher("/semantic/map", PointCloud2, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher("/semantic/markers", MarkerArray, queue_size=1)
        self.status_pub = rospy.Publisher("/semantic/status", String, queue_size=5)

        self.image_sub = message_filters.Subscriber(
            "/camera/image", Image, queue_size=self.sync_queue_size, buff_size=2**22
        )
        self.info_sub = message_filters.Subscriber(
            "/camera/camera_info", CameraInfo, queue_size=self.sync_queue_size
        )
        self.cloud_sub = message_filters.Subscriber(
            "/velodyne_points", PointCloud2, queue_size=self.sync_queue_size, buff_size=2**22
        )
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.info_sub, self.cloud_sub],
            queue_size=self.sync_queue_size,
            slop=self.sync_slop,
            allow_headerless=False,
        )
        self.synchronizer.registerCallback(self.handle_synced)
        rospy.loginfo(
            "[语义SLAM] 已启用近似时间同步: queue=%d slop=%.0fms max_delta=%.0fms",
            self.sync_queue_size,
            self.sync_slop * 1000.0,
            self.max_sync_delta * 1000.0,
        )

