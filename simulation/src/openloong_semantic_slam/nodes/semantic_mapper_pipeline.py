#!/usr/bin/env python3
"""Synchronized frame processing pipeline for the semantic mapper."""

import json
import time

import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridgeError
from sensor_msgs import point_cloud2
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Header, String

from semantic_inference import cloud_message
from semantic_mapper_core import normalize_frame_id


class SemanticMapperPipeline:
    @staticmethod
    def stamp_delta(left: rospy.Time, right: rospy.Time) -> float:
        return abs((left - right).to_sec())

    def handle_synced(
        self, image_message: Image, info_message: CameraInfo, cloud_msg: PointCloud2
    ) -> None:
        started = time.perf_counter()
        image_cloud_delta = self.stamp_delta(image_message.header.stamp, cloud_msg.header.stamp)
        info_cloud_delta = self.stamp_delta(info_message.header.stamp, cloud_msg.header.stamp)
        observed_delta = max(image_cloud_delta, info_cloud_delta)
        if observed_delta > self.max_sync_delta:
            self.sync_rejected += 1
            rospy.logwarn_throttle(
                2.0,
                "[语义SLAM] 拒绝不同步数据: image-cloud=%.1fms info-cloud=%.1fms 门限=%.1fms",
                image_cloud_delta * 1000.0,
                info_cloud_delta * 1000.0,
                self.max_sync_delta * 1000.0,
            )
            return

        try:
            image = self.bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8")
        except CvBridgeError as error:
            rospy.logwarn_throttle(2.0, "[语义SLAM] 图像转换失败: %s", error)
            return

        xyz = np.asarray(
            list(
                point_cloud2.read_points(
                    cloud_msg, field_names=("x", "y", "z"), skip_nans=True
                )
            ),
            dtype=np.float32,
        )
        if xyz.size == 0:
            return

        cloud_frame = normalize_frame_id(cloud_msg.header.frame_id)
        camera_frame = normalize_frame_id(
            info_message.header.frame_id or image_message.header.frame_id
        )
        if not cloud_frame or not camera_frame:
            rospy.logwarn_throttle(
                2.0,
                "[语义SLAM] 缺少传感器 frame_id: cloud=%r camera=%r",
                cloud_msg.header.frame_id,
                info_message.header.frame_id or image_message.header.frame_id,
            )
            return

        timeout = rospy.Duration(self.tf_timeout)
        try:
            map_transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                cloud_frame,
                cloud_msg.header.stamp,
                timeout,
            )
            camera_transform = self.tf_buffer.lookup_transform(
                camera_frame,
                cloud_frame,
                cloud_msg.header.stamp,
                timeout,
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as error:
            rospy.logwarn_throttle(
                2.0,
                "[语义SLAM] 等待点云到 map/camera 的 TF 变换: %s",
                error,
            )
            return

        inference_started = time.perf_counter()
        detections, overlay = self.detector.infer(image)
        inference_ms = (time.perf_counter() - inference_started) * 1000.0
        xyz_camera = self.transform_points(xyz, camera_transform)
        labels, confidence, colors, instance_ids, association = self.associate(
            xyz_camera,
            detections,
            image.shape,
            info_message,
            camera_frame,
        )
        xyz_map = self.transform_points(xyz, map_transform)
        voxel_updates = self.fuse_voxels(xyz_map, labels, confidence, colors, detections)

        output_header = Header(stamp=cloud_msg.header.stamp, frame_id=self.map_frame)
        self.cloud_pub.publish(cloud_message(output_header, xyz_map, colors, labels, confidence))
        # ROS Noetic cv_bridge still calls ndarray.tostring(), removed by NumPy 2.
        # Construct the standard bgr8 Image directly to keep the isolated environment current.
        overlay = np.ascontiguousarray(overlay, dtype=np.uint8)
        overlay_message = Image()
        overlay_message.header = image_message.header
        overlay_message.height, overlay_message.width = overlay.shape[:2]
        overlay_message.encoding = "bgr8"
        overlay_message.is_bigendian = False
        overlay_message.step = overlay_message.width * 3
        overlay_message.data = overlay.tobytes()
        self.image_pub.publish(overlay_message)
        self.publish_markers(output_header, xyz_map, instance_ids, detections)

        now = rospy.Time.now()
        if (now - self.last_map_publish).to_sec() >= self.map_publish_period:
            self.publish_maps(output_header)
            self.last_map_publish = now

        self.frame_count += 1
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        detected_names = [detection["name"] for detection in detections]
        status = {
            "frame": self.frame_count,
            "inference_ms": round(inference_ms, 1),
            "pipeline_ms": round(elapsed_ms, 1),
            "image_cloud_delta_ms": round(image_cloud_delta * 1000.0, 3),
            "camera_info_cloud_delta_ms": round(info_cloud_delta * 1000.0, 3),
            "sync_rejected": self.sync_rejected,
            "cloud_frame": cloud_frame,
            "camera_frame": camera_frame,
            "detections": detected_names,
            "associated_points": int(np.count_nonzero(labels)),
            "geometry_voxels": len(self.geometry_voxels),
            "semantic_voxels": len(self.semantic_voxels),
            "geometry_capacity_rejections": self.geometry_capacity_rejections,
            "semantic_capacity_rejections": self.semantic_capacity_rejections,
            "voxel_updates": voxel_updates,
            "association": association,
        }
        self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))
        if self.frame_count % 5 == 0:
            names = ", ".join(detected_names) if detected_names else "无"
            rospy.loginfo(
                "[语义分析] 帧=%d 同步=%.1fms 推理=%.1fms 总耗时=%.1fms 类别=[%s] "
                "关联点=%d 语义体素=%d",
                self.frame_count,
                image_cloud_delta * 1000.0,
                inference_ms,
                elapsed_ms,
                names,
                int(np.count_nonzero(labels)),
                len(self.semantic_voxels),
            )

