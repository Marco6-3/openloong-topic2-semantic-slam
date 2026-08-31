#!/usr/bin/env python3
"""Projection, voxel fusion, and publication methods for the semantic mapper."""

import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from semantic_inference import cloud_message, quaternion_matrix
from semantic_mapper_core import project_points, update_geometry_voxel, update_semantic_voxel


class SemanticMapperFusion:
    def associate(
        self,
        xyz_camera: np.ndarray,
        detections: list[dict],
        image_shape: tuple[int, ...],
        camera_info: CameraInfo,
        camera_frame: str,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, dict]:
        labels = np.zeros(xyz_camera.shape[0], dtype=np.uint16)
        confidence = np.zeros(xyz_camera.shape[0], dtype=np.float32)
        colors = np.full((xyz_camera.shape[0], 3), 150, dtype=np.uint8)
        instance_ids = np.full(xyz_camera.shape[0], -1, dtype=np.int16)
        try:
            point_indices, u, v, convention = project_points(
                xyz_camera,
                camera_info.K,
                image_shape,
                frame_id=camera_frame,
                convention=self.camera_frame_convention,
                min_depth=self.min_camera_depth,
            )
        except ValueError as error:
            rospy.logwarn_throttle(2.0, "[语义SLAM] 相机投影参数无效: %s", error)
            return labels, confidence, colors, instance_ids, {
                "projected": 0,
                "camera_convention": "invalid",
            }
        if not detections or point_indices.size == 0:
            return labels, confidence, colors, instance_ids, {
                "projected": int(point_indices.size),
                "camera_convention": convention,
            }

        ordered = sorted(enumerate(detections), key=lambda item: item[1]["score"])
        for detection_index, detection in ordered:
            matched = detection["mask"][v, u]
            matched_indices = point_indices[matched]
            labels[matched_indices] = int(detection["class_id"]) + 1
            confidence[matched_indices] = float(detection["score"])
            colors[matched_indices] = np.asarray(detection["rgb"], dtype=np.uint8)
            instance_ids[matched_indices] = detection_index
        return labels, confidence, colors, instance_ids, {
            "projected": int(point_indices.size),
            "matched": int(np.count_nonzero(labels)),
            "camera_convention": convention,
        }

    def transform_points(self, xyz: np.ndarray, transform) -> np.ndarray:
        q = transform.transform.rotation
        rotation = quaternion_matrix(q.x, q.y, q.z, q.w)
        t = transform.transform.translation
        translation = np.asarray((t.x, t.y, t.z), dtype=np.float32)
        return xyz @ rotation.T + translation

    def fuse_voxels(
        self,
        xyz_map: np.ndarray,
        labels: np.ndarray,
        confidence: np.ndarray,
        colors: np.ndarray,
        detections: list[dict],
    ) -> dict[str, int]:
        keys = np.floor(xyz_map / self.voxel_size).astype(np.int32)
        dynamin_labels = {
            detection["class_id"] + 1
            for detection in detections
            if detection["name"] in self.dynamic_names
        }
        stats = {
            "geometry_inserted": 0,
            "geometry_updated": 0,
            "geometry_rejected": 0,
            "semantic_inserted": 0,
            "semantic_updated": 0,
            "semantic_unchanged": 0,
            "semantic_rejected": 0,
        }
        for index, key_array in enumerate(keys):
            key = tuple(int(value) for value in key_array)
            label = int(labels[index])
            if label not in dynamic_labels:
                geometry_result = update_geometry_voxel(
                    self.geometry_voxels,
                    key,
                    (xyz_map[index].copy(), (145, 145, 145)),
                    self.max_voxels,
                )
                stats[f"geometry_{geometry_result}"] += 1
                if geometry_result == "rejected":
                    self.geometry_capacity_rejections += 1

            # Dynamic instances stay visible in the current cloud/markers but are
            # deliberately excluded from the persistent semantic map.
            if label > 0 and label not in dynamic_labels:
                semantic_result = update_semantic_voxel(
                    self.semantic_voxels,
                    key,
                    (
                        xyz_map[index].copy(),
                        tuple(int(value) for value in colors[index]),
                        label,
                        float(confidence[index]),
                    ),
                    self.max_voxels,
                )
                stats[f"semantic_{semantic_result}"] += 1
                if semantic_result == "rejected":
                    self.semantic_capacity_rejections += 1
        return stats

    def publish_maps(self, header: Header) -> None:
        if self.geometry_voxels:
            geometry_values = list(self.geometry_voxels.values())
            xyz = np.asarray([value[0] for value in geometry_values], dtype=np.float32)
            colors = np.asarray([value[1] for value in geometry_values], dtype=np.uint8)
            self.geometry_pub.publish(cloud_message(header, xyz, colors))
        if self.semantic_voxels:
            semantic_values = list(self.semantic_voxels.values())
            xyz = np.asarray([value[0] for value in semantic_values], dtype=np.float32)
            colors = np.asarray([value[1] for value in semantic_values], dtype=np.uint8)
            labels = np.asarray([value[2] for value in semantic_values], dtype=np.uint16)
            confidence = np.asarray([value[3] for value in semantic_values], dtype=np.float32)
            self.map_pub.publish(cloud_message(header, xyz, colors, labels, confidence))

    def publish_markers(
        self, header: Header, xyz_map: np.ndarray, instance_ids: np.ndarray, detections: list[dict]
    ) -> None:
        array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        array.markers.append(clear)
        for marker_id, detection in enumerate(detections):
            points = xyz_map[instance_ids == marker_id]
            if points.shape[0] < 3:
                continue
            center = np.median(points, axis=0)
            marker = Marker()
            marker.header = header
            marker.ns = "semantic_labels"
            marker.id = marker_id
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = float(center[0])
            marker.pose.position.y = float(center[1])
            marker.pose.position.z = float(center[2] + 0.35)
            marker.pose.orientation.w = 1.0
            marker.scale.z = 0.28
            rgb = detection["rgb"]
            marker.color.r = rgb[0] / 255.0
            marker.color.g = rgb[1] / 255.0
            marker.color.b = rgb[2] / 255.0
            marker.color.a = 1.0
            marker.text = f"{detection['name']} {detection['score']:.2f} ({points.shape[0]} pts)"
            marker.lifetime = rospy.Duration(0.6)
            array.markers.append(marker)
        self.marker_pub.publish(array)


