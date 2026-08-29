#!/usr/bin/env python3
"""YOLO instance masks projected onto VLP-16 points and fused in the SLAM map."""

import json
import os
import time

import cv2
import numpy as np
import rospy
import tf2_ros
import yaml
from cv_bridge import CvBridge
from sensor_msgs import point_cloud2
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker, MarkerArray


def sigmoid(values: np.ndarray) -> np.ndarray:
    values = np.clip(values, -30.0, 30.0)
    return 1.0 / (1.0 + np.exp(-values))


def letterbox(image: np.ndarray, size: int) -> tuple[np.ndarray, float, int, int]:
    height, width = image.shape[:2]
    scale = min(size / width, size / height)
    resized_width = int(round(width * scale))
    resized_height = int(round(height * scale))
    resized = cv2.resize(image, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)
    left = (size - resized_width) // 2
    top = (size - resized_height) // 2
    canvas = np.full((size, size, 3), 114, dtype=np.uint8)
    canvas[top : top + resized_height, left : left + resized_width] = resized
    return canvas, scale, left, top


def palette_color(class_id: int) -> tuple[int, int, int]:
    # Deterministic, high-contrast RGB color; class_id is zero based.
    hue = (class_id * 0.61803398875) % 1.0
    hsv = np.uint8([[[int(hue * 179), 210, 255]]])
    bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0]
    return int(bgr[2]), int(bgr[1]), int(bgr[0])


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


def pack_rgb(colors: np.ndarray) -> np.ndarray:
    packed = (
        (colors[:, 0].astype(np.uint32) << 16)
        | (colors[:, 1].astype(np.uint32) << 8)
        | colors[:, 2].astype(np.uint32)
    )
    return packed.view(np.float32)


def cloud_message(
    header: Header,
    xyz: np.ndarray,
    colors: np.ndarray,
    labels: np.ndarray | None = None,
    confidence: np.ndarray | None = None,
) -> PointCloud2:
    semantic = labels is not None and confidence is not None
    if semantic:
        dtype = np.dtype(
            {
                "names": ["x", "y", "z", "rgb", "label", "confidence"],
                "formats": ["<f4", "<f4", "<f4", "<f4", "<u2", "<f4"],
                "offsets": [0, 4, 8, 12, 16, 20],
                "itemsize": 24,
            }
        )
    else:
        dtype = np.dtype(
            {
                "names": ["x", "y", "z", "rgb"],
                "formats": ["<f4", "<f4", "<f4", "<f4"],
                "offsets": [0, 4, 8, 12],
                "itemsize": 16,
            }
        )
    records = np.zeros(xyz.shape[0], dtype=dtype)
    records["x"], records["y"], records["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    records["rgb"] = pack_rgb(colors)
    if semantic:
        records["label"] = labels
        records["confidence"] = confidence

    message = PointCloud2()
    message.header = header
    message.height = 1
    message.width = int(xyz.shape[0])
    message.is_bigendian = False
    message.is_dense = bool(np.all(np.isfinite(xyz)))
    message.point_step = dtype.itemsize
    message.row_step = message.point_step * message.width
    message.fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
        PointField("rgb", 12, PointField.FLOAT32, 1),
    ]
    if semantic:
        message.fields.extend(
            [
                PointField("label", 16, PointField.UINT16, 1),
                PointField("confidence", 20, PointField.FLOAT32, 1),
            ]
        )
    message.data = records.tobytes()
    return message


class YoloSegmentation:
    def __init__(self, model_path: str, labels_path: str, input_size: int, confidence: float, iou: float) -> None:
        if not os.path.isfile(model_path):
            raise FileNotFoundError(f"semantic model not found: {model_path}")
        with open(labels_path, "r", encoding="utf-8") as stream:
            self.names = list(yaml.safe_load(stream)["names"])
        self.net = cv2.dnn.readNetFromONNX(model_path)
        self.input_size = input_size
        self.confidence = confidence
        self.iou = iou

    def infer(self, image: np.ndarray) -> tuple[list[dict], np.ndarray]:
        padded, scale, left, top = letterbox(image, self.input_size)
        blob = cv2.dnn.blobFromImage(padded, 1.0 / 255.0, swapRB=True, crop=False)
        self.net.setInput(blob)
        outputs = self.net.forward(self.net.getUnconnectedOutLayersNames())
        prediction = next(output for output in outputs if output.ndim == 3)
        prototype = next(output for output in outputs if output.ndim == 4)
        candidates = prediction[0].T
        class_count = len(self.names)
        class_scores = candidates[:, 4 : 4 + class_count]
        class_ids = np.argmax(class_scores, axis=1)
        scores = class_scores[np.arange(class_scores.shape[0]), class_ids]
        keep = scores >= self.confidence
        candidates, class_ids, scores = candidates[keep], class_ids[keep], scores[keep]
        if candidates.shape[0] == 0:
            return [], image.copy()

        boxes_center = candidates[:, :4]
        boxes_xywh = np.column_stack(
            (
                boxes_center[:, 0] - boxes_center[:, 2] / 2.0,
                boxes_center[:, 1] - boxes_center[:, 3] / 2.0,
                boxes_center[:, 2],
                boxes_center[:, 3],
            )
        )
        indices = cv2.dnn.NMSBoxes(boxes_xywh.tolist(), scores.tolist(), self.confidence, self.iou)
        if len(indices) == 0:
            return [], image.copy()
        indices = np.asarray(indices).reshape(-1)

        proto = prototype[0]
        proto_flat = proto.reshape(proto.shape[0], -1)
        original_height, original_width = image.shape[:2]
        resized_width = int(round(original_width * scale))
        resized_height = int(round(original_height * scale))
        overlay = image.copy()
        detections: list[dict] = []
        for index in indices:
            class_id = int(class_ids[index])
            score = float(scores[index])
            coeff = candidates[index, 4 + class_count :]
            mask_small = sigmoid(coeff @ proto_flat).reshape(proto.shape[1], proto.shape[2])
            mask_input = cv2.resize(mask_small, (self.input_size, self.input_size), interpolation=cv2.INTER_LINEAR)
            mask = mask_input[top : top + resized_height, left : left + resized_width]
            mask = cv2.resize(mask, (original_width, original_height), interpolation=cv2.INTER_LINEAR)

            x, y, width, height = boxes_xywh[index]
            x1 = int(np.clip((x - left) / scale, 0, original_width - 1))
            y1 = int(np.clip((y - top) / scale, 0, original_height - 1))
            x2 = int(np.clip((x + width - left) / scale, 0, original_width - 1))
            y2 = int(np.clip((y + height - top) / scale, 0, original_height - 1))
            instance_mask = mask >= 0.5
            box_mask = np.zeros_like(instance_mask)
            box_mask[y1 : y2 + 1, x1 : x2 + 1] = True
            instance_mask &= box_mask
            if not np.any(instance_mask):
                continue

            rgb = palette_color(class_id)
            color_bgr = (rgb[2], rgb[1], rgb[0])
            overlay[instance_mask] = (
                overlay[instance_mask].astype(np.float32) * 0.45
                + np.asarray(color_bgr, dtype=np.float32) * 0.55
            ).astype(np.uint8)
            cv2.rectangle(overlay, (x1, y1), (x2, y2), color_bgr, 2)
            label_text = f"{self.names[class_id]} {score:.2f}"
            cv2.putText(overlay, label_text, (x1, max(14, y1 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color_bgr, 1, cv2.LINE_AA)
            detections.append(
                {
                    "class_id": class_id,
                    "name": self.names[class_id],
                    "score": score,
                    "mask": instance_mask,
                    "rgb": rgb,
                }
            )
        return detections, overlay


class SemanticMapper:
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
        self.map_frame = str(rospy.get_param("~map_frame", "map"))
        self.voxel_size = float(rospy.get_param("~voxel_size", 0.10))
        self.max_voxels = int(rospy.get_param("~max_voxels", 200000))
        self.map_publish_period = float(rospy.get_param("~map_publish_period", 2.0))
        self.dynamic_names = set(rospy.get_param("~dynamic_classes", ["person", "bicycle", "car", "motorcycle", "bus", "truck"]))
        self.latest_image = None
        self.camera_info = None
        self.geometry_voxels: dict[tuple[int, int, int], tuple[np.ndarray, tuple[int, int, int]]] = {}
        self.semantic_voxels: dict[tuple[int, int, int], tuple[np.ndarray, tuple[int, int, int], int, float]] = {}
        self.frame_count = 0
        self.last_map_publish = rospy.Time(0)

        self.image_pub = rospy.Publisher("/semantic/image", Image, queue_size=1)
        self.cloud_pub = rospy.Publisher("/semantic/cloud", PointCloud2, queue_size=1)
        self.geometry_pub = rospy.Publisher("/semantic/geometry_map", PointCloud2, queue_size=1, latch=True)
        self.map_pub = rospy.Publisher("/semantic/map", PointCloud2, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher("/semantic/markers", MarkerArray, queue_size=1)
        self.status_pub = rospy.Publisher("/semantic/status", String, queue_size=5)
        self.image_sub = rospy.Subscriber("/camera/image", Image, self.handle_image, queue_size=1, buff_size=2**22)
        self.info_sub = rospy.Subscriber("/camera/camera_info", CameraInfo, self.handle_info, queue_size=1)
        self.cloud_sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.handle_cloud, queue_size=1, buff_size=2**22)

    def handle_image(self, message: Image) -> None:
        self.latest_image = (message.header, self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8"))

    def handle_info(self, message: CameraInfo) -> None:
        self.camera_info = message

    def handle_cloud(self, message: PointCloud2) -> None:
        if self.latest_image is None or self.camera_info is None:
            return
        started = time.perf_counter()
        xyz = np.asarray(
            list(point_cloud2.read_points(message, field_names=("x", "y", "z"), skip_nans=True)),
            dtype=np.float32,
        )
        if xyz.size == 0:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, message.header.frame_id, message.header.stamp, rospy.Duration(0.15)
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as error:
            rospy.logwarn_throttle(2.0, "[语义SLAM] 等待map点云变换: %s", error)
            return

        image_header, image = self.latest_image
        inference_started = time.perf_counter()
        detections, overlay = self.detector.infer(image)
        inference_ms = (time.perf_counter() - inference_started) * 1000.0
        labels, confidence, colors, instance_ids, association = self.associate(xyz, detections, image.shape)
        xyz_map = self.transform_points(xyz, transform)
        self.fuse_voxels(xyz_map, labels, confidence, colors, detections)

        output_header = Header(stamp=message.header.stamp, frame_id=self.map_frame)
        self.cloud_pub.publish(cloud_message(output_header, xyz_map, colors, labels, confidence))
        # ROS Noetic cv_bridge still calls ndarray.tostring(), removed by NumPy 2.
        # Construct the standard bgr8 Image directly to keep the isolated environment current.
        overlay = np.ascontiguousarray(overlay, dtype=np.uint8)
        overlay_message = Image()
        overlay_message.header = image_header
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
            "detections": detected_names,
            "associated_points": int(np.count_nonzero(labels)),
            "geometry_voxels": len(self.geometry_voxels),
            "semantic_voxels": len(self.semantic_voxels),
            "association": association,
        }
        self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))
        if self.frame_count % 5 == 0:
            names = ", ".join(detected_names) if detected_names else "无"
            rospy.loginfo(
                "[语义分析] 帧=%d 推理=%.1fms 总耗时=%.1fms 类别=[%s] 关联点=%d 语义体素=%d",
                self.frame_count,
                inference_ms,
                elapsed_ms,
                names,
                int(np.count_nonzero(labels)),
                len(self.semantic_voxels),
            )

    def associate(
        self, xyz: np.ndarray, detections: list[dict], image_shape: tuple[int, ...]
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, dict]:
        labels = np.zeros(xyz.shape[0], dtype=np.uint16)
        confidence = np.zeros(xyz.shape[0], dtype=np.float32)
        colors = np.full((xyz.shape[0], 3), 150, dtype=np.uint8)
        instance_ids = np.full(xyz.shape[0], -1, dtype=np.int16)
        if not detections:
            return labels, confidence, colors, instance_ids, {"projected": 0}

        fx, fy = float(self.camera_info.K[0]), float(self.camera_info.K[4])
        cx, cy = float(self.camera_info.K[2]), float(self.camera_info.K[5])
        forward = xyz[:, 0]
        camera_up = xyz[:, 2] + 0.0377
        projected = forward > 0.10
        indices = np.nonzero(projected)[0]
        u = np.rint(cx - fx * xyz[indices, 1] / forward[indices]).astype(np.int32)
        v = np.rint(cy - fy * camera_up[indices] / forward[indices]).astype(np.int32)
        height, width = image_shape[:2]
        inside = (u >= 0) & (u < width) & (v >= 0) & (v < height)
        point_indices = indices[inside]
        u, v = u[inside], v[inside]

        ordered = sorted(enumerate(detections), key=lambda item: item[1]["score"])
        for detection_index, detection in ordered:
            matched = detection["mask"][v, u]
            matched_indices = point_indices[matched]
            labels[matched_indices] = int(detection["class_id"]) + 1
            confidence[matched_indices] = float(detection["score"])
            colors[matched_indices] = np.asarray(detection["rgb"], dtype=np.uint8)
            instance_ids[matched_indices] = detection_index
        return labels, confidence, colors, instance_ids, {"projected": int(point_indices.size)}

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
    ) -> None:
        keys = np.floor(xyz_map / self.voxel_size).astype(np.int32)
        dynamic_labels = {
            detection["class_id"] + 1
            for detection in detections
            if detection["name"] in self.dynamic_names
        }
        for index, key_array in enumerate(keys):
            key = tuple(int(value) for value in key_array)
            label = int(labels[index])
            if label not in dynamic_labels and len(self.geometry_voxels) < self.max_voxels:
                self.geometry_voxels[key] = (xyz_map[index].copy(), (145, 145, 145))
            # Dynamic instances stay visible in the current cloud/markers but are
            # deliberately excluded from the persistent semantic map.
            if label > 0 and label not in dynamic_labels and len(self.semantic_voxels) < self.max_voxels:
                previous = self.semantic_voxels.get(key)
                if previous is None or float(confidence[index]) >= previous[3]:
                    self.semantic_voxels[key] = (
                        xyz_map[index].copy(),
                        tuple(int(value) for value in colors[index]),
                        label,
                        float(confidence[index]),
                    )

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


if __name__ == "__main__":
    rospy.init_node("semantic_mapper")
    try:
        SemanticMapper()
    except Exception as exception:
        rospy.logfatal("[语义分析] 启动失败: %s", exception)
        raise
    rospy.spin()
