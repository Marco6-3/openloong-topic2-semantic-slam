#!/usr/bin/env python3
"""YOLO segmentation and PointCloud2 construction helpers for the semantic mapper."""

import os

import cv2
import numpy as np
import yaml
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


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


