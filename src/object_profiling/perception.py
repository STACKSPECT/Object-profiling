from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from .config import SensorConfig
from .contracts import CameraObservation, RejectionReason, ScanView
from .sensors import backproject_depth


@dataclass(frozen=True)
class SegmentationResult:
    mask: np.ndarray
    touches_border: bool
    reason: RejectionReason | None


def segment_foreground(depth_m: np.ndarray, background_depth_m: np.ndarray, config: SensorConfig) -> SegmentationResult:
    finite = np.isfinite(depth_m) & np.isfinite(background_depth_m) & (depth_m > 0.0) & (background_depth_m > 0.0)
    raw = finite & (depth_m < background_depth_m - config.foreground_margin_m)
    mask = raw.astype(np.uint8) * 255
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if count <= 1:
        return SegmentationResult(np.zeros_like(raw), False, RejectionReason.INSUFFICIENT_FOREGROUND)
    component = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    area = int(stats[component, cv2.CC_STAT_AREA])
    selected = labels == component
    if area < config.min_component_pixels:
        return SegmentationResult(selected, False, RejectionReason.INSUFFICIENT_FOREGROUND)
    margin = config.border_margin_px
    touches = bool(np.any(selected[:margin]) or np.any(selected[-margin:]) or np.any(selected[:, :margin]) or np.any(selected[:, -margin:]))
    return SegmentationResult(selected, touches, RejectionReason.FRAME_BORDER_CONTACT if touches else None)


def observation_to_scan_view(observation: CameraObservation, background_depth_m: np.ndarray, config: SensorConfig) -> tuple[ScanView | None, RejectionReason | None]:
    segmentation = segment_foreground(observation.depth_m, background_depth_m, config)
    if segmentation.reason is not None:
        return None, segmentation.reason
    points_world = backproject_depth(observation, segmentation.mask)
    center = np.asarray(config.scan_center_world_m)
    half_extent = np.asarray(config.scan_half_extent_m)
    points_world = points_world[np.all(np.abs(points_world - center) <= half_extent, axis=1)]
    world_to_tool = np.linalg.inv(observation.tool_to_world)
    homogeneous = np.column_stack([points_world, np.ones(points_world.shape[0])])
    points_tool = (world_to_tool @ homogeneous.T).T[:, :3]
    return ScanView(observation.angle_deg, observation.rgb, observation.depth_m, segmentation.mask, points_tool, False), None
