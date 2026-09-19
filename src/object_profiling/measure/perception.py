from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from ..config import AppConfig, SensorConfig
from ..contracts import CameraObservation, RejectionReason
from .backproject import backproject_depth


@dataclass(frozen=True)
class ScanView:
    """Una vista ya segmentada y retroproyectada. Tipo interno del pipeline."""

    pose_name: str
    target_yaw_deg: int
    target_tilt_deg: int
    rgb: np.ndarray
    depth_m: np.ndarray
    mask: np.ndarray
    points_tool_m: np.ndarray
    touches_border: bool


@dataclass(frozen=True)
class SegmentationResult:
    """Mascara de la caja obtenida solo de RGB-D.

    `mask` es el componente conexo completo. `interior_mask` le quitaría el
    anillo de silueta si `mask_erosion_px > 0`. Con el valor actual (0) ambas
    coinciden: EXP-006 midio que ese anillo es el que define los extremos.
    """

    mask: np.ndarray
    interior_mask: np.ndarray
    touches_border: bool
    reason: RejectionReason | None


def segment_foreground(
    depth_m: np.ndarray,
    background_depth_m: np.ndarray,
    config: SensorConfig,
) -> SegmentationResult:
    """Aisla la caja restando el fondo de la misma pose.

    No interviene ningun identificador de geometria de MuJoCo: el brazo, el
    terminal y el apoyo desaparecen porque ya estaban en el fondo.
    """

    empty = np.zeros(depth_m.shape, dtype=bool)
    finite = (
        np.isfinite(depth_m)
        & np.isfinite(background_depth_m)
        & (depth_m > 0.0)
        & (background_depth_m > 0.0)
    )
    closer = finite & (depth_m < background_depth_m - config.foreground_margin_m)

    kernel = np.ones((5, 5), np.uint8)
    cleaned = cv2.morphologyEx(closer.astype(np.uint8), cv2.MORPH_OPEN, kernel)
    cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel)

    count, labels, stats, _ = cv2.connectedComponentsWithStats(cleaned, connectivity=8)
    if count <= 1:
        return SegmentationResult(empty, empty, False, RejectionReason.INSUFFICIENT_FOREGROUND)

    component = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    selected = labels == component
    if int(stats[component, cv2.CC_STAT_AREA]) < config.min_component_pixels:
        return SegmentationResult(selected, empty, False, RejectionReason.INSUFFICIENT_FOREGROUND)

    margin = config.border_margin_px
    touches = bool(
        np.any(selected[:margin])
        or np.any(selected[-margin:])
        or np.any(selected[:, :margin])
        or np.any(selected[:, -margin:])
    )
    if touches:
        return SegmentationResult(selected, empty, True, RejectionReason.FRAME_BORDER_CONTACT)

    erosion = 2 * config.mask_erosion_px + 1
    interior = cv2.erode(selected.astype(np.uint8), np.ones((erosion, erosion), np.uint8)).astype(bool)
    if not interior.any():
        return SegmentationResult(selected, interior, False, RejectionReason.INSUFFICIENT_FOREGROUND)
    return SegmentationResult(selected, interior, False, None)


def tool_volume_bounds_m(config: AppConfig) -> tuple[np.ndarray, np.ndarray]:
    """Volumen de recorte en el marco del terminal.

    Se deriva del rango de cajas declarado y de la geometria del terminal, no de
    la caja concreta. El mismo limite se aplica a los dos ejes horizontales para
    no presuponer cual de ellos lleva la longitud. El limite superior en z es el
    plano de contacto de las copas, contra el que se apoya la cara superior.
    """

    sensor = config.sensor
    box_range = config.box_range
    horizontal = max(box_range.length_m[1], box_range.width_m[1]) / 2.0 + sensor.tool_volume_margin_m
    lower = np.asarray(
        [-horizontal, -horizontal, sensor.tool_to_box_offset_m - sensor.cup_plane_margin_m]
    )
    upper = np.asarray(
        [
            horizontal,
            horizontal,
            sensor.tool_to_box_offset_m + box_range.height_m[1] + sensor.tool_volume_margin_m,
        ]
    )
    return lower, upper


def points_in_tool_frame(
    observation: CameraObservation,
    mask: np.ndarray,
    config: AppConfig,
) -> np.ndarray:
    """Retroproyecta la mascara y la lleva al marco actual del terminal."""

    points_world = backproject_depth(observation, mask)
    homogeneous = np.column_stack([points_world, np.ones(points_world.shape[0])])
    points_tool = (np.linalg.inv(observation.tool_to_world) @ homogeneous.T).T[:, :3]
    lower, upper = tool_volume_bounds_m(config)
    inside = np.all((points_tool >= lower) & (points_tool <= upper), axis=1)
    return points_tool[inside]


def observation_to_scan_view(
    observation: CameraObservation,
    background_depth_m: np.ndarray,
    config: AppConfig,
) -> tuple[ScanView | None, RejectionReason | None]:
    segmentation = segment_foreground(observation.depth_m, background_depth_m, config.sensor)
    if segmentation.reason is not None:
        return None, segmentation.reason

    points_tool = points_in_tool_frame(observation, segmentation.interior_mask, config)
    if points_tool.shape[0] == 0:
        return None, RejectionReason.INSUFFICIENT_FOREGROUND
    return ScanView(
        observation.pose_name,
        observation.target_yaw_deg,
        observation.target_tilt_deg,
        observation.rgb,
        observation.depth_m,
        segmentation.mask,
        points_tool,
        segmentation.touches_border,
    ), None
