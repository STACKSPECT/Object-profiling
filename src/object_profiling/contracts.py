from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import StrEnum
from typing import Any

import numpy as np


OBJECT_DIMENSIONS_SCHEMA_VERSION = 2


class RejectionReason(StrEnum):
    INSUFFICIENT_FOREGROUND = "INSUFFICIENT_FOREGROUND"
    FRAME_BORDER_CONTACT = "FRAME_BORDER_CONTACT"
    INSUFFICIENT_VIEWS = "INSUFFICIENT_VIEWS"
    REGISTRATION_INCONSISTENT = "REGISTRATION_INCONSISTENT"
    OUT_OF_RANGE = "OUT_OF_RANGE"
    HIGH_UNCERTAINTY = "HIGH_UNCERTAINTY"
    MOTION_TIMEOUT = "MOTION_TIMEOUT"
    RENDER_FAILURE = "RENDER_FAILURE"
    MISSING_BACKGROUND = "MISSING_BACKGROUND"
    INSUFFICIENT_FACE_COVERAGE = "INSUFFICIENT_FACE_COVERAGE"


@dataclass(frozen=True)
class Extent3D:
    """Tripleta por eje sin orden impuesto.

    Incertidumbres y errores se expresan asi porque la incertidumbre de la
    longitud puede ser menor que la de la anchura.
    """

    length: float
    width: float
    height: float

    def as_array(self) -> np.ndarray:
        return np.asarray([self.length, self.width, self.height], dtype=np.float64)


@dataclass(frozen=True)
class Dimensions3D(Extent3D):
    def __post_init__(self) -> None:
        if self.length < self.width:
            raise ValueError("length must be greater than or equal to width")


@dataclass(frozen=True)
class BoxSpec:
    object_id: str
    dimensions_m: Dimensions3D
    mass_kg: float
    rgba: tuple[float, float, float, float]


@dataclass(frozen=True)
class CameraIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass(frozen=True)
class ViewDescriptor:
    """Vista utilizada, identificada por nombre de pose y angulos en grados."""

    pose_name: str
    yaw_deg: int
    tilt_deg: int


@dataclass(frozen=True)
class CameraObservation:
    timestamp_s: float
    pose_name: str
    target_yaw_deg: int
    target_tilt_deg: int
    rgb: np.ndarray
    depth_m: np.ndarray
    intrinsics: CameraIntrinsics
    camera_to_world: np.ndarray
    tool_to_world: np.ndarray


@dataclass(frozen=True)
class ScanView:
    pose_name: str
    target_yaw_deg: int
    target_tilt_deg: int
    rgb: np.ndarray
    depth_m: np.ndarray
    mask: np.ndarray
    points_tool_m: np.ndarray
    touches_border: bool


@dataclass(frozen=True)
class ObjectDimensions:
    object_id: str
    timestamp_s: float
    frame_id: str
    dimensions_m: Dimensions3D | None
    uncertainty_m: Extent3D | None
    views_used: tuple[ViewDescriptor, ...]
    confidence: float
    valid: bool
    rejection_reason: RejectionReason | None
    schema_version: int = OBJECT_DIMENSIONS_SCHEMA_VERSION

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["rejection_reason"] = self.rejection_reason.value if self.rejection_reason else None
        payload["views_used"] = [asdict(view) for view in self.views_used]
        return payload


@dataclass(frozen=True)
class EvaluationRecord:
    seed: int
    ground_truth_m: Dimensions3D
    prediction: ObjectDimensions
    absolute_error_m: Extent3D | None
    perception_latency_s: float

    def to_dict(self) -> dict[str, Any]:
        return {
            "seed": self.seed,
            "ground_truth_m": asdict(self.ground_truth_m),
            "prediction": self.prediction.to_dict(),
            "absolute_error_m": asdict(self.absolute_error_m) if self.absolute_error_m else None,
            "perception_latency_s": self.perception_latency_s,
        }
