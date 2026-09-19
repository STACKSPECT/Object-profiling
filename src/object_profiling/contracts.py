"""Contrato publico de Object Profiling.

Este modulo es la frontera con `hackspain/Simulation`. Contiene lo que el
consumidor necesita construir (`CameraObservation`) y lo que recibe
(`ObjectDimensions`). No contiene tipos de generacion de escena ni de
evaluacion: `BoxSpec` vive en `environment.py`, `ScanView` en `perception.py` y
`EvaluationRecord` en `evaluation.py`.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import StrEnum
from typing import Any

import numpy as np

# 4: anade condition, routing y el informe de dano estructural.
OBJECT_DIMENSIONS_SCHEMA_VERSION = 4

AXIS_NAMES = ("x", "y", "z")


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


class BoxCondition(StrEnum):
    INTACT = "INTACT"
    DAMAGED = "DAMAGED"
    UNKNOWN = "UNKNOWN"


class RoutingHint(StrEnum):
    NORMAL = "NORMAL"
    ERROR_ZONE = "ERROR_ZONE"


class StructuralDamageKind(StrEnum):
    CRUSHED_CORNER = "CRUSHED_CORNER"
    DENTED_FACE = "DENTED_FACE"
    BUCKLED_PANEL = "BUCKLED_PANEL"


@dataclass(frozen=True)
class DamageReport:
    kind: StructuralDamageKind
    severity_m: float
    location: str
    evidence: dict[str, float]


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


def snap_to_catalogue(dimensions: Dimensions3D, step_m: float) -> Dimensions3D | None:
    """Redondea cada arista al paso de catalogo y reimpone length >= width.

    Las cajas del equipo viven en multiplos de ese paso. La medida continua se
    conserva aparte; esta es la que consume el paletizado.
    """

    if step_m <= 0.0:
        return None
    snapped = np.round(dimensions.as_array() / step_m) * step_m
    ordered = sorted((float(snapped[0]), float(snapped[1])), reverse=True)
    return Dimensions3D(ordered[0], ordered[1], float(snapped[2]))


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
    """Una captura RGB-D con todo lo necesario para medir.

    Es la entrada de `measurement.measure()`. Un consumidor externo puede
    construirla sin MuJoCo: basta con su propia camara calibrada y la cinematica
    directa de su robot.
    """

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
class CuboidPose:
    """Cuboide ajustado, situado en el marco declarado.

    Sin esto un consumidor solo tiene tres numeros y no puede transformar el
    volumen al mundo, calcular el centro de masas geometrico ni decidir una
    rotacion de colocacion.
    """

    center_m: tuple[float, float, float]
    # Extension por eje del marco, en orden x, y, z. No esta reordenada.
    extent_by_axis_m: tuple[float, float, float]
    # Que eje del marco lleva cada dimension nombrada: 0 = x, 1 = y, 2 = z.
    length_axis: int
    width_axis: int
    height_axis: int
    # Del origen del marco al plano de contacto de las copas, donde se apoya la
    # cara agarrada. El cuboide vive mas alla de ese plano.
    grasp_plane_offset_m: float
    # Cara del cuboide contra las copas, en notacion de eje con signo.
    grasp_face: str

    @property
    def axis_names(self) -> tuple[str, str, str]:
        return (
            AXIS_NAMES[self.length_axis],
            AXIS_NAMES[self.width_axis],
            AXIS_NAMES[self.height_axis],
        )

    def center_array(self) -> np.ndarray:
        return np.asarray(self.center_m, dtype=np.float64)


@dataclass(frozen=True)
class ObjectDimensions:
    object_id: str
    timestamp_s: float
    frame_id: str
    dimensions_m: Dimensions3D | None
    # Misma caja ajustada al paso del catalogo declarado. Es `None` si no se
    # declara ningun paso.
    dimensions_snapped_m: Dimensions3D | None
    uncertainty_m: Extent3D | None
    # Ordenada igual que `dimensions_m`; `pose` dice a que eje corresponde cada
    # componente.
    pose: CuboidPose | None
    views_used: tuple[ViewDescriptor, ...]
    confidence: float
    valid: bool
    rejection_reason: RejectionReason | None
    condition: BoxCondition = BoxCondition.UNKNOWN
    routing: RoutingHint = RoutingHint.NORMAL
    damage: DamageReport | None = None
    schema_version: int = OBJECT_DIMENSIONS_SCHEMA_VERSION

    def catalogue_dimensions(self) -> Dimensions3D | None:
        """Medida que baja al paletizado: catalogo si existe, si no la continua."""

        return self.dimensions_snapped_m if self.dimensions_snapped_m is not None else self.dimensions_m

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["rejection_reason"] = self.rejection_reason.value if self.rejection_reason else None
        payload["condition"] = self.condition.value
        payload["routing"] = self.routing.value
        payload["views_used"] = [asdict(view) for view in self.views_used]
        if self.damage is not None:
            payload["damage"]["kind"] = self.damage.kind.value
        if self.pose is not None:
            payload["pose"]["axis_names"] = list(self.pose.axis_names)
        return payload
