"""Generadores procedurales de dano estructural.

Operan sobre vertices de autoria, antes de llevarlos al marco compilado de
MuJoCo. Un solo defecto por caja. La cara de agarre (`-z` del cuerpo, la que
apoya contra las copas) no se dania: el weld de succion no modela el sellado.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum

import numpy as np

from ..config import AppConfig
from ..contracts import Dimensions3D


class DamageKind(StrEnum):
    INTACT = "INTACT"
    CRUSHED_CORNER = "CRUSHED_CORNER"
    DENTED_FACE = "DENTED_FACE"
    BUCKLED_PANEL = "BUCKLED_PANEL"


# La cara contra las copas, en el marco del cuerpo de la caja. El terminal
# agarra por -Z: el centro de la caja vive en +Z del gripper.
GRASP_FACE_AXIS = 2
GRASP_FACE_SIGN = -1

FACE_LOCATIONS: tuple[str, ...] = ("+x", "-x", "+y", "-y", "+z", "-z")
CORNER_LOCATIONS: tuple[str, ...] = (
    "+x+y+z",
    "+x+y-z",
    "+x-y+z",
    "+x-y-z",
    "-x+y+z",
    "-x+y-z",
    "-x-y+z",
    "-x-y-z",
)


@dataclass(frozen=True)
class DamageSpec:
    """Ground truth de dano. Generacion de escena, no contrato publico."""

    kind: DamageKind
    severity_m: float
    location: str
    radius_m: float = 0.0

    @property
    def damaged(self) -> bool:
        return self.kind is not DamageKind.INTACT


INTACT_DAMAGE = DamageSpec(kind=DamageKind.INTACT, severity_m=0.0, location="")


def _parse_face(location: str) -> tuple[int, int]:
    axis_name = location[-1]
    sign = 1 if location[0] == "+" else -1
    return "xyz".index(axis_name), sign


def _parse_corner(location: str) -> np.ndarray:
    signs = np.ones(3, dtype=np.float64)
    for index, axis in enumerate("xyz"):
        token = location[location.index(axis) - 1]
        signs[index] = 1.0 if token == "+" else -1.0
    return signs


def shortest_edge_m(dimensions: Dimensions3D) -> float:
    return float(min(dimensions.length, dimensions.width, dimensions.height))


def face_is_grasp(location: str) -> bool:
    if len(location) == 2:
        axis, sign = _parse_face(location)
        return axis == GRASP_FACE_AXIS and sign == GRASP_FACE_SIGN
    signs = _parse_corner(location)
    return int(signs[GRASP_FACE_AXIS]) == GRASP_FACE_SIGN


def damageable_faces() -> tuple[str, ...]:
    return tuple(location for location in FACE_LOCATIONS if not face_is_grasp(location))


def damageable_corners() -> tuple[str, ...]:
    # Las cuatro esquinas de la cara de agarre estan sobre el plano de las
    # copas: cualquier chaflan las danaria. Solo se aplastan las opuestas.
    return tuple(location for location in CORNER_LOCATIONS if not face_is_grasp(location))


def generate_damage_spec(
    rng: np.random.Generator,
    dimensions: Dimensions3D,
    config: AppConfig,
) -> DamageSpec:
    if float(rng.random()) >= config.damage.rate:
        return INTACT_DAMAGE
    kind = rng.choice(
        np.asarray([DamageKind.CRUSHED_CORNER, DamageKind.DENTED_FACE, DamageKind.BUCKLED_PANEL])
    )
    shortest = shortest_edge_m(dimensions)
    fraction = float(rng.uniform(*config.damage.severity_fraction))
    severity = fraction * shortest
    if kind == DamageKind.CRUSHED_CORNER:
        location = str(rng.choice(np.asarray(damageable_corners())))
        return DamageSpec(kind=DamageKind.CRUSHED_CORNER, severity_m=severity, location=f"corner:{location}")
    location = str(rng.choice(np.asarray(damageable_faces())))
    if kind == DamageKind.DENTED_FACE:
        radius = 0.35 * shortest
        return DamageSpec(
            kind=DamageKind.DENTED_FACE,
            severity_m=severity,
            location=f"face:{location}",
            radius_m=radius,
        )
    return DamageSpec(kind=DamageKind.BUCKLED_PANEL, severity_m=severity, location=f"face:{location}")


def _on_face(vertices: np.ndarray, half: np.ndarray, axis: int, sign: int, atol: float = 1e-9) -> np.ndarray:
    return np.abs(vertices[:, axis] - sign * half[axis]) <= atol


def _on_grasp_face(vertices: np.ndarray, half: np.ndarray) -> np.ndarray:
    return _on_face(vertices, half, GRASP_FACE_AXIS, GRASP_FACE_SIGN)


def dent_face(vertices: np.ndarray, half: np.ndarray, location: str, depth_m: float, radius_m: float) -> np.ndarray:
    out = vertices.copy()
    axis, sign = _parse_face(location)
    on_face = _on_face(out, half, axis, sign) & ~_on_grasp_face(out, half)
    others = [index for index in range(3) if index != axis]
    radial = out[:, others[0]] ** 2 + out[:, others[1]] ** 2
    sigma = max(float(radius_m) / 2.0, 1e-4)
    falloff = np.exp(-radial / (2.0 * sigma * sigma))
    out[on_face, axis] -= sign * float(depth_m) * falloff[on_face]
    return out


def buckle_panel(vertices: np.ndarray, half: np.ndarray, location: str, amplitude_m: float) -> np.ndarray:
    """Pandeo hacia dentro con amplitud cero en el borde, para no cambiar el envolvente."""

    out = vertices.copy()
    axis, sign = _parse_face(location)
    on_face = _on_face(out, half, axis, sign) & ~_on_grasp_face(out, half)
    others = [index for index in range(3) if index != axis]
    u = out[on_face, others[0]] / max(float(half[others[0]]), 1e-9)
    v = out[on_face, others[1]] / max(float(half[others[1]]), 1e-9)
    envelope = np.cos(0.5 * np.pi * np.clip(u, -1.0, 1.0)) * np.cos(0.5 * np.pi * np.clip(v, -1.0, 1.0))
    out[on_face, axis] -= sign * float(amplitude_m) * envelope
    return out


def crush_corner(vertices: np.ndarray, half: np.ndarray, location: str, depth_m: float) -> np.ndarray:
    out = vertices.copy()
    signs = _parse_corner(location)
    corner = signs * half
    normal = signs / np.linalg.norm(signs)
    plane_d = float(normal @ corner) - float(depth_m)
    signed = out @ normal - plane_d
    beyond = signed > 0.0
    out[beyond] -= np.outer(signed[beyond], normal)
    grasp = _on_grasp_face(vertices, half)
    out[grasp] = vertices[grasp]
    return out


def apply_damage(vertices: np.ndarray, half_extents_m: np.ndarray, spec: DamageSpec) -> np.ndarray:
    if not spec.damaged:
        return vertices
    half = np.asarray(half_extents_m, dtype=np.float64).reshape(3)
    if spec.kind is DamageKind.CRUSHED_CORNER:
        return crush_corner(vertices, half, spec.location.split(":", 1)[1], spec.severity_m)
    if spec.kind is DamageKind.DENTED_FACE:
        return dent_face(
            vertices,
            half,
            spec.location.split(":", 1)[1],
            spec.severity_m,
            spec.radius_m,
        )
    if spec.kind is DamageKind.BUCKLED_PANEL:
        return buckle_panel(vertices, half, spec.location.split(":", 1)[1], spec.severity_m)
    raise ValueError(spec.kind)


def grasp_plane_reached(vertices: np.ndarray, half_extents_m: np.ndarray, margin_m: float) -> bool:
    """True si algun vertice cruza el plano de contacto de las copas hacia ellas."""

    grasp_z = GRASP_FACE_SIGN * float(half_extents_m[GRASP_FACE_AXIS])
    # La cara de agarre esta en z = -hz. Un vertice con z < -hz - margen se ha
    # salido del cuboide hacia las copas (hacia fuera). Un chaflan hacia dentro
    # tiene z > -hz. La pregunta del plan es si el chaflan alcanza el plano:
    # vertices de la cara de agarre que se hayan movido.
    original_on_grasp = np.abs(vertices[:, GRASP_FACE_AXIS] - grasp_z) <= 1e-9
    # Tras deformar, comparamos contra una copia intacta en el llamador.
    return bool(np.any(original_on_grasp))
