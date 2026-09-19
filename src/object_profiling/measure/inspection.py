"""Inspeccion geometrica de dano sobre la nube fusionada.

No importa MuJoCo ni el ground truth de escena. Las senales se calculan contra
el cuboide ya ajustado: planaridad de cara, rectitud de arista, ocupacion de
esquina, agrupacion del residuo hacia dentro y el residuo global.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..config import AppConfig
from ..contracts import BoxCondition, DamageReport, RoutingHint, StructuralDamageKind
from .geometry import CuboidEstimate
from .registration import FusedCloud, distance_to_box_surface_m

FACE_NAMES = ("+x", "-x", "+y", "-y", "+z", "-z")
CORNER_NAMES = (
    "-x-y-z",
    "+x-y-z",
    "-x+y-z",
    "+x+y-z",
    "-x-y+z",
    "+x-y+z",
    "-x+y+z",
    "+x+y+z",
)


def signed_distance_to_box_m(points_m: np.ndarray, lower_m: np.ndarray, upper_m: np.ndarray) -> np.ndarray:
    center = (lower_m + upper_m) / 2.0
    half_extent = (upper_m - lower_m) / 2.0
    offset = np.abs(points_m - center) - half_extent
    outside = np.linalg.norm(np.maximum(offset, 0.0), axis=1)
    inside = np.minimum(np.max(offset, axis=1), 0.0)
    return outside + inside


def _face_planes(lower_m: np.ndarray, upper_m: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Normales hacia fuera y puntos sobre cada plano, en orden FACE_NAMES."""

    normals = np.asarray(
        [[1.0, 0.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, -1.0]]
    )
    origins = np.asarray(
        [
            [upper_m[0], 0.0, 0.0],
            [lower_m[0], 0.0, 0.0],
            [0.0, upper_m[1], 0.0],
            [0.0, lower_m[1], 0.0],
            [0.0, 0.0, upper_m[2]],
            [0.0, 0.0, lower_m[2]],
        ]
    )
    return normals, origins


def _assign_faces(points_m: np.ndarray, lower_m: np.ndarray, upper_m: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    normals, origins = _face_planes(lower_m, upper_m)
    signed = (points_m[:, None, :] - origins[None, :, :]) * normals[None, :, :]
    signed = signed.sum(axis=2)
    nearest = np.argmin(np.abs(signed), axis=1)
    return nearest, signed[np.arange(points_m.shape[0]), nearest]


def _corners(lower_m: np.ndarray, upper_m: np.ndarray) -> np.ndarray:
    corners = np.empty((8, 3), dtype=np.float64)
    index = 0
    for z in (lower_m[2], upper_m[2]):
        for y in (lower_m[1], upper_m[1]):
            for x in (lower_m[0], upper_m[0]):
                corners[index] = (x, y, z)
                index += 1
    return corners


def _edges(lower_m: np.ndarray, upper_m: np.ndarray) -> list[tuple[np.ndarray, np.ndarray]]:
    c = _corners(lower_m, upper_m)
    # indices: 0 --x-y-z  1 +x-y-z  2 -x+y-z  3 +x+y-z
    #          4 -x-y+z  5 +x-y+z  6 -x+y+z  7 +x+y+z
    pairs = (
        (0, 1),
        (2, 3),
        (4, 5),
        (6, 7),
        (0, 2),
        (1, 3),
        (4, 6),
        (5, 7),
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),
    )
    return [(c[a], c[b]) for a, b in pairs]


def _point_to_segment_m(points: np.ndarray, start: np.ndarray, end: np.ndarray) -> np.ndarray:
    direction = end - start
    length2 = float(direction @ direction)
    if length2 < 1e-18:
        return np.linalg.norm(points - start, axis=1)
    t = np.clip((points - start) @ direction / length2, 0.0, 1.0)
    closest = start + t[:, None] * direction
    return np.linalg.norm(points - closest, axis=1)


def _fitted_line_residual_p95_m(points: np.ndarray) -> float:
    """Residuo p95 a la recta PCA de los puntos, no a la arista del cuboide.

    Un entorno cilindrico alrededor de la arista incluye una franja de cara.
    Medir contra el segmento del cuboide devolveria ~el radio, no la rectitud.
    """

    centered = points - points.mean(axis=0)
    _, _, vt_matrix = np.linalg.svd(centered, full_matrices=False)
    direction = vt_matrix[0]
    residual = np.linalg.norm(centered - np.outer(centered @ direction, direction), axis=1)
    return float(np.percentile(residual, 95))


@dataclass(frozen=True)
class InspectionMetrics:
    residual_p95_m: float
    face_planarity_p95_m: tuple[float, ...]
    face_inward_p95_m: tuple[float, ...]
    edge_straightness_p95_m: tuple[float, ...]
    corner_support: tuple[int, ...]
    max_inward_m: float
    inward_location: str
    inward_fraction: float
    weakest_corner_index: int
    weakest_corner_support: int
    max_edge_p95_m: float
    max_face_planarity_p95_m: float


@dataclass(frozen=True)
class DamageAssessment:
    condition: BoxCondition
    routing: RoutingHint
    report: DamageReport | None
    metrics: InspectionMetrics


def inspect_cloud(cloud: FusedCloud, estimate: CuboidEstimate, config: AppConfig) -> InspectionMetrics:
    points = cloud.points_m
    lower, upper = estimate.lower_m, estimate.upper_m
    residual = distance_to_box_surface_m(points, lower, upper)
    residual_p95 = float(np.percentile(residual, 95))
    face_index, signed = _assign_faces(points, lower, upper)
    planarity = []
    inward = []
    for index in range(6):
        selected = signed[face_index == index]
        if selected.size == 0:
            planarity.append(0.0)
            inward.append(0.0)
            continue
        planarity.append(float(np.percentile(np.abs(selected), 95)))
        inward_only = np.maximum(-selected, 0.0)
        inward.append(float(np.percentile(inward_only, 95)) if inward_only.size else 0.0)
    edges = _edges(lower, upper)
    edge_p95 = []
    radius = config.damage.edge_radius_m
    for start, end in edges:
        distance = _point_to_segment_m(points, start, end)
        nearby = points[distance <= radius]
        if nearby.shape[0] < 8:
            edge_p95.append(0.0)
            continue
        edge_p95.append(_fitted_line_residual_p95_m(nearby))
    corners = _corners(lower, upper)
    support = []
    for corner in corners:
        support.append(int(np.count_nonzero(np.linalg.norm(points - corner, axis=1) <= config.damage.corner_radius_m)))
    face_inward = np.asarray(inward)
    worst_face = int(np.argmax(face_inward))
    inward_mask = signed < -config.damage.cluster_inward_m
    inward_fraction = float(np.mean(inward_mask)) if points.shape[0] else 0.0
    max_inward = float(np.max(np.maximum(-signed, 0.0))) if points.shape[0] else 0.0
    weakest = int(np.argmin(support)) if support else 0
    return InspectionMetrics(
        residual_p95_m=residual_p95,
        face_planarity_p95_m=tuple(planarity),
        face_inward_p95_m=tuple(inward),
        edge_straightness_p95_m=tuple(edge_p95),
        corner_support=tuple(support),
        max_inward_m=max_inward,
        inward_location=f"face:{FACE_NAMES[worst_face]}",
        inward_fraction=inward_fraction,
        weakest_corner_index=weakest,
        weakest_corner_support=int(support[weakest]) if support else 0,
        max_edge_p95_m=float(max(edge_p95) if edge_p95 else 0.0),
        max_face_planarity_p95_m=float(max(planarity) if planarity else 0.0),
    )


def stacking_threshold_m(extent_m: np.ndarray, config: AppConfig) -> float:
    shortest = float(np.min(extent_m))
    return max(config.damage.relative_threshold * shortest, config.damage.absolute_floor_m)


def assess_damage(metrics: InspectionMetrics, extent_m: np.ndarray, config: AppConfig) -> DamageAssessment:
    threshold = stacking_threshold_m(extent_m, config)
    supports = np.asarray(metrics.corner_support, dtype=np.float64)
    median_support = float(np.median(supports)) if supports.size else 0.0
    # Las esquinas ocluidas de una caja sana tienen menos puntos, no cero. Un
    # chaflan deja la esfera practicamente vacia; el suelo absoluto evita el
    # falso positivo del percentil relativo sobre una esquina solo parcialmente
    # vista.
    corner_outlier = (
        median_support >= config.damage.minimum_corner_support
        and metrics.weakest_corner_support < config.damage.minimum_corner_support
    )
    face_fail = metrics.max_inward_m >= threshold
    edge_fail = metrics.max_edge_p95_m >= threshold
    if not (face_fail or edge_fail or corner_outlier):
        return DamageAssessment(BoxCondition.INTACT, RoutingHint.NORMAL, None, metrics)

    if corner_outlier:
        kind = StructuralDamageKind.CRUSHED_CORNER
        location = f"corner:{CORNER_NAMES[metrics.weakest_corner_index]}"
        severity = max(threshold, metrics.max_inward_m)
    elif metrics.inward_fraction > 0.04:
        kind = StructuralDamageKind.BUCKLED_PANEL
        location = metrics.inward_location
        severity = metrics.max_inward_m
    else:
        kind = StructuralDamageKind.DENTED_FACE
        location = metrics.inward_location
        severity = metrics.max_inward_m
    report = DamageReport(
        kind=kind,
        severity_m=float(severity),
        location=location,
        evidence={
            "face_planarity_p95_m": metrics.max_face_planarity_p95_m,
            "edge_straightness_p95_m": metrics.max_edge_p95_m,
            "weakest_corner_support": float(metrics.weakest_corner_support),
            "max_inward_m": metrics.max_inward_m,
            "residual_p95_m": metrics.residual_p95_m,
        },
    )
    return DamageAssessment(BoxCondition.DAMAGED, RoutingHint.ERROR_ZONE, report, metrics)
