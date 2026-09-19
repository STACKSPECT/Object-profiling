from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .config import AppConfig
from .contracts import Dimensions3D, Extent3D, RejectionReason
from .registration import FusedCloud, axis_aligned_extremes_m, distance_to_box_surface_m


@dataclass(frozen=True)
class FaceCoverage:
    """Soporte observado en cada extremo de cada eje del terminal.

    Un percentil no distingue un extremo bien visto de uno que solo aparece en
    un punado de puntos de silueta. Esta cuenta si.
    """

    slab_m: float
    minimum_support: int
    lower_support: tuple[int, int, int]
    upper_support: tuple[int, int, int]

    @property
    def weakest_support(self) -> int:
        return min(min(self.lower_support), min(self.upper_support))

    @property
    def satisfied(self) -> bool:
        return self.weakest_support >= self.minimum_support


@dataclass(frozen=True)
class CuboidEstimate:
    dimensions: Dimensions3D
    uncertainty: Extent3D
    confidence: float
    lower_m: np.ndarray
    upper_m: np.ndarray
    points: int
    residual_p95_m: float
    view_plane_residual_m: float
    view_extent_disagreement_m: np.ndarray
    coverage: FaceCoverage


def axis_assignment(extent_m: np.ndarray) -> tuple[int, int, int]:
    """Que eje del marco lleva la longitud, la anchura y la altura.

    La altura es siempre el eje Z del terminal, del que cuelga la caja. Entre los
    dos horizontales, la longitud es el mayor. Publicar esta correspondencia es
    lo que permite al consumidor recuperar el eje fisico, que el orden
    `length >= width` por si solo borra.
    """

    length_axis, width_axis = (0, 1) if extent_m[0] >= extent_m[1] else (1, 0)
    return length_axis, width_axis, 2


def dimensions_from_extent(extent_m: np.ndarray) -> Dimensions3D:
    """Aplica la convencion length >= width sobre los dos ejes horizontales."""

    length_axis, width_axis, height_axis = axis_assignment(extent_m)
    return Dimensions3D(
        float(extent_m[length_axis]), float(extent_m[width_axis]), float(extent_m[height_axis])
    )


def face_coverage(
    points_m: np.ndarray,
    lower_m: np.ndarray,
    upper_m: np.ndarray,
    config: AppConfig,
) -> FaceCoverage:
    estimator = config.estimator
    slab = estimator.coverage_slab_m
    lower_support = tuple(
        int(np.count_nonzero(points_m[:, axis] <= lower_m[axis] + slab)) for axis in range(3)
    )
    upper_support = tuple(
        int(np.count_nonzero(points_m[:, axis] >= upper_m[axis] - slab)) for axis in range(3)
    )
    return FaceCoverage(
        slab_m=slab,
        minimum_support=estimator.minimum_face_support_points,
        lower_support=lower_support,
        upper_support=upper_support,
    )


def _bootstrap_uncertainty_m(points_m: np.ndarray, config: AppConfig, seed: int) -> np.ndarray:
    """Dispersion del extremo robusto al remuestrear la nube."""

    estimator = config.estimator
    rng = np.random.default_rng(seed)
    sample_pool = points_m
    if points_m.shape[0] > estimator.bootstrap_point_cap:
        indices = rng.choice(points_m.shape[0], estimator.bootstrap_point_cap, replace=False)
        sample_pool = points_m[indices]

    extents = np.empty((estimator.bootstrap_samples, 3))
    for index in range(estimator.bootstrap_samples):
        resampled = sample_pool[rng.choice(sample_pool.shape[0], sample_pool.shape[0], replace=True)]
        lower, upper = axis_aligned_extremes_m(
            resampled, estimator.percentile_low, estimator.percentile_high
        )
        extents[index] = upper - lower
    spread = np.percentile(extents, 97.5, axis=0) - np.percentile(extents, 2.5, axis=0)
    return spread / 2.0


def combine_uncertainty_m(
    bootstrap_m: np.ndarray,
    lateral_pitch_m: float,
    residual_p95_m: float,
    view_plane_residual_m: float,
    coverage: FaceCoverage,
) -> np.ndarray:
    """Suma en cuadratura las fuentes de incertidumbre de cada eje.

    - `bootstrap_m`: dispersion estadistica al remuestrear.
    - `lateral_pitch_m`: resolucion lateral de la camara a la distancia de
      trabajo. Es el suelo con el que se puede situar un borde de silueta, y
      cada dimension depende de dos bordes.
    - `residual_p95_m`: calidad del ajuste del cuboide.
    - `view_plane_residual_m`: consistencia entre vistas, es decir registro.
    - `coverage`: un extremo con poco soporte se localiza peor, en proporcion a
      la raiz del soporte que le falta.
    """

    edge_terms = np.full(3, np.sqrt(2.0) * lateral_pitch_m / 2.0)
    coverage_deficit = np.asarray(
        [
            max(0.0, coverage.minimum_support / max(1, support) - 1.0)
            for support in np.minimum(coverage.lower_support, coverage.upper_support)
        ]
    )
    coverage_terms = lateral_pitch_m * np.sqrt(coverage_deficit)
    return np.sqrt(
        np.square(bootstrap_m)
        + np.square(edge_terms)
        + np.square(residual_p95_m)
        + np.square(view_plane_residual_m)
        + np.square(coverage_terms)
    )


def estimate_cuboid(
    cloud: FusedCloud,
    config: AppConfig,
    *,
    seed: int,
    lateral_pitch_m: float,
) -> tuple[CuboidEstimate | None, RejectionReason | None]:
    """Ajusta un cuboide alineado al marco del terminal.

    La caja esta soldada al terminal con el agarre centrado, asi que sus caras
    quedan alineadas con los ejes del marco de fusion y no hace falta buscar la
    orientacion.
    """

    estimator = config.estimator
    if cloud.view_count < estimator.minimum_views:
        return None, RejectionReason.INSUFFICIENT_VIEWS
    points = cloud.points_m
    if points.shape[0] < estimator.minimum_points:
        return None, RejectionReason.INSUFFICIENT_FOREGROUND

    lower, upper = axis_aligned_extremes_m(points, estimator.percentile_low, estimator.percentile_high)
    dimensions = dimensions_from_extent(upper - lower)
    coverage = face_coverage(points, lower, upper, config)
    if not coverage.satisfied:
        return None, RejectionReason.INSUFFICIENT_FACE_COVERAGE

    ranges = config.box_range
    minimum = np.asarray([ranges.length_m[0], ranges.width_m[0], ranges.height_m[0]])
    maximum = np.asarray([ranges.length_m[1], ranges.width_m[1], ranges.height_m[1]])
    tolerance = config.sensor.tool_volume_margin_m / 2.0
    values = dimensions.as_array()
    if np.any(values < minimum - tolerance) or np.any(values > maximum + tolerance):
        return None, RejectionReason.OUT_OF_RANGE

    residual = distance_to_box_surface_m(points, lower, upper)
    residual_p95 = float(np.percentile(residual, 95))
    # Percentil alto, no mediana: una vista desplazada mantiene la mayoria de sus
    # puntos sobre las caras laterales comunes, y la mediana no lo nota.
    per_view = np.asarray(
        [
            float(np.percentile(distance_to_box_surface_m(cloud.points_for_view(index), lower, upper), 95))
            if cloud.points_for_view(index).shape[0] > 0
            else np.inf
            for index in range(cloud.view_count)
        ]
    )
    view_plane_residual = float(np.max(per_view))
    if view_plane_residual > estimator.max_view_plane_residual_m:
        return None, RejectionReason.REGISTRATION_INCONSISTENT

    extents_per_view = np.asarray(
        [
            np.ptp(cloud.points_for_view(index), axis=0)
            for index in range(cloud.view_count)
            if cloud.points_for_view(index).shape[0] > 0
        ]
    )
    disagreement = extents_per_view.max(axis=0) - extents_per_view.min(axis=0)

    bootstrap = _bootstrap_uncertainty_m(points, config, seed)
    uncertainty = combine_uncertainty_m(
        bootstrap, lateral_pitch_m, residual_p95, view_plane_residual, coverage
    )
    if np.any(uncertainty > estimator.max_uncertainty_m):
        return None, RejectionReason.HIGH_UNCERTAINTY

    confidence = _confidence(points.shape[0], residual_p95, uncertainty, coverage, config)
    return (
        CuboidEstimate(
            dimensions=dimensions,
            uncertainty=Extent3D(*(float(value) for value in uncertainty)),
            confidence=confidence,
            lower_m=lower,
            upper_m=upper,
            points=int(points.shape[0]),
            residual_p95_m=residual_p95,
            view_plane_residual_m=view_plane_residual,
            view_extent_disagreement_m=disagreement,
            coverage=coverage,
        ),
        None,
    )


def _confidence(
    point_count: int,
    residual_p95_m: float,
    uncertainty_m: np.ndarray,
    coverage: FaceCoverage,
    config: AppConfig,
) -> float:
    estimator = config.estimator
    point_score = min(1.0, point_count / 20_000.0)
    residual_score = float(np.exp(-residual_p95_m / estimator.cuboid_residual_scale_m))
    uncertainty_score = float(np.exp(-float(np.mean(uncertainty_m)) / estimator.max_uncertainty_m))
    coverage_score = min(1.0, coverage.weakest_support / (4.0 * estimator.minimum_face_support_points))
    return float(
        np.clip(
            0.15 * point_score + 0.25 * residual_score + 0.3 * uncertainty_score + 0.3 * coverage_score,
            0.0,
            1.0,
        )
    )
