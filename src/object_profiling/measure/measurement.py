"""Medicion pura: observaciones dentro, `ObjectDimensions` fuera.

Este modulo no importa MuJoCo ni el entorno de simulacion. Es el punto de
integracion: cualquier consumidor que pueda producir `CameraObservation` y un
`BackgroundSet` puede medir, sin arrastrar la escena de este repositorio.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, replace

import numpy as np

from ..config import AppConfig
from ..contracts import (
    BoxCondition,
    CameraObservation,
    CuboidPose,
    ObjectDimensions,
    RejectionReason,
    RoutingHint,
    ViewDescriptor,
    snap_to_catalogue,
)
from .background import BackgroundSet, MissingBackgroundError
from .backproject import lateral_pitch_m
from .geometry import CuboidEstimate, axis_assignment, estimate_cuboid, provisional_cuboid
from .inspection import DamageAssessment, InspectionMetrics, assess_damage, inspect_cloud
from .perception import ScanView, observation_to_scan_view
from .registration import TOOL_FRAME_ID, FusedCloud, fuse_scan_views

# Seed del bootstrap de incertidumbre. Es deliberadamente independiente de la
# seed que genera la escena: reutilizarla acoplaria el estimador al episodio.
DEFAULT_BOOTSTRAP_SEED = 20260919

# Cara del cuboide apoyada contra las copas. El terminal agarra por arriba y la
# caja cuelga hacia +Z del marco del terminal.
GRASP_FACE = "-z"


@dataclass(frozen=True)
class MeasurementResult:
    """Salida publica mas los diagnosticos que no forman parte del contrato."""

    dimensions: ObjectDimensions
    estimate: CuboidEstimate | None
    cloud: FusedCloud | None
    views: tuple[ScanView, ...]
    rejected_views: tuple[tuple[str, str], ...]
    inspection: InspectionMetrics | None = None
    assessment: DamageAssessment | None = None


def _cuboid_pose(estimate: CuboidEstimate, config: AppConfig) -> CuboidPose:
    extent = estimate.upper_m - estimate.lower_m
    length_axis, width_axis, height_axis = axis_assignment(extent)
    center = (estimate.lower_m + estimate.upper_m) / 2.0
    return CuboidPose(
        center_m=tuple(float(value) for value in center),
        extent_by_axis_m=tuple(float(value) for value in extent),
        length_axis=length_axis,
        width_axis=width_axis,
        height_axis=height_axis,
        grasp_plane_offset_m=config.sensor.tool_to_box_offset_m,
        grasp_face=GRASP_FACE,
    )


def _rejected(
    object_id: str,
    timestamp_s: float,
    reason: RejectionReason,
    views: Sequence[ScanView],
) -> ObjectDimensions:
    return ObjectDimensions(
        object_id=object_id,
        timestamp_s=timestamp_s,
        frame_id=TOOL_FRAME_ID,
        dimensions_m=None,
        dimensions_snapped_m=None,
        uncertainty_m=None,
        pose=None,
        views_used=tuple(
            ViewDescriptor(view.pose_name, view.target_yaw_deg, view.target_tilt_deg) for view in views
        ),
        confidence=0.0,
        valid=False,
        rejection_reason=reason,
        condition=BoxCondition.UNKNOWN,
        routing=RoutingHint.NORMAL,
        damage=None,
    )


def rejected_measurement(
    object_id: str,
    timestamp_s: float,
    reason: RejectionReason,
) -> MeasurementResult:
    """Rechazo sin vistas utiles, para fallos anteriores a la percepcion."""

    return MeasurementResult(
        dimensions=_rejected(object_id, timestamp_s, reason, ()),
        estimate=None,
        cloud=None,
        views=(),
        rejected_views=(),
    )


def _scan_views(
    observations: Sequence[CameraObservation],
    backgrounds: BackgroundSet,
    config: AppConfig,
) -> tuple[list[ScanView], list[tuple[str, str]], list[float]]:
    views: list[ScanView] = []
    rejected: list[tuple[str, str]] = []
    pitches: list[float] = []
    for observation in observations:
        try:
            background = backgrounds.depth_for(observation.pose_name)
        except MissingBackgroundError:
            rejected.append((observation.pose_name, RejectionReason.MISSING_BACKGROUND.value))
            continue
        view, reason = observation_to_scan_view(observation, background, config)
        if view is None:
            rejected.append((observation.pose_name, reason.value if reason else "UNKNOWN"))
            continue
        views.append(view)
        pitches.append(lateral_pitch_m(observation, view.mask))
    return views, rejected, pitches


def measure(
    observations: Sequence[CameraObservation],
    backgrounds: BackgroundSet,
    config: AppConfig | None = None,
    *,
    object_id: str,
    bootstrap_seed: int = DEFAULT_BOOTSTRAP_SEED,
    inspection_observations: Sequence[CameraObservation] = (),
) -> MeasurementResult:
    """Segmenta, registra, ajusta el cuboide y valida.

    No recibe el entorno de simulacion, solo observaciones y fondos. Es lo que
    hace la medicion trasladable a otra escena.

    `observations` alimentan L/W/H. `inspection_observations` se fusionan
    solo para el inspector, para cubrir las caras que las vistas de medida
    no ven (el +y del yaw 180 desde la camara baja).
    """

    config = config or AppConfig()
    timestamp = max(
        (observation.timestamp_s for observation in (*observations, *inspection_observations)),
        default=0.0,
    )

    views, rejected, pitches = _scan_views(observations, backgrounds, config)
    inspection_views, inspection_rejected, _ = _scan_views(
        inspection_observations, backgrounds, config
    )
    rejected.extend(inspection_rejected)

    if not views:
        reason = RejectionReason(rejected[0][1]) if rejected else RejectionReason.INSUFFICIENT_VIEWS
        return MeasurementResult(
            dimensions=_rejected(object_id, timestamp, reason, ()),
            estimate=None,
            cloud=None,
            views=(),
            rejected_views=tuple(rejected),
        )

    cloud = fuse_scan_views(views)
    inspection_cloud = fuse_scan_views(views + inspection_views) if inspection_views else cloud
    pitch = float(np.median(pitches))
    estimate, reason = estimate_cuboid(cloud, config, seed=bootstrap_seed, lateral_pitch_m=pitch)

    def accepted(estimate: CuboidEstimate, assessment: DamageAssessment, inspection: InspectionMetrics) -> MeasurementResult:
        return MeasurementResult(
            dimensions=ObjectDimensions(
                object_id=object_id,
                timestamp_s=timestamp,
                frame_id=cloud.frame_id,
                dimensions_m=estimate.dimensions,
                dimensions_snapped_m=snap_to_catalogue(estimate.dimensions, config.catalogue_step_m),
                uncertainty_m=estimate.uncertainty,
                pose=_cuboid_pose(estimate, config),
                views_used=cloud.views,
                confidence=estimate.confidence,
                valid=True,
                rejection_reason=None,
                condition=assessment.condition,
                routing=assessment.routing,
                damage=assessment.report,
            ),
            estimate=estimate,
            cloud=cloud,
            views=tuple(views),
            rejected_views=tuple(rejected),
            inspection=inspection,
            assessment=assessment,
        )

    if estimate is not None:
        inspection = inspect_cloud(inspection_cloud, estimate, config)
        assessment = assess_damage(inspection, estimate.dimensions.as_array(), config)
        return accepted(estimate, assessment, inspection)

    dimensions = _rejected(object_id, timestamp, reason, views)
    fallback = provisional_cuboid(cloud, config)
    if fallback is None:
        return MeasurementResult(
            dimensions=dimensions,
            estimate=None,
            cloud=cloud,
            views=tuple(views),
            rejected_views=tuple(rejected),
        )
    inspection = inspect_cloud(inspection_cloud, fallback, config)
    assessment = assess_damage(inspection, fallback.dimensions.as_array(), config)
    if assessment.condition is BoxCondition.DAMAGED:
        relaxed, _ = estimate_cuboid(
            cloud,
            config,
            seed=bootstrap_seed,
            lateral_pitch_m=pitch,
            ignore_inward_residual=True,
        )
        if relaxed is not None:
            inspection = inspect_cloud(inspection_cloud, relaxed, config)
            assessment = assess_damage(inspection, relaxed.dimensions.as_array(), config)
            return accepted(relaxed, assessment, inspection)
        dimensions = replace(
            dimensions,
            condition=assessment.condition,
            routing=assessment.routing,
            damage=assessment.report,
        )
    return MeasurementResult(
        dimensions=dimensions,
        estimate=None,
        cloud=cloud,
        views=tuple(views),
        rejected_views=tuple(rejected),
        inspection=inspection,
        assessment=assessment,
    )
