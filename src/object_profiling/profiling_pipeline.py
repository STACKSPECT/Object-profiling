from __future__ import annotations

import argparse
import json
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .background import MissingBackgroundError
from .config import AppConfig
from .contracts import ObjectDimensions, RejectionReason, ScanView, ViewDescriptor
from .controller import MotionError
from .environment import ProfilingEnvironment
from .geometry import CuboidEstimate, estimate_cuboid
from .perception import observation_to_scan_view
from .registration import TOOL_FRAME_ID, FusedCloud, fuse_scan_views
from .scanning import run_fixed_scan
from .sensors import RGBDSensor, lateral_pitch_m


@dataclass(frozen=True)
class ProfilingResult:
    """Salida publica mas los diagnosticos que no forman parte del contrato."""

    dimensions: ObjectDimensions
    estimate: CuboidEstimate | None
    cloud: FusedCloud | None
    views: tuple[ScanView, ...]
    rejected_views: tuple[tuple[str, str], ...]
    perception_latency_s: float
    cycle_duration_s: float


def _rejected(
    object_id: str,
    timestamp_s: float,
    reason: RejectionReason,
    views: tuple[ScanView, ...],
) -> ObjectDimensions:
    return ObjectDimensions(
        object_id=object_id,
        timestamp_s=timestamp_s,
        frame_id=TOOL_FRAME_ID,
        dimensions_m=None,
        uncertainty_m=None,
        views_used=tuple(
            ViewDescriptor(view.pose_name, view.target_yaw_deg, view.target_tilt_deg) for view in views
        ),
        confidence=0.0,
        valid=False,
        rejection_reason=reason,
    )


def profile(
    environment: ProfilingEnvironment,
    *,
    seed: int,
    config: AppConfig | None = None,
    on_state: Callable[[str], None] | None = None,
    on_step: Callable[[], None] | None = None,
) -> ProfilingResult:
    """Ejecuta el ciclo fijo completo y devuelve `ObjectDimensions`.

    La trayectoria no depende del resultado: las tres poses se recorren siempre.
    Recibe el episodio ya construido y solo su etiqueta, nunca su geometria.
    """

    config = config or AppConfig()
    object_id = environment.object_id
    sensor = RGBDSensor(environment)
    report = on_state or (lambda _name: None)

    views: list[ScanView] = []
    rejected: list[tuple[str, str]] = []
    pitches: list[float] = []
    perception_latency = 0.0

    def on_capture(pose, observation, backgrounds) -> None:
        nonlocal perception_latency
        started = time.perf_counter()
        try:
            background = backgrounds.depth_for(pose.name)
        except MissingBackgroundError:
            rejected.append((pose.name, RejectionReason.MISSING_BACKGROUND.value))
            return
        view, reason = observation_to_scan_view(observation, background, config)
        perception_latency += time.perf_counter() - started
        if view is None:
            rejected.append((pose.name, reason.value if reason else "UNKNOWN"))
            return
        views.append(view)
        pitches.append(lateral_pitch_m(observation, view.mask))

    cycle_started = time.perf_counter()
    try:
        run_fixed_scan(environment, sensor, on_state=report, on_step=on_step, on_capture=on_capture)
    except MotionError as error:
        return ProfilingResult(
            dimensions=_rejected(
                object_id, float(environment.data.time), error.reason, tuple(views)
            ),
            estimate=None,
            cloud=None,
            views=tuple(views),
            rejected_views=tuple(rejected),
            perception_latency_s=perception_latency,
            cycle_duration_s=time.perf_counter() - cycle_started,
        )
    finally:
        sensor.close()

    timestamp = float(environment.data.time)
    cycle_duration = time.perf_counter() - cycle_started

    report("ESTIMATE")
    if not views:
        reason = RejectionReason(rejected[0][1]) if rejected else RejectionReason.INSUFFICIENT_VIEWS
        return ProfilingResult(
            dimensions=_rejected(object_id, timestamp, reason, ()),
            estimate=None,
            cloud=None,
            views=(),
            rejected_views=tuple(rejected),
            perception_latency_s=perception_latency,
            cycle_duration_s=cycle_duration,
        )

    started = time.perf_counter()
    cloud = fuse_scan_views(views)
    estimate, reason = estimate_cuboid(
        cloud, config, seed=seed, lateral_pitch_m=float(np.median(pitches))
    )
    perception_latency += time.perf_counter() - started

    report("VALIDATE")
    if estimate is None:
        return ProfilingResult(
            dimensions=_rejected(object_id, timestamp, reason, tuple(views)),
            estimate=None,
            cloud=cloud,
            views=tuple(views),
            rejected_views=tuple(rejected),
            perception_latency_s=perception_latency,
            cycle_duration_s=cycle_duration,
        )

    report("PROFILE_READY")
    return ProfilingResult(
        dimensions=ObjectDimensions(
            object_id=object_id,
            timestamp_s=timestamp,
            frame_id=cloud.frame_id,
            dimensions_m=estimate.dimensions,
            uncertainty_m=estimate.uncertainty,
            views_used=cloud.views,
            confidence=estimate.confidence,
            valid=True,
            rejection_reason=None,
        ),
        estimate=estimate,
        cloud=cloud,
        views=tuple(views),
        rejected_views=tuple(rejected),
        perception_latency_s=perception_latency,
        cycle_duration_s=cycle_duration,
    )


def profile_seed(
    seed: int,
    *,
    config: AppConfig | None = None,
    on_state: Callable[[str], None] | None = None,
) -> ProfilingResult:
    config = config or AppConfig()
    environment = ProfilingEnvironment.for_seed(seed, config, attach_box=False)
    return profile(environment, seed=seed, config=config, on_state=on_state)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Mide una caja suspendida con la trayectoria fija y emite ObjectDimensions."
    )
    parser.add_argument("--seed", type=int, default=42, help="Seed que genera la caja del episodio.")
    parser.add_argument("--quiet", action="store_true", help="No imprime la maquina de estados.")
    parser.add_argument("--output", type=Path, help="Ruta opcional para el JSON de salida.")
    args = parser.parse_args()

    announce = None if args.quiet else (lambda name: print(f"[profiling] {name}"))
    result = profile_seed(args.seed, on_state=announce)
    payload = json.dumps(result.dimensions.to_dict(), indent=2, sort_keys=True)
    print(payload)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload + "\n", encoding="utf-8")
    return 0 if result.dimensions.valid else 1


if __name__ == "__main__":
    raise SystemExit(main())
