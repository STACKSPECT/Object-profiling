"""Orquestador del ciclo en MuJoCo.

Es el adaptador entre la escena de este repositorio y `measure.measure()`.
Todo lo que depende de MuJoCo vive aqui; la medicion en si no depende de nada de
esto.
"""

from __future__ import annotations

import argparse
import json
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from ..config import AppConfig
from ..contracts import ObjectDimensions, RejectionReason, RoutingHint
from ..measure.background import BackgroundSet
from ..measure.measurement import (
    DEFAULT_BOOTSTRAP_SEED,
    MeasurementResult,
    measure,
    rejected_measurement,
)
from ..measure.perception import ScanView
from ..measure.registration import FusedCloud
from .camera import RGBDSensor, RenderError
from .controller import MotionError
from .discard import discard_to_error_zone
from .environment import ProfilingEnvironment
from .poses import INSPECTION_POSES, SCAN_POSES
from .scanning import calibrate_backgrounds, run_fixed_scan


@dataclass(frozen=True)
class ProfilingResult:
    dimensions: ObjectDimensions
    measurement: MeasurementResult
    perception_latency_s: float
    # Tiempo de pared del simulador. No es el tiempo de ciclo de la estacion.
    wall_clock_duration_s: float
    # Tiempo simulado del ciclo, que es el que tardaria el robot.
    simulated_duration_s: float
    # Pose caja→terminal al terminar el escaneo, antes de un posible descarte.
    # Solo evaluacion de registro: el estimador no la ve.
    box_to_tool_m: np.ndarray | None = None

    @property
    def cycle_duration_s(self) -> float:
        return self.wall_clock_duration_s

    @property
    def estimate(self):
        return self.measurement.estimate

    @property
    def cloud(self) -> FusedCloud | None:
        return self.measurement.cloud

    @property
    def views(self) -> tuple[ScanView, ...]:
        return self.measurement.views

    @property
    def rejected_views(self) -> tuple[tuple[str, str], ...]:
        return self.measurement.rejected_views


def profile(
    environment: ProfilingEnvironment,
    *,
    seed: int | None = None,
    config: AppConfig | None = None,
    bootstrap_seed: int = DEFAULT_BOOTSTRAP_SEED,
    backgrounds: BackgroundSet | None = None,
    on_state: Callable[[str], None] | None = None,
    on_step: Callable[[], None] | None = None,
) -> ProfilingResult:
    """Ejecuta el ciclo fijo y devuelve `ObjectDimensions`.

    La trayectoria no depende del resultado: las dos poses de medida se recorren
    siempre; el yaw 180 de inspeccion no entra en `measure()`.
    Recibe el episodio ya construido y solo su etiqueta, nunca su geometria.
    """

    config = config or AppConfig()
    _ = seed
    object_id = environment.object_id
    sensor = RGBDSensor(environment)
    report = on_state or (lambda _name: None)

    started = time.perf_counter()
    simulation_started = float(environment.data.time)
    try:
        cycle = run_fixed_scan(
            environment,
            sensor,
            backgrounds=backgrounds,
            on_state=report,
            on_step=on_step,
        )
    except (MotionError, RenderError) as error:
        failure = rejected_measurement(object_id, float(environment.data.time), error.reason)
        report(error.reason.value)
        return ProfilingResult(
            dimensions=failure.dimensions,
            measurement=failure,
            perception_latency_s=0.0,
            wall_clock_duration_s=time.perf_counter() - started,
            simulated_duration_s=float(environment.data.time) - simulation_started,
        )
    finally:
        sensor.close()

    box_to_tool = environment.attachment_box_transform()

    report("ESTIMATE")
    perception_started = time.perf_counter()
    result = measure(
        cycle.observations,
        cycle.backgrounds,
        config,
        object_id=object_id,
        bootstrap_seed=bootstrap_seed,
    )
    perception_latency = time.perf_counter() - perception_started

    report("VALIDATE")
    if result.dimensions.routing is RoutingHint.ERROR_ZONE:
        try:
            discard_to_error_zone(environment, on_state=report, on_step=on_step)
        except MotionError as error:
            report(error.reason.value)
    report("PROFILE_READY" if result.dimensions.valid else str(result.dimensions.rejection_reason))
    return ProfilingResult(
        dimensions=result.dimensions,
        measurement=result,
        perception_latency_s=perception_latency,
        wall_clock_duration_s=time.perf_counter() - started,
        simulated_duration_s=float(environment.data.time) - simulation_started,
        box_to_tool_m=box_to_tool,
    )


def profile_session(
    start_seed: int,
    count: int,
    *,
    config: AppConfig | None = None,
    environment: ProfilingEnvironment | None = None,
    on_state: Callable[[str], None] | None = None,
    on_step: Callable[[], None] | None = None,
    on_result: Callable[[ProfilingResult], None] | None = None,
) -> tuple[ProfilingResult, ...]:
    """Mide `count` cajas seguidas en la misma estacion.

    Cada ciclo genera una caja, la mide, decide si aceptarla o descartarla, y
    retira la caja antes de generar la siguiente. Los fondos se calibran una
    sola vez: ya no dependen de la geometria del episodio.
    """

    if count < 1:
        raise ValueError("count must be at least 1")

    config = config or AppConfig()
    report = on_state or (lambda _name: None)
    if environment is None:
        environment = ProfilingEnvironment.for_seed(start_seed, config, attach_box=False)

    sensor = RGBDSensor(environment)
    try:
        report("CALIBRATE_BACKGROUND")
        backgrounds = calibrate_backgrounds(
            environment, sensor, poses=(*SCAN_POSES, *INSPECTION_POSES)
        )
    finally:
        sensor.close()

    results: list[ProfilingResult] = []
    for index in range(count):
        seed = start_seed + index
        report(f"BOX_{index + 1}_OF_{count}")
        environment.load_seed(seed, attach_box=False)
        result = profile(
            environment,
            seed=seed,
            config=config,
            backgrounds=backgrounds,
            on_state=on_state,
            on_step=on_step,
        )
        if on_result is not None:
            on_result(result)
        results.append(result)
        if index + 1 < count:
            environment.set_box_visible(False)
            report("CLEAR_BOX")
    return tuple(results)


def profile_seed(
    seed: int,
    *,
    config: AppConfig | None = None,
    on_state: Callable[[str], None] | None = None,
) -> ProfilingResult:
    config = config or AppConfig()
    environment = ProfilingEnvironment.for_seed(seed, config, attach_box=False)
    return profile(environment, config=config, on_state=on_state)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Mide cajas suspendidas con la trayectoria fija y emite ObjectDimensions."
    )
    parser.add_argument("--seed", type=int, default=42, help="Seed de la primera caja. Las siguientes usan seed+i.")
    parser.add_argument(
        "--count",
        type=int,
        default=1,
        help="Numero de cajas del bucle. Cada una se mide, se acepta o se descarta, y se retira.",
    )
    parser.add_argument("--quiet", action="store_true", help="No imprime la maquina de estados.")
    parser.add_argument("--output", type=Path, help="Ruta opcional para el JSON de salida.")
    args = parser.parse_args()
    if args.count < 1:
        parser.error("--count must be at least 1")

    announce = None if args.quiet else (lambda name: print(f"[profiling] {name}"))
    results = profile_session(args.seed, args.count, on_state=announce)
    payload_obj: object
    if len(results) == 1:
        payload_obj = results[0].dimensions.to_dict()
    else:
        payload_obj = [result.dimensions.to_dict() for result in results]
    payload = json.dumps(payload_obj, indent=2, sort_keys=True)
    print(payload)
    if not args.quiet:
        for result in results:
            print(
                f"[profiling] {result.dimensions.object_id}"
                f"   {result.dimensions.condition.value}"
                f"   {result.dimensions.routing.value}"
                f"   ciclo simulado {result.simulated_duration_s:.3f} s"
                f"   percepcion {result.perception_latency_s * 1000:.0f} ms"
            )
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload + "\n", encoding="utf-8")
    return 0 if all(result.dimensions.valid for result in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
