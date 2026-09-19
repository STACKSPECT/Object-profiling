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

from ..config import AppConfig
from ..contracts import ObjectDimensions, RejectionReason
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
from .environment import ProfilingEnvironment
from .scanning import run_fixed_scan


@dataclass(frozen=True)
class ProfilingResult:
    dimensions: ObjectDimensions
    measurement: MeasurementResult
    perception_latency_s: float
    # Tiempo de pared del simulador. No es el tiempo de ciclo de la estacion.
    wall_clock_duration_s: float
    # Tiempo simulado del ciclo, que es el que tardaria el robot.
    simulated_duration_s: float

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

    La trayectoria no depende del resultado: las poses fijas se recorren siempre.
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

    wall_clock = time.perf_counter() - started
    simulated = float(environment.data.time) - simulation_started

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
    report("PROFILE_READY" if result.dimensions.valid else str(result.dimensions.rejection_reason))
    return ProfilingResult(
        dimensions=result.dimensions,
        measurement=result,
        perception_latency_s=perception_latency,
        wall_clock_duration_s=wall_clock,
        simulated_duration_s=simulated,
    )


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
    if not args.quiet:
        print(
            f"[profiling] ciclo simulado {result.simulated_duration_s:.3f} s"
            f"   percepcion {result.perception_latency_s * 1000:.0f} ms"
            f"   wall clock {result.wall_clock_duration_s:.3f} s"
        )
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload + "\n", encoding="utf-8")
    return 0 if result.dimensions.valid else 1


if __name__ == "__main__":
    raise SystemExit(main())
