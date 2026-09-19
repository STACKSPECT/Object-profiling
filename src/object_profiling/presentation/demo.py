from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2
import numpy as np

from ..config import AppConfig
from ..contracts import ObjectDimensions
from ..station.environment import ProfilingEnvironment
from ..station.pipeline import ProfilingResult, profile
from .panels import (
    ACCENT,
    EVALUATION_INK,
    HEADER_HEIGHT,
    INK,
    TILE,
    WARNING,
    Line,
    cloud_tile,
    depth_tile,
    text_panel,
    tile,
)


def _format_mm(values_m: np.ndarray) -> str:
    millimetres = values_m * 1000.0
    return f"{millimetres[0]:7.2f} x {millimetres[1]:7.2f} x {millimetres[2]:7.2f}"


def _solution_lines(result: ObjectDimensions) -> list[Line]:
    lines: list[Line] = [("SOLUCION", INK)]
    if result.valid and result.dimensions_m is not None:
        initial = result.dimensions_m.as_array() * 1000.0
        published = result.catalogue_dimensions()
        measured = published.as_array() * 1000.0 if published is not None else initial
        uncertainty = result.uncertainty_m.as_array() * 1000.0
        for index, label in enumerate(("longitud", "anchura", "altura")):
            lines.append(
                (
                    f"{label:9s} ini {initial[index]:7.2f}  med {measured[index]:7.2f}"
                    f"  +/- {uncertainty[index]:4.2f}",
                    ACCENT,
                )
            )
        lines.append((f"confianza {result.confidence:.3f}", INK))
    else:
        lines.append(("RECHAZADO", WARNING))
        lines.append((f"motivo {result.rejection_reason}", WARNING))
        lines.append(("", INK))
        lines.append(("", INK))
    lines.append((f"{len(result.views_used)} vistas  marco {result.frame_id}", INK))
    return lines


def _evaluation_lines(result: ObjectDimensions, truth_m: np.ndarray) -> list[Line]:
    lines: list[Line] = [("EVALUACION (fuera de la solucion)", EVALUATION_INK)]
    for label, value in zip(("longitud", "anchura", "altura"), truth_m * 1000.0):
        lines.append((f"real {label:9s} {value:7.2f} mm", EVALUATION_INK))
    if result.valid and result.dimensions_m is not None:
        error = (result.dimensions_m.as_array() - truth_m) * 1000.0
        covered = np.all(np.abs(error) <= result.uncertainty_m.as_array() * 1000.0)
        lines.append((f"error inicial {np.round(error, 3).tolist()} mm", EVALUATION_INK))
        lines.append((f"dentro de incertidumbre: {'si' if covered else 'no'}", EVALUATION_INK))
    return lines


def _console_row(result: ObjectDimensions, truth_m: np.ndarray) -> str:
    if not result.valid or result.dimensions_m is None:
        return f"RECHAZADO {result.rejection_reason}"
    published = result.catalogue_dimensions() or result.dimensions_m
    error = (result.dimensions_m.as_array() - truth_m) * 1000.0
    return (
        f"medido_inicial {_format_mm(result.dimensions_m.as_array())}  "
        f"medido {_format_mm(published.as_array())}  "
        f"real {_format_mm(truth_m)}  "
        f"error {error[0]:+6.3f} {error[1]:+6.3f} {error[2]:+6.3f} mm"
    )


def compose_panels(result: ProfilingResult, truth_m: np.ndarray, state: str) -> np.ndarray:
    """Compone el mosaico de la demo de una caja.

    La mascara mostrada es la observable, la que produce la solucion. El ground
    truth aparece unicamente en el panel de evaluacion.
    """

    rgb_row = [
        tile(cv2.cvtColor(view.rgb, cv2.COLOR_RGB2BGR), f"RGB {view.pose_name}")
        for view in result.views
    ]
    depth_row = [
        depth_tile(view.depth_m, view.mask, f"Profundidad {view.pose_name}") for view in result.views
    ]
    mask_row = [
        tile(view.mask.astype(np.uint8) * 255, f"Mascara observable {view.pose_name}")
        for view in result.views
    ]

    points = result.cloud.points_m if result.cloud is not None else np.empty((0, 3))
    lower = result.estimate.lower_m if result.estimate is not None else None
    upper = result.estimate.upper_m if result.estimate is not None else None

    width = TILE[0] * 3
    rows = [np.hstack(row) for row in (rgb_row, depth_row, mask_row) if row]
    tile_height = TILE[1] + HEADER_HEIGHT
    bottom = np.hstack(
        [
            cloud_tile(points, lower, upper, (0, 1), "Nube fusionada  terminal XY"),
            cloud_tile(points, lower, upper, (0, 2), "Nube fusionada  terminal XZ"),
            np.vstack(
                [
                    text_panel(_solution_lines(result.dimensions), TILE[0], tile_height // 2),
                    text_panel(
                        _evaluation_lines(result.dimensions, truth_m),
                        TILE[0],
                        tile_height - tile_height // 2,
                    ),
                ]
            ),
        ]
    )
    header = text_panel(
        [
            (
                f"{result.dimensions.object_id}   estado {state}   "
                f"ciclo {result.cycle_duration_s:.2f} s   percepcion {result.perception_latency_s:.3f} s",
                INK,
            )
        ],
        width,
        HEADER_HEIGHT,
    )
    return np.vstack([header] + rows + [bottom])


def run_demo(
    seed: int,
    *,
    visual: bool,
    speed: float,
    config: AppConfig | None = None,
) -> tuple[ProfilingResult, np.ndarray]:
    """Ejecuta la demo. Visual y headless comparten el mismo `profile()`."""

    config = config or AppConfig()
    environment = ProfilingEnvironment.for_seed(seed, config, attach_box=False)
    # El ground truth se lee aqui, en la capa de presentacion, y solo alimenta el
    # panel de evaluacion.
    truth_m = environment.box_spec.dimensions_m.as_array()
    states: list[str] = []

    def announce(name: str) -> None:
        states.append(name)
        print(f"[demo] {name}")

    if not visual:
        result = profile(environment, seed=seed, config=config, on_state=announce)
        print(f"[demo] {_console_row(result.dimensions, truth_m)}")
        return result, compose_panels(result, truth_m, states[-1] if states else "DONE")

    import mujoco.viewer

    with mujoco.viewer.launch_passive(environment.model, environment.data) as viewer:

        def animate() -> None:
            viewer.sync()
            time.sleep(environment.model.opt.timestep / speed)

        def announce_visual(name: str) -> None:
            announce(name)
            viewer.sync()
            time.sleep(0.5 / speed)

        result = profile(
            environment, seed=seed, config=config, on_state=announce_visual, on_step=animate
        )
        announce_visual("DONE")
    print(f"[demo] {_console_row(result.dimensions, truth_m)}")
    return result, compose_panels(result, truth_m, states[-1] if states else "DONE")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Demo del ciclo de medicion: paneles de RGB, profundidad, mascara, nube y resultado."
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--visual", action="store_true", help="Abre el visor de MuJoCo. Requiere mjpython.")
    mode.add_argument("--headless", action="store_true", help="Sin ventana. Modo por defecto.")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Ruta del mosaico PNG. Por defecto artifacts/demo/seed-NNNN.png.",
    )
    args = parser.parse_args()

    result, panels = run_demo(args.seed, visual=args.visual, speed=args.speed)
    output = args.output or Path("artifacts/demo") / f"seed-{args.seed:04d}.png"
    output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output), panels)
    print(f"[demo] mosaico en {output}")
    return 0 if result.dimensions.valid else 1


if __name__ == "__main__":
    raise SystemExit(main())
