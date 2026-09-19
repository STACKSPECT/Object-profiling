from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2
import numpy as np

from .config import AppConfig
from .contracts import ObjectDimensions
from .environment import ProfilingEnvironment
from .profiling_pipeline import ProfilingResult, profile

TILE = (320, 240)
HEADER_HEIGHT = 34
FONT = cv2.FONT_HERSHEY_SIMPLEX
INK = (235, 235, 235)
ACCENT = (120, 220, 120)
WARNING = (110, 140, 250)
EVALUATION_INK = (200, 180, 120)


def _tile(image: np.ndarray, caption: str) -> np.ndarray:
    resized = cv2.resize(image, TILE, interpolation=cv2.INTER_AREA)
    if resized.ndim == 2:
        resized = cv2.cvtColor(resized, cv2.COLOR_GRAY2BGR)
    panel = np.zeros((TILE[1] + HEADER_HEIGHT, TILE[0], 3), dtype=np.uint8)
    panel[HEADER_HEIGHT:] = resized
    cv2.putText(panel, caption, (8, 22), FONT, 0.45, INK, 1, cv2.LINE_AA)
    cv2.rectangle(panel, (0, 0), (TILE[0] - 1, TILE[1] + HEADER_HEIGHT - 1), (60, 60, 60), 1)
    return panel


def _depth_tile(depth_m: np.ndarray, mask: np.ndarray, caption: str) -> np.ndarray:
    inside = depth_m[mask] if mask.any() else depth_m[np.isfinite(depth_m)]
    low, high = float(np.min(inside)), float(np.max(inside))
    span = max(high - low, 1e-6)
    normalised = np.clip((depth_m - low) / span, 0.0, 1.0)
    coloured = cv2.applyColorMap((255 * (1.0 - normalised)).astype(np.uint8), cv2.COLORMAP_TURBO)
    return _tile(coloured, caption)


def _cloud_tile(
    points_m: np.ndarray,
    lower_m: np.ndarray | None,
    upper_m: np.ndarray | None,
    axes: tuple[int, int],
    caption: str,
) -> np.ndarray:
    canvas = np.full((TILE[1], TILE[0], 3), 18, dtype=np.uint8)
    if points_m.shape[0] == 0:
        return _tile(canvas, caption)

    horizontal, vertical = axes
    span = 0.46
    centre = np.asarray([0.0, 0.0, 0.089 + 0.165])
    scale = min(TILE[0], TILE[1]) / span

    def to_pixel(values: np.ndarray) -> np.ndarray:
        column = (values[:, horizontal] - centre[horizontal]) * scale + TILE[0] / 2.0
        row = (values[:, vertical] - centre[vertical]) * scale + TILE[1] / 2.0
        return np.column_stack([column, row]).astype(np.int32)

    pixels = to_pixel(points_m)
    inside = (
        (pixels[:, 0] >= 0) & (pixels[:, 0] < TILE[0]) & (pixels[:, 1] >= 0) & (pixels[:, 1] < TILE[1])
    )
    canvas[pixels[inside, 1], pixels[inside, 0]] = (108, 122, 130)

    if lower_m is not None and upper_m is not None:
        box = to_pixel(np.asarray([lower_m, upper_m]))
        cv2.rectangle(canvas, tuple(box[0]), tuple(box[1]), ACCENT, 2)
    return _tile(canvas, caption)


def _text_panel(lines: list[tuple[str, tuple[int, int, int]]], width: int, height: int) -> np.ndarray:
    panel = np.zeros((height, width, 3), dtype=np.uint8)
    for index, (text, colour) in enumerate(lines):
        cv2.putText(panel, text, (12, 26 + 21 * index), FONT, 0.46, colour, 1, cv2.LINE_AA)
    cv2.rectangle(panel, (0, 0), (width - 1, height - 1), (60, 60, 60), 1)
    return panel


def _solution_lines(result: ObjectDimensions) -> list[tuple[str, tuple[int, int, int]]]:
    lines: list[tuple[str, tuple[int, int, int]]] = [("SOLUCION", INK)]
    if result.valid and result.dimensions_m is not None:
        dimensions = result.dimensions_m.as_array() * 1000.0
        uncertainty = result.uncertainty_m.as_array() * 1000.0
        for label, value, error in zip(("longitud", "anchura", "altura"), dimensions, uncertainty):
            lines.append((f"{label:9s} {value:7.1f} +/- {error:4.2f} mm", ACCENT))
        lines.append((f"confianza {result.confidence:.3f}", INK))
    else:
        lines.append(("RECHAZADO", WARNING))
        lines.append((f"motivo {result.rejection_reason}", WARNING))
        lines.append(("", INK))
        lines.append(("", INK))
    lines.append((f"{len(result.views_used)} vistas  marco {result.frame_id}", INK))
    return lines


def _evaluation_lines(
    result: ObjectDimensions, truth_m: np.ndarray
) -> list[tuple[str, tuple[int, int, int]]]:
    lines: list[tuple[str, tuple[int, int, int]]] = [("EVALUACION (fuera de la solucion)", EVALUATION_INK)]
    for label, value in zip(("longitud", "anchura", "altura"), truth_m * 1000.0):
        lines.append((f"real {label:9s} {value:7.1f} mm", EVALUATION_INK))
    if result.valid and result.dimensions_m is not None:
        error = (result.dimensions_m.as_array() - truth_m) * 1000.0
        covered = np.all(np.abs(error) <= result.uncertainty_m.as_array() * 1000.0)
        lines.append((f"error {np.round(error, 3).tolist()} mm", EVALUATION_INK))
        lines.append((f"dentro de incertidumbre: {'si' if covered else 'no'}", EVALUATION_INK))
    return lines


def compose_panels(result: ProfilingResult, truth_m: np.ndarray, state: str) -> np.ndarray:
    """Compone el mosaico de la demo.

    La mascara mostrada es la observable, la que produce la solucion. El ground
    truth aparece unicamente en el panel de evaluacion.
    """

    rgb_row = [
        _tile(cv2.cvtColor(view.rgb, cv2.COLOR_RGB2BGR), f"RGB {view.pose_name}")
        for view in result.views
    ]
    depth_row = [
        _depth_tile(view.depth_m, view.mask, f"Profundidad {view.pose_name}") for view in result.views
    ]
    mask_row = [
        _tile(view.mask.astype(np.uint8) * 255, f"Mascara observable {view.pose_name}")
        for view in result.views
    ]

    points = result.cloud.points_m if result.cloud is not None else np.empty((0, 3))
    lower = result.estimate.lower_m if result.estimate is not None else None
    upper = result.estimate.upper_m if result.estimate is not None else None
    bottom_row = [
        _cloud_tile(points, lower, upper, (0, 1), "Nube fusionada  terminal XY"),
        _cloud_tile(points, lower, upper, (0, 2), "Nube fusionada  terminal XZ"),
    ]

    width = TILE[0] * 3
    rows = [np.hstack(row) for row in (rgb_row, depth_row, mask_row) if row]
    tile_height = TILE[1] + HEADER_HEIGHT
    text_height = tile_height
    bottom = np.hstack(
        bottom_row
        + [
            np.vstack(
                [
                    _text_panel(_solution_lines(result.dimensions), TILE[0], text_height // 2),
                    _text_panel(
                        _evaluation_lines(result.dimensions, truth_m), TILE[0], text_height - text_height // 2
                    ),
                ]
            )
        ]
    )
    header = _text_panel(
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
    print(f"[demo] valido={result.dimensions.valid} motivo={result.dimensions.rejection_reason}")
    return 0 if result.dimensions.valid else 1


if __name__ == "__main__":
    raise SystemExit(main())
