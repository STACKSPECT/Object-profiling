from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass, replace
from pathlib import Path

import cv2
import numpy as np

from ..config import AppConfig
from ..contracts import ObjectDimensions
from ..station.environment import BoxSpec, ProfilingEnvironment, generate_box_spec
from ..station.pipeline import ProfilingResult, profile
from .panels import (
    ACCENT,
    BORDER,
    EVALUATION_INK,
    FONT,
    HEADER_HEIGHT,
    INK,
    WARNING,
    Line,
    bar_row,
    text_panel,
    tile,
)

CASE_TILE = (260, 195)
TEXT_WIDTH = 440
SHEET_WIDTH = CASE_TILE[0] * 2 + TEXT_WIDTH
# Escala de las barras del panel agregado. El objetivo del plan es 5 mm de MAE.
ERROR_BAR_LIMIT_MM = 5.0
SHOWCASE_CASE_COUNT = 6


@dataclass(frozen=True)
class ShowcaseCase:
    """Caja del recorrido. Si no trae spec, se genera con la seed del catalogo."""

    label: str
    seed: int
    box_spec: BoxSpec | None = None

    def spec(self, config: AppConfig) -> BoxSpec:
        if self.box_spec is not None:
            return self.box_spec
        intact = replace(config, damage=replace(config.damage, rate=0.0))
        return generate_box_spec(self.seed, intact)


def contrast_cases(*, base_seed: int) -> tuple[ShowcaseCase, ...]:
    """Seis cajas aleatorias del rango, en la rejilla de 5 mm.

    `generate_box_spec` ya recorta al rango y ajusta al catalogo. Las dimensiones
    reales no salen de la capa de presentacion: el estimador solo ve RGB-D.
    """

    return tuple(
        ShowcaseCase(f"aleatoria {index + 1}", base_seed + index)
        for index in range(SHOWCASE_CASE_COUNT)
    )


@dataclass(frozen=True)
class ShowcaseOutcome:
    case: ShowcaseCase
    result: ProfilingResult
    truth_m: np.ndarray

    @property
    def dimensions(self) -> ObjectDimensions:
        return self.result.dimensions

    @property
    def error_mm(self) -> np.ndarray | None:
        estimated = self.dimensions.dimensions_m
        if estimated is None:
            return None
        return (estimated.as_array() - self.truth_m) * 1000.0


def measure_case(
    case: ShowcaseCase,
    environment: ProfilingEnvironment,
    *,
    config: AppConfig | None = None,
    on_state=None,
    on_step=None,
) -> ShowcaseOutcome:
    config = config or AppConfig()
    environment.load_box(case.spec(config))
    # Ground truth solo para el panel de evaluacion. `profile()` no lo recibe.
    truth_m = environment.box_spec.dimensions_m.as_array()
    result = profile(environment, seed=case.seed, config=config, on_state=on_state, on_step=on_step)
    return ShowcaseOutcome(case, result, truth_m)


def _format_mm(values_m: np.ndarray) -> str:
    millimetres = values_m * 1000.0
    return f"{millimetres[0]:7.2f} x {millimetres[1]:7.2f} x {millimetres[2]:7.2f}"


def _comparison_lines(outcome: ShowcaseOutcome) -> list[Line]:
    dimensions = outcome.dimensions
    lines: list[Line] = [
        (f"{outcome.case.label}   seed {outcome.case.seed}", INK),
        ("              inicial     medido      real      error", INK),
    ]
    if not dimensions.valid or dimensions.dimensions_m is None:
        lines.append(("RECHAZADO", WARNING))
        lines.append((f"motivo {dimensions.rejection_reason}", WARNING))
        return lines

    initial = dimensions.dimensions_m.as_array() * 1000.0
    snapped = dimensions.catalogue_dimensions()
    measured = snapped.as_array() * 1000.0 if snapped is not None else initial
    truth = outcome.truth_m * 1000.0
    uncertainty = dimensions.uncertainty_m.as_array() * 1000.0
    error = outcome.error_mm
    for index, label in enumerate(("longitud", "anchura", "altura")):
        lines.append(
            (
                f"{label:9s} {initial[index]:8.2f}    {measured[index]:8.2f}    {truth[index]:8.2f}"
                f"   {error[index]:+7.3f} mm",
                ACCENT,
            )
        )
    covered = bool(np.all(np.abs(error) <= uncertainty))
    lines.append(
        (
            f"incertidumbre +/- {uncertainty.max():.2f} mm   cubre el error: {'si' if covered else 'NO'}",
            EVALUATION_INK if covered else WARNING,
        )
    )
    lines.append(
        (
            f"confianza {dimensions.confidence:.3f}   ciclo {outcome.result.cycle_duration_s:.2f} s"
            f"   percepcion {outcome.result.perception_latency_s * 1000:.0f} ms",
            INK,
        )
    )
    return lines


def _mask_window(mask: np.ndarray, aspect: float, padding: int = 26) -> tuple[slice, slice]:
    """Ventana centrada en la caja, con la proporcion del recuadro destino.

    Recortar al contorno observado hace visible el contraste de forma entre
    cajas, que a tamano completo se pierde entre suelo y brazo.
    """

    rows, columns = np.nonzero(mask)
    if rows.size == 0:
        return slice(0, mask.shape[0]), slice(0, mask.shape[1])

    top, bottom = int(rows.min()) - padding, int(rows.max()) + padding
    left, right = int(columns.min()) - padding, int(columns.max()) + padding
    height, width = bottom - top, right - left
    if width / height < aspect:
        width = int(round(height * aspect))
    else:
        height = int(round(width / aspect))

    centre_row, centre_column = (top + bottom) // 2, (left + right) // 2
    top = min(max(0, centre_row - height // 2), max(0, mask.shape[0] - height))
    left = min(max(0, centre_column - width // 2), max(0, mask.shape[1] - width))
    return slice(top, top + height), slice(left, left + width)


def _case_row(outcome: ShowcaseOutcome) -> np.ndarray:
    views = outcome.result.views
    row_height = CASE_TILE[1] + HEADER_HEIGHT

    def _view_rgb(index: int, fallback: str) -> np.ndarray:
        if index >= len(views):
            return tile(np.zeros((CASE_TILE[1], CASE_TILE[0], 3), dtype=np.uint8), fallback, CASE_TILE)
        view = views[index]
        rows, columns = _mask_window(view.mask, CASE_TILE[0] / CASE_TILE[1])
        rgb = cv2.cvtColor(view.rgb, cv2.COLOR_RGB2BGR)[rows, columns]
        return tile(rgb, f"RGB  {view.pose_name}", CASE_TILE)

    return np.hstack(
        [
            _view_rgb(0, "RGB  SCAN_YAW_0"),
            _view_rgb(1, "RGB  SCAN_YAW_90"),
            text_panel(_comparison_lines(outcome), TEXT_WIDTH, row_height, scale=0.44),
        ]
    )


def _aggregate_panel(outcomes: list[ShowcaseOutcome], height: int = 176) -> np.ndarray:
    canvas = np.zeros((height, SHEET_WIDTH, 3), dtype=np.uint8)
    valid = [outcome for outcome in outcomes if outcome.error_mm is not None]
    cv2.putText(
        canvas,
        f"RESUMEN   {len(valid)} de {len(outcomes)} perfiles validos"
        f"   barra a escala del objetivo de {ERROR_BAR_LIMIT_MM:.0f} mm de MAE",
        (12, 24),
        FONT,
        0.48,
        INK,
        1,
        cv2.LINE_AA,
    )
    if valid:
        errors = np.abs(np.asarray([outcome.error_mm for outcome in valid]))
        for index, label in enumerate(("longitud", "anchura", "altura")):
            bar_row(
                canvas,
                44 + 26 * index,
                f"MAE {label}",
                float(np.mean(errors[:, index])),
                ERROR_BAR_LIMIT_MM,
                ACCENT,
            )
        bar_row(
            canvas,
            44 + 26 * 3,
            "peor error absoluto",
            float(np.max(errors)),
            ERROR_BAR_LIMIT_MM,
            EVALUATION_INK,
        )
        cv2.putText(
            canvas,
            "El panel de la derecha de cada fila compara contra el ground truth."
            " La solucion no lo consume.",
            (12, height - 14),
            FONT,
            0.42,
            EVALUATION_INK,
            1,
            cv2.LINE_AA,
        )
    cv2.rectangle(canvas, (0, 0), (SHEET_WIDTH - 1, height - 1), BORDER, 1)
    return canvas


def compose_sheet(outcomes: list[ShowcaseOutcome]) -> np.ndarray:
    """Hoja resumen: una fila por caja mas el panel agregado."""

    header = text_panel(
        [
            (
                "Object Profiling   medicion suspendida con trayectoria fija"
                "   SCAN_YAW_0 / SCAN_YAW_90   cajas aleatorias 5 mm",
                INK,
            )
        ],
        SHEET_WIDTH,
        HEADER_HEIGHT,
    )
    return np.vstack([header] + [_case_row(outcome) for outcome in outcomes] + [_aggregate_panel(outcomes)])


def run_showcase(
    *,
    visual: bool,
    speed: float = 1.0,
    cases: tuple[ShowcaseCase, ...] | None = None,
    config: AppConfig | None = None,
    announce: bool = True,
    seed: int | None = None,
) -> tuple[list[ShowcaseOutcome], np.ndarray]:
    """Mide varias cajas seguidas. Visual y headless comparten `profile()`."""

    config = config or AppConfig()
    if cases is None:
        if seed is None:
            seed = int(np.random.default_rng().integers(0, 1_000_000))
        cases = contrast_cases(base_seed=seed)
        if announce:
            print(
                f"[showcase] seis cajas aleatorias  seed base {seed}"
                f"  (reproducir con --seed {seed})"
            )
    # Un solo entorno para todas las cajas: MuJoCo admite un visor por proceso,
    # y las dimensiones se reconfiguran sobre el mismo modelo.
    environment = ProfilingEnvironment.create(cases[0].spec(config), config, attach_box=False)
    outcomes: list[ShowcaseOutcome] = []

    def measure(case: ShowcaseCase, on_state, on_step) -> None:
        outcome = measure_case(case, environment, config=config, on_state=on_state, on_step=on_step)
        outcomes.append(outcome)
        if announce:
            print(_table_row(outcome))

    if not visual:
        for index, case in enumerate(cases, start=1):
            prefix = f"[showcase {index}/{len(cases)}]"
            reporter = (lambda name, prefix=prefix: print(f"{prefix} {name}")) if announce else None
            measure(case, reporter, None)
        return outcomes, compose_sheet(outcomes)

    import mujoco.viewer

    with mujoco.viewer.launch_passive(environment.model, environment.data) as viewer:
        viewer.opt.geomgroup[5] = 1

        def animate() -> None:
            viewer.sync()
            time.sleep(environment.model.opt.timestep / speed)

        for index, case in enumerate(cases, start=1):
            prefix = f"[showcase {index}/{len(cases)}] {case.label}"

            def reporter(name: str, prefix=prefix) -> None:
                print(f"{prefix}  {name}")
                viewer.sync()
                time.sleep(0.4 / speed)

            measure(case, reporter, animate)

    return outcomes, compose_sheet(outcomes)


def _table_row(outcome: ShowcaseOutcome) -> str:
    dimensions = outcome.dimensions
    if not dimensions.valid or dimensions.dimensions_m is None:
        return f"  {outcome.case.label:24s} seed {outcome.case.seed:5d}  RECHAZADO {dimensions.rejection_reason}"
    published = dimensions.catalogue_dimensions()
    if published is None:
        published = dimensions.dimensions_m
    error = outcome.error_mm
    return (
        f"  {outcome.case.label:24s} seed {outcome.case.seed:5d}  "
        f"medido_inicial {_format_mm(dimensions.dimensions_m.as_array())}  "
        f"medido {_format_mm(published.as_array())}  "
        f"real {_format_mm(outcome.truth_m)}  "
        f"error {error[0]:+6.3f} {error[1]:+6.3f} {error[2]:+6.3f} mm"
    )


def summary(outcomes: list[ShowcaseOutcome]) -> dict:
    valid = [outcome for outcome in outcomes if outcome.error_mm is not None]
    errors = np.abs(np.asarray([outcome.error_mm for outcome in valid])) if valid else np.empty((0, 3))
    return {
        "schema_version": 1,
        "checkpoint": "multi_box_showcase",
        "cases": len(outcomes),
        "valid_profiles": len(valid),
        "mae_mm": {
            axis: float(np.mean(errors[:, index])) if valid else None
            for index, axis in enumerate(("length", "width", "height"))
        },
        "worst_absolute_error_mm": float(np.max(errors)) if valid else None,
        "results": [
            {
                "label": outcome.case.label,
                "seed": outcome.case.seed,
                "ground_truth_mm": (outcome.truth_m * 1000.0).tolist(),
                "error_mm": outcome.error_mm.tolist() if outcome.error_mm is not None else None,
                "prediction": outcome.dimensions.to_dict(),
            }
            for outcome in outcomes
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Mide seis cajas aleatorias del catalogo y compara medido frente a real."
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--visual", action="store_true", help="Abre el visor de MuJoCo por caja. Requiere mjpython."
    )
    mode.add_argument("--headless", action="store_true", help="Sin ventana. Modo por defecto.")
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Seed base de las seis cajas. Si se omite, se elige al azar.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("artifacts/showcase/showcase.png"),
        help="Ruta de la hoja resumen PNG.",
    )
    parser.add_argument("--report", type=Path, help="Ruta opcional para el resumen JSON.")
    args = parser.parse_args()

    outcomes, sheet = run_showcase(visual=args.visual, speed=args.speed, seed=args.seed)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(args.output), sheet)

    report = summary(outcomes)
    print(f"\n[showcase] {report['valid_profiles']} de {report['cases']} perfiles validos")
    if report["worst_absolute_error_mm"] is not None:
        print(f"[showcase] peor error absoluto {report['worst_absolute_error_mm']:.3f} mm")
    print(f"[showcase] hoja en {args.output}")
    if args.report is not None:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0 if report["valid_profiles"] == report["cases"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
