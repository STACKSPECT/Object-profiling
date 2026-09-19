from __future__ import annotations

import cv2
import numpy as np

TILE = (320, 240)
HEADER_HEIGHT = 34
FONT = cv2.FONT_HERSHEY_SIMPLEX
INK = (235, 235, 235)
ACCENT = (120, 220, 120)
WARNING = (110, 140, 250)
EVALUATION_INK = (200, 180, 120)
BORDER = (60, 60, 60)

Line = tuple[str, tuple[int, int, int]]


def tile(image: np.ndarray, caption: str, size: tuple[int, int] = TILE) -> np.ndarray:
    """Encaja una imagen en un recuadro con titulo."""

    resized = cv2.resize(image, size, interpolation=cv2.INTER_AREA)
    if resized.ndim == 2:
        resized = cv2.cvtColor(resized, cv2.COLOR_GRAY2BGR)
    panel = np.zeros((size[1] + HEADER_HEIGHT, size[0], 3), dtype=np.uint8)
    panel[HEADER_HEIGHT:] = resized
    cv2.putText(panel, caption, (8, 22), FONT, 0.45, INK, 1, cv2.LINE_AA)
    cv2.rectangle(panel, (0, 0), (size[0] - 1, size[1] + HEADER_HEIGHT - 1), BORDER, 1)
    return panel


def depth_tile(
    depth_m: np.ndarray,
    mask: np.ndarray,
    caption: str,
    size: tuple[int, int] = TILE,
) -> np.ndarray:
    """Colorea la profundidad normalizando en el rango que ocupa la caja."""

    inside = depth_m[mask] if mask.any() else depth_m[np.isfinite(depth_m)]
    low, high = float(np.min(inside)), float(np.max(inside))
    normalised = np.clip((depth_m - low) / max(high - low, 1e-6), 0.0, 1.0)
    coloured = cv2.applyColorMap((255 * (1.0 - normalised)).astype(np.uint8), cv2.COLORMAP_TURBO)
    return tile(coloured, caption, size)


def cloud_tile(
    points_m: np.ndarray,
    lower_m: np.ndarray | None,
    upper_m: np.ndarray | None,
    axes: tuple[int, int],
    caption: str,
    size: tuple[int, int] = TILE,
    span_m: float = 0.46,
) -> np.ndarray:
    """Proyeccion ortografica de la nube en el marco del terminal."""

    canvas = np.full((size[1], size[0], 3), 18, dtype=np.uint8)
    if points_m.shape[0] == 0:
        return tile(canvas, caption, size)

    horizontal, vertical = axes
    centre = np.asarray([0.0, 0.0, 0.089 + 0.165])
    scale = min(size) / span_m

    def to_pixel(values: np.ndarray) -> np.ndarray:
        column = (values[:, horizontal] - centre[horizontal]) * scale + size[0] / 2.0
        row = (values[:, vertical] - centre[vertical]) * scale + size[1] / 2.0
        return np.column_stack([column, row]).astype(np.int32)

    pixels = to_pixel(points_m)
    inside = (
        (pixels[:, 0] >= 0) & (pixels[:, 0] < size[0]) & (pixels[:, 1] >= 0) & (pixels[:, 1] < size[1])
    )
    canvas[pixels[inside, 1], pixels[inside, 0]] = (108, 122, 130)

    if lower_m is not None and upper_m is not None:
        box = to_pixel(np.asarray([lower_m, upper_m]))
        cv2.rectangle(canvas, tuple(box[0]), tuple(box[1]), ACCENT, 2)
    return tile(canvas, caption, size)


def text_panel(lines: list[Line], width: int, height: int, *, scale: float = 0.46) -> np.ndarray:
    panel = np.zeros((height, width, 3), dtype=np.uint8)
    step = int(round(21 * scale / 0.46))
    for index, (text, colour) in enumerate(lines):
        cv2.putText(panel, text, (12, 24 + step * index), FONT, scale, colour, 1, cv2.LINE_AA)
    cv2.rectangle(panel, (0, 0), (width - 1, height - 1), BORDER, 1)
    return panel


def bar_row(
    canvas: np.ndarray,
    top: int,
    label: str,
    value_mm: float,
    limit_mm: float,
    colour: tuple[int, int, int],
    *,
    label_width: int = 150,
) -> None:
    """Dibuja una barra horizontal con su etiqueta y su valor."""

    width = canvas.shape[1]
    left = label_width
    right = width - 150
    cv2.putText(canvas, label, (12, top + 12), FONT, 0.44, INK, 1, cv2.LINE_AA)
    cv2.rectangle(canvas, (left, top), (right, top + 14), (42, 42, 42), -1)
    filled = int(left + (right - left) * min(1.0, value_mm / limit_mm))
    cv2.rectangle(canvas, (left, top), (filled, top + 14), colour, -1)
    cv2.putText(canvas, f"{value_mm:6.3f} mm", (right + 12, top + 12), FONT, 0.44, colour, 1, cv2.LINE_AA)
