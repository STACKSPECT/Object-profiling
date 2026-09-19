from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any

import mujoco
import numpy as np

from ..contracts import Dimensions3D, Extent3D, ObjectDimensions
from ..station.environment import ProfilingEnvironment


@dataclass(frozen=True)
class EvaluationRecord:
    """Prediccion emparejada con el ground truth de su episodio."""

    seed: int
    ground_truth_m: Dimensions3D
    prediction: ObjectDimensions
    absolute_error_m: Extent3D | None
    perception_latency_s: float

    def to_dict(self) -> dict[str, Any]:
        return {
            "seed": self.seed,
            "ground_truth_m": asdict(self.ground_truth_m),
            "prediction": self.prediction.to_dict(),
            "absolute_error_m": asdict(self.absolute_error_m) if self.absolute_error_m else None,
            "perception_latency_s": self.perception_latency_s,
        }


TERMINAL_GEOM_NAMES = (
    "gripper_hub",
    "gripper_beam_x",
    "gripper_beam_y",
    "cup_center",
    "cup_x_pos",
    "cup_x_neg",
    "cup_y_pos",
    "cup_y_neg",
)


@dataclass(frozen=True)
class GroundTruthMasks:
    box: np.ndarray
    terminal: np.ndarray


@dataclass(frozen=True)
class RegistrationMetrics:
    """Error de registro medido contra la pose y dimensiones reales.

    Combina el error de retroproyeccion, el de la cadena camara-mundo-terminal y
    el de la propia segmentacion. Es la vara de medir del registro, no una
    entrada del estimador.
    """

    points: int
    mean_surface_distance_m: float
    p95_surface_distance_m: float
    maximum_surface_distance_m: float


@dataclass(frozen=True)
class SegmentationMetrics:
    predicted_pixels: int
    ground_truth_pixels: int
    intersection_over_union: float
    precision: float
    recall: float
    false_terminal_pixels: int
    missed_box_pixels: int
    false_other_pixels: int


class GroundTruthRenderer:
    """Renderiza mascaras ground truth para evaluar, nunca para estimar.

    Depende de IDs de geometria de MuJoCo. Ningun modulo de la solucion debe
    importar este.
    """

    def __init__(self, environment: ProfilingEnvironment):
        self.environment = environment
        sensor = environment.config.sensor
        self.renderer = mujoco.Renderer(environment.model, height=sensor.height, width=sensor.width)
        self.box_geom_id = mujoco.mj_name2id(environment.model, mujoco.mjtObj.mjOBJ_GEOM, "box_geom")
        self.terminal_geom_ids = tuple(
            mujoco.mj_name2id(environment.model, mujoco.mjtObj.mjOBJ_GEOM, name)
            for name in TERMINAL_GEOM_NAMES
        )

    def close(self) -> None:
        self.renderer.close()

    def masks(self, camera_name: str = "scan_rgbd_cam") -> GroundTruthMasks:
        self.renderer.enable_segmentation_rendering()
        self.renderer.update_scene(self.environment.data, camera=camera_name)
        segmentation = self.renderer.render()[:, :, 0].copy()
        self.renderer.disable_segmentation_rendering()
        return GroundTruthMasks(
            box=segmentation == self.box_geom_id,
            terminal=np.isin(segmentation, self.terminal_geom_ids),
        )


def box_to_tool(environment: ProfilingEnvironment) -> np.ndarray:
    """Pose real de la caja en el marco del terminal. Solo evaluacion."""

    return np.linalg.inv(environment.tool_to_world()) @ environment.body_to_world("profiling_box")


def evaluate_registration(
    points_tool_m: np.ndarray,
    box_to_tool_transform: np.ndarray,
    dimensions_m: np.ndarray,
) -> RegistrationMetrics:
    """Distancia de cada punto observado a la superficie real de la caja."""

    rotation = box_to_tool_transform[:3, :3]
    translation = box_to_tool_transform[:3, 3]
    points_box = (points_tool_m - translation) @ rotation
    offset = np.abs(points_box) - dimensions_m / 2.0
    outside = np.linalg.norm(np.maximum(offset, 0.0), axis=1)
    inside = np.minimum(np.max(offset, axis=1), 0.0)
    distance = np.abs(outside + inside)
    return RegistrationMetrics(
        points=int(distance.size),
        mean_surface_distance_m=float(np.mean(distance)),
        p95_surface_distance_m=float(np.percentile(distance, 95)),
        maximum_surface_distance_m=float(np.max(distance)),
    )


def evaluate_segmentation(predicted: np.ndarray, truth: GroundTruthMasks) -> SegmentationMetrics:
    intersection = int(np.count_nonzero(predicted & truth.box))
    union = int(np.count_nonzero(predicted | truth.box))
    predicted_pixels = int(np.count_nonzero(predicted))
    truth_pixels = int(np.count_nonzero(truth.box))
    false_positive = predicted & ~truth.box
    return SegmentationMetrics(
        predicted_pixels=predicted_pixels,
        ground_truth_pixels=truth_pixels,
        intersection_over_union=intersection / union if union else 0.0,
        precision=intersection / predicted_pixels if predicted_pixels else 0.0,
        recall=intersection / truth_pixels if truth_pixels else 0.0,
        false_terminal_pixels=int(np.count_nonzero(false_positive & truth.terminal)),
        missed_box_pixels=truth_pixels - intersection,
        false_other_pixels=int(np.count_nonzero(false_positive & ~truth.terminal)),
    )
