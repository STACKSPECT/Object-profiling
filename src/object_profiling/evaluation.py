from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from .environment import ProfilingEnvironment


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
