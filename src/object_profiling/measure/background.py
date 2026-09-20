"""MERGE: fondos de estacion vacia por pose. Solo el tipo; la captura es OVERLAP."""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass

import numpy as np

from ..contracts import RejectionReason


class MissingBackgroundError(KeyError):
    """No existe fondo calibrado para una pose de escaneo."""

    def __init__(self, pose_name: str):
        super().__init__(pose_name)
        self.pose_name = pose_name
        self.reason = RejectionReason.MISSING_BACKGROUND


@dataclass(frozen=True)
class PoseBackground:
    """Profundidad de la estacion vacia en una pose concreta.

    El brazo y el terminal cambian de sitio entre poses, asi que un unico
    fondo no sirve para las tres. La asociacion es por `pose_name`.
    """

    pose_name: str
    depth_m: np.ndarray
    joint_positions_rad: np.ndarray


@dataclass(frozen=True)
class BackgroundSet:
    backgrounds: tuple[PoseBackground, ...]

    def depth_for(self, pose_name: str) -> np.ndarray:
        """In: nombre de pose. Out: profundidad (H, W) en metros. Raise si falta."""
        for background in self.backgrounds:
            if background.pose_name == pose_name:
                return background.depth_m
        raise MissingBackgroundError(pose_name)

    def joint_positions_for(self, pose_name: str) -> np.ndarray:
        for background in self.backgrounds:
            if background.pose_name == pose_name:
                return background.joint_positions_rad
        raise MissingBackgroundError(pose_name)

    @property
    def pose_names(self) -> tuple[str, ...]:
        return tuple(background.pose_name for background in self.backgrounds)

    def covers(self, pose_names: Iterable[str]) -> bool:
        available = set(self.pose_names)
        return all(name in available for name in pose_names)
