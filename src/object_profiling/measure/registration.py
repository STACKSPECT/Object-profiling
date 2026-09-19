from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..contracts import ViewDescriptor
from .perception import ScanView


# Marco comun de fusion: el sitio de acople de la muneca del UR10e, que es el
# marco que `ProfilingEnvironment.tool_to_world()` expone. La caja permanece
# rigida respecto a el durante las tres capturas, asi que sus coordenadas deben
# coincidir aunque el brazo haya rotado.
TOOL_FRAME_ID = "ur10e_attachment_site"


@dataclass(frozen=True)
class FusedCloud:
    frame_id: str
    points_m: np.ndarray
    view_index: np.ndarray
    views: tuple[ViewDescriptor, ...]

    @property
    def view_count(self) -> int:
        return len(self.views)

    def points_for_view(self, index: int) -> np.ndarray:
        return self.points_m[self.view_index == index]


def fuse_scan_views(views: list[ScanView]) -> FusedCloud:
    """Concatena en el marco del terminal las nubes de las vistas validas."""

    if not views:
        raise ValueError("no hay vistas que fusionar")
    clouds = [view.points_tool_m for view in views]
    view_index = np.concatenate(
        [np.full(cloud.shape[0], index, dtype=np.int32) for index, cloud in enumerate(clouds)]
    )
    return FusedCloud(
        frame_id=TOOL_FRAME_ID,
        points_m=np.concatenate(clouds, axis=0),
        view_index=view_index,
        views=tuple(
            ViewDescriptor(view.pose_name, view.target_yaw_deg, view.target_tilt_deg) for view in views
        ),
    )


def axis_aligned_extremes_m(
    points_m: np.ndarray,
    percentile_low: float,
    percentile_high: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Extremos robustos por eje en el marco de la nube."""

    lower = np.percentile(points_m, percentile_low, axis=0)
    upper = np.percentile(points_m, percentile_high, axis=0)
    return lower, upper


def signed_distance_to_box_m(
    points_m: np.ndarray,
    lower_m: np.ndarray,
    upper_m: np.ndarray,
) -> np.ndarray:
    """Distancia firmada a la superficie: negativa dentro, positiva fuera."""

    center = (lower_m + upper_m) / 2.0
    half_extent = (upper_m - lower_m) / 2.0
    offset = np.abs(points_m - center) - half_extent
    outside = np.linalg.norm(np.maximum(offset, 0.0), axis=1)
    inside = np.minimum(np.max(offset, axis=1), 0.0)
    return outside + inside


def distance_to_box_surface_m(
    points_m: np.ndarray,
    lower_m: np.ndarray,
    upper_m: np.ndarray,
) -> np.ndarray:
    """Distancia de cada punto a la superficie del cuboide dado.

    Sirve como residuo: si una vista quedase mal registrada respecto a las
    demas, sus puntos dejarian de apoyarse en las caras comunes.
    """

    return np.abs(signed_distance_to_box_m(points_m, lower_m, upper_m))


def view_plane_residuals_m(
    cloud: FusedCloud,
    percentile_low: float,
    percentile_high: float,
) -> np.ndarray:
    """Residuo de cada vista frente al cuboide comun de la fusion.

    No usa ground truth: el cuboide de referencia sale de la propia nube
    fusionada. Se toma el percentil 95 y no la mediana, porque una vista
    desplazada conserva la mayoria de sus puntos sobre las caras laterales
    comunes y la mediana no acusa el desplazamiento.
    """

    lower, upper = axis_aligned_extremes_m(cloud.points_m, percentile_low, percentile_high)
    residuals = np.empty(cloud.view_count)
    for index in range(cloud.view_count):
        points = cloud.points_for_view(index)
        if points.shape[0] == 0:
            residuals[index] = np.inf
            continue
        residuals[index] = float(np.percentile(distance_to_box_surface_m(points, lower, upper), 95))
    return residuals


def view_extent_disagreement_m(cloud: FusedCloud) -> np.ndarray:
    """Discrepancia por eje entre las extensiones que ve cada vista.

    Solo compara el eje vertical del terminal y los dos horizontales tal como
    los ve cada vista; una vista que no observe un extremo dara una extension
    menor, asi que la discrepancia mide cobertura tanto como registro.
    """

    if cloud.view_count < 2:
        return np.zeros(3)
    extents = np.asarray(
        [
            np.ptp(cloud.points_for_view(index), axis=0)
            for index in range(cloud.view_count)
            if cloud.points_for_view(index).shape[0] > 0
        ]
    )
    return extents.max(axis=0) - extents.min(axis=0)
