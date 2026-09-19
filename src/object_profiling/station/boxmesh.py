"""Malla de cuboide subdividida, con winding hacia fuera.

La topologia es identica para todas las cajas: seis caras, cada una una
rejilla. Eso permite deformar vertices (hundimientos, chaflanes) sin cambiar
el numero de elementos, y reutilizar el mismo activo compilado de MuJoCo.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# 16 divisiones por arista: el defecto mas pequeno que queremos ver (2 % de
# 80 mm = 1,6 mm) cubre varios triangulos en una cara de 120 mm.
MESH_SUBDIVISIONS = 16

# Semiextensiones con las que se compila el activo. Distintas entre si para
# que los ejes principales de inercia no sean degenerados y MuJoCo no reoriente
# de forma arbitraria. Coincide con la caja nominal del XML.
REFERENCE_HALF_EXTENTS_M = np.asarray([0.15, 0.10, 0.075], dtype=np.float64)


@dataclass(frozen=True)
class CuboidMesh:
    vertices: np.ndarray
    faces: np.ndarray
    subdivisions: int

    @property
    def vertex_count(self) -> int:
        return int(self.vertices.shape[0])

    @property
    def face_count(self) -> int:
        return int(self.faces.shape[0])


def expected_counts(subdivisions: int = MESH_SUBDIVISIONS) -> tuple[int, int]:
    vertices = 6 * (subdivisions + 1) ** 2
    faces = 6 * 2 * subdivisions**2
    return vertices, faces


def _add_face(
    verts: list[np.ndarray],
    faces: list[list[int]],
    origin: np.ndarray,
    u: np.ndarray,
    v: np.ndarray,
    subdivisions: int,
) -> None:
    base = len(verts)
    for i in range(subdivisions + 1):
        for j in range(subdivisions + 1):
            verts.append(origin + (i / subdivisions) * u + (j / subdivisions) * v)
    for i in range(subdivisions):
        for j in range(subdivisions):
            v00 = base + i * (subdivisions + 1) + j
            v10 = base + (i + 1) * (subdivisions + 1) + j
            v01 = v00 + 1
            v11 = v10 + 1
            faces.append([v00, v10, v11])
            faces.append([v00, v11, v01])


def orient_outward(vertices: np.ndarray, faces: np.ndarray) -> np.ndarray:
    """Invierte cualquier triangulo cuya normal apunte hacia el interior."""

    a, b, c = vertices[faces[:, 0]], vertices[faces[:, 1]], vertices[faces[:, 2]]
    normals = np.cross(b - a, c - a)
    centroids = (a + b + c) / 3.0
    inward = np.einsum("ij,ij->i", normals, centroids) < 0.0
    oriented = faces.copy()
    oriented[inward] = oriented[inward][:, [0, 2, 1]]
    return oriented


def cuboid_mesh(
    half_extents_m: np.ndarray,
    subdivisions: int = MESH_SUBDIVISIONS,
) -> CuboidMesh:
    """Cuboide centrado en el origen, aristas alineadas con los ejes.

    `half_extents_m` es (x, y, z) en metros, el mismo convenio que `geom_size`
    de una primitiva caja: longitud/2, anchura/2, altura/2.
    """

    hx, hy, hz = (float(value) for value in np.asarray(half_extents_m, dtype=np.float64).reshape(3))
    origin = lambda x, y, z: np.asarray([x, y, z], dtype=np.float64)
    verts: list[np.ndarray] = []
    faces: list[list[int]] = []
    # Cada cara se construye para que u × v apunte hacia fuera; orient_outward
    # lo comprueba de todos modos, porque un winding invertido hace que la
    # profundidad de MuJoCo muestre la cara interior y el error es silencioso.
    _add_face(verts, faces, origin(-hx, -hy, hz), origin(0.0, 2 * hy, 0.0), origin(2 * hx, 0.0, 0.0), subdivisions)
    _add_face(verts, faces, origin(-hx, -hy, -hz), origin(2 * hx, 0.0, 0.0), origin(0.0, 2 * hy, 0.0), subdivisions)
    _add_face(verts, faces, origin(hx, -hy, -hz), origin(0.0, 0.0, 2 * hz), origin(0.0, 2 * hy, 0.0), subdivisions)
    _add_face(verts, faces, origin(-hx, -hy, -hz), origin(0.0, 2 * hy, 0.0), origin(0.0, 0.0, 2 * hz), subdivisions)
    _add_face(verts, faces, origin(-hx, hy, -hz), origin(2 * hx, 0.0, 0.0), origin(0.0, 0.0, 2 * hz), subdivisions)
    _add_face(verts, faces, origin(-hx, -hy, -hz), origin(0.0, 0.0, 2 * hz), origin(2 * hx, 0.0, 0.0), subdivisions)
    vertices = np.asarray(verts, dtype=np.float64)
    oriented = orient_outward(vertices, np.asarray(faces, dtype=np.int32))
    return CuboidMesh(vertices, oriented, subdivisions)


def triangle_normals(vertices: np.ndarray, faces: np.ndarray) -> np.ndarray:
    a, b, c = vertices[faces[:, 0]], vertices[faces[:, 1]], vertices[faces[:, 2]]
    normals = np.cross(b - a, c - a)
    length = np.linalg.norm(normals, axis=1, keepdims=True)
    length = np.maximum(length, 1e-12)
    return normals / length


def all_faces_outward(vertices: np.ndarray, faces: np.ndarray, *, atol: float = 0.0) -> bool:
    a, b, c = vertices[faces[:, 0]], vertices[faces[:, 1]], vertices[faces[:, 2]]
    normals = np.cross(b - a, c - a)
    centroids = (a + b + c) / 3.0
    return bool(np.all(np.einsum("ij,ij->i", normals, centroids) > atol))


def axis_aligned_bounds(vertices: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return vertices.min(axis=0), vertices.max(axis=0)


def rigid_align(source: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Kabsch: `target ≈ source @ R.T + t`, con R ∈ SO(3).

    El compilador de MuJoCo recentra y reorienta la malla hacia sus ejes
    principales. Esta transformacion permite escribir vertices de autoria en el
    marco ya compilado, sin recompilar.
    """

    source_mean = source.mean(axis=0)
    target_mean = target.mean(axis=0)
    centered_source = source - source_mean
    centered_target = target - target_mean
    covariance = centered_source.T @ centered_target
    u_matrix, _singular, vt_matrix = np.linalg.svd(covariance)
    rotation = vt_matrix.T @ u_matrix.T
    if np.linalg.det(rotation) < 0.0:
        vt_matrix[-1] *= -1.0
        rotation = vt_matrix.T @ u_matrix.T
    translation = target_mean - rotation @ source_mean
    return rotation, translation


def apply_rigid(vertices: np.ndarray, rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    return vertices @ rotation.T + translation
