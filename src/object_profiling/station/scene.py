"""Carga y compilacion de la escena de perfilado.

`MjSpec.from_file` no abre la ruta Unicode de este repositorio ni la ruta
corta de Windows (la extension queda en mayusculas y el despachador de
decodificadores distingue). El directorio de trabajo tiene que ser el de la
escena, tanto al parsear como al compilar, porque `meshdir` es relativo.
"""

from __future__ import annotations

import os
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from ..config import PROJECT_ROOT
from .boxmesh import (
    REFERENCE_HALF_EXTENTS_M,
    apply_rigid,
    cuboid_mesh,
    rigid_align,
    triangle_normals,
)

SCENE_DIRECTORY = PROJECT_ROOT / "assets" / "universal_robots_ur10e"
SCENE_FILE_NAME = "profiling_scene.xml"
BOX_MESH_NAME = "box_mesh"


@contextmanager
def scene_directory() -> Path:
    previous = os.getcwd()
    os.chdir(SCENE_DIRECTORY)
    try:
        yield SCENE_DIRECTORY
    finally:
        os.chdir(previous)


def load_scene_spec() -> mujoco.MjSpec:
    with scene_directory():
        return mujoco.MjSpec.from_file(SCENE_FILE_NAME)


def compile_scene_spec(spec: mujoco.MjSpec) -> mujoco.MjModel:
    with scene_directory():
        return spec.compile()


@dataclass
class BoxMeshBinding:
    """Como escribir vertices de autoria sobre el activo ya compilado."""

    mesh_id: int
    vertex_address: int
    vertex_count: int
    normal_address: int
    normal_count: int
    face_address: int
    rotation: np.ndarray
    translation: np.ndarray
    authored_faces: np.ndarray
    reference_half_extents_m: np.ndarray
    compiled_geom_size: np.ndarray


def inject_box_mesh(spec: mujoco.MjSpec) -> tuple[np.ndarray, np.ndarray]:
    """Sustituye `box_geom` por una malla de cuboide de topologia fija."""

    mesh = cuboid_mesh(REFERENCE_HALF_EXTENTS_M)
    asset = spec.add_mesh()
    asset.name = BOX_MESH_NAME
    asset.uservert = mesh.vertices.flatten().tolist()
    asset.userface = mesh.faces.flatten().tolist()
    geom = spec.geom("box_geom")
    geom.type = mujoco.mjtGeom.mjGEOM_MESH
    geom.meshname = BOX_MESH_NAME
    geom.contype = 0
    geom.conaffinity = 0
    geom.group = 2
    return mesh.vertices, mesh.faces


def bind_box_mesh(model: mujoco.MjModel, authored_vertices: np.ndarray, authored_faces: np.ndarray) -> BoxMeshBinding:
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "box_geom")
    mesh_id = int(model.geom_dataid[geom_id])
    vertex_address = int(model.mesh_vertadr[mesh_id])
    vertex_count = int(model.mesh_vertnum[mesh_id])
    if vertex_count != authored_vertices.shape[0]:
        raise RuntimeError(
            f"la malla compilada tiene {vertex_count} vertices, la de autoria {authored_vertices.shape[0]}"
        )
    stored = np.asarray(model.mesh_vert[vertex_address : vertex_address + vertex_count], dtype=np.float64)
    rotation, translation = rigid_align(authored_vertices, stored)
    return BoxMeshBinding(
        mesh_id=mesh_id,
        vertex_address=vertex_address,
        vertex_count=vertex_count,
        normal_address=int(model.mesh_normaladr[mesh_id]),
        normal_count=int(model.mesh_normalnum[mesh_id]),
        face_address=int(model.mesh_faceadr[mesh_id]),
        rotation=rotation,
        translation=translation,
        authored_faces=np.asarray(authored_faces, dtype=np.int32),
        reference_half_extents_m=REFERENCE_HALF_EXTENTS_M.copy(),
        compiled_geom_size=np.asarray(model.geom_size[geom_id], dtype=np.float64).copy(),
    )


def write_box_vertices(
    model: mujoco.MjModel,
    binding: BoxMeshBinding,
    authored_vertices: np.ndarray,
    half_extents_m: np.ndarray,
) -> None:
    """Escribe vertices de autoria en el marco compilado y actualiza normales.

    No recompila: el visor y el renderer conservan el mismo `mjModel`.
    `geom_aabb` y `geom_rbound` viven en el marco del geom, no en el de la
    malla reorientada: se actualizan con el envolvente de autoria.
    """

    compiled = apply_rigid(authored_vertices, binding.rotation, binding.translation).astype(np.float32)
    start = binding.vertex_address
    model.mesh_vert[start : start + binding.vertex_count] = compiled
    compiled_normals = triangle_normals(compiled.astype(np.float64), binding.authored_faces)
    vertex_normals = np.zeros((binding.vertex_count, 3), dtype=np.float64)
    np.add.at(vertex_normals, binding.authored_faces[:, 0], compiled_normals)
    np.add.at(vertex_normals, binding.authored_faces[:, 1], compiled_normals)
    np.add.at(vertex_normals, binding.authored_faces[:, 2], compiled_normals)
    lengths = np.linalg.norm(vertex_normals, axis=1, keepdims=True)
    lengths = np.maximum(lengths, 1e-12)
    vertex_normals = (vertex_normals / lengths).astype(np.float32)
    if binding.normal_count == binding.vertex_count:
        model.mesh_normal[binding.normal_address : binding.normal_address + binding.normal_count] = vertex_normals
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "box_geom")
    compiled64 = compiled.astype(np.float64)
    center = 0.5 * (compiled64.max(axis=0) + compiled64.min(axis=0))
    half = 0.5 * (compiled64.max(axis=0) - compiled64.min(axis=0))
    # geom_size en una malla escala vertices que ya estan en metros. Se deja el
    # valor de compilacion y solo se actualizan AABB y radio de culling.
    _ = half_extents_m
    model.geom_size[geom_id] = binding.compiled_geom_size
    model.geom_rbound[geom_id] = float(np.linalg.norm(compiled64, axis=1).max()) + 0.02
    model.geom_aabb[geom_id, 0:3] = center
    model.geom_aabb[geom_id, 3:6] = half
