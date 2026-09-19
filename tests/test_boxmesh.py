from __future__ import annotations

import numpy as np
import pytest
import mujoco
import cv2

from object_profiling.config import AppConfig
from object_profiling.evaluation.checkpoint import MAXIMUM_BOX, MINIMUM_BOX, NOMINAL_BOX
from object_profiling.station.boxmesh import (
    MESH_SUBDIVISIONS,
    REFERENCE_HALF_EXTENTS_M,
    all_faces_outward,
    apply_rigid,
    axis_aligned_bounds,
    cuboid_mesh,
    expected_counts,
    rigid_align,
)
from object_profiling.station.environment import ProfilingEnvironment
from object_profiling.station.scene import compile_scene_spec, load_scene_spec

SCAN = np.asarray([-0.174, 0.735, 0.650])


@pytest.mark.parametrize(
    "half",
    [
        np.asarray([0.075, 0.06, 0.04]),
        np.asarray([0.15, 0.10, 0.075]),
        np.asarray([0.20, 0.15, 0.125]),
        REFERENCE_HALF_EXTENTS_M,
    ],
)
def test_cuboid_mesh_is_outward_and_matches_the_envelope(half: np.ndarray) -> None:
    mesh = cuboid_mesh(half)
    vertices, faces = expected_counts(MESH_SUBDIVISIONS)

    assert mesh.vertex_count == vertices
    assert mesh.face_count == faces
    assert all_faces_outward(mesh.vertices, mesh.faces)
    lower, upper = axis_aligned_bounds(mesh.vertices)
    assert lower == pytest.approx(-half, abs=1e-9)
    assert upper == pytest.approx(half, abs=1e-9)


def test_topology_is_identical_across_sizes() -> None:
    small = cuboid_mesh(np.asarray([0.075, 0.06, 0.04]))
    large = cuboid_mesh(np.asarray([0.20, 0.15, 0.125]))

    assert np.array_equal(small.faces, large.faces)
    assert small.vertex_count == large.vertex_count


def test_rigid_align_recovers_a_known_transform() -> None:
    source = cuboid_mesh(REFERENCE_HALF_EXTENTS_M).vertices
    rotation = np.asarray([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]])
    translation = np.asarray([0.01, -0.02, 0.03])
    target = apply_rigid(source, rotation, translation)
    recovered_r, recovered_t = rigid_align(source, target)

    assert recovered_r == pytest.approx(rotation, abs=1e-9)
    assert recovered_t == pytest.approx(translation, abs=1e-9)
    assert apply_rigid(source, recovered_r, recovered_t) == pytest.approx(target, abs=1e-9)


def _place(model, data) -> None:
    data.qpos[:6] = np.asarray(AppConfig().motion.home_qpos, dtype=np.float64)
    data.ctrl[:6] = data.qpos[:6]
    joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "box_free")
    address = model.jnt_qposadr[joint]
    data.qpos[address : address + 3] = SCAN
    data.qpos[address + 3 : address + 7] = [1.0, 0.0, 0.0, 0.0]
    mujoco.mj_forward(model, data)


def _depth(model, data) -> np.ndarray:
    renderer = mujoco.Renderer(model, height=480, width=640)
    renderer.enable_depth_rendering()
    renderer.update_scene(data, camera="scan_rgbd_cam")
    depth = renderer.render().copy()
    renderer.close()
    return depth


def _box_masked_depth(model, data) -> tuple[np.ndarray, np.ndarray]:
    geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "box_geom")
    with_box = _depth(model, data)
    alpha = float(model.geom_rgba[geom, 3])
    model.geom_rgba[geom, 3] = 0.0
    without = _depth(model, data)
    model.geom_rgba[geom, 3] = alpha
    mask = np.abs(with_box - without) > 1e-4
    return with_box, mask


@pytest.mark.parametrize("box_spec", [MINIMUM_BOX, NOMINAL_BOX, MAXIMUM_BOX])
def test_intact_mesh_depth_matches_primitive_box(box_spec) -> None:
    spec = load_scene_spec()
    primitive = compile_scene_spec(spec)
    pdata = mujoco.MjData(primitive)
    geom = mujoco.mj_name2id(primitive, mujoco.mjtObj.mjOBJ_GEOM, "box_geom")
    collision = mujoco.mj_name2id(primitive, mujoco.mjtObj.mjOBJ_GEOM, "box_collision")
    half = box_spec.dimensions_m.as_array() / 2.0
    primitive.geom_size[geom] = half
    primitive.geom_size[collision] = half
    _place(primitive, pdata)
    primitive_depth, primitive_mask = _box_masked_depth(primitive, pdata)

    environment = ProfilingEnvironment.create(box_spec, attach_box=False)
    gid = mujoco.mj_name2id(environment.model, mujoco.mjtObj.mjOBJ_GEOM, "box_geom")
    assert environment.model.geom_type[gid] == mujoco.mjtGeom.mjGEOM_MESH
    _place(environment.model, environment.data)
    mesh_depth, mesh_mask = _box_masked_depth(environment.model, environment.data)

    # Interseccion, no union: con la camara baja de EXP-009 la caja minima
    # deja un borde de silueta donde una representacion ve caja y la otra
    # fondo. El acuerdo que importa es la profundidad sobre superficie
    # compartida.
    shared = primitive_mask & mesh_mask
    interior = cv2.erode(shared.astype(np.uint8), np.ones((5, 5), np.uint8)).astype(bool)
    if not interior.any():
        raise AssertionError("la caja no ocupa el interior de la mascara")
    diff = np.abs(mesh_depth - primitive_depth)[interior]
    assert float(np.percentile(diff, 95)) < 0.001
