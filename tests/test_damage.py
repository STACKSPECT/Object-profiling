from __future__ import annotations

import numpy as np
import pytest

from object_profiling.config import AppConfig
from object_profiling.contracts import Dimensions3D
from object_profiling.evaluation.checkpoint import NOMINAL_BOX
from object_profiling.station.boxmesh import axis_aligned_bounds, cuboid_mesh
from object_profiling.station.damage import (
    INTACT_DAMAGE,
    DamageKind,
    DamageSpec,
    apply_damage,
    damageable_corners,
    damageable_faces,
    face_is_grasp,
    generate_damage_spec,
)
from object_profiling.station.environment import ProfilingEnvironment, generate_box_spec
from tests.conftest import damaged_config, intact_config


HALF = np.asarray([0.15, 0.10, 0.075])
DIMENSIONS = Dimensions3D(0.30, 0.20, 0.15)


def test_grasp_face_is_excluded_from_generators() -> None:
    assert face_is_grasp("-z")
    assert not face_is_grasp("+z")
    assert "-z" not in damageable_faces()
    assert all(not face_is_grasp(location[-2:]) for location in damageable_corners())
    assert len(damageable_corners()) == 4


def test_damage_does_not_grow_the_envelope() -> None:
    intact = cuboid_mesh(HALF).vertices
    lower0, upper0 = axis_aligned_bounds(intact)
    cases = (
        DamageSpec(DamageKind.CRUSHED_CORNER, 0.03, "corner:+x+y+z"),
        DamageSpec(DamageKind.DENTED_FACE, 0.02, "face:+x", radius_m=0.05),
        DamageSpec(DamageKind.BUCKLED_PANEL, 0.02, "face:+y"),
    )
    for spec in cases:
        damaged = apply_damage(intact, HALF, spec)
        lower, upper = axis_aligned_bounds(damaged)
        assert np.all(lower >= lower0 - 1e-9), spec.kind
        assert np.all(upper <= upper0 + 1e-9), spec.kind


def test_grasp_face_vertices_do_not_move() -> None:
    intact = cuboid_mesh(HALF).vertices
    grasp_z = -HALF[2]
    on_grasp = np.abs(intact[:, 2] - grasp_z) <= 1e-9
    for spec in (
        DamageSpec(DamageKind.CRUSHED_CORNER, 0.04, "corner:+x+y+z"),
        DamageSpec(DamageKind.DENTED_FACE, 0.025, "face:+x", radius_m=0.05),
        DamageSpec(DamageKind.BUCKLED_PANEL, 0.02, "face:+z"),
    ):
        damaged = apply_damage(intact, HALF, spec)
        delta = np.linalg.norm(damaged[on_grasp] - intact[on_grasp], axis=1).max()
        assert delta == pytest.approx(0.0, abs=1e-12), spec.kind


def test_declared_severity_moves_vertices_monotonically() -> None:
    intact = cuboid_mesh(HALF).vertices
    previous = 0.0
    for severity in (0.010, 0.020, 0.040):
        damaged = apply_damage(
            intact,
            HALF,
            DamageSpec(DamageKind.CRUSHED_CORNER, severity, "corner:+x+y+z"),
        )
        delta = float(np.linalg.norm(damaged - intact, axis=1).max())
        assert delta > previous
        previous = delta


def test_default_damage_rate_is_the_operational_ten_percent() -> None:
    assert AppConfig().damage.rate == 0.1


def test_one_defect_per_box_and_intact_at_zero_rate() -> None:
    intact = generate_box_spec(42, intact_config())
    assert intact.damage == INTACT_DAMAGE

    rng_kinds: set[DamageKind] = set()
    damaged_count = 0
    for seed in range(200, 400):
        spec = generate_box_spec(seed, damaged_config(0.5))
        rng_kinds.add(spec.damage.kind)
        damaged_count += int(spec.damage.damaged)
        assert spec.damage.kind is DamageKind.INTACT or spec.damage.location
    assert DamageKind.INTACT in rng_kinds
    assert damaged_count > 40
    assert generate_box_spec(42, intact_config()) == generate_box_spec(42, intact_config())


def test_same_seed_reproduces_damage() -> None:
    config = damaged_config(1.0)
    first = generate_box_spec(7, config)
    second = generate_box_spec(7, config)
    assert first.damage == second.damage
    assert first.damage.damaged


def test_load_box_keeps_the_same_compiled_model() -> None:
    environment = ProfilingEnvironment.create(NOMINAL_BOX, AppConfig(), attach_box=False)
    model = environment.model
    crushed = DamageSpec(DamageKind.CRUSHED_CORNER, 0.03, "corner:+x+y+z")
    environment.load_box(
        dataclasses_replace_damage(NOMINAL_BOX, crushed),
        attach_box=False,
    )
    assert environment.model is model
    assert environment.box_spec.damage.kind is DamageKind.CRUSHED_CORNER


def dataclasses_replace_damage(box, damage: DamageSpec):
    from dataclasses import replace

    return replace(box, damage=damage)
