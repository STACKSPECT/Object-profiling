from __future__ import annotations

import ast
from pathlib import Path

import pytest

PACKAGE = Path(__file__).resolve().parents[1] / "src" / "object_profiling"

# Modulos que consumen observaciones y producen la medida. No pueden mirar la
# escena por dentro.
SOLUTION_MODULES = (
    "background.py",
    "config.py",
    "contracts.py",
    "controller.py",
    "geometry.py",
    "perception.py",
    "poses.py",
    "profiling_pipeline.py",
    "registration.py",
    "scanning.py",
    "sensors.py",
)

# `environment.py` construye la escena y por tanto conoce la caja: es quien la
# crea. Los modulos de auditoria miden contra la verdad por definicion.
EVALUATION_MODULES = (
    "benchmark.py",
    "camera_audit.py",
    "checkpoint.py",
    "demo.py",
    "depth_audit.py",
    "evaluation.py",
    "geometry_audit.py",
    "registration_audit.py",
    "segmentation_audit.py",
)

PRIVILEGED_TOKENS = (
    "box_geom",
    "profiling_box",
    "box_spec",
    "body_to_world",
    "gripper_to_box",
    "mjOBJ_GEOM",
    "enable_segmentation_rendering",
    "mj_ray",
)

FORBIDDEN_IMPORTS = {
    "camera_audit",
    "checkpoint",
    "depth_audit",
    "evaluation",
    "registration_audit",
    "segmentation_audit",
}


def _existing(modules: tuple[str, ...]) -> list[Path]:
    return [PACKAGE / name for name in modules if (PACKAGE / name).exists()]


def _local_imports(source: str) -> set[str]:
    imported: set[str] = set()
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, ast.ImportFrom) and node.level:
            if node.module:
                imported.add(node.module.split(".")[0])
            imported.update(alias.name for alias in node.names)
        elif isinstance(node, ast.Import):
            for alias in node.names:
                if alias.name.startswith("object_profiling."):
                    imported.add(alias.name.split(".")[1])
    return imported


@pytest.mark.parametrize("path", _existing(SOLUTION_MODULES), ids=lambda path: path.name)
def test_solution_modules_do_not_read_privileged_scene_state(path: Path) -> None:
    source = path.read_text(encoding="utf-8")
    offenders = [token for token in PRIVILEGED_TOKENS if token in source]

    assert not offenders, f"{path.name} usa informacion privilegiada: {offenders}"


@pytest.mark.parametrize("path", _existing(SOLUTION_MODULES), ids=lambda path: path.name)
def test_solution_modules_do_not_import_evaluation(path: Path) -> None:
    imported = _local_imports(path.read_text(encoding="utf-8"))
    offenders = sorted(imported & FORBIDDEN_IMPORTS)

    assert not offenders, f"{path.name} importa modulos de evaluacion: {offenders}"


def test_the_boundary_check_would_catch_a_violation() -> None:
    """Una prueba que nunca puede fallar no demuestra nada.

    Los modulos de evaluacion si usan la verdad de la escena, asi que deben
    disparar el mismo criterio que protege a la solucion.
    """

    evaluation_sources = [path.read_text(encoding="utf-8") for path in _existing(EVALUATION_MODULES)]

    assert evaluation_sources
    for source in evaluation_sources:
        assert any(token in source for token in PRIVILEGED_TOKENS)


def test_every_solution_module_is_listed() -> None:
    """Un modulo nuevo no puede quedar fuera de la comprobacion por olvido."""

    known = set(SOLUTION_MODULES) | set(EVALUATION_MODULES) | {"__init__.py", "environment.py"}
    present = {path.name for path in PACKAGE.glob("*.py")}

    assert present <= known, f"modulos sin clasificar: {sorted(present - known)}"
