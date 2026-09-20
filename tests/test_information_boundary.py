from __future__ import annotations

import ast
from pathlib import Path

import pytest

PACKAGE = Path(__file__).resolve().parents[1] / "src" / "object_profiling"
MEASURE_DIR = PACKAGE / "measure"

PUBLIC = (
    PACKAGE / "config.py",
    PACKAGE / "contracts.py",
)

SOLUTION_PATHS = tuple(MEASURE_DIR.glob("*.py")) + PUBLIC

PRIVILEGED_TOKENS = (
    "box_geom",
    "box_collision",
    "box_mesh",
    "profiling_box",
    "box_spec",
    "DamageSpec",
    "damage_spec",
    "body_to_world",
    "gripper_to_box",
    "mjOBJ_GEOM",
    "enable_segmentation_rendering",
    "mj_ray",
)

FORBIDDEN_IMPORTS = {
    "audits",
    "benchmark",
    "checkpoint",
    "evaluation",
    "metrics",
    "station",
    "presentation",
}


def _existing(paths: tuple[Path, ...]) -> list[Path]:
    return [path for path in paths if path.exists() and path.name != "__init__.py"]


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


@pytest.mark.parametrize("path", _existing(SOLUTION_PATHS), ids=lambda path: str(path.relative_to(PACKAGE)))
def test_solution_modules_do_not_read_privileged_scene_state(path: Path) -> None:
    source = path.read_text(encoding="utf-8")
    offenders = [token for token in PRIVILEGED_TOKENS if token in source]

    assert not offenders, f"{path.relative_to(PACKAGE)} usa informacion privilegiada: {offenders}"


@pytest.mark.parametrize("path", _existing(SOLUTION_PATHS), ids=lambda path: str(path.relative_to(PACKAGE)))
def test_solution_modules_do_not_import_simulation_adapters(path: Path) -> None:
    imported = _local_imports(path.read_text(encoding="utf-8"))
    offenders = sorted(imported & FORBIDDEN_IMPORTS)

    assert not offenders, f"{path.relative_to(PACKAGE)} importa adaptadores de escena: {offenders}"


def test_every_python_module_is_classified() -> None:
    known = {path.resolve() for path in _existing(SOLUTION_PATHS)}
    known.add((PACKAGE / "__init__.py").resolve())
    known.add((PACKAGE / "measure" / "__init__.py").resolve())

    present = {path.resolve() for path in PACKAGE.rglob("*.py") if "__pycache__" not in path.parts}
    assert present <= known, f"modulos sin clasificar: {sorted(p.relative_to(PACKAGE) for p in present - known)}"


def test_package_does_not_import_mujoco() -> None:
    for path in PACKAGE.rglob("*.py"):
        if "__pycache__" in path.parts:
            continue
        source = path.read_text(encoding="utf-8")
        assert "mujoco" not in source, f"{path.relative_to(PACKAGE)} no puede depender de MuJoCo"
