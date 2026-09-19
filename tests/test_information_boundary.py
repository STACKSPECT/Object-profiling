from __future__ import annotations

import ast
from pathlib import Path

import pytest

PACKAGE = Path(__file__).resolve().parents[1] / "src" / "object_profiling"

# El medidor no puede mirar la escena por dentro ni importar evaluacion.
MEASURE_DIR = PACKAGE / "measure"

# Estacion sin environment.py: el entorno es quien crea la caja.
STATION_WITHOUT_SCENE = (
    PACKAGE / "station" / "backgrounds.py",
    PACKAGE / "station" / "camera.py",
    PACKAGE / "station" / "controller.py",
    PACKAGE / "station" / "pipeline.py",
    PACKAGE / "station" / "poses.py",
    PACKAGE / "station" / "scanning.py",
)

PUBLIC = (
    PACKAGE / "config.py",
    PACKAGE / "contracts.py",
)

PRESENTATION_SAFE = (PACKAGE / "presentation" / "panels.py",)

SOLUTION_PATHS = tuple(MEASURE_DIR.glob("*.py")) + STATION_WITHOUT_SCENE + PUBLIC + PRESENTATION_SAFE

EVALUATION_GLOBS = (
    PACKAGE / "evaluation",
    PACKAGE / "presentation",
    PACKAGE / "station" / "environment.py",
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
    "audits",
    "benchmark",
    "checkpoint",
    "evaluation",
    "metrics",
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
def test_solution_modules_do_not_import_evaluation(path: Path) -> None:
    imported = _local_imports(path.read_text(encoding="utf-8"))
    offenders = sorted(imported & FORBIDDEN_IMPORTS)

    assert not offenders, f"{path.relative_to(PACKAGE)} importa evaluacion: {offenders}"


def test_the_boundary_check_would_catch_a_violation() -> None:
    evaluation_paths = [PACKAGE / "station" / "environment.py"]
    evaluation_paths.extend((PACKAGE / "evaluation").rglob("*.py"))
    evaluation_paths.extend((PACKAGE / "presentation").glob("*.py"))
    sources = [
        path.read_text(encoding="utf-8")
        for path in evaluation_paths
        if path.exists() and path.name not in {"__init__.py", "panels.py"}
    ]

    assert sources
    for source in sources:
        assert any(token in source for token in PRIVILEGED_TOKENS)


def test_every_python_module_is_classified() -> None:
    known = {path.resolve() for path in _existing(SOLUTION_PATHS)}
    known.add((PACKAGE / "__init__.py").resolve())
    known.add((PACKAGE / "station" / "environment.py").resolve())
    known.add((PACKAGE / "station" / "__init__.py").resolve())
    known.add((PACKAGE / "measure" / "__init__.py").resolve())
    known.add((PACKAGE / "presentation" / "__init__.py").resolve())
    known.add((PACKAGE / "presentation" / "demo.py").resolve())
    known.add((PACKAGE / "presentation" / "showcase.py").resolve())
    known.add((PACKAGE / "evaluation" / "__init__.py").resolve())
    known.add((PACKAGE / "evaluation" / "audits" / "__init__.py").resolve())
    known.update(path.resolve() for path in (PACKAGE / "evaluation").rglob("*.py"))

    present = {path.resolve() for path in PACKAGE.rglob("*.py") if "__pycache__" not in path.parts}
    assert present <= known, f"modulos sin clasificar: {sorted(p.relative_to(PACKAGE) for p in present - known)}"


def test_measure_package_does_not_import_mujoco() -> None:
    for path in MEASURE_DIR.glob("*.py"):
        source = path.read_text(encoding="utf-8")
        assert "mujoco" not in source, f"{path.name} no puede depender de MuJoCo"
        assert "from ..station" not in source
        assert "from ...station" not in source
