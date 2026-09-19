from __future__ import annotations

from object_profiling.checkpoint import NOMINAL_BOX
from object_profiling.depth_audit import audit_depth_accuracy
from object_profiling.poses import SCAN_POSES


def test_rendered_depth_matches_analytic_rays_on_the_box_interior() -> None:
    """El aviso de ARB_clip_control no impide medir en milimetros.

    Sobre el interior de la caja la profundidad renderizada coincide con el
    trazado de rayos analitico muy por debajo del milimetro, asi que el error
    dimensional no puede atribuirse al buffer de profundidad.
    """

    records = audit_depth_accuracy(NOMINAL_BOX)

    assert [record.pose_name for record in records] == [pose.name for pose in SCAN_POSES]
    for record in records:
        assert record.interior_pixels > 1_000
        assert record.interior_p95_absolute_error_m < 1e-4
        assert abs(record.interior_signed_bias_m) < 1e-4
        assert record.quantization_step_m < 1e-3


def test_silhouette_pixels_are_the_real_depth_hazard() -> None:
    """Un solo pixel de silueta puede errar centimetros.

    Justifica erosionar la mascara antes de estimar dimensiones, en lugar de
    confiar en percentiles sobre todos los puntos visibles.
    """

    records = audit_depth_accuracy(NOMINAL_BOX)
    worst_interior = max(record.interior_maximum_absolute_error_m for record in records)
    worst_silhouette = max(record.silhouette_maximum_absolute_error_m for record in records)

    assert worst_interior < 1e-3
    assert worst_silhouette > worst_interior
