"""Umbrales de sensor, estimador y dano. Sin escena ni qpos de robot."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class BoxRange:
    """Rango declarado de cajas. `estimate_cuboid` rechaza fuera de el."""

    length_m: tuple[float, float] = (0.15, 0.40)
    width_m: tuple[float, float] = (0.12, 0.30)
    height_m: tuple[float, float] = (0.08, 0.25)


@dataclass(frozen=True)
class SensorConfig:
    width: int = 640
    height: int = 480
    # Absorbe el asentamiento del terminal entre la estacion vacia del fondo y
    # la caja suspendida, que EXP-003 midio por debajo de 0,1 mm.
    foreground_margin_m: float = 0.004
    # EXP-002 midio 5.010 px visibles en el peor caso, la caja minima. El umbral
    # de colocacion de camara de la auditoria son 4.000 px; aqui solo hace falta
    # descartar restos que no puedan ser la caja.
    min_component_pixels: int = 2_000
    border_margin_px: int = 3
    # EXP-006: la silueta es la unica fuente de los extremos que ninguna cara
    # observada define. Con 0, `interior_mask` coincide con la mascara completa
    # y es la que se retroproyecta. No reactivar: 2 px costaban ~5 mm de anchura.
    mask_erosion_px: int = 0
    # Holgura del volumen de recorte sobre el tamano maximo de caja declarado.
    tool_volume_margin_m: float = 0.04
    # Separacion entre el marco del terminal y el plano de contacto de las copas,
    # donde se apoya la cara superior de la caja. Es geometria del terminal.
    tool_to_box_offset_m: float = 0.089
    # Holgura por encima de ese plano. Las copas ocupan z entre 0,065 y 0,089 en
    # el marco del terminal: un margen generoso los mete en la nube y estira la
    # altura hasta 30 mm.
    cup_plane_margin_m: float = 0.002


@dataclass(frozen=True)
class EstimatorConfig:
    # EXP-006: recortar medio punto porcentual por extremo costaba hasta 11 mm
    # de anchura, porque los extremos que solo define la silueta tienen poca
    # densidad. Con 0,05 el error queda por debajo del milimetro.
    percentile_low: float = 0.05
    percentile_high: float = 99.95
    bootstrap_samples: int = 24
    bootstrap_point_cap: int = 12_000
    minimum_points: int = 800
    minimum_views: int = 2
    # Cobertura: cuantos puntos deben apoyar cada extremo dentro de la loncha.
    # El extremo peor medido sobre el rango de cajas aporta 121 puntos.
    coverage_slab_m: float = 0.002
    minimum_face_support_points: int = 50
    # Residuo p95 por vista medido sobre el rango de cajas: 0,805 mm en el peor
    # caso. El umbral deja margen y detecta un desplazamiento de 10 mm.
    max_view_plane_residual_m: float = 0.003
    max_uncertainty_m: float = 0.012
    cuboid_residual_scale_m: float = 0.012
    # Puntos mas hacia dentro que esto no cuentan como fallo de registro: son
    # el hueco de un defecto, no una vista desplazada. El envolvente (percentil)
    # sigue midiendo L/W/H. El inspector usa esos puntos aparte.
    inward_residual_ignore_m: float = 0.002


@dataclass(frozen=True)
class DamageConfig:
    relative_threshold: float = 0.05
    # Suelo absoluto: ruido de intactas en el entorno ideal es < 1 mm.
    absolute_floor_m: float = 0.003
    corner_radius_m: float = 0.020
    edge_radius_m: float = 0.004
    minimum_corner_support: int = 15
    cluster_inward_m: float = 0.002


@dataclass(frozen=True)
class AppConfig:
    # Paso del catalogo de cajas. La salida publica incluye la medida ajustada a
    # este paso; con 0 no se publica ninguna medida ajustada.
    catalogue_step_m: float = 0.005
    box_range: BoxRange = field(default_factory=BoxRange)
    sensor: SensorConfig = field(default_factory=SensorConfig)
    estimator: EstimatorConfig = field(default_factory=EstimatorConfig)
    damage: DamageConfig = field(default_factory=DamageConfig)
