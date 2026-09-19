# EXP-009 — Cámara baja (espejo en Z) y yaw 180° para defectos

## Estado

Implementado en esta rama. Cámara baja y yaw 180 de inspección están en el
ciclo de estación. `measure()` sigue usando solo Y0+Y90.

Las dos tareas no comparten el mismo conjunto de vistas:

- **Dimensiones:** las mismas dos vistas de EXP-008, con la cámara **debajo**.
- **Detección de caras defectuosas:** misma cámara; un yaw extra a 180° para
  la lateral que 0°+90° no enseñan. Esa vista **no** entra en `measure()`.

## Hipótesis

1. Si se conserva el `(x, y)` de `scan_rgbd_cam` y se espeja su `z` respecto al
   punto de mira `scan_target`, la cámara queda tan por debajo de la caja como
   ahora queda por encima, mira hacia arriba (~17°) y ve la cara inferior más
   las laterales, sin tilt.
2. Adaptar solo esa geometría a Y0+Y90 **no degrada** L/W/H: el AABB gana
   apoyo en la base, el eje flojo de EXP-008.
3. El retorno 90° → 0° es sentido contrario y no aporta una lateral nueva. Un
   segundo `+90°` (90° → 180°) es solo para inspección; el estimador sigue
   usando dos capturas.

## Entorno

- Rama: `exp-009-camara-baja-yaw-180`, sobre `main` (`782a705`).
- Simulador: MuJoCo 3.13.0. El XML de producción de esta rama ya tiene
  `z = 0,38025` m. El sondeo inicial mutó `cam_pos` en RAM sobre la escena alta.
- Cajas de cobertura: mínima, nominal, máxima (checkpoint).
- Medida L/W/H: caja nominal, `measure()` de producción con **exactamente**
  Y0+Y90. Y180 se midió aparte como control, no como ciclo propuesto.
- Ideal: sin ruido, weld rígido.

## Geometría actual y propuesta

Cámara actual: `(0,5566, 1,22835, 0,91975)` m. Mira a
`(-0,174, 0,735, 0,650)` m. Distancia horizontal 0,882 m. Elevación
**+17,0°** sobre la horizontal (`z` de la cámara 0,270 m por encima del
target).

Propuesta para **ambas** tareas (mismo XY, `z` espejado):

`(0,5566, 1,22835, 0,38025)` m → elevación **−17,0°** (mira hacia arriba).
Holgura al suelo: 0,38 m. El apoyo de recogida (`z` 0,355–0,455 m) no coincide
con el XY de la cámara.

El FOV vertical (50°) y la distancia no cambian: el cubo máximo (altura 0,25 m)
sigue cabiendo en ángulo.

## Sistema de medición

El aceptado en `main`: dos yaws, `minimum_views = 2`. Fondos de Y0 y Y90
recalculados con la cámara mutada. Y180 no se pasa a `measure()` en el ciclo
propuesto. El ground truth no entra en el estimador.

## Método

Espejo: `z' = 2 · z_target − z_cámara`. Cobertura con los umbrales de EXP-002.
El yaw 180 se sondea para cinemática y encuadre de inspección, no para el
contrato `ObjectDimensions`.

## Protocolo

```bash
source .venv/bin/activate
python -m object_profiling.evaluation.low_camera_viability \
  --output results/exp-009-viability.json
```

La fila que cuenta para dimensiones es `proposed_y0_y90`. Las filas con Y180
son control de inspección / ablación.

## Resultados

### Movimiento

| Transición | Alcanzada | Error articular máx. | Uso |
|---|---|---:|---|
| 0° → 90° | sí | 0,0038 rad | medida + inspección |
| 90° → 180° | sí | 0,0038 rad | solo inspección |

No hace falta IK nueva: `lift_qpos` con el último eje a +π.

### Cobertura (criterios EXP-002)

Cámara **alta** (producción), Y0 y Y90: 6/6 válidas. Peor fracción visible
0,866 (mínima; las copas tapan desde arriba). Peor margen 76 px (máxima).

Cámara **baja**, Y0 y Y90 (las de medida): 6/6 válidas, fracción visible
**1,000**, margen ≥ 168 px. Y180 (inspección) también 3/3 válidas en
mínima/nominal/máxima. Profundidad de la caja ~0,65–1,12 m.

Desde abajo el bastidor de copas deja de ocluir. La cara **superior** queda
del lado del terminal, como se pide para defectos.

### L/W/H, caja nominal (error continuo en mm, snap a 5 mm)

| Cámara y vistas | Rol | Válido | Snap | ΔL | ΔW | ΔH |
|---|---|---|---|---:|---:|---:|
| Alta, Y0+Y90 | producción hoy | sí | sí | +0,017 | −0,242 | **+0,828** |
| **Baja, Y0+Y90** | **medida propuesta** | sí | sí | +0,015 | −0,234 | **−0,071** |
| Baja, Y0+Y180 | control, no medida | sí | sí | +0,016 | +0,051 | −0,097 |
| Baja, Y0+Y90+Y180 | control, no medida | sí | sí | +0,016 | +0,049 | −0,078 |

Adaptar las dos vistas actuales a cámara baja **mejora la altura** en la
nominal (se ve la base). No hace falta meter Y180 en el estimador.

Confirmación rápida, 20 seeds (1000–1019), cámara baja, Y0+Y90:

| Eje | MAE | p95 | Máximo |
|---|---:|---:|---:|
| longitud | 0,016 mm | 0,021 mm | 0,022 mm |
| anchura | 0,195 mm | 0,268 mm | 0,404 mm |
| altura | 0,102 mm | 0,144 mm | 0,144 mm |

20 / 20 válidos, snap 20 / 20, `meets_targets` verdadero. La altura mejora un
orden de magnitud respecto a EXP-008 con cámara alta (~1,07 mm MAE).

## Observaciones y fallos

- Y0+Y90 oblicuas ven tres laterales; la cuarta sale en Y180. Eso es de
  **inspección**, no de cuboide.
- No hay clasificador de defectos en este repositorio. EXP-009 solo fija
  visibilidad.
- No se ha repetido el benchmark de 300 seeds con cámara baja (sí cobertura
  min/nom/máx y una medida nominal).
- Fondo hacia arriba: en la nominal `measure()` con Y0+Y90 no falló.
- Caja máxima en YAW_0: ~1626 px que no son caja (apoyo de recogida en el
  recorte oblicuo inferior). IoU 0,966, aún sobre el umbral 0,95 de EXP-004.
  No estiró L/W/H fuera del objetivo en la auditoría de geometría (0,42 mm).
- Tras Y90, el ciclo de medida puede volver a vertical o, si la estación
  sigue a inspección, continuar a 180°. Eso no cambia `SCAN_POSES` de
  `measure()`.
- No mover XY: se perdería el tres cuartos.

## Decisión

**Aplicado en esta rama:** XML con `z = 0,38025` m; `SCAN_POSES = (Y0, Y90)`;
después de medir, la estación gira a `SCAN_YAW_180` y captura RGB-D de
inspección que **no** entra en `measure()`.

Pendiente: repetir EXP-007 a 300 seeds con la cámara baja y, más adelante, el
clasificador de defectos.

Descartado: meter Y180 en el contrato de dimensiones; tilt; cambiar XY.
