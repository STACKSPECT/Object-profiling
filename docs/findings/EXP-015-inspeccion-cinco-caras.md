# EXP-015 — Inspección de cinco caras y envolvente con daño

## Estado

Implementado, ejecutado y medido. Seeds de comprobación recorridas de una en
una (`--count 1`), no con el bucle de sesión.

## Hipótesis

1. La cámara baja de EXP-009 ya ve, en cada yaw, el fondo y dos laterales. El
   segundo giro +90° (`SCAN_YAW_180`) enseña la lateral `+y` que Y0+Y90 no
   cubren. Fusionar esa vista **solo** en el inspector cubre las cinco caras
   no agarradas.
2. Un hundimiento visible no debe tumbar L/W/H: el envolvente sigue siendo el
   cuboide nominal. El residuo de registro se relaja hacia dentro **después**
   de declarar daño, no antes, para no confundir una vista desplazada con un
   defecto.

## Entorno

- Cámara `scan_rgbd_cam` en `(0,5566, 1,22835, 0,38025)` m, target
  `(-0,174, 0,735, 0,650)` m: 0,27 m por debajo del centro de escaneo,
  elevación −17°.
- Ciclo: `SCAN_YAW_0` → `SCAN_YAW_90` → `SCAN_YAW_180`.
- Seeds individuales, `damage.rate = 0,5`, entorno ideal, weld rígido.
- Comprobación CLI: `python -m object_profiling.station.pipeline --seed 42 --count 1`
  y `--seed 78 --count 1`.

## Sistema de medición

- L/W/H: Y0+Y90, igual que EXP-008/009. `views_used` no incluye Y180.
- Inspección: nube de Y0+Y90+Y180 contra el cuboide de medida.
- Esquinas de la cara de agarre (`-z`) no votan `CRUSHED_CORNER`.
- Si el cuboide estricto falla por residuo y el inspector declara `DAMAGED`,
  se reestima ignorando puntos hacia dentro (`ignore_inward_residual`).

## Método

Cobertura por pose, caja nominal:

| Pose | Caras con soporte |
|---|---|
| `SCAN_YAW_0` | `+x`, `-y`, `+z` |
| `SCAN_YAW_90` | `-x`, `-y`, `+z` |
| `SCAN_YAW_180` | `-x`, `+y`, `+z` |

Unión: `+x -x +y -y +z`. `-z` es agarre y queda fuera del daño.

## Protocolo

```bash
python -m object_profiling.station.pipeline --seed 42 --count 1
python -m object_profiling.station.pipeline --seed 78 --count 1
python -m pytest -p no:cacheprovider tests/test_inspection.py tests/test_geometry.py \
  tests/test_profiling_pipeline.py tests/test_discard.py tests/test_camera_audit.py
```

No se usó `--count` mayor que 1 para esta medición.

## Resultados

Antes: seed 78 (`DENTED_FACE` en `+y`) salía `INTACT`; seeds 67/54/44
salían `DAMAGED` con `valid: false` y sin dimensiones.

Después, una seed por ciclo:

| Seed | Verdad | Condición | Válido | Error L/W/H (mm) |
|---:|---|---|---|---:|
| 42 | INTACT | INTACT | sí | 0,019 / 0,299 / 0,146 |
| 78 | DENTED_FACE `+y` | DAMAGED | sí | 0,021 / 0,300 / 0,093 |
| 56 | BUCKLED_PANEL `+y` | DAMAGED | sí | 0,015 / 0,218 / 0,153 |
| 67 | DENTED_FACE `+x` | DAMAGED | sí | 0,009 / 0,184 / 0,081 |
| 54 | BUCKLED_PANEL `+z` | DAMAGED | sí | 0,015 / 0,126 / 0,097 |
| 44 | DENTED_FACE `-y` | DAMAGED | sí | 0,016 / 0,200 / 0,041 |
| 52 | CRUSHED_CORNER | DAMAGED | sí | 0,017 / 0,381 / 0,111 |
| 45–51, 53, 55, 57 | INTACT | INTACT | sí | MAE 0,018 / 0,255 / 0,101 |

Cero falsos positivos en las intactas de esa banda. Las dañadas de `+y` y del
fondo se detectan. El CLI de seed 78 publica `location: face:+y`,
`routing: ERROR_ZONE` y dimensiones con snap a 5 mm.

## Observaciones y fallos

La clase publicada no siempre coincide con `DamageSpec`: un hundimiento amplio
puede etiquetarse `BUCKLED_PANEL`, y un chaflán pequeño `DENTED_FACE`. El
enrutado a zona de error es el que cuenta.

Un registro realmente desplazado sigue rechazándose: el residuo estricto no
se relaja en cajas intactas.

## Decisión

Conservar la cámara baja y el split Y0+Y90 / Y180. Fusionar Y180 en el
inspector. Publicar el envolvente de una caja dañada. Siguiente: repetir un
benchmark amplio si se quiere comparar con EXP-009 a 300 seeds; no hace falta
para cerrar este fallo de cobertura.
