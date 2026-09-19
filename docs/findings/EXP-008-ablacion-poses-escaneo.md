# EXP-008 — Ablación de las tres poses de escaneo

## Estado

Implementado, ejecutado y medido en simulación. En esta rama experimental el
ciclo de producción es `SCAN_YAW_0` + `SCAN_YAW_90` (`minimum_views = 2`). El
tilt queda fuera del showcase y del demo.

## Hipótesis

`SCAN_YAW_0` y `SCAN_YAW_90` ya ven pares laterales complementarios. El tilt de
35° aporta sobre todo la base. Si el error de altura con solo las dos verticales
sigue por debajo de 2,5 mm, el redondeo a 5 mm recupera el SKU y la tercera pose
no es estrictamente necesaria para la fase 1.

## Entorno

- Rama: `exp-008-pose-ablation`, sobre la modularización en cuatro paquetes.
- Simulador: MuJoCo 3.13.0, backend OpenGL sin `ARB_clip_control`.
- Contraste: cajas mínima, nominal, máxima y seeds 42, 72, 731 (ya en rejilla
  de 5 mm).
- Dimensiones variables: seeds 1000–1049 (50 escenas), mismo generador que
  EXP-007.
- Tiempo de ciclo: 8 seeds (1000–1007) ejecutando la trayectoria real recortada,
  no un subconjunto de las mismas capturas.
- Entorno ideal: sin ruido, sin slip, weld rígido.

## Sistema de medición

El aceptado, con `minimum_views = 1` solo en esta evaluación para poder medir
subconjuntos de una vista. En el ciclo de esta rama el estimador exige 2 vistas.

Captura de calidad: las tres poses de siempre. Medición: subconjuntos de esas
observaciones, mismos fondos.

Ciclo recortado: fondos y capturas solo de las poses del subconjunto, más
retorno a vertical.

## Método

AABB en el marco del terminal, snap a `catalogue_step_m = 0.005`. El ground
truth no entra en `measure()`. Un snap acierta si
`dimensions_snapped_m` coincide con las aristas reales.

## Protocolo

```bash
source .venv/bin/activate
python -m pytest tests/test_pose_ablation.py
python -m object_profiling.evaluation.pose_ablation \
  --start 1000 --count 50 --cycle-count 8 \
  --output results/pose-ablation.json
```

## Resultados

### Contraste, 6 cajas

| Subconjunto | Válidos | Snap 5 mm | MAE L/W/H (mm) | Peor error (mm) |
|---|---:|---:|---|---:|
| Y0 | 4 / 6 | 4 / 4 | 0,50 / 0,29 / 1,00 | 1,44 |
| Y90 | 5 / 6 | 5 / 5 | 0,28 / 0,43 / 0,98 | 1,29 |
| T35 | 3 / 6 | 3 / 3 | 0,61 / 0,31 / 0,26 | 0,99 |
| **Y0+Y90** | **6 / 6** | **6 / 6** | 0,017 / 0,27 / 0,83 | 1,37 |
| Y0+T35 | 6 / 6 | 6 / 6 | 0,46 / 0,20 / 0,55 | 1,22 |
| Y90+T35 | 6 / 6 | 6 / 6 | 0,025 / 0,21 / 0,56 | 1,02 |
| all3 | 6 / 6 | 6 / 6 | 0,025 / 0,20 / 0,69 | 1,29 |

Las vistas sueltas fallan por `INSUFFICIENT_FACE_COVERAGE` (mínima; tilt también
en nominal y alargada).

### Seeds 1000–1049, 50 escenas

| Subconjunto | Válidos | Snap 5 mm | MAE L/W/H (mm) | p95 H (mm) | Máximo (mm) |
|---|---:|---:|---|---:|---:|
| Y0 | 35 / 50 | 35 / 35 | 0,41 / 0,19 / 1,07 | 1,72 | 1,88 |
| Y90 | 41 / 50 | 41 / 41 | 0,25 / 0,33 / 1,15 | 1,87 | 1,90 |
| T35 | 31 / 50 | 31 / 31 | 0,48 / 0,15 / 0,25 | 0,30 | 1,00 |
| **Y0+Y90** | **50 / 50** | **50 / 50** | 0,016 / 0,22 / 1,15 | 1,85 | 1,90 |
| Y0+T35 | 50 / 50 | 50 / 50 | 0,41 / 0,13 / 0,96 | 1,77 | 1,99 |
| Y90+T35 | 50 / 50 | 50 / 50 | 0,024 / 0,17 / 0,97 | 1,83 | 1,99 |
| all3 | 50 / 50 | 50 / 50 | 0,024 / 0,16 / 1,07 | 1,83 | 2,01 |

Cualquier par de vistas recupera el catálogo en las 50. Una sola vista no es
estable. Y0+Y90 iguala la longitud de las tres vistas; la altura MAE sube ~0,08
mm. Y90+T35 es el mejor par (anchura y altura). El máximo de tres vistas (2,01
mm) sigue por debajo de medio paso de catálogo.

### Tiempo de ciclo real recortado, 8 seeds

Tiempo **simulado** (el del robot), p50:

| Trayectoria | Simulado p50 | Pared p50 |
|---|---:|---:|
| Y0+Y90 | 2,70 s | 0,18 s |
| Y90+T35 | 2,79 s | 0,19 s |
| all3 | 3,59 s | 0,24 s |

Dos poses ahorran ~0,8–0,9 s de tiempo de robot (~25 %), fondos incluidos. La
pared del Mac no es el dato de estación.

### Confirmación: protocolo EXP-007 con dos yaws

Mismo protocolo que EXP-007 (seeds 1000–1099 y 5000–5199, 300 escenas), con el
ciclo de producción de esta rama. `meets_targets` verdadero en ambos rangos.
Snap a 5 mm: 300 / 300.

| Eje | MAE | p95 | Máximo |
|---|---:|---:|---:|
| longitud | 0,016 mm | 0,021 mm | 0,023 mm |
| anchura | 0,242 mm | 0,380 mm | 0,500 mm |
| altura | 1,066 mm | 1,795 mm | 1,901 mm |

300 / 300 válidos, reproducibles, peor error 1,901 mm (EXP-007 con tres poses:
2,003 mm). La longitud mejora; la anchura sube ~0,06 mm MAE; la altura ~0,09 mm
MAE. Sigue por debajo de medio paso de catálogo y del MAE de 5 mm del plan.
Ciclo de pared p50 ~0,23 s (EXP-007: 0,28 s).

```bash
python -m object_profiling.evaluation.benchmark --start 1000 --count 100 \
  --output results/benchmark-1000-1099-two-yaw.json
python -m object_profiling.evaluation.benchmark --start 5000 --count 200 --quiet \
  --output results/benchmark-5000-5199-two-yaw.json
```

## Observaciones y fallos

- Tilt solo: mejor altura cuando es válido (ve la base), pero 19 / 50 rechazos.
- Y0+T35: longitud peor (~0,4 mm MAE) porque falta el lateral a 90°.
- Fusionar las tres no siempre reduce el máximo de altura respecto a Y0+Y90
  (2,01 mm vs 1,90 mm en este rango). El tilt no es un seguro de altura en el
  peor caso.
- `minimum_views = 3` en producción habría rechazado los pares. Esa puerta es
  de protocolo, no de geometría.

## Decisión

**Las dos verticales bastan para el SKU de 5 mm** en este entorno. El tilt no es
estrictamente necesario para la fase 1.

En esta rama experimental el ciclo de producción es `SCAN_YAW_0` + `SCAN_YAW_90`
con `minimum_views = 2`. El showcase y el demo recorren solo esas dos poses.
`SCAN_TILT_35` queda en el módulo de ablación para comparar, no en la trayectoria
visible.

Alternativa no aplicada: `SCAN_YAW_90` + `SCAN_TILT_35` (mejor par de altura;
evita volver a yaw 0 para inclinar).
