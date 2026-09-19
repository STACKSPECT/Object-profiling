# EXP-010 — Malla única y generación de daño

## Estado

Implementado, ejecutado y medido en simulación. La malla es la representación
de todas las cajas. Los generadores de daño están medidos por render, no solo
diseñados.

## Hipótesis

Sustituir la primitiva `box` por una malla de cuboide de topología fija no
debe degradar la precisión dimensional de las cajas sanas. Sobre esa malla, un
solo defecto hacia dentro (esquina, abolladura o pandeo), sin tocar la cara de
agarre, debe ser observable en profundidad y no crecer el envolvente.

## Entorno

- Simulador: MuJoCo 3.13.0 en Windows, `scan_rgbd_cam` 640 × 480.
- Seeds de no regresión: 1000–1029, `damage.rate = 0.0`.
- Caja nominal de generación: 0,30 × 0,20 × 0,15 m.
- Sin ruido de profundidad. Succión: `equality weld` rígido declarado.

## Sistema de medición

El ciclo fijo de EXP-007. La caja visual es `box_geom` tipo malla; la colisión
sigue siendo `box_collision`, el envolvente intacto. `load_box` escribe
vértices en el marco compilado (Kabsch) sin recompilar, para un solo visor.

## Método

- `station/boxmesh.py`: cuboide 16 subdivisiones, winding hacia fuera.
- `station/damage.py`: `CRUSHED_CORNER`, `DENTED_FACE`, `BUCKLED_PANEL`.
- Cara de agarre `-z` excluida. Un defecto por caja. Severidad 2–25 % de la
  arista más corta.

## Protocolo

```bash
python -m pytest -p no:cacheprovider
object-profiling-damage-audit --artifacts artifacts/damage-audit --output results/damage-audit.json
object-profiling-benchmark --start 1000 --count 30 --determinism-samples 2 --output results/benchmark-1000-1029.json
```

## Resultados

### No regresión dimensional (30 seeds intactas, 1000–1029)

| Eje | MAE | p95 | Máximo | EXP-007 MAE |
|---|---:|---:|---:|---:|
| longitud | 0,024 mm | 0,032 mm | 0,034 mm | 0,024 mm |
| anchura | 0,154 mm | 0,264 mm | 0,286 mm | 0,179 mm |
| altura | 0,985 mm | 1,827 mm | 1,940 mm | 0,979 mm |

30 / 30 válidos, reproducible, `meets_targets: true`. El MAE no empeora más
allá del ruido de reejecución.

### Observabilidad del daño (caja nominal en `SCAN_YAW_0`)

Desviación de profundidad sobre píxeles que siguen viendo superficie, no
el fondo a través de la silueta recortada:

| Caso | Píxeles de superficie | Pico | Píxeles de silueta |
|---|---:|---:|---:|
| INTACT | 0 | 0,0 mm | 0 |
| CRUSHED_15 | 27 | 5,4 mm | 84 |
| CRUSHED_40 | 84 | 7,0 mm | 567 |
| DENT_10 | 3.656 | 14,0 mm | 1 |
| DENT_25 | 4.638 | 34,7 mm | 2 |
| BUCKLE_20 | 7.827 | 28,0 mm | 0 |

La esquina aplastada se ve sobre todo como cambio de silueta, no como residuo
de cara. Hundimiento y pandeo en `+x` son ampliamente observables y crecen con
la severidad declarada.

## Observaciones y fallos

- `geom_size` de una malla escala vértices ya métricos. El tamaño lo llevan
  los vértices; solo se actualizan AABB y `rbound`.
- El compilador reorienta la malla; hay que escribir en el marco compilado.
- El casco de colisión es el envolvente intacto (R2): no se valida el apoyo
  físico de una cara hundida.

## Decisión

La malla queda como representación única. Los generadores de CP1 se conservan.
Siguiente: recoger las cajas dañadas (EXP-011).
