# EXP-012 — Señales de inspección, todavía sin umbral operativo

## Estado

Implementado, ejecutado y medido sobre seeds 8000–8019 con `damage.rate = 0.5`.
Las distribuciones se usaron para fijar el detector de EXP-013; esta entrada
conserva la evidencia, no el umbral.

## Hipótesis

1. El residuo global del cuboide **no** separa `CRUSHED_CORNER`.
2. La ocupación de esquina sí: un chaflán deja la esfera casi vacía.
3. Hundimiento y pandeo aparecen como residuo hacia dentro en la cara vista.
4. Ninguna señal consume `DamageSpec`.

## Entorno

- 20 seeds, 12 intactas y 8 dañadas (2 esquinas, 4 abolladuras, 2 pandeos).
- Inspector sobre la nube fusionada y, si el cuboide se rechaza, sobre un AABB
  provisional de la misma nube.

## Sistema de medición

Ciclo fijo. `measure/inspection.py` no importa MuJoCo ni `station.damage`.

## Método

Cinco señales: planaridad por cara, rectitud de arista (residuo PCA, no
distancia al segmento del cuboide), ocupación de esquina, fracción hacia
dentro y `residual_p95_m` global.

## Protocolo

```bash
object-profiling-inspection-audit --start 8000 --count 20 --damage-rate 0.5 --output results/inspection-audit-8000-8019.json
```

Banda de evaluación disjunta de la de descarte (8100+).

## Resultados

Intactas (12): `max_inward_m` ≤ 1,9 mm, `max_edge_p95_m` ≤ 3,2 mm,
`weakest_corner_support` ≥ 29, `residual_p95_m` ≤ 1,7 mm.

| Clase | n | `weakest_corner` típico | `max_inward` cuando se ve |
|---|---:|---:|---|
| INTACT | 12 | 29–45 | < 2 mm |
| CRUSHED_CORNER | 2 | 0 | 14–16 mm, residuo global ~1,5 mm |
| DENTED_FACE / BUCKLED_PANEL vistas | 4 | — | 10–30 mm |
| Defecto en cara no observada | 2 | como intacta | < 2 mm |

La predicción del plan se cumple: el residuo global de las dos esquinas
aplastadas es 1,4–1,6 mm, indistinguible de una sana; el soporte de esquina
cae a 0.

## Observaciones y fallos

Medir la arista contra el segmento del cuboide devolvía ~el radio del
entorno (~12 mm) en cajas sanas. La recta PCA lo corrige: intactas ≤ 3,2 mm.

Las caras `+y`, `-x` y `-y` a menudo no entran en la nube de las tres poses.
Un defecto ahí no mueve las señales.

## Decisión

Suelo absoluto de 3 mm y umbral relativo del 5 % de la arista más corta, más
`minimum_corner_support = 15`. Calibración válida solo sin ruido (R5).
Siguiente: clasificar (EXP-013).
