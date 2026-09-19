# EXP-013 — Reconocimiento de cajas dañadas

## Estado

Implementado, ejecutado y medido. Umbrales tomados de las intactas de
EXP-012, no de la banda con la que se reporta el descarte.

## Hipótesis

Con `umbral = max(0.05 * arista_más_corta, 3 mm)`, cero falsos positivos en
el entorno ideal, y detección de todo defecto **observable** por encima del
umbral. No se afirma detección de defectos en caras no vistas.

## Entorno

- Calibración: intactas de seeds 8000–8019.
- Evaluación de detección: la misma banda, 50 % de daño (20 seeds).
- Pruebas dirigidas: esquina `+x+y+z` y abolladura `+x` sobre la caja nominal.

## Sistema de medición

`assess_damage` publica `condition`, `routing` y `DamageReport`. `valid` no
cambia por la clasificación: si el cuboide falla por residuo de vista, se
inspecciona un AABB provisional y se puede emitir `DAMAGED` con `valid: false`.

## Protocolo

```bash
object-profiling-inspection-audit --start 8000 --count 20 --damage-rate 0.5 --output results/inspection-audit-8000-8019.json
python -m pytest -p no:cacheprovider tests/test_inspection.py tests/test_discard.py
```

## Resultados

| | Intactas | Dañadas |
|---|---:|---:|
| n | 12 | 8 |
| Clasificadas `DAMAGED` | 0 | 6 |
| Falso positivo | 0 | — |
| Fallos | — | 2 (cara no observada) |

Por clase: esquina 2/2, abolladura 3/4, pandeo 1/2. Las dos pérdidas están
en `+y` / no vistas, no por debajo del umbral en una cara observada.

Las pruebas dirigidas sobre `+x` y una esquina visible enrutan a
`ERROR_ZONE` y dejan la caja en el contenedor.

## Observaciones y fallos

Un hundimiento o pandeo visible suele romper `max_view_plane_residual_m`
(3 mm). Sin el AABB provisional, esas cajas salían `UNKNOWN` y no se
descartaban. Con él, `valid` puede ser falso y `condition` `DAMAGED`.

La clase publicada no siempre coincide con `DamageSpec`: una esquina con
residuo hacia dentro se etiqueta `CRUSHED_CORNER` si el soporte es 0.

Calibración válida solo sin ruido de profundidad.

## Decisión

El detector se acepta para el entorno ideal, con la limitación de caras no
vistas declarada. Siguiente: descarte visible (EXP-014).
