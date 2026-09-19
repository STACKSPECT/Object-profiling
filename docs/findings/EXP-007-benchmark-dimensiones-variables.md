# EXP-007 — Benchmark de dimensiones variables

## Estado

Implementado, ejecutado y medido en simulación sobre 300 seeds. Medido en el
entorno ideal: sin ruido de profundidad, sin error de calibración, sin
oscilación, sin deslizamiento y con succión abstraída como unión rígida.

## Hipótesis

El ciclo fijo debe medir longitud, anchura y altura de cajas con dimensiones
aleatorias de forma reproducible, cumpliendo los criterios originales del plan.

## Entorno

- Base de código: commit `b4a7d60` más los cambios de EXP-007.
- Simulador: MuJoCo 3.13.0 en macOS, backend OpenGL sin `ARB_clip_control`.
- Generación de escenas: `generate_box_spec()`, longitud 0,15–0,40 m, anchura
  0,12–0,30 m con `width <= length`, altura 0,08–0,25 m, masa 0,5–5 kg.
- Rangos ejecutados: seeds 1000–1099 (100 escenas) y 5000–5199 (200 escenas).
- Agarre centrado, sin ruido añadido, sin oclusiones externas.

## Sistema de medición

El del ciclo aceptado: trayectoria fija `SCAN_YAW_0`, `SCAN_YAW_90` y
`SCAN_TILT_35`, fondos por pose en pasada previa, segmentación por resta de
fondo, registro en `ur10e_attachment_site` y cuboide alineado a ejes.

## Protocolo

```bash
source .venv/bin/activate
python -m pytest -p no:cacheprovider
object-profiling-benchmark --start 1000 --count 100 \
  --output results/benchmark-1000-1099.json
object-profiling-benchmark --start 5000 --count 200 --quiet \
  --output results/benchmark-5000-5199.json
```

## Resultados

### Seeds 1000–1099, 100 escenas

| Métrica | Resultado |
|---|---:|
| Tasa de perfiles válidos | 100 / 100 |
| Rechazos | ninguno |
| Peor error absoluto | 2,003 mm |
| Latencia de percepción p50 / p95 | 28,7 / 36,7 ms |
| Tiempo de ciclo p50 / p95 | 0,278 / 0,318 s |
| Error de registro p95, media / máximo | 0,145 / 0,263 mm |
| Confianza media | 0,931 |
| Reproducible al repetir seed | sí |

Por dimensión, en milímetros:

| Eje | MAE | RMSE | p95 | Máximo | Incertidumbre media | Cobertura |
|---|---:|---:|---:|---:|---:|---:|
| longitud | 0,024 | 0,025 | 0,035 | 0,037 | 1,919 | 1,000 |
| anchura | 0,179 | 0,191 | 0,304 | 0,371 | 1,930 | 1,000 |
| altura | 0,979 | 1,150 | 1,796 | 2,003 | 1,947 | 1,000 |

### Seeds 5000–5199, 200 escenas

| Eje | MAE | p95 | Máximo | Cobertura |
|---|---:|---:|---:|---:|
| longitud | 0,024 mm | 0,035 mm | 0,036 mm | 1,000 |
| anchura | 0,187 mm | 0,317 mm | 0,407 mm | 1,000 |
| altura | 0,958 mm | 1,758 mm | 1,953 mm | 1,000 |

200 / 200 válidos, peor error 1,953 mm, latencia p50 29,5 ms, ciclo p50 0,289 s,
reproducible. Las cifras coinciden con el primer rango en la tercera cifra
decimal, así que no dependen del rango de seeds elegido.

### Criterios originales

| Criterio | Objetivo | Medido | Cumple |
|---|---|---|---|
| Perfiles válidos | ≥ 98 % | 100 % en 300 escenas | sí |
| MAE por dimensión | ≤ 5 mm | 0,024 / 0,187 / 0,979 mm | sí |
| p95 por dimensión | ≤ 10 mm | 0,035 / 0,317 / 1,796 mm | sí |
| Ningún resultado fuera de tolerancia marcado como válido | — | ninguno | sí |
| Resultado idéntico al repetir seed | — | sin discrepancias | sí |

## Observaciones y fallos

- **La altura es cuarenta veces peor que la longitud**, 0,979 mm frente a
  0,024 mm de MAE. La longitud queda determinada por dos caras laterales bien
  vistas; la altura depende del borde inferior, que solo aparece en la silueta.
  Es el mismo patrón que EXP-006 midió sobre las cajas dirigidas, ahora
  confirmado sobre 300 escenas.
- **La calibración de la incertidumbre está justa en altura.** Cubre el error en
  el 100 % de los casos, pero la incertidumbre media de altura es 1,947 mm y el
  error máximo observado 2,003 mm. El margen es prácticamente nulo en el peor
  caso, así que la cobertura del 100 % no debe leerse como holgura. En longitud y
  anchura la incertidumbre es claramente conservadora: 1,9 mm frente a errores de
  0,02 a 0,4 mm, porque está dominada por el término de resolución lateral.
- **Cero rechazos significa que los criterios de rechazo no se han ejercitado en
  este rango.** Su comportamiento está comprobado con pruebas dirigidas por
  configuración, no por escenas que fallen de verdad. Hasta que EXP-008 añada
  ruido y oclusiones no hay evidencia de que discriminen bien.
- El benchmark sin perfiles válidos informa `null` en las métricas por eje en
  lugar de un número inventado, y `meets_targets: false`.

## Límites de esta medición

Estas cifras describen el entorno ideal. No demuestran:

- comportamiento con ruido de profundidad ni error de calibración;
- tolerancia a oclusiones o a un agarre descentrado;
- nada sobre el sellado de la ventosa, fugas, cartón poroso, deformación ni
  deslizamiento: la succión sigue siendo un `equality weld` rígido declarado;
- recogida desde la cinta, que no forma parte de este incremento.

## Decisión

La medición suspendida con trayectoria fija queda aceptada para integrarse por
contrato. El siguiente experimento debe introducir realismo, empezando por ruido
de profundidad y error de calibración, y medir cuánto se degradan estas cifras y
si los criterios de rechazo discriminan. Después, la recogida desde la cinta.
