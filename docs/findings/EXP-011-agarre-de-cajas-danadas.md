# EXP-011 — Agarres de cajas dañadas

## Estado

Implementado, ejecutado y medido en simulación sobre 16 seeds con
`damage.rate = 1.0` y sobre las tres clases dirigidas de la caja nominal.

## Hipótesis

El weld de succión y la colisión por envolvente permiten presentar una caja
dañada en las tres poses de escaneo sin `MOTION_TIMEOUT` ni penetración
por encima de la tolerancia del checkpoint.

## Entorno

- Seeds 200–215, todas dañadas.
- Cajas nominales con `CRUSHED_CORNER` 30 mm, `DENTED_FACE` 25 mm y
  `BUCKLED_PANEL` 20 mm.
- Colisión: primitiva `box_collision`, no la malla hundida.

## Sistema de medición

Checkpoint de movimiento `grasp_lift_yaw_tilt_return`. Sin medición.

## Método

`active_cup_names()` sigue decidiendo por L × W del envolvente. Un chaflán
puede dejar una copa exterior sobre el vacío; el weld las sella igual. Queda
declarado: no valida el vacío.

## Protocolo

```bash
python -m pytest -p no:cacheprovider tests/test_checkpoint.py tests/test_damage.py
object-profiling-checkpoint --start 200 --count 16 --damage-rate 1.0 --output results/checkpoint-damaged-200-215.json
```

## Resultados

| Métrica | Resultado |
|---|---:|
| Semillas dañadas | 16 / 16 |
| Agarres establecidos | 16 / 16 |
| `MOTION_TIMEOUT` | 0 |
| Penetraciones inesperadas | 0 |
| Caja nominal, tres clases | 3 / 3 |

## Observaciones y fallos

La colisión no ve el hundimiento. El checkpoint demuestra que el ciclo de
movimiento no se rompe, no que una ventosa selle sobre cartón dañado.

## Decisión

El agarre de cajas dañadas se acepta bajo la abstracción ya declarada.
Siguiente: señales de inspección (EXP-012).
