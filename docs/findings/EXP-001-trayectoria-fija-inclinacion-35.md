# EXP-001 — Trayectoria fija con inclinación de 35°

## Estado

Implementado y verificado automáticamente en simulación. Pendiente de
aceptación visual manual por el equipo.

## Hipótesis

Una secuencia fija con dos orientaciones verticales y una inclinación de 35°
puede presentar siempre las tres dimensiones de una caja rectangular sin tomar
decisiones adaptativas durante el ciclo.

Este experimento valida únicamente el movimiento. Todavía no demuestra que las
tres vistas permitan recuperar las dimensiones mediante una cámara.

## Entorno

- Simulador: MuJoCo 3.13.0.
- Robot: UR10e de MuJoCo Menagerie.
- Terminal: bastidor abierto con cinco copas conceptuales de 40 mm.
- Agarre: unión rígida `equality weld` declarada.
- Cajas auditadas:
  - mínima: 0,15 × 0,12 × 0,08 m y 0,5 kg;
  - nominal: 0,30 × 0,20 × 0,15 m y 2 kg;
  - máxima: 0,40 × 0,30 × 0,25 m y 5 kg.
- El apoyo inicial se alinea con la base de cada caja como preparación del
  escenario. Esta información no interviene en la trayectoria.
- Cámara y estimación de dimensiones: no utilizadas.

## Movimiento fijo

```text
ATTACH_SUCTION
→ LIFT / SCAN_YAW_0
→ SCAN_YAW_90
→ SCAN_TILT_35
→ RETURN_VERTICAL
```

`SCAN_TILT_35` es una solución de cinemática inversa calculada para inclinar el
terminal 35° alrededor de su eje X local y mantener fija la posición del TCP.
La misma pose articular se usa con cualquier caja; no depende de sus medidas.

La transición utiliza interpolación articular minimum-jerk y control de
posición. La caja se audita durante cada paso para detectar contactos después
de abandonar el apoyo.

## Criterios de aceptación

- Completar los cuatro estados finales en el orden establecido.
- Orientación de la pose inclinada: 35° ± 0,5°.
- Regresar a vertical con error angular menor de 0,5°.
- Elevar la caja al menos 0,10 m.
- Error articular máximo menor o igual a 0,025 rad.
- Deriva terminal-caja menor o igual a 1 mm y 0,5°.
- Cero muestras con contacto de la caja fuera de la fase inicial de elevación.
- Superar los casos mínimo, nominal y máximo con la misma trayectoria.

## Protocolo

```bash
source .venv/bin/activate
python -m pytest -p no:cacheprovider
object-profiling-checkpoint --headless --seed 42 \
  --output results/checkpoint-seed-42.json
```

Demo visual:

```bash
mjpython -m object_profiling.checkpoint --visual --seed 42 --speed 1.0
```

## Resultados

Ejecución headless del 19 de septiembre de 2026:

| Métrica | Resultado |
|---|---:|
| Pruebas automatizadas | 13/13 superadas |
| Casos dimensionales | mínimo, nominal y máximo superados |
| Inclinación nominal observada | 34,513° |
| Elevación nominal observada | 0,114202 m |
| Error articular nominal máximo | 0,005226 rad |
| Deriva lineal nominal máxima | 0,000345 m |
| Deriva angular nominal máxima | 0,008697° |
| Contactos nominales inesperados | 0 |
| Resultado nominal | `success: true` |

Durante el desarrollo, la caja máxima reveló que cambiar la posición de una
geometría estática requiere recalcular las constantes compiladas de MuJoCo. La
auditoría detectó el contacto con el apoyo; se corrigió la preparación de escena
sin relajar los umbrales ni alterar la trayectoria fija.

## Decisión

Se conserva esta trayectoria como ciclo determinista para el siguiente
experimento. El siguiente paso, no incluido aquí, será capturar RGB-D en
`SCAN_YAW_0`, `SCAN_YAW_90` y `SCAN_TILT_35`, comprobando primero encuadre y
cobertura sin implementar aún la estimación de dimensiones.
