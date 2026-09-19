# EXP-000 — Agarre, elevación y rotación

## Estado

Superado por [EXP-001](EXP-001-trayectoria-fija-inclinacion-35.md). Se conserva
como registro del primer movimiento verificado, limitado al giro vertical.

## Objetivo del checkpoint

Comprobar de forma aislada que el UR10e puede tomar una caja nominal ya
presentada bajo el terminal, elevarla y detenerse en orientaciones de 0°, 90° y
180°. Este experimento no evalúa percepción ni medición.

## Entorno

- Revisión: árbol inicial sin commit; estado reproducible mediante los archivos
  actuales del repositorio.
- Simulador: MuJoCo 3.13.0.
- Modelo: UR10e de MuJoCo Menagerie, revisión
  `8161bba264d7fa7c99ca301e91e7fb44737676ad`.
- Seed registrada: 42. La geometría de EXP-000 es fija; la seed no altera la
  escena.
- Caja: cuboide rígido de 0,30 × 0,20 × 0,15 m y 2 kg.
- Terminal: bastidor abierto con cinco copas conceptuales Piab BCP de 40 mm.
- Posición inicial: caja centrada, alineada y apoyada bajo las cinco copas.

## Solución implementada

- `ATTACH_SUCTION`: activa una unión rígida `equality weld` entre terminal y
  caja en su pose relativa actual.
- `LIFT`: trayectoria articular suave hasta una pose obtenida por IK que eleva
  el TCP 0,12 m y conserva su orientación.
- `ROTATE_90` y `ROTATE_180`: giro de la última articulación del UR10e mientras
  se mantiene la pose elevada.
- Control: actuadores de posición, interpolación minimum-jerk, espera de
  estabilización y timeout.
- Auditoría: en cada pose final se registra estado articular, posición de la
  caja y deriva de la transformación relativa terminal-caja.

La unión rígida es una simplificación explícita. EXP-000 no demuestra sellado,
fugas, deformación del cartón, deslizamiento ni capacidad física real de las
ventosas.

## Criterios de aceptación

- Secuencia completa con estados finales 0°, 90° y 180°.
- Elevación vertical observada de al menos 0,10 m.
- Error articular máximo menor o igual a 0,025 rad.
- Deriva terminal-caja menor o igual a 1 mm y 0,5°.
- Unión activa durante todos los estados auditados.
- Código de salida 0 y `success: true`.

## Protocolo

```bash
source .venv/bin/activate
python -m pytest -p no:cacheprovider
object-profiling-checkpoint --headless --seed 42 --output results/checkpoint-seed-42.json
```

Para inspección visual, `--speed MULTIPLICADOR` cambia solamente el ritmo de
reproducción del visor; el timestep y el estado simulado permanecen iguales.

El informe JSON se genera desde el estado del simulador. Las dimensiones y la
pose reales se usan aquí únicamente para construir y auditar el escenario; no
existe todavía un estimador de percepción.

## Resultado observado

Ejecución del 19 de septiembre de 2026:

| Métrica | Resultado |
|---|---:|
| Pruebas automatizadas | 3/3 superadas |
| Secuencia | 0°, 90° y 180° completada |
| Elevación observada | 0,114257 m |
| Error articular máximo | 0,005226 rad |
| Deriva lineal máxima terminal-caja | 0,000340 m |
| Deriva angular máxima terminal-caja | 0,015827° |
| Resultado headless | `success: true` |

## Límites y siguiente decisión

Este resultado valida solo la cadena cinemática, el control básico y la
abstracción rígida de agarre en simulación. No valida la factibilidad física de
la ventosa ni la visibilidad de la caja ante una cámara.

El desarrollo se detiene en este checkpoint hasta que el equipo ejecute y
acepte la demo visual. No se inicia la medición ni la percepción como parte de
EXP-000.
