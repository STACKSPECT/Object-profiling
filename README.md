# Object Profiling

Repositorio de exploración para medir cajas rectangulares mientras un UR10e las
mantiene suspendidas y las presenta desde varias orientaciones ante una cámara
RGB-D fija.

Este trabajo está aislado del repositorio de integración `hackspain/Simulation`.
Su salida principal serán las tres dimensiones de la caja y su incertidumbre,
consumibles por el resto del sistema. Este repositorio sí incluye el agarre y
los movimientos del UR10e necesarios para medir; no incluye el cálculo del
centro de masas, la reconstrucción del palé ni la planificación de colocación.

## Estado actual: medición completa con trayectoria fija

El ciclo mide las tres dimensiones de extremo a extremo:

```text
CALIBRATE_BACKGROUND
-> PRESENT_BOX
-> ATTACH_SUCTION
-> SCAN_YAW_0
-> SCAN_YAW_90
-> RETURNED_VERTICAL
-> ESTIMATE
-> VALIDATE
-> PROFILE_READY / rechazo con motivo
```

La trayectoria es fija: no elige vistas según el resultado. Todas las cajas
recorren las mismas dos poses (`SCAN_YAW_0` y `SCAN_YAW_90`; EXP-008).

La tabla siguiente es EXP-007 (ciclo de tres poses). En esta rama el ciclo es
el de dos yaw; calidad y tiempo en
[EXP-008](docs/findings/EXP-008-ablacion-poses-escaneo.md). EXP-007, 300 escenas
(seeds 1000–1099 y 5000–5199), entorno ideal y sin ruido:

| Eje | MAE | p95 | Máximo |
|---|---:|---:|---:|
| longitud | 0,024 mm | 0,035 mm | 0,037 mm |
| anchura | 0,179 mm | 0,304 mm | 0,371 mm |
| altura | 0,979 mm | 1,796 mm | 2,003 mm |

300 de 300 perfiles válidos, reproducibles por seed, con latencia de percepción
p50 de 29 ms y ciclo p50 de 0,28 s. Detalle y límites en
[EXP-007](docs/findings/EXP-007-benchmark-dimensiones-variables.md).

La succión se abstrae mediante un `equality weld` rígido declarado de MuJoCo. Eso
**no** valida sellado, fugas, cartón poroso, deformación ni deslizamiento.

### Contrato de salida

`ObjectDimensions`, versión de esquema 3, en el marco `ur10e_attachment_site`.
`dimensions_m` es la medida continua; el palé debe usar `dimensions_snapped_m`
(múltiplos de 5 mm). `pose` es opcional.

```json
{
  "schema_version": 3,
  "object_id": "box-0042",
  "timestamp_s": 3.588,
  "frame_id": "ur10e_attachment_site",
  "dimensions_m": {"length": 0.3435, "width": 0.1988, "height": 0.2273},
  "dimensions_snapped_m": {"length": 0.345, "width": 0.200, "height": 0.225},
  "uncertainty_m": {"length": 0.0023, "width": 0.0023, "height": 0.0023},
  "pose": null,
  "views_used": [{"pose_name": "SCAN_YAW_0", "yaw_deg": 0, "tilt_deg": 0}],
  "confidence": 0.92,
  "valid": true,
  "rejection_reason": null
}
```

Convenciones: distancias en metros, ángulos articulares en radianes, nombres de
pose en grados, `length >= width` y la altura sobre el eje Z del terminal.

Motivos de rechazo: `INSUFFICIENT_FOREGROUND`, `FRAME_BORDER_CONTACT`,
`INSUFFICIENT_VIEWS`, `REGISTRATION_INCONSISTENT`, `OUT_OF_RANGE`,
`HIGH_UNCERTAINTY`, `MOTION_TIMEOUT`, `RENDER_FAILURE`, `MISSING_BACKGROUND` e
`INSUFFICIENT_FACE_COVERAGE`.

El estimador no puede consumir la pose ni las dimensiones internas de MuJoCo. Una
prueba estática comprueba que los módulos de solución no nombran ninguna de esas
entradas ni importan módulos de evaluación.

## Ejecución

Medir una seed y emitir el contrato:

```bash
source .venv/bin/activate
object-profiling-profile --seed 42 --output results/profile-seed-42.json
```

Demo de una caja, con mosaico de RGB, profundidad, máscara observable, nube
fusionada y resultado. El ground truth aparece solo en el panel de evaluación:

```bash
object-profiling-demo --headless --seed 42
```

Demo de varias cajas, con una fila por caja comparando medido frente a real y un
panel agregado. Recorre seis cajas de contraste: mínima y máxima del rango, una
muy alargada, una prácticamente cúbica y dos aleatorias:

```bash
object-profiling-showcase --headless --report results/showcase.json
```

En macOS las dos con visor de MuJoCo necesitan `mjpython`. El showcase mide las
seis cajas seguidas en un único visor, porque MuJoCo solo admite uno por proceso:

```bash
mjpython -m object_profiling.presentation.demo --visual --seed 42 --speed 1.0
mjpython -m object_profiling.presentation.showcase --visual --speed 1.0
```

`--speed` controla únicamente la reproducción del visor. No modifica el timestep,
la trayectoria simulada ni las métricas físicas.

Benchmark sobre un rango de seeds:

```bash
object-profiling-benchmark --start 1000 --count 100 \
  --output results/benchmark-1000-1099.json
```

### Checkpoint de movimiento y auditorías

Cada etapa tiene su comando reproducible. Todos necesitan acceso gráfico, aunque
no abran ventana, porque MuJoCo crea un contexto OpenGL para renderizar.

```bash
object-profiling-checkpoint --headless --seed 42 --output results/checkpoint-seed-42.json
object-profiling-camera-audit --artifacts artifacts/camera-audit --output results/camera-audit.json
object-profiling-depth-audit --output results/depth-audit.json
object-profiling-segmentation-audit --artifacts artifacts/segmentation-audit --output results/segmentation-audit.json
object-profiling-registration-audit --artifacts artifacts/registration-audit --output results/registration-audit.json
object-profiling-geometry-audit --output results/geometry-audit.json
```

## Fuera de este incremento

La recogida desde la cinta no está implementada. La cinta y `infeed_rgbd_cam`
están presentes en la escena para esa evolución. Tampoco se cubren el centro de
masas, la reconstrucción del palé ni la planificación de colocación.

## Puesta en marcha

Activar el entorno local e instalar las dependencias de Python:

```bash
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e '.[dev]'
```

Ejecutar las pruebas:

```bash
python -m pytest -p no:cacheprovider
```

## Documentación

- [Reto THEKER](docs/track-theker-hackspain-26.md)
- [Alcance técnico de Object Profiling](docs/object-profiling-scope.md)
- [Comparación de enfoques de medición](docs/measurement-approaches-handoff.md)
- [Registro de experimentos](docs/findings/README.md)
- [EXP-000: agarre, elevación y rotación](docs/findings/EXP-000-agarre-elevacion-rotacion.md)
- [EXP-001: trayectoria fija con inclinación](docs/findings/EXP-001-trayectoria-fija-inclinacion-35.md)
- [EXP-002: cámara RGB-D fija y cobertura](docs/findings/EXP-002-camara-rgbd-cobertura.md)
- [EXP-003: fondo por pose y veracidad de la profundidad](docs/findings/EXP-003-fondo-por-pose-y-profundidad.md)
- [EXP-004: segmentación observable](docs/findings/EXP-004-segmentacion-observable.md)
- [EXP-005: registro multivista](docs/findings/EXP-005-registro-multivista.md)
- [EXP-006: estimación del cuboide](docs/findings/EXP-006-estimacion-cuboide.md)
- [EXP-007: benchmark de dimensiones variables](docs/findings/EXP-007-benchmark-dimensiones-variables.md)
