# Object Profiling

Repositorio de exploración para medir cajas rectangulares mientras un UR10e las
mantiene suspendidas y las presenta desde varias orientaciones ante una cámara
RGB-D fija.

Este trabajo está aislado del repositorio de integración `hackspain/Simulation`.
Su salida principal serán las tres dimensiones de la caja y su incertidumbre,
consumibles por el resto del sistema. Este repositorio sí incluye el agarre y
los movimientos del UR10e necesarios para medir; no incluye el cálculo del
centro de masas, la reconstrucción del palé ni la planificación de colocación.

## Estado actual: medición y condición estructural

El ciclo mide las tres dimensiones y decide si la caja sigue siendo un
cuboide apilable. Si no lo es, el brazo la lleva al contenedor de rechazo:

```text
CALIBRATE_BACKGROUND
-> BOX_k_OF_n
-> PRESENT_BOX
-> ATTACH_SUCTION
-> SCAN_YAW_0
-> SCAN_YAW_90
-> SCAN_YAW_180
-> ESTIMATE
-> VALIDATE
-> MOVE_TO_ERROR_ZONE / RELEASE  (solo si routing = ERROR_ZONE)
-> PROFILE_READY / rechazo con motivo
-> CLEAR_BOX                     (si quedan cajas)
```

`--count n` repite el ciclo n veces. Cada caja se genera, se mide, se acepta o
se descarta, y desaparece antes de que aparezca la siguiente. Los fondos se
calibran una sola vez. La trayectoria es fija: no elige vistas según el
resultado. La medida usa `SCAN_YAW_0` y `SCAN_YAW_90` (EXP-008). `SCAN_YAW_180`
es el segundo giro en el mismo sentido: se fusiona **solo** para inspección, de
modo que las cinco caras no agarradas (fondo + cuatro laterales) entren en el
detector (EXP-015). Cámara RGB-D fija por debajo de la caja (EXP-009).

La tabla de 300 seeds siguiente es EXP-008 (cámara **alta**). Con la cámara
baja de esta rama, 20 seeds (1000–1019): MAE L/W/H 0,016 / 0,195 / 0,102 mm,
20/20 válidos y snap; detalle en
[EXP-009](docs/findings/EXP-009-camara-baja-yaw-180.md). Aún no se ha repetido
el protocolo de 300 seeds.

Medido sobre 300 escenas (seeds 1000–1099 y 5000–5199), ciclo de dos yaws y
cámara alta, entorno ideal y sin ruido:

| Eje | MAE | p95 | Máximo |
|---|---:|---:|---:|
| longitud | 0,016 mm | 0,021 mm | 0,023 mm |
| anchura | 0,242 mm | 0,380 mm | 0,500 mm |
| altura | 1,066 mm | 1,795 mm | 1,901 mm |

300 de 300 perfiles válidos, snap a 5 mm en las 300, reproducibles por seed,
latencia de percepción p50 de 25 ms y ciclo p50 de 0,23 s. Comparación con el
ciclo de tres poses en
[EXP-007](docs/findings/EXP-007-benchmark-dimensiones-variables.md) y
[EXP-008](docs/findings/EXP-008-ablacion-poses-escaneo.md).
Tras sustituir la primitiva por malla, 30 seeds (1000–1029) dan MAE
0,024 / 0,154 / 0,985 mm
([EXP-010](docs/findings/EXP-010-malla-y-generacion-de-dano.md)).

El 10 % de las cajas se generan dañadas. El inspector geométrico no usa el
ground truth de escena. `valid` habla de la calidad de la medida, no de si
la caja es apilable. Limitaciones y tasas en
[EXP-013](docs/findings/EXP-013-reconocimiento-de-dano.md) y
[EXP-014](docs/findings/EXP-014-descarte-y-prevalencia.md).

La succión se abstrae mediante un `equality weld` rígido declarado de MuJoCo. Eso
**no** valida sellado, fugas, cartón poroso, deformación ni deslizamiento.

### Contrato de salida

`ObjectDimensions`, versión de esquema 4, en el marco `ur10e_attachment_site`.
`dimensions_m` es la medida continua; el palé debe usar `dimensions_snapped_m`
(múltiplos de 5 mm). `pose` es opcional. `condition` / `routing` / `damage`
describen si la caja es un cuboide apilable; no sustituyen a `valid`.

```json
{
  "schema_version": 4,
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
  "rejection_reason": null,
  "condition": "DAMAGED",
  "routing": "ERROR_ZONE",
  "damage": {
    "kind": "CRUSHED_CORNER",
    "severity_m": 0.031,
    "location": "corner:+x+y-z",
    "evidence": {
      "face_planarity_p95_m": 0.0008,
      "edge_straightness_p95_m": 0.0219,
      "weakest_corner_support": 3
    }
  }
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

Medir n cajas seguidas en la misma estación. La seed es la de la primera caja;
las siguientes usan `seed+i`. Con `--count` mayor que 1 el JSON es una lista:

```bash
object-profiling-profile --seed 42 --count 8 --output results/profile-42-49.json
```

Demo de una caja, con mosaico de RGB, profundidad, máscara observable, nube
fusionada y resultado. El ground truth aparece solo en el panel de evaluación:

```bash
object-profiling-demo --headless --seed 42
```

El mismo visor recorre n cajas: mide, acepta o descarta, retira la caja y genera
la siguiente. `--count` vale 1 por defecto:

```bash
object-profiling-demo --headless --seed 42 --count 8
```

Demo de varias cajas, con una fila por caja comparando medido frente a real y un
panel agregado. Recorre seis cajas aleatorias del rango, en múltiplos de 5 mm.
Sin `--seed` elige una seed base al azar y la imprime para poder reproducir:

```bash
object-profiling-showcase --headless --report results/showcase.json
object-profiling-showcase --headless --seed 42 --report results/showcase.json
```

En macOS las dos con visor de MuJoCo necesitan `mjpython`. El showcase mide las
seis cajas seguidas en un único visor, porque MuJoCo solo admite uno por proceso:

```bash
mjpython -m object_profiling.presentation.demo --visual --seed 42 --count 8 --speed 1.0
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
object-profiling-damage-audit --artifacts artifacts/damage-audit --output results/damage-audit.json
object-profiling-inspection-audit --start 8000 --count 20 --damage-rate 0.5 --output results/inspection-audit.json
object-profiling-discard-audit --start 8100 --count 8 --output results/discard-audit.json
object-profiling-checkpoint --start 200 --count 16 --damage-rate 1.0 --output results/checkpoint-damaged.json
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
- [Detección de cajas dañadas: ground truth y plan](docs/damaged-box-detection-plan.md)
- [Registro de experimentos](docs/findings/README.md)
- [EXP-000: agarre, elevación y rotación](docs/findings/EXP-000-agarre-elevacion-rotacion.md)
- [EXP-001: trayectoria fija con inclinación](docs/findings/EXP-001-trayectoria-fija-inclinacion-35.md)
- [EXP-002: cámara RGB-D fija y cobertura](docs/findings/EXP-002-camara-rgbd-cobertura.md)
- [EXP-003: fondo por pose y veracidad de la profundidad](docs/findings/EXP-003-fondo-por-pose-y-profundidad.md)
- [EXP-004: segmentación observable](docs/findings/EXP-004-segmentacion-observable.md)
- [EXP-005: registro multivista](docs/findings/EXP-005-registro-multivista.md)
- [EXP-006: estimación del cuboide](docs/findings/EXP-006-estimacion-cuboide.md)
- [EXP-007: benchmark de dimensiones variables](docs/findings/EXP-007-benchmark-dimensiones-variables.md)
- [EXP-008: ablación de poses de escaneo](docs/findings/EXP-008-ablacion-poses-escaneo.md)
- [EXP-009: cámara baja para medir con dos vistas; yaw 180 solo defectos](docs/findings/EXP-009-camara-baja-yaw-180.md)
- [EXP-010: malla única y generación de daño](docs/findings/EXP-010-malla-y-generacion-de-dano.md)
- [EXP-011: agarre de cajas dañadas](docs/findings/EXP-011-agarre-de-cajas-danadas.md)
- [EXP-012: señales de inspección](docs/findings/EXP-012-senales-de-inspeccion.md)
- [EXP-013: reconocimiento de daño](docs/findings/EXP-013-reconocimiento-de-dano.md)
- [EXP-014: descarte y prevalencia 10 %](docs/findings/EXP-014-descarte-y-prevalencia.md)
- [EXP-015: inspección de cinco caras y envolvente con daño](docs/findings/EXP-015-inspeccion-cinco-caras.md)
