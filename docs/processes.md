# Procesos y contratos

Guía de emparejado con el resto del sistema. Este paquete cubre medición e
inspección. El picking, el brazo, el palé y el CoM son de los compañeros.

Convenciones: metros, `length >= width`, altura sobre Z del terminal. Marco:
`ur10e_attachment_site`.

## Dónde encaja

```text
1.   Recoger la caja                         → compañeros
2.1  Medir L/W/H con visión                  → ESTE PAQUETE
2.2  Detectar caja rota                      → ESTE PAQUETE
2.3  Soltar en zona de error si DAMAGED      → decisión aquí; motion compañeros
2.4  Dejar la caja sujeta si INTACT          → ESTE PAQUETE (handoff)
3.   Centro de masas moviendo la caja        → siguiente módulo (caja ya sujeta)
4–7. Palé, scoring, colocación, calidad      → compañeros
```

## Procesos

### P0 — Fondos

| | |
|---|---|
| **Entrada** | Estación **sin caja**. Una profundidad por `pose_name`. |
| **Salida** | `BackgroundSet` (`pose_name` → `depth_m` HxW en metros). |
| **Dueño** | El integrador captura con su cámara. Este repo solo define el tipo. |

### P2 — Escaneo

Robot detenido. Yaw 0° y 90° miden; yaw 180° solo inspecciona.

| | |
|---|---|
| **Entrada** | Caja sujeta, fondos de P0, RGB-D calibrada, `camera_to_world` y `tool_to_world` 4×4. |
| **Salida** | `observations` (medida) y `inspection_observations` (yaw 180). |
| **Dueño** | El integrador mueve su UR10e y rellena `CameraObservation`. |

`CameraObservation`: `timestamp_s`, `pose_name`, `target_yaw_deg`,
`target_tilt_deg`, `rgb` (H,W,3), `depth_m` (H,W), `intrinsics`,
`camera_to_world`, `tool_to_world`.

### P3 — Medir L/W/H

```text
measure(observations, backgrounds, config, object_id=..., inspection_observations=...)
    -> MeasurementResult
```

| | |
|---|---|
| **Entrada** | Vistas de medida (≥2), fondos, `object_id`. Prohibido leer geometría interna del simulador. |
| **Salida** | `ObjectDimensions`. Si `valid=True`: `dimensions_m`, `dimensions_snapped_m`, `uncertainty_m`, `pose`. |
| **Fallos** | `INSUFFICIENT_FOREGROUND`, `FRAME_BORDER_CONTACT`, `INSUFFICIENT_VIEWS`, `REGISTRATION_INCONSISTENT`, `OUT_OF_RANGE`, `HIGH_UNCERTAINTY`, `MISSING_BACKGROUND`, `INSUFFICIENT_FACE_COVERAGE`. El integrador puede inyectar `MOTION_TIMEOUT` / `RENDER_FAILURE` con `rejected_measurement`. |

El palé usa `dimensions_snapped_m` (múltiplos de 5 mm). `valid` no significa
apilable.

### P4 — Inspeccionar daño

Lo llama `measure()`. Señales: planaridad, rectitud, soporte de esquinas
anti-agarre, residuo hacia dentro. Umbral `max(5% del lado corto, 3 mm)`.

| | |
|---|---|
| **Salida** | `condition`: `INTACT` / `DAMAGED` / `UNKNOWN`. `routing`: `NORMAL` / `ERROR_ZONE`. `damage`: `DamageReport` o `null`. |

### P5 — Descarte

Si `routing == ERROR_ZONE`, el orquestador suelta la caja. Este paquete no
mueve el brazo.

### P6 — Handoff al CoM

Si `valid`, `INTACT` y `NORMAL`, la caja **sigue sujeta**.

```text
HeldBoxHandoff(dimensions, held=True, grasp_face="-z", frame_id="ur10e_attachment_site")
ready_for_com() -> bool
```

El módulo de CoM no reagarra. Si `held=False`, no arranca.

## `ObjectDimensions` (schema 4)

```text
schema_version, object_id, timestamp_s, frame_id
dimensions_m / dimensions_snapped_m / uncertainty_m
pose, views_used, confidence
valid, rejection_reason
condition, routing, damage
```

| Ellos necesitan | Campo |
|---|---|
| Tamaño para el palé | `dimensions_snapped_m` |
| ¿Medida usable? | `valid`, `uncertainty_m`, `rejection_reason` |
| ¿Contenedor o CoM? | `routing` |
| Tipo de defecto | `damage` |
| Prior geométrico | `pose` + `HeldBoxHandoff.held` |
| RGB-D sin MuJoCo | `CameraObservation` + `BackgroundSet` |

## Funciones públicas

| Función | In | Out |
|---|---|---|
| `measure` | observaciones, fondos, config, `object_id` | `MeasurementResult` |
| `rejected_measurement` | id, timestamp, motivo | resultado inválido sin nube |
| `segment_foreground` | profundidad, fondo, `SensorConfig` | máscara o rechazo |
| `observation_to_scan_view` | captura + fondo | `ScanView` o rechazo |
| `fuse_scan_views` | `ScanView` en TCP | `FusedCloud` |
| `estimate_cuboid` | nube + config | cuboide o rechazo |
| `inspect_cloud` / `assess_damage` | nube + cuboide | métricas / `condition`+`routing` |
| `snap_to_catalogue` | L/W/H + paso 0.005 | aristas de catálogo |
| `HeldBoxHandoff.ready_for_com` | — | bool |
