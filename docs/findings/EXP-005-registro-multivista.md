# EXP-005 — Registro de las tres vistas en el marco del terminal

## Estado

Implementado, ejecutado y medido en simulación.

## Hipótesis

Como la caja permanece rígida respecto al terminal, retroproyectar cada vista y
llevarla al marco del terminal con la cinemática del UR10e debe hacer que las
tres nubes coincidan, sin usar la pose interna de la caja.

## Entorno

- Base de código: commit `bf6e29d` más los cambios de EXP-005.
- Simulador: MuJoCo 3.13.0.
- Casos: cajas mínima (0,5 kg), nominal (2 kg) y máxima (5 kg).
- Poses: `SCAN_YAW_0`, `SCAN_YAW_90` y `SCAN_TILT_35`.

## Sistema de medición

- Marco común declarado: `ur10e_attachment_site`, el sitio de acople de la
  muñeca que expone `ProfilingEnvironment.tool_to_world()`. Es el valor de
  `frame_id` en la salida.
- Cadena: profundidad → retroproyección con intrínsecos → cámara a mundo →
  mundo al marco actual del terminal → recorte → fusión.
- Registro: `registration.py`. Métricas contra ground truth: `evaluation.py`.
- Auditoría: `registration_audit.py`.

## Método

La solución no conoce la pose de la caja. El registro se apoya solo en:

- los intrínsecos derivados del FOV;
- la pose de la cámara, fija y calibrada;
- el estado articular del UR10e, a través de la cinemática directa hasta el
  sitio de acople.

Para rechazar sin ground truth se define el **residuo de plano por vista**: se
ajusta un cuboide alineado a ejes sobre la nube fusionada y se mide la distancia
mediana de los puntos de cada vista a sus caras. Una vista mal registrada deja
de apoyarse en las caras comunes y su residuo crece.

La evaluación mide la distancia de cada punto a la superficie real de la caja, y
separa deliberadamente dos cosas:

- **por vista**, usando la pose real correspondiente a esa captura: mide la
  cadena de retroproyección;
- **fusionada**, usando la pose real de la primera captura para todos los
  puntos: mide además cuánto se ha movido la caja respecto al terminal entre
  poses, que es precisamente el error de registro entre vistas.

## Protocolo

```bash
source .venv/bin/activate
python -m pytest -p no:cacheprovider
object-profiling-registration-audit \
  --artifacts artifacts/registration-audit \
  --output results/registration-audit.json
```

Criterios: p95 de distancia a la superficie de la nube fusionada ≤ 3 mm y
residuo de plano por vista ≤ 4 mm.

## Resultados

### Hallazgo principal: la rigidez del montaje del terminal dominaba el error

La primera ejecución separó las dos contribuciones de forma inequívoca:

| Caja | p95 por vista | p95 fusionada |
|---|---:|---:|
| mínima | 0,002 mm | 0,774 mm |
| nominal | 0,002 mm | 2,801 mm |
| máxima | 0,002 mm | 9,647 mm |

La cadena cámara-mundo-terminal era exacta a nivel de micras. Todo el error
aparecía al fusionar, y crecía con la masa. La causa medida:

| Caja | Deriva caja→terminal en `SCAN_TILT_35` | De la cual, muñeca→terminal |
|---|---:|---:|
| nominal | 2,477 mm / 0,726° | 0,730° |
| máxima | 8,556 mm / 2,113° | 2,121° |

La unión **succión → caja** apenas cedía 0,009°. La que flexaba era
`wrist_to_gripper`, es decir el terminal atornillado a la brida de la muñeca. Un
montaje atornillado que gira 2,1° no es físicamente creíble; era un artefacto de
`solref="0.01 1"`.

Se endurecieron ambas uniones a `solref="0.004 1"`, el doble del timestep, con
`solimp="0.99 0.9999 0.001 0.5 2"`.

### Resultados tras endurecer las uniones

| Caja | Deriva en `SCAN_TILT_35` | Mejora |
|---|---:|---:|
| mínima | 0,042 mm / 0,014° | — |
| nominal | 0,097 mm / 0,029° | 26× |
| máxima | 0,222 mm / 0,054° | 39× |

| Métrica | Peor caso de tres |
|---|---:|
| p95 de distancia a superficie, por vista | 0,002 mm |
| p95 de distancia a superficie, fusionada | 0,264 mm |
| Media de distancia a superficie, fusionada | 0,043 mm |
| Residuo de plano por vista | 0,031 mm |
| `valid` global | `true` |

Detalle de la nube fusionada:

| Caja | Puntos | Media | p95 | Máximo |
|---|---:|---:|---:|---:|
| mínima | 13.445 | 0,012 mm | 0,047 mm | 0,060 mm |
| nominal | 53.281 | 0,022 mm | 0,112 mm | 0,149 mm |
| máxima | 131.474 | 0,043 mm | 0,264 mm | 0,354 mm |

### Efecto colateral en el checkpoint de movimiento

El mismo cambio mejoró EXP-001 sin tocar su lógica: la deriva de traslación pasa
del orden del milímetro a 0,0059 mm y la de rotación a 0,0001°. El checkpoint
sigue en `success: true`.

### Auditorías reejecutadas tras el cambio de escena

| Auditoría | Resultado |
|---|---|
| Cámara | `valid: true`, 9/9, peor caso 4.998 px y fracción visible 86,62 % |
| Profundidad | p95 interior 0,008 mm, cuantización 0,076 mm |
| Segmentación | `valid: true`, IoU mínimo 0,9953, 150 falsos del terminal, 0 falsos en otro sitio |
| Movimiento | `success: true`, elevación 0,1149 m, cero contactos inesperados |

Las cifras de cámara y segmentación se mueven en la tercera cifra decimal porque
la pose asentada cambia ligeramente. Los criterios de aceptación se mantienen.

## Barrera de información privilegiada

Se añade una comprobación estática: los módulos de solución no pueden contener
`box_geom`, `profiling_box`, `box_spec`, `body_to_world`, `gripper_to_box`,
`mjOBJ_GEOM`, `enable_segmentation_rendering` ni `mj_ray`, y no pueden importar
ningún módulo de evaluación o auditoría.

La comprobación incluye dos garantías adicionales para no ser decorativa:

- los módulos de evaluación deben disparar el mismo criterio, de modo que la
  prueba fallaría si el criterio dejara de detectar nada;
- todo módulo nuevo del paquete debe quedar clasificado como solución o como
  evaluación, así que no se puede añadir uno y saltarse la comprobación.

## Observaciones y fallos

- La discrepancia de extensión entre vistas llega a 7,34 mm en la caja máxima.
  **No es error de registro**, porque el registro quedó en 0,264 mm: es falta de
  cobertura. Cada vista observa extremos distintos de cada eje, así que por sí
  sola mide de menos. Es exactamente el riesgo que el plan señala para la
  estimación y debe tratarse allí como cobertura, no como inconsistencia.
- La distancia a superficie de la nube fusionada sigue siendo un límite
  inferior del error dimensional: mide si los puntos están sobre la caja, no si
  cubren sus extremos.
- `np.ndarray.ptp()` desapareció en NumPy 2; se usa `np.ptp()`.

## Decisión

Se conservan el marco `ur10e_attachment_site`, la fusión por concatenación en
ese marco y las uniones endurecidas. El siguiente experimento debe estimar el
cuboide y, antes de confiar en percentiles, verificar por eje que ambos extremos
han sido observados.
