# EXP-003 — Fondo por pose y veracidad de la profundidad

## Estado

Implementado, ejecutado y medido en simulación.

## Hipótesis

Dos cuestiones se evalúan juntas porque la segunda condiciona cualquier
afirmación de precisión que dependa de la primera:

1. Un fondo RGB-D independiente por pose permite aislar la caja sin usar IDs de
   geometría de MuJoCo. Un único fondo no puede servir para las tres poses
   porque el brazo y el terminal cambian de sitio entre ellas.
2. El aviso `ARB_clip_control unavailable while mjDEPTH_ZEROFAR requested` del
   backend OpenGL de este Mac podría impedir medir con tolerancia milimétrica.

## Entorno

- Base de código: commit `6710c97` más los cambios de EXP-003.
- Simulador: MuJoCo 3.13.0, backend OpenGL de macOS sin `ARB_clip_control`.
- Robot, trayectoria y cámara: los aceptados en EXP-001 y EXP-002.
- Casos: cajas mínima, nominal y máxima.
- Poses por caja: `SCAN_YAW_0`, `SCAN_YAW_90` y `SCAN_TILT_35`.
- Plano cercano del render: 0,0298 m. Plano lejano: 14,877 m.

## Sistema de medición

- Fondos: `background.py`, con `BackgroundSet` indexado por `pose_name`.
- Retirada de la caja: `ProfilingEnvironment.set_box_visible(False)`.
- Auditoría de profundidad: `depth_audit.py`, con `mj_ray` como referencia
  analítica.
- Modalidades: profundidad métrica; el RGB no participa en esta medición.

## Método

### Fondo por pose

La calibración se ejecuta como **pasada previa**, antes de activar la succión:
se recorren las tres poses con la estación vacía, se registra la profundidad y
la configuración articular, y solo después se agarra la caja y se ejecuta el
ciclo de medición. La alternativa descrita en el handoff, ocultar y restituir la
caja dentro del ciclo, obligaría a reconstruir el weld de succión tres veces a
mitad de ciclo y rompería la continuidad de la auditoría de deriva aceptada en
EXP-001.

`capture_pose_backgrounds()` acepta además configuraciones articulares medidas.
Con ellas el brazo se coloca exactamente en la configuración observada en lugar
de volver a estabilizarse, lo que elimina la diferencia de carga entre la
estación vacía y la caja suspendida. Esta vía queda disponible y se usará si
EXP-004 mide demasiados píxeles falsos sobre el terminal.

### Veracidad de la profundidad

Para cada pose se compara la profundidad renderizada contra la intersección
geométrica exacta obtenida con `mj_ray`, restringida a la máscara ground truth
de la caja. **En esta auditoría** la máscara GT se erosiona 2 px para separar
dos poblaciones muy distintas: el interior de las caras y el anillo de silueta.
Esa erosión no forma parte del estimador.

`mj_ray` y la máscara ground truth son herramientas de auditoría. No forman
parte del camino de la solución.

## Protocolo

```bash
source .venv/bin/activate
python -m pytest -p no:cacheprovider
object-profiling-depth-audit --output results/depth-audit.json
```

## Resultados

### Los tres fondos son necesariamente distintos

| Comparación | Píxeles que difieren más de 0,1 mm |
|---|---:|
| `SCAN_YAW_0` contra `SCAN_YAW_90` | 18.198 |
| `SCAN_YAW_0` contra `SCAN_TILT_35` | 42.450 |

Reutilizar un solo fondo introduciría decenas de miles de píxeles de primer
plano espurio. La asociación por `pose_name` queda justificada.

### La profundidad del render no es el factor limitante

Peor caso sobre las nueve combinaciones de caja y pose:

| Métrica | Resultado |
|---|---:|
| Error p95 en el interior de la caja | 0,006 mm |
| Sesgo medio en el interior | 0,001 mm |
| Error máximo en el interior | 0,032 mm |
| Salto de cuantización observado | 0,070 mm |
| Error máximo en el anillo de silueta | 97,7 mm |

El interior de las caras se rinde con error de micras y sin sesgo apreciable.
La cuantización efectiva, 0,07 mm, queda dos órdenes de magnitud por debajo del
objetivo de 5 mm de MAE. **El aviso de `ARB_clip_control` no impide la medición
milimétrica en esta escena**, y por tanto el objetivo de precisión de la Fase 8
no necesita reinterpretarse por este motivo.

### El riesgo real son los píxeles de silueta

Un único píxel del borde puede errar 97,7 mm, porque el rasterizado y el rayo
que pasa por el centro del píxel caen en superficies distintas: la copa a
0,88 m o el suelo a 5,6 m. Es un error de tres órdenes de magnitud sobre el
interior.

Una primera medición ingenua daba 0,55–1,26 mm de sesgo y hasta 7 mm de p95.
Aquella cifra estaba dominada por el suelo ajedrezado visto en ángulo rasante,
no por las caras de la caja, y no describía lo que el estimador va a medir.

## Observaciones y fallos

- `set_box_visible(False)` aparcaba la caja en `z = -3 m`, bajo el plano del
  suelo. La penetración la expulsaba hasta `z ≈ 12,9 m`, todavía dentro del
  plano lejano de 14,877 m, de modo que podía reaparecer en un fondo. Ahora se
  aparca detrás de la cámara y con los contactos desactivados.
- `body_gravcomp` no actúa sobre el freejoint de la caja: `qfrc_gravcomp`
  permanece a cero y la caja aparcada cae libremente. Se reaparca antes de cada
  captura en lugar de intentar congelarla.
- Colocar el brazo en una configuración articular medida reproduce el fondo con
  un residuo por debajo de 0,1 mm en el percentil 99, pero no exactamente: el
  terminal cuelga del weld y se asienta unas centésimas de milímetro por debajo
  de su posición cinemática. Ese desplazamiento conmuta píxeles de silueta de
  las copas. Confirma que la segmentación necesita apertura morfológica y
  componente conexo principal, no solo un umbral.

## Decisión

Se conservan el fondo por pose mediante pasada previa y la auditoría de
profundidad como referencia reproducible.

La silueta tiene error centimétrico en profundidad; el interior, de micras. Eso
no implica erosionar la máscara del estimador antes de retroproyectar: esos
píxeles de borde son los únicos que marcan extremos no vistos de frente.
EXP-006 midió que erosionar 2 px costaba ~5 mm de anchura y dejó
`mask_erosion_px = 0`. El siguiente experimento (entonces) debía segmentar la
caja usando solo RGB-D y medir IoU, precisión, recall y píxeles falsos del
terminal contra la máscara ground truth.
