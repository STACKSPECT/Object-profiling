# EXP-004 — Segmentación observable de la caja suspendida

## Estado

Implementado, ejecutado y medido en simulación.

## Hipótesis

Restar el fondo calibrado de la misma pose basta para aislar la caja suspendida
sin usar el ID de `box_geom` ni ninguna otra propiedad interna de MuJoCo, y el
resultado se acerca lo suficiente a la máscara ground truth para sostener una
medición dimensional.

## Entorno

- Base de código: commit `c00f996` más los cambios de EXP-004.
- Simulador: MuJoCo 3.13.0.
- Robot, trayectoria, cámara y fondos: los aceptados en EXP-001, EXP-002 y
  EXP-003.
- Casos: cajas mínima, nominal y máxima por tres poses; nueve combinaciones.

## Sistema de medición

- Segmentación: `perception.py`.
- Ciclo compartido de calibración, agarre, escaneo y retorno: `scanning.py`.
- Métricas contra ground truth: `evaluation.py`.
- Auditoría ejecutable: `segmentation_audit.py`.

`evaluation.py` es el único módulo que abre IDs de geometría, y ningún módulo de
la solución lo importa.

## Método

```text
profundidad de la pose
− fondo calibrado de la misma pose
→ umbral con margen de 4 mm
→ apertura y cierre morfológico 5×5
→ componente conexo mayor
→ validación de área y de contacto con el borde
→ erosión de 2 px
→ retroproyección y recorte en el marco del terminal
```

Dos decisiones cambian respecto al prototipo heredado:

1. **Recorte en el marco del terminal.** El prototipo recortaba en coordenadas
   de mundo alrededor de un centro fijo con media extensión de 0,34 m, lo que
   depende de que la caja caiga siempre en el mismo punto del mundo. Ahora el
   volumen se deriva del rango de cajas declarado: el mismo límite horizontal
   para los dos ejes, de modo que no se presupone cuál lleva la longitud.
2. **Máscara interior separada (en este experimento).** EXP-003 midió que un
   píxel de silueta puede errar 97,7 mm frente a 0,03 mm en el interior. Aquí se
   conservó la máscara completa para diagnóstico y se retroproyectó la
   erosionada. EXP-006 revirtió esa decisión: el estimador actual retroproyecta
   la máscara completa (`mask_erosion_px = 0`).

El umbral de área se revisó: el prototipo usaba 500 px y la auditoría de cámara
4.000 px. Son criterios distintos. 4.000 px es un criterio de colocación de
cámara, mientras que aquí solo hace falta descartar restos que no puedan ser la
caja. Queda en 2.000 px, con margen frente a los 5.010 px del peor caso medido
en EXP-002.

## Protocolo

```bash
source .venv/bin/activate
python -m pytest -p no:cacheprovider
object-profiling-segmentation-audit \
  --artifacts artifacts/segmentation-audit \
  --output results/segmentation-audit.json
```

Criterios de aceptación: IoU ≥ 0,95 y como máximo 200 píxeles falsos sobre el
terminal en cada una de las nueve capturas.

## Resultados

| Métrica | Peor caso de nueve |
|---|---:|
| IoU | 0,9950 |
| Precisión | 0,9964 |
| Recall | 0,9966 |
| Píxeles falsos sobre el terminal | 163 |
| Píxeles falsos en cualquier otro sitio | 0 |
| Píxeles de caja perdidos | 31 |
| `valid` global | `true` |

Detalle por caso:

| Caja | Pose | IoU | Falsos del terminal | Perdidos |
|---|---|---:|---:|---:|
| mínima | `SCAN_YAW_0` | 0,9964 | 7 | 11 |
| mínima | `SCAN_YAW_90` | 0,9950 | 12 | 14 |
| mínima | `SCAN_TILT_35` | 0,9966 | 0 | 19 |
| nominal | `SCAN_YAW_0` | 0,9962 | 58 | 14 |
| nominal | `SCAN_YAW_90` | 0,9954 | 69 | 21 |
| nominal | `SCAN_TILT_35` | 0,9989 | 5 | 16 |
| máxima | `SCAN_YAW_0` | 0,9961 | 146 | 26 |
| máxima | `SCAN_YAW_90` | 0,9957 | 163 | 31 |
| máxima | `SCAN_TILT_35` | 0,9998 | 2 | 10 |

**Cero píxeles falsos fuera del terminal en las nueve capturas.** El brazo, el
apoyo, la cinta y el suelo desaparecen limpiamente con la resta de fondo; no
hace falta ningún criterio adicional para excluirlos.

Todos los falsos positivos se concentran en el anillo de silueta donde las cinco
copas se apoyan sobre la cara superior. Las imágenes de desacuerdo muestran una
línea de un píxel en la muesca de las copas, exactamente el efecto que EXP-003
predijo a partir del asentamiento submilimétrico del terminal. Las poses
verticales lo sufren más que `SCAN_TILT_35`, donde la cara superior queda casi
fuera de la vista.

### Casos límite cubiertos por pruebas

- Máscara vacía: `INSUFFICIENT_FOREGROUND`.
- Diferencia de profundidad por debajo del margen de 4 mm: sin primer plano.
- Componente menor que el umbral de área: `INSUFFICIENT_FOREGROUND`.
- Componente tocando el borde del encuadre: `FRAME_BORDER_CONTACT`.
- Dos componentes: gana el mayor; `interior_mask` no sale de `mask`.
- Fondo de otra pose: el IoU cae más de 0,05 respecto al fondo correcto, así que
  una calibración mal asociada no pasa desapercibida.
- Repetición: máscara idéntica bit a bit.

## Observaciones y fallos

- Falta por medir la consecuencia geométrica de los 163 píxeles falsos y de los
  31 perdidos. Un IoU de 0,995 no implica error dimensional pequeño: los píxeles
  afectados están precisamente en el borde superior, que es donde se lee la
  altura.
- En este experimento el anillo de silueta se retiró por erosión fija de 2 px.
  En la caja mínima esa erosión se llevaba un 14 % de los píxeles, frente a un
  5 % en la máxima. EXP-006 midió el coste dimensional y dejó la erosión en 0.

## Decisión

Se conserva la segmentación por resta de fondo por pose. La erosión de 2 px
quedó como decisión de este experimento y EXP-006 la revirtió para el
estimador. El siguiente experimento debía retroproyectar, registrar las tres
vistas en el marco del terminal y medir el error de registro.
