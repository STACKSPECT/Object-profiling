# EXP-002 — Cámara RGB-D fija y cobertura

## Estado

Implementado, ejecutado y medido en simulación. Aceptado como configuración de
cámara fija: la auditoría se reprodujo y la inspección visual se completó (ver
«Reproducción de cierre»).

## Hipótesis

Una única cámara RGB-D fija, situada en diagonal y con una elevación baja,
puede mantener dentro del encuadre todas las cajas del rango durante las tres
poses fijas, limitando a la vez la oclusión causada por el UR10e y el terminal.

## Entorno

- Base de código: commit `0ddb597` más los cambios no confirmados de EXP-002.
- Simulador: MuJoCo 3.13.0.
- Robot y trayectoria: los aceptados en EXP-001.
- Casos: cajas mínima, nominal y máxima.
- Poses por caja: `SCAN_YAW_0`, `SCAN_YAW_90` y `SCAN_TILT_35`.
- Total: nueve capturas RGB-D.

## Cámara

- Nombre: `scan_rgbd_cam`.
- Posición mundial: `(0,5566, 1,22835, 0,91975) m`.
- Punto fijo de observación: `(-0,174, 0,735, 0,650) m`.
- Resolución: 640 × 480 px.
- FOV vertical: 50°.
- Modalidades utilizadas: RGB y profundidad métrica de MuJoCo.
- Montaje: externo al UR10e y representado mediante una carcasa sin colisión.

La dirección diagonal permite ver simultáneamente dos caras laterales. Se
eligió una elevación baja para que las copas y la muñeca no oculten en exceso la
caja mínima. Acercar la cámara respecto a EXP-001 aumenta su representación de
aproximadamente 2.250 a 5.010 píxeles en el peor caso.

## Método de selección

Se compararon varias posiciones más altas y más cercanas. Para cada candidata
se renderizaron las nueve combinaciones de tamaño y pose y se midieron:

- píxeles visibles de la caja;
- margen mínimo hasta el borde de imagen;
- fracción visible respecto a un render aislado de la misma caja;
- disponibilidad de profundidad válida sobre todos los píxeles visibles.

Los IDs de geometría de MuJoCo se emplean solo en esta auditoría para construir
la máscara ground truth. No forman parte de la observación permitida para el
estimador de dimensiones.

## Criterios de aceptación

- Al menos 4.000 píxeles visibles por captura.
- Margen mínimo de 60 px respecto al borde.
- Al menos 80 % de los píxeles visibles frente a la caja aislada.
- Profundidad finita y positiva para toda la máscara visible.
- Las ocho esquinas ground truth dentro del encuadre con al menos 40 px de
  margen en las pruebas geométricas.

## Protocolo

```bash
source .venv/bin/activate
python -m pytest -p no:cacheprovider
object-profiling-camera-audit \
  --artifacts artifacts/camera-audit \
  --output results/camera-audit.json
```

## Resultados

Ejecución del 19 de septiembre de 2026:

| Métrica | Resultado |
|---|---:|
| Pruebas automáticas totales | 17/17 superadas |
| Capturas válidas | 9/9 |
| Píxeles visibles, peor caso | 5.010 |
| Fracción visible, peor caso | 86,83 % |
| Margen al borde, peor caso | 76 px |
| Píxeles visibles, mejor caso | 48.223 |
| Rango global de profundidad observado | 0,636–1,138 m |
| Cajas evaluadas | mínima, nominal y máxima |
| Poses evaluadas por caja | 3 |

La inspección visual de la caja nominal confirma que las poses verticales
muestran caras laterales complementarias y que `SCAN_TILT_35` aporta una vista
claramente distinta sin sacar la caja del encuadre.

## Reproducción de cierre

Segunda ejecución tras trasladar la definición de las tres poses de
`camera_audit.py` a `poses.py`, de modo que la auditoría y el futuro
orquestador comparten la misma secuencia fija:

| Métrica | Resultado |
|---|---:|
| Pruebas automáticas totales | 26/26 superadas |
| `valid` global del informe | `true` |
| Capturas válidas | 9/9 |
| Píxeles visibles, peor caso | 5.010 |
| Fracción visible, peor caso | 86,83 % |
| Margen al borde, peor caso | 76 px |
| Profundidad finita y positiva | 9/9 mapas, cero píxeles no positivos |
| Rango global de profundidad observado | 0,636–1,138 m |

La inspección visual de los nueve RGB no muestra truncamiento. En
`SCAN_YAW_0` son visibles la cara superior y dos caras laterales, porque el
bastidor de copas es abierto y no tapa la cara superior completa. Esto es
favorable para la observabilidad de la altura, y debe confirmarse por eje en el
experimento de estimación, no asumirse aquí.

## Límites y decisión

EXP-002 demuestra encuadre, profundidad disponible y cobertura aparente en una
escena ideal. No demuestra todavía que la segmentación observable separe la
caja del terminal ni que la nube obtenida permita medir con el error objetivo.
El backend OpenGL disponible en este Mac avisó de precisión de profundidad
limitada al no disponer de `ARB_clip_control`; su impacto métrico deberá medirse
antes de aceptar tolerancias milimétricas.

Se conserva esta cámara como configuración fija. El siguiente experimento debe
calibrar fondos por pose y segmentar la caja usando únicamente RGB-D, sin IDs de
MuJoCo, antes de ajustar un cuboide o calcular dimensiones.
