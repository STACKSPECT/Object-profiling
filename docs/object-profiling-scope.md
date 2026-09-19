# Alcance técnico: medición suspendida de cajas

## Problema

Una cinta transportadora entrega, una a una, cajas rígidas y rectangulares de
dimensiones desconocidas. La posición y orientación de llegada podrán variar.
El sistema debe recoger cada caja, mantenerla suspendida, presentarla desde
varias orientaciones ante una cámara y estimar:

- longitud;
- anchura;
- altura;
- incertidumbre de cada dimensión;
- validez de la medición y causa de rechazo;
- condición estructural (`INTACT` / `DAMAGED` / `UNKNOWN`) y pista de
  enrutado (`NORMAL` / `ERROR_ZONE`).

Las tres dimensiones deben obtenerse de observaciones sensóricas. La geometría,
pose y dimensiones internas de MuJoCo solo pueden utilizarse como ground truth
durante la evaluación.

El repositorio cubre el agarre y los movimientos necesarios para medir. No
cubre la estimación del centro de masas, la reconstrucción del palé ni la
elección o ejecución de una colocación sobre él; esas responsabilidades
pertenecen a otros módulos de `hackspain/Simulation`.

## Hipótesis iniciales

- Cada objeto es un cuboide rígido, opaco y con caras aproximadamente planas.
- Solo hay una caja en el ciclo de medición.
- La ventosa se acopla a una cara superior apta para vacío.
- La caja permanece rígida respecto al terminal durante cada escaneo.
- Las capturas se realizan con el UR10e detenido y la caja estabilizada.
- Las caras ocultas se infieren mediante la hipótesis de cuboide; no se afirma
  que se haya observado un mesh completo.

## Sistema físico propuesto

### Brazo

Se utilizará un **Universal Robots UR10e**. Sus seis grados de libertad son
suficientes para situar la caja en el volumen de escaneo y aplicar las
rotaciones discretas necesarias. La zona de escaneo se colocará en una región
despejada y alejada de límites articulares y singularidades.

El UR10e forma parte del experimento. Su estado articular y cinemática directa
se usarán para relacionar las observaciones tomadas en distintas poses, pero no
se leerá la pose interna de la caja del simulador.

### Terminal de vacío

El terminal será un **bastidor abierto multicap**, inspirado en componentes
industriales Piab:

- cinco copas BCP de 40 mm;
- una copa central y cuatro exteriores en cruz o cuadrado;
- una válvula piSAVE Sense por copa para aislar las copas descubiertas;
- una zona central y una exterior como opción de control;
- cuerpo estrecho, cableado y vacío encaminados hacia la muñeca;
- separación ajustable antes de fijar la geometría definitiva.

Una caja pequeña podrá agarrarse con el subconjunto central. Una caja mayor
podrá usar también las copas exteriores para incrementar fuerza y resistencia a
torsión. El bastidor abierto evita que una placa sólida tape la cara superior y
los bordes laterales.

Referencias de diseño:

- [Piab BCP, copa específica para cartón](https://www.piab.com/en-us/suction-cups-and-soft-grippers/round-suction-cups/multibellows-suction-cups/s.bcp)
- [Piab piSAVE Sense, aislamiento de copas descubiertas](https://www.piab.com/en-us/suction-cups-and-soft-grippers/accessories-for-suction-cups/suction-cup-valves/0202425)

El diámetro, separación y número final de copas no se considerarán validados
hasta fijar el rango mínimo y máximo de dimensiones y masa de las cajas.

En el primer simulador, el vacío podrá representarse mediante una unión rígida
activable después de comprobar contacto y alineamiento. Esta abstracción debe
quedar registrada y no contará como validación física del agarre. Las
evoluciones introducirán subconjuntos de copas selladas, límites de fuerza y
momento, fugas y deslizamiento.

### Cámaras

El diseño integrado contempla dos funciones sensóricas:

1. **`infeed_rgbd_cam`: cámara RGB-D cenital sobre una ventana de la cinta.**
   Localiza aproximadamente la caja, su yaw y un punto de succión seguro. En el
   primer experimento puede omitirse, porque la caja comienza ya agarrada.
2. **`scan_rgbd_cam`: cámara RGB-D fija y oblicua.** Observa la caja suspendida
   y permanece inmóvil respecto al mundo mientras el UR10e la rota. Una
   resolución inicial de 640 × 480 es suficiente para establecer la baseline.

La cámara de escaneo no será estrictamente cenital: debe observar caras
laterales para que la altura sea directamente medible. Su pose se elegirá para
ver al menos dos contornos verticales y un borde superior sin que el UR10e o el
terminal los oculten simultáneamente.

## Secuencia de operación

La secuencia integrada objetivo es:

```text
WAIT_FOR_BOX
→ DETECT_AND_LOCALIZE
→ STOP_CONVEYOR
→ APPROACH
→ ATTACH_SUCTION
→ VERIFY_GRASP
→ LIFT
→ MOVE_TO_SCAN
→ SETTLE
→ CAPTURE_0
→ ROTATE_90
→ SETTLE
→ CAPTURE_90
→ [CAPTURE_180 si hace falta]
→ ESTIMATE
→ VALIDATE
→ PROFILE_READY / REJECTED
```

La baseline comenzará en `MOVE_TO_SCAN`, con una caja ya unida al terminal. La
recogida desde la cinta se añadirá después de validar la medición.

## Pipeline de percepción y geometría

Para cada pose de captura:

1. Obtener RGB, profundidad, intrínsecos, timestamp y pose de cámara.
2. Convertir la profundidad válida en una nube de puntos.
3. Recortar al volumen de escaneo.
4. Segmentar la caja y excluir geometría conocida del UR10e y del terminal.
5. Transformar los puntos al marco del terminal usando la calibración
   cámara-mundo y la cinemática directa del brazo.
6. Fusionar las observaciones en un marco común ligado al terminal.
7. Filtrar outliers y comprobar cobertura de caras y consistencia entre vistas.
8. Ajustar tres direcciones ortogonales o un cuboide orientado.
9. Estimar longitud, anchura y altura mediante estadísticas robustas.
10. Calcular incertidumbre y emitir resultado o rechazo explicable.

Para el primer MVP puede evitarse una reconstrucción general: una vista estima
altura y un lado horizontal; la vista rotada 90° estima altura y el segundo
lado. La fusión completa será una mejora posterior si aporta precisión medida.

## Contrato de salida

La salida mínima será equivalente a:

```text
ObjectDimensions
  object_id
  timestamp
  frame_id
  dimensions_lwh_m
  uncertainty_lwh_m
  views_used
  confidence
  valid
  rejection_reason
  condition
  routing
  damage
```

El centro, orientación, puntos segmentados y cuboide reconstruido pueden
conservarse como diagnósticos, pero no forman parte del resultado obligatorio
de esta exploración. Las unidades y la convención que distingue longitud de
anchura deben quedar fijadas antes de integrar con `Simulation`.

## Evolución incremental

1. **EXP-001 — Caja suspendida fija.** Caja ya unida al terminal, una pose de
   escaneo y dimensiones conocidas. Validar cámara, segmentación y marcos.
2. **EXP-002 — Rotación multivista.** Capturas detenidas a 0° y 90°; añadir
   180° únicamente para comprobar oclusiones y consistencia.
3. **EXP-003 — Dimensiones variables.** Aleatorizar longitud, anchura y altura
   con seed reproducible, manteniendo el agarre centrado.
4. **EXP-004 — Terminal multicap.** Variar qué copas contactan y sellan;
   introducir agarres ligeramente descentrados y criterios de rechazo.
5. **EXP-005 — Recogida desde cinta parada.** Cámara de entrada, pose variable,
   selección de punto de succión y secuencia completa hasta la medición.
6. **EXP-006 — Realismo.** Ruido de profundidad y calibración, oscilación,
   deslizamiento, cartón parcialmente poroso y oclusiones.
7. **EXP-007 — Escaneo adaptativo.** Capturar otra vista solo cuando la
   incertidumbre de alguna dimensión supere el umbral.

La cinta en movimiento y una política aprendida de selección de vistas quedan
fuera del MVP.

## Evaluación

Medir por separado:

- éxito y estabilidad del agarre;
- porcentaje de perfiles válidos;
- MAE, RMSE y p95 por dimensión;
- discrepancia de altura entre vistas;
- error de registro entre nubes;
- número de vistas utilizadas;
- latencia de percepción y tiempo total de ciclo;
- rechazos y causa concreta.

Como objetivo inicial del entorno ideal, la baseline debe producir 50 de 50
perfiles válidos y un error p95 inferior a 5 mm por dimensión. Este umbral es
una meta de ingeniería, no un resultado observado, y deberá revisarse al añadir
ruido y física de agarre.

## Criterio de integración

La exploración estará preparada para integrarse cuando:

- mida dimensiones variables de manera reproducible;
- el resultado no dependa del ground truth de MuJoCo;
- el registro entre poses use calibración y estado articular;
- los fallos de agarre no se contabilicen como mediciones correctas;
- el contrato de salida sea estable;
- las métricas y limitaciones estén documentadas en `docs/findings/`.
