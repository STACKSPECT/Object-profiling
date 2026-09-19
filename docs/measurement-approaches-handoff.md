# Handoff: enfoques para medir paquetes mediante visión y manipulación

## Objetivo

Diseñar el subsistema que recibe un paquete rectangular heterogéneo por una cinta,
estima longitud, anchura, altura, centro y orientación, y entrega un perfil
geométrico a hackspain/Simulation. Este documento compara opciones y deja una
instrucción para el siguiente agente; no implementa código.

## Decisión adoptada

Tras comparar los enfoques, se adopta **D: recoger, sostener y rotar ante una
cámara fija**. La medición objetivo se limita inicialmente a longitud, anchura y
altura de cuboides rígidos.

- **Brazo:** Universal Robots UR10e.
- **Terminal:** bastidor abierto y compacto con cinco copas Piab BCP de 40 mm,
  una central y cuatro exteriores.
- **Aislamiento:** una válvula piSAVE Sense por copa; opcionalmente una zona
  central y otra exterior.
- **Cámara de entrada:** RGB-D cenital para localización y agarre cuando se
  integre la cinta.
- **Cámara de escaneo:** RGB-D fija y oblicua para observar caras laterales sin
  que el terminal tape sus contornos.
- **Captura inicial:** poses discretas con el robot detenido; no escaneo
  continuo.

El diseño multicap permite agarrar cajas pequeñas con el subconjunto central y
cajas mayores con más copas. El bastidor será abierto para reducir oclusiones.
El diámetro y la separación finales quedan condicionados por las dimensiones y
el peso mínimos y máximos que se definan para los experimentos.

La arquitectura recomendada es:

    Cámara de entrada → detectar → UR10e + ventosa → zona de escaneo
    → rotaciones discretas ante RGB-D fija → fusionar nubes
    → ajustar cuboide → ObjectDimensions + incertidumbre

Desarrollar en dos niveles:

1. Baseline: caja ya agarrada y suspendida en la zona de escaneo.
2. MVP integrado: detectar en cinta, recoger, rotar delante de una cámara fija y medir.

Las estimaciones suponen una persona, MuJoCo instalado y cuboides como primer
tipo de objeto.

| Enfoque | Dificultad | Robustez esperada | Valor demo |
|---|---:|---|---|
| A. Medir sobre superficie | 2/5 | Alta en caja aislada | Media |
| B. Dimensionador multi-cámara | 3/5 | Alta ante posiciones variables | Media-alta |
| C. Recoger, depositar y medir | 3/5 | Muy alta si el depósito es repetible | Media |
| D. Recoger, sostener y rotar | 4/5 | Alta si no desliza la ventosa | Muy alta |
| E. Escaneo continuo | 5/5 | Dependiente de sincronización | Alta |
| F. Next-best-view | 5/5 | Potencialmente general | Muy alta |

## A. Medir sobre una superficie

Una caja aparece sobre un plano conocido. Una RGB-D cenital elimina el plano,
segmenta la caja y ajusta un rectángulo orientado. La altura se obtiene entre la
superficie superior y el plano de apoyo.

Entorno y pipeline:

- una RGB-D cenital;
- plano conocido;
- una caja por episodio;
- posición y yaw inicialmente fijos;
- UR10e ausente o inactivo;
- nube de puntos → RANSAC → componente de caja → rectángulo orientado →
  altura robusta → validación.

Dificultad: 2/5. Implementación estimada: 0,5–1,5 días. Debe ser el primer
benchmark aunque no sea la demo final.

Referencias:

- [POC_measuringCuboids](https://github.com/Bramke/POC_measuringCuboids):
  medición de cuboides desde nube con Open3D, NumPy y RANSAC.
- [Luxonis box measurement](https://github.com/luxonis/oak-examples/tree/main/depth-measurement/3d-measurement/box-measurement):
  RGB-D, segmentación, nube de puntos y ajuste de cuboide.

## B. Dimensionador multi-cámara en la cinta

La caja se mide directamente con cámaras fijas. La cinta puede detenerse para
capturar o moverse con seguimiento temporal.

Componentes:

- cámara de entrada;
- dos o más RGB-D oblicuas;
- calibración de todas en un marco común;
- estado observable de la cinta.

Pipeline:

1. Detectar la llegada.
2. Segmentar en cada cámara.
3. Crear y transformar cada nube.
4. Fusionar y filtrar outliers.
5. Ajustar planos o cuboide.
6. Emitir dimensiones y confianza.

No depende de un agarre y se aproxima a un dimensionador industrial, pero exige
calibración entre cámaras y aporta menos valor al movimiento del robot.

Dificultad: 3/5 con cinta parada, 4/5 en movimiento. Estimación: 1–3 días para
la versión parada y 2–4 días adicionales para seguimiento.

Referencia: [Precise Measurement of Cargo Boxes for Gantry Robot Palletization](https://portal.fis.tum.de/en/publications/precise-measurement-of-cargo-boxes-for-gantry-robot-palletization/).
El trabajo de TUM describe varias RGB-D, fusión de nubes, extracción de
primitivas y corrección frente a ruido y oclusiones, con precisión
subcentimétrica reportada en un espacio grande.

## C. Recoger, depositar y medir

El UR10e recoge y deposita el paquete en una estación estable:

    detectar → recoger → depositar → estabilizar → medir → retirar

Componentes: RGB-D de entrada, UR10e con ventosa, superficie o jaula de medición
con topes, RGB-D cenital en la estación y cámaras laterales opcionales.

El robot debe verificar el agarre, depositar, esperar a que cese el movimiento y
ejecutar A o B. La caja estable separa fallos de recogida y medición, pero añade
depósito y una estación física.

Dificultad: 3/5. Implementación: 1,5–3 días. Topes y verificación de estabilidad:
0,5–1,5 días adicionales.

Elegirlo si la prioridad es robustez rápida. Mantenerlo como fallback para
objetos que no puedan medirse suspendidos.

## D. Recoger, sostener y rotar ante cámara fija

El UR10e recoge la caja, la lleva a una zona de escaneo y la gira mientras una
RGB-D fija captura varias vistas. Es el enfoque recomendado para la demo.

Configuración:

- robot: Universal Robots UR10e;
- terminal: ventosa sobre la cara superior;
- cámara de entrada: RGB-D cenital sobre una ventana de recogida;
- cámara de escaneo: RGB-D fija y oblicua;
- cuboides con dimensiones, posición y yaw variables;
- cinta inicialmente parada durante la recogida.

La cámara de escaneo debe ser fija respecto al mundo. No debe ser la única
cámara montada en la muñeca: si cámara y caja se mueven juntas, el giro no
produce vistas suficientemente distintas.

Máquina de estados:

    WAIT → DETECT → STOP_CONVEYOR → PLAN_APPROACH → ATTACH_SUCTION
    → VERIFY_GRASP → LIFT → MOVE_TO_SCAN
    → CAPTURE_0 → ROTATE_90 → CAPTURE_90
    → ROTATE_180 → CAPTURE_180
    → ESTIMATE → VALIDATE → RELEASE → DONE / REJECTED

El primer MVP usa 0°, 90° y 180°. Añadir 270° solo si los resultados lo
justifican.

La cámara de entrada debe devolver máscara, centro, yaw, normal superior, punto
de succión y confianza. El punto debe guardar margen a los bordes; con confianza
insuficiente se repite o rechaza la recogida.

Para cada vista:

1. Capturar RGB, profundidad, intrínsecos y cámara-mundo.
2. Segmentar caja y eliminar ventosa y brazo.
3. Transformar puntos al marco del terminal con pose articular y cinemática
   directa del UR10e.
4. Fusionar nubes en el marco del terminal.
5. Ajustar planos y cuboide.
6. Calcular dimensiones robustas y comparar vistas.

No usar la pose interna de la caja en MuJoCo para registrar nubes. El registro
debe depender de sensores, calibración y estado articular.

Separar los fallos: detección, aproximación, succión, pérdida durante giro,
balanceo, profundidad incompleta, calibración, registro, cuboide inconsistente y
perfil fuera de tolerancia.

Dificultad: 4/5. Escaneo con caja ya agarrada: 2–4 días. Detección y recogida
desde cinta: 1–3 días adicionales. Verificación de succión y estabilidad:
1–2 días adicionales.

Referencias:

- [Robotic In-Hand 3D Object Modeling](https://rse-lab.cs.washington.edu/projects/3d-in-hand/):
  calibración sensor-robot, seguimiento, modelado 3D y planificación de vistas.
- [RGB-D In-Hand Object Scanning](https://www.rgbdinhandmanipulation.com/):
  dataset y resultados de escaneo durante manipulación y reorientación.
- [In-Hand 3D Object Scanning from an RGB Sequence](https://rgbinhandscanning.github.io/):
  reconstrucción de objetos desconocidos durante manipulación.

## E. Escaneo continuo

El UR10e gira continuamente mientras la cámara captura RGB-D y las nubes se
integran online. Añade sincronización cámara-estado articular, interpolación por
timestamp, latencia, desenfoque, pérdida de profundidad, control de velocidad y
detección de balanceo.

Dificultad: 5/5. Prototipo después de D: 3–5 días. Versión robusta:
5–10 días adicionales.

No usarlo para el primer MVP: las capturas discretas permiten atribuir cada error
a una fase concreta.

## F. Percepción activa y next-best-view

Tras cada vista, el estimador decide si las dimensiones son suficientes:

    capturar → estimar incertidumbre
      → suficiente: finalizar
      → falta lateral: girar 90°
      → falta altura: inclinar o cambiar vista
      → riesgo de agarre: detener y rechazar

Primero debe ser una política geométrica explicable. Después se puede entrenar
Meta-RL para minimizar vistas, tiempo, riesgo de pérdida y error dimensional.

Dificultad: 5/5. Política geométrica: 2–4 días tras D. Meta-RL:
1–2 semanas adicionales.

Referencia: [NeU-NBV](https://github.com/dmar-bonn/neu-nbv), planificación de la
siguiente vista guiada por incertidumbre. No es un controlador listo para UR10e.

## Cámara en muñeca: limitación

Una cámara en el terminal puede localizar la caja y verificar el agarre, pero no
debe ser la única cámara de escaneo. Si cámara y objeto comparten movimiento,
rotar el brazo no cambia suficientemente la vista relativa.

Para escanear usar una cámara fija respecto a la estación, una fija más otra
lateral, o cámara de muñeca combinada con un movimiento que cambie realmente la
pose relativa.

## UR10e y MuJoCo

El modelo de [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/main/universal_robots_ur10e)
es una base reutilizable. Su documentación lo describe como MJCF simplificado y
advierte de posibles inestabilidades al combinarlo directamente con Robotiq
2F-85. Se recomienda modelar una ventosa específica.

Si se usa una abstracción para mantener la caja agarrada, debe declararse en el
finding y medirse por separado; no puede convertirse silenciosamente en un
éxito de agarre.

## Contrato de salida

    ObjectDimensions
      object_id, timestamp, frame_id
      dimensions_lwh_m
      views_used
      confidence, uncertainty_lwh_m
      valid, rejection_reason

Separar Observation (sensores y poses), ObjectSegment (máscara y puntos),
GeometricEstimate (dimensiones y cuboide) y ObjectDimensions (resultado
validado). Centro, yaw, puntos y vértices pueden conservarse como diagnóstico.

## Roadmap de implementación

1. Caja ya agarrada: pose fija, una captura y validación de marcos.
2. Rotación suspendida: dos o tres capturas y registro con cinemática.
3. Recogida desde cinta: cámara de entrada, cinta parada, succión y verificación.
4. Decisión adaptativa: otra vista solo si la incertidumbre lo requiere.
5. Cinta en movimiento y Meta-RL únicamente tras reproducibilidad.

## Métricas y aceptación

| Área | Métricas |
|---|---|
| Detección | tasa, error de centro y yaw |
| Recogida | tasa de agarre, pérdidas durante giro |
| Medición | MAE, RMSE y p95 por dimensión |
| Registro | error de alineamiento entre vistas |
| Decisión | vistas por objeto, rechazos, perfiles falsos |
| Rendimiento | latencia p50/p95 y tiempo de ciclo |

Objetivo inicial ideal: 50/50 perfiles válidos en baseline, p95 menor de 5 mm por
eje, ningún fallo de agarre contado como medición correcta y todas las
ejecuciones reproducibles con seed. Los umbrales con ruido, oclusión, movimiento
y cajas no cuboides deben definirse en nuevos findings.

## Instrucción final

Implementar D desde una baseline con la caja ya agarrada. Incorporar después la
recogida desde cinta. Conservar A como benchmark geométrico opcional y C como
fallback si el agarre suspendido no resulta estable. E y F son evoluciones
posteriores.
