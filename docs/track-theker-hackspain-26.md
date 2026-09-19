# THEKER Robotics · HackSpain '26

## Track

> **Automatiza una tarea que hoy hace una persona**

## Contexto del reto

THEKER Robotics tiene como misión:

> «Automatizar el 100 % del trabajo manual y crear la empresa más grande del mundo.»

El reto consiste en elegir una tarea real, ambiciosa y técnicamente difícil que actualmente realiza una persona, y construir un sistema que demuestre que puede dejar de hacerla una persona.

## 1. El reto

En Europa existe una escasez estructural de mano de obra industrial. Cada vez resulta más difícil cubrir los turnos de fábrica y muchas tareas manuales siguen sin automatizar porque exigen manejar niveles de variabilidad que la robótica tradicional todavía resuelve mal:

- Objetos distintos.
- Posiciones impredecibles.
- Deformaciones.
- Solapamientos.
- Oclusiones.
- Imprevistos y cambios de escena.

El objetivo es elegir una tarea manual que hoy se realiza en un entorno industrial y construir un sistema capaz de resolverla de forma autónoma, de principio a fin.

### Alcance técnico

Cada equipo debe definir el alcance de su sistema:

1. Qué tarea manual quiere automatizar.
2. Qué variabilidad debe ser capaz de manejar el sistema: tipos de objeto, tamaños, posiciones, orientaciones, deformaciones, oclusiones o cambios de escena.
3. Qué debe percibir, decidir y ejecutar el sistema para completar la tarea sin intervención humana.
4. Hasta qué punto la solución puede extenderse a nuevas variantes de la tarea sin reprogramarla desde cero.

La solución es libre. Puede combinar, entre otros enfoques:

- Manipulación robótica.
- Robótica móvil.
- Visión artificial.
- Planificación.
- Aprendizaje automático.
- Control y ejecución física.

Resolver bien una tarea variable de forma autónoma ya es un problema difícil. Hacer que el mismo sistema pueda adaptarse además a nuevos objetos, formatos o variantes de la tarea con cambios mínimos es el siguiente nivel. La generalización no es obligatoria, pero será especialmente valorada.

### Preguntas del jurado

El jurado debe poder responder a estas tres preguntas:

1. ¿El sistema resuelve la tarea de principio a fin y de forma repetible?
2. ¿La tarea elegida representa un problema real que hoy sigue requiriendo trabajo humano?
3. ¿Hasta dónde funciona la solución cuando aumenta la variabilidad?

## 2. Evaluación

La evaluación se estructura en dos ejes.

### Eje A — Visión, criterio y ejecución

- **Ambición del problema y originalidad del enfoque.** La tarea debe ser real y suficientemente exigente, y la solución debe aportar un enfoque técnico propio, ingenioso y bien pensado. Se valorará más una forma original de resolver un problema relevante que una tarea llamativa pero trivial.
- **Resolución de problemas durante el desarrollo.** Se tendrá en cuenta qué obstáculos aparecieron, cómo se abordaron y cómo evolucionó el diseño a partir de lo aprendido.

### Eje B — Profundidad técnica y resultados

- **Dificultad técnica asumida.** Complejidad real resuelta, nivel de variabilidad asumido y profundidad alcanzada por el sistema.
- **Autonomía y calidad de ingeniería.** El sistema debe completar la tarea de principio a fin, sin intervención humana y de forma reproducible.
- **Iteración y mejora medida.** Se deben definir indicadores propios y aportar datos que respalden cuánto ha mejorado la solución durante el desarrollo.
- **Generalización.** Se valorará hasta qué punto la solución puede extenderse a nuevos objetos, condiciones o variantes de la tarea con cambios mínimos.

### Puntos extra — Hardware y realismo físico

Se valorará especialmente una escena que represente de forma creíble el proceso real y, cuando aplique, un diseño de terminal o sistema físico coherente con la tarea.

## 3. Ejemplos de tareas

Los siguientes casos sirven únicamente como inspiración; no constituyen un menú cerrado:

- **Inducción de objetos:** colocar bultos sueltos, uno a uno y bien orientados, en una línea de clasificación.
- **Paletizado:** apilar bultos de distinto tamaño y peso en un palé que aguante el transporte.

Partir de uno de estos casos es perfectamente válido. Se valorará más la originalidad y la ambición del enfoque técnico que la elección de una tarea especialmente llamativa: el objetivo es demostrar qué nivel de variabilidad se asume, cómo se plantea la solución y hasta dónde se consigue hacerla robusta y generalizable.

También es posible combinar estos casos con robótica móvil, navegación u otros componentes.

## 4. Entregables

El formato es libre, pero la entrega debe incluir:

- **Código**, acompañado de lo mínimo necesario para entender qué contiene y cómo se ejecuta.
- **Demostración en directo** del sistema en funcionamiento.
- **Vídeo breve** del sistema en funcionamiento, opcional. Puede servir como complemento para enseñar aquello que no dé tiempo a ejecutar o que pueda fallar durante la presentación.
- **Una diapositiva**, opcional. Debe dejar claro de un vistazo el problema abordado y la solución propuesta. Puede incluir métricas, diagramas, arquitectura, datos del proceso o cualquier otro elemento útil para entender la propuesta sin explicación oral.

## 5. Recursos para empezar

Ninguna de estas herramientas es obligatoria. Son puntos de partida; investigar y encontrar las herramientas adecuadas forma parte del reto.

### Simuladores

- [MuJoCo](https://mujoco.readthedocs.io/) — física rápida y precisa. Instalación habitual: `pip install mujoco`.
- [Gazebo](https://gazebosim.org/) — estándar del ecosistema ROS.
- [NVIDIA Isaac Sim / Isaac Lab](https://developer.nvidia.com/isaac/sim) — fotorrealismo y aprendizaje por refuerzo; requiere GPU.

### Robots, pinzas y objetos

- [MELFA ROS 2 Driver](https://github.com/Mitsubishi-Electric-Asia/melfa_ros2_driver) — URDF, mallas y configuración MoveIt para brazos Mitsubishi Electric.
- [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) — robots listos para simular: brazos, pinzas, robots móviles y humanoides.

### Middleware y control

- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/) — estándar industrial para comunicar nodos, sensores y actuadores.
- [MoveIt 2](https://moveit.ai/) — planificación de movimiento con colisiones para brazos dentro de ROS.
- [mink](https://github.com/kevinzakka/mink) — cinemática inversa diferencial nativa de MuJoCo, con límites articulares y evitación de colisiones.
- [Pinocchio](https://github.com/stack-of-tasks/pinocchio) — cinemática y dinámica rápidas, sin ROS.

## 6. Criterio de éxito para el equipo

La propuesta debe demostrar, con una tarea industrial concreta y métricas observables, que el sistema:

1. Percibe el estado de la escena.
2. Decide qué acción ejecutar.
3. Ejecuta la acción físicamente o en una simulación representativa.
4. Detecta y gestiona errores o variaciones razonables.
5. Completa el proceso de principio a fin sin intervención humana.
6. Mantiene un rendimiento repetible y medible.
7. Puede extenderse a nuevas variantes con cambios mínimos, si el alcance lo permite.

> **Idea central:** no basta con que el sistema haga una demostración llamativa una vez. Debe quedar claro qué trabajo humano sustituye, qué variabilidad resuelve y con qué resultados repetibles.
