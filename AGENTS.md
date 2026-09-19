# AGENTS.md

## Propósito del repositorio

Este repositorio investiga cómo medir una caja rectangular mientras un
Universal Robots UR10e la sostiene y la rota delante de una cámara RGB-D fija.
Debe producir una estimación de longitud, anchura y altura, con incertidumbre y
métricas de error, mediante una interfaz integrable en `hackspain/Simulation`.

Mantén este repositorio independiente de la simulación completa. La prioridad
es que cada experimento sea pequeño, reproducible y fácil de comparar.

## Alcance

Incluido:

- configuración de la estación de medición y sus sensores;
- modelo y control del UR10e para recoger, elevar y presentar la caja;
- terminal de vacío multicap y abstracción explícita de succión;
- adquisición y calibración de RGB y profundidad;
- segmentación de la caja suspendida y exclusión de brazo y terminal;
- registro y fusión de observaciones tomadas en varias poses discretas;
- reconstrucción de nube de puntos y ajuste de un cuboide rectangular;
- estimación de longitud, anchura y altura;
- incertidumbre, validación, métricas y documentación de experimentos;
- definición del contrato de salida hacia el repositorio de integración.

Fuera de alcance:

- estimar su centro de masas;
- reconstruir el palé;
- decidir la posición de colocación o ejecutar el paletizado;
- optimizar el patrón completo del palé;
- entrenar controladores generales del brazo que no sean necesarios para el
  ciclo de medición.

## Principios de implementación

1. Empezar por el caso mínimo: cuboide ya sujeto rígidamente por el UR10e,
   cámara RGB-D fija y oblicua, y dos o tres poses de escaneo predefinidas.
2. Separar entorno, sensores, percepción, geometría, evaluación y presentación.
3. No usar como entrada del algoritmo la pose ni las dimensiones internas de
   MuJoCo. El ground truth solo puede utilizarse en evaluación.
4. Hacer explícitos los marcos de coordenadas, unidades, convenciones de ejes y
   transformaciones.
5. Toda aleatoriedad debe aceptar una seed y quedar registrada.
6. Añadir complejidad de una en una: dimensiones, rotación, subconjunto de copas
   selladas, posición de recogida, ruido, deslizamiento y oclusiones.
7. Rechazar una medida dudosa con una razón explícita antes que devolver un
   resultado aparentemente preciso.
8. Evitar acoplar código a nombres o estructura interna de `Simulation`; la
   integración debe hacerse mediante un contrato de datos estable.
9. Declarar cualquier unión rígida usada para simular succión. No confundir un
   agarre abstraído con una validación física de la ventosa.
10. Capturar con el robot detenido antes de intentar escaneo continuo.

## Organización esperada

Cuando empiece la implementación, favorecer esta separación:

```text
src/object_profiling/
  environment/     # escena, objetos y generación de episodios
  robot/           # UR10e, poses de escaneo y secuencia de movimiento
  gripper/         # contactos, copas, vacío y estado de agarre
  sensors/         # cámaras, calibración y back-projection
  perception/      # ROI, segmentación y nube de puntos
  registration/    # transformaciones y fusión entre vistas
  geometry/        # ajuste de cuboide y dimensiones
  evaluation/      # ground truth, métricas y benchmarks
  contracts.py     # salida pública ObjectDimensions
tests/
docs/findings/
```

No crear módulos vacíos solo para imitar esta estructura; añadirlos cuando
exista una responsabilidad real.

## Experimentos y evidencia

Cada cambio experimental relevante debe tener una entrada en `docs/findings/`
siguiendo su plantilla. Debe distinguir:

- estado del entorno;
- configuración física y sensórica;
- poses del UR10e y configuración del terminal;
- método evaluado;
- protocolo, seeds y dataset de escenas;
- resultados medidos;
- fallos observados y siguiente decisión.

No describir como validado algo que solo esté diseñado o implementado. Separar
claramente `propuesto`, `implementado`, `ejecutado` y `medido`.

## Calidad mínima

- Ejecutar las pruebas relacionadas antes de cerrar un cambio.
- Incluir un benchmark reproducible para cualquier mejora de precisión.
- Mantener los artefactos pesados fuera de Git; conservar en el hallazgo los
  comandos, parámetros y métricas necesarios para reproducirlos.
- Documentar cualquier desviación del alcance antes de implementarla.
