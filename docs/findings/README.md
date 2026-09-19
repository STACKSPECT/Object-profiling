# Findings de Object Profiling

Esta carpeta contiene un documento por experimento. Su objetivo es conservar
qué se probó, bajo qué condiciones y qué resultado produjo, no escribir un
diario cronológico.

## Nombres

Usar nombres estables y ordenables:

```text
EXP-001-caja-suspendida-vista-fija.md
EXP-002-rotacion-multivista.md
EXP-003-dimensiones-variables.md
```

## Plantilla

```markdown
# EXP-NNN — Título

## Estado

Propuesto | Implementado | Ejecutado | Medido | Descartado

## Hipótesis

Qué cambio se evalúa y por qué debería mejorar la medición.

## Entorno

- Commit:
- Simulador y versión:
- Seed o seeds:
- Número de escenas:
- Estado de cinta o posición inicial de la caja:
- Volumen de escaneo y obstáculos:
- Distribución de longitud, anchura, altura, posición y orientación:
- Ruido, iluminación y oclusiones:

## Sistema de medición

- Modelo del UR10e y controlador:
- Pose de recogida y poses de escaneo:
- Trayectoria, velocidades y tiempo de estabilización:
- Terminal, patrón y diámetro de copas:
- Copas en contacto, copas selladas y zonas activas:
- Modelo de succión o abstracción de unión rígida:
- Número y tipo de cámaras:
- Pose, resolución, FOV e intrínsecos por cámara:
- Modalidades usadas: RGB, profundidad, segmentación u otras:
- Calibración y marcos de referencia:

## Método

- Preprocesado y ROI:
- Segmentación de caja y exclusión de brazo/terminal:
- Reconstrucción de nube de puntos:
- Registro y fusión entre poses:
- Ajuste geométrico:
- Estimación de longitud, anchura y altura:
- Criterios de validez e incertidumbre:

## Protocolo

Comando exacto, configuración, datos evaluados y forma de comparar contra el
ground truth. El ground truth no puede entrar en el estimador.

## Resultados

| Métrica | Resultado |
|---|---:|
| Tasa de perfiles válidos | |
| Tasa de agarres válidos | |
| MAE longitud | |
| MAE anchura | |
| MAE altura | |
| Error p95 por dimensión | |
| Error de registro entre vistas | |
| Vistas utilizadas por caja | |
| Latencia de medición p50 / p95 | |
| Tiempo de ciclo p50 / p95 | |

## Observaciones y fallos

Casos concretos, capturas o artefactos relevantes y explicación respaldada por
evidencia.

## Decisión

Qué se conserva, qué se descarta y cuál es el siguiente experimento.
```

Si una métrica no aplica, marcarla como tal. No omitir condiciones que impidan
comparar dos experimentos.
