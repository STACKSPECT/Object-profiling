# AGENTS.md

## Propósito

Este paquete mide L/W/H de una caja ya sujeta y decide si es un cuboide
apilable. La integración es `measure()` más `ObjectDimensions` /
`HeldBoxHandoff`. No incluye escena MuJoCo, control del UR10e ni CoM.

## Alcance

Incluido:

- segmentación RGB-D por sustracción de fondo;
- fusión de vistas en el marco del terminal;
- ajuste de cuboide, incertidumbre y catálogo de 5 mm;
- inspección geométrica de daño y pista de enrutado.

Fuera de alcance:

- picking, IK, soldadura de succión y descarte físico;
- centro de masas, palé y colocación;
- demos, audits y generación de episodios.

## Principios

1. `measure()` no lee pose ni dimensiones internas del simulador.
2. `valid` es calidad de medida; `condition`/`routing` son apilabilidad.
3. Unidades en metros. `length >= width`. Altura sobre Z del TCP.
4. Rechazar con `RejectionReason` antes que devolver un número dudoso.
5. El orquestador deja la caja sujeta (`HeldBoxHandoff.held`) si `NORMAL`.

## Organización

```text
src/object_profiling/
  contracts.py      # CameraObservation, ObjectDimensions, HeldBoxHandoff
  config.py         # umbrales
  measure/          # measure(), inspección, geometría, registro
tests/
docs/processes.md
docs/merge-inventory.md
```

## Calidad mínima

Ejecutar `python -m pytest -p no:cacheprovider` antes de cerrar un cambio.
No reintroducir MuJoCo ni adaptadores de estación en este paquete.
