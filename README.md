# Object Profiling

Biblioteca para **medir** una caja rectangular ya sujeta y **decidir** si está
rota. El picking, el movimiento del brazo, el palé y el centro de masas viven
en el repositorio de integración.

Entrada: `CameraObservation` + `BackgroundSet`.
Salida: `ObjectDimensions` y, si la caja es apilable, `HeldBoxHandoff` con
`held=True` para el módulo de centro de masas.

```python
from object_profiling.measure import measure
from object_profiling.contracts import HeldBoxHandoff

result = measure(
    observations,            # SCAN_YAW_0 y SCAN_YAW_90
    backgrounds,
    object_id="box-0042",
    inspection_observations=inspection,  # SCAN_YAW_180, solo daño
)
handoff = HeldBoxHandoff(dimensions=result.dimensions, held=True)
if result.dimensions.routing.value == "ERROR_ZONE":
    # el orquestador suelta la caja en el contenedor
    ...
elif handoff.ready_for_com():
    # el siguiente módulo mueve la caja sin reagarrar
    ...
```

Contratos detallados: [docs/processes.md](docs/processes.md).
Archivos a copiar: [docs/merge-inventory.md](docs/merge-inventory.md).

## Qué hace y qué no

Hace:

- segmentar la caja restando el fondo de cada pose;
- fusionar nubes en `ur10e_attachment_site`;
- estimar L/W/H, incertidumbre y snap a 5 mm;
- inspeccionar daño estructural y emitir `condition` / `routing`.

No hace: recoger la caja, mover el UR10e, renderizar MuJoCo, calcular el
centro de masas, ni puntuar huecos del palé.

## Puesta en marcha

```bash
python -m pip install -e '.[dev]'
python -m pytest -p no:cacheprovider
```

Dependencias: `numpy`, `opencv-python`. No requiere MuJoCo.
