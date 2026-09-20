# Inventario de merge

Este repositorio **es** el payload. Copia el paquete entero.

```text
src/object_profiling/contracts.py
src/object_profiling/config.py
src/object_profiling/__init__.py
src/object_profiling/measure/__init__.py
src/object_profiling/measure/measurement.py
src/object_profiling/measure/perception.py
src/object_profiling/measure/geometry.py
src/object_profiling/measure/inspection.py
src/object_profiling/measure/registration.py
src/object_profiling/measure/background.py
src/object_profiling/measure/backproject.py
```

Dependencias: `numpy`, `opencv-python`. Sin MuJoCo.

```python
from object_profiling.measure import measure
from object_profiling.contracts import CameraObservation, HeldBoxHandoff
```

El integrador construye `CameraObservation` con su cámara y su FK, un
`BackgroundSet` de la estación vacía, llama a `measure()`, y:

- si `routing == ERROR_ZONE`, suelta la caja;
- si `HeldBoxHandoff.ready_for_com()`, pasa al centro de masas con el weld activo.

Snapshot previo a recortar la estación: `40e3baf`.
