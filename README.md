# Object Profiling

Repositorio de exploración para medir cajas rectangulares mientras un UR10e las
mantiene suspendidas y las presenta desde varias orientaciones ante una cámara
RGB-D fija.

Este trabajo está aislado del repositorio de integración `hackspain/Simulation`.
Su salida principal serán las tres dimensiones de la caja y su incertidumbre,
consumibles por el resto del sistema. Este repositorio sí incluye el agarre y
los movimientos del UR10e necesarios para medir; no incluye el cálculo del
centro de masas, la reconstrucción del palé ni la planificación de colocación.

## Estado actual: checkpoint de movimiento fijo

La implementación se detiene deliberadamente en este flujo:

```text
ATTACH_SUCTION
-> LIFT / SCAN_YAW_0
-> SCAN_YAW_90
-> SCAN_TILT_35
-> RETURN_VERTICAL
```

La demo usa una caja de 0,30 × 0,20 × 0,15 m y 2 kg, presentada bajo las cinco
copas. La misma trayectoria se verifica automáticamente con las cajas mínima,
nominal y máxima del rango. La succión se abstrae mediante un `equality weld`
rígido de MuJoCo. No se detecta todavía la caja con cámaras y no se calculan sus
medidas.

El modo headless valida que la caja se eleve al menos 0,10 m, que el robot
alcance todas las poses, vuelva a vertical y que la transformación relativa terminal-caja no derive
más de 1 mm ni 0,5°:

```bash
source .venv/bin/activate
object-profiling-checkpoint --headless --seed 42 --output results/checkpoint-seed-42.json
```

En macOS, la demo visual debe lanzarse mediante `mjpython`:

```bash
source .venv/bin/activate
mjpython -m object_profiling.checkpoint --visual --seed 42 --speed 1.0
```

`--speed` controla únicamente la reproducción del visor: `2.0` muestra la
secuencia al doble de velocidad y `0.5` a la mitad. No modifica el timestep, la
trayectoria simulada ni las métricas físicas.

La ventana muestra el mismo flujo que usa el test headless. En la terminal se
imprimen los cambios de estado y, al terminar, el informe JSON. Un resultado
correcto termina con `"success": true` y código de salida 0.

## Puesta en marcha

En macOS, Open3D necesita `libusb` como dependencia nativa:

```bash
brew install libusb
```

Después, activar el entorno local e instalar las dependencias de Python:

```bash
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e '.[dev]'
```

Ejecutar las pruebas del checkpoint:

```bash
python -m pytest -p no:cacheprovider
```

## Documentación

- [Reto THEKER](docs/track-theker-hackspain-26.md)
- [Alcance técnico de Object Profiling](docs/object-profiling-scope.md)
- [Comparación de enfoques de medición](docs/measurement-approaches-handoff.md)
- [Registro de experimentos](docs/findings/README.md)
- [EXP-000: agarre, elevación y rotación](docs/findings/EXP-000-agarre-elevacion-rotacion.md)
- [EXP-001: trayectoria fija con inclinación](docs/findings/EXP-001-trayectoria-fija-inclinacion-35.md)
