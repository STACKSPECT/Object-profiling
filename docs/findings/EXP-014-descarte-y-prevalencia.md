# EXP-014 — Descarte a zona de error y prevalencia 10 %

## Estado

Implementado, ejecutado y medido. El contrato pasa a esquema 4.
`damage.rate` operativo es 0,1.

## Hipótesis

1. El contenedor no entra en el encuadre de `scan_rgbd_cam` (grupo 5).
2. Tras `VALIDATE`, `routing = ERROR_ZONE` lleva la caja al contenedor, la
   suelta y vuelve a casa; una intacta hace el retorno normal.
3. Bajar la prevalencia al 10 % no degrada el MAE de las cajas sanas.

## Entorno

- Contenedor en `(-0.85, 0.15, 0.0)`, alcanzable desde `lift_qpos`.
- Seeds 8100–8107, `damage.rate = 0.5`, para la política de descarte.
- Precisión dimensional: 1000–1029 con rate 0 (EXP-010). Las tests de
  contrato usan seeds que siguen sanas con rate 0,1 (42, 3000–3005).

## Sistema de medición

Ciclo fijo más `MOVE_TO_ERROR_ZONE` → `RELEASE` → `RETURN_HOME`. La
liberación se simula: se corta el weld y se integran 0,8 s de caída.

## Protocolo

```bash
python -m pytest -p no:cacheprovider
object-profiling-discard-audit --start 8100 --count 8 --damage-rate 0.5 --artifacts artifacts/discard-audit --output results/discard-audit-8100-8107.json
```

El visor activa `geomgroup[5]` para ver el contenedor. La cámara de escaneo
no lo renderiza.

## Resultados

### Contenedor y fondos

0 píxeles de diferencia en `SCAN_YAW_0` al ocultar los geoms del contenedor.
El MAE de intactas (EXP-010) permanece en 0,024 / 0,154 / 0,985 mm.

### Política, 8 seeds al 50 %

| Seed | Verdad | Condición | En contenedor | Soldada | Política |
|---|---|---|---|---|---|
| 8100 | INTACT | INTACT | no | sí | sí |
| 8101 | CRUSHED_CORNER | DAMAGED | sí | no* | sí |
| 8102 | INTACT | INTACT | no | sí | sí |
| 8103 | CRUSHED_CORNER | DAMAGED | sí | no | sí |
| 8104 | CRUSHED_CORNER | INTACT | no | sí | sí (fallo de detección, cara/esquina no decisiva) |
| 8105 | INTACT | INTACT | no | sí | sí |
| 8106 | INTACT | INTACT | no | sí | sí |
| 8107 | BUCKLED_PANEL | INTACT | no | sí | sí (fallo de detección) |

\* Un primer recorrido dejó 8101 soldada sobre el contenedor porque
`MOVE_TO_ERROR_ZONE` expiró antes de `RELEASE`. El `finally` suelta siempre.

### Contrato

`schema_version = 4`: `condition`, `routing`, `damage`. `valid` no se apaga
por daño.

### Prevalencia 10 %

`DamageConfig.rate = 0.1`. Con 10 % de prevalencia, la precisión importa más
que el recall: en 8000–8019 no hubo falsos positivos, así que al 10 % no se
espera una avalancha de falsas alarmas. El recall sigue limitado por las
caras no observadas.

## Observaciones y fallos

No se abrió el visor interactivo en esta máquina de implementación; las
capturas son mosaicos RGB de `scan_rgbd_cam` y del estado final del
descarte. El modo `--visual` de demo, showcase y checkpoint enciende el
grupo 5.

## Decisión

El incremento de cajas dañadas se cierra para el entorno ideal. No se da por
validado el sellado sobre cartón dañado, ni el detector con ruido de
profundidad. El pandeo hacia fuera y el daño en la cara de agarre siguen
fuera de alcance.
