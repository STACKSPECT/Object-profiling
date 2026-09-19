# EXP-006 — Estimación del cuboide y cobertura de extremos

## Estado

Implementado, ejecutado y medido en simulación sobre tres cajas dirigidas. No
medido todavía sobre dimensiones aleatorias; eso es EXP-007.

## Hipótesis

Como la caja queda alineada con el marco del terminal, las extensiones robustas
por eje deberían recuperar longitud, anchura y altura sin buscar orientación. La
duda que el plan señalaba era si los percentiles de los puntos visibles
recuperan un extremo que ninguna cara observada define.

## Entorno

- Base de código: commit `b72cfe7` más los cambios de EXP-006.
- Casos: cajas mínima, nominal y máxima. Seed 42.
- Poses: las tres de la trayectoria fija.

## Sistema de medición

- Estimador: `geometry.py`.
- Comparación de métodos: `geometry_audit.py`.
- Resolución lateral: `sensors.lateral_pitch_m()`.

## Método

Baseline: extensiones robustas por eje en el marco del terminal, con
`length = max(extent_x, extent_y)`, `width = min(extent_x, extent_y)` y
`height = extent_z`.

### La cobertura de extremos resultó ser el problema real, y su causa fue nuestra

Las tres dimensiones no se observan igual:

| Eje | Puntos a 2 mm del extremo inferior | Del superior |
|---|---:|---:|
| x | 11.348 | 49.559 |
| y | 63.750 | 406 |
| z | 15.181 | 976 |

Medido sobre la caja máxima. El extremo `+y` y la cara inferior **no los define
ninguna cara observada**: solo aparecen en la silueta, el contorno de la caja
contra el fondo.

De ahí salieron dos errores propios, ambos de nuestra configuración:

1. **La erosión de 2 px que EXP-003 recomendó borraba justamente esos
   extremos.** A 1,6 mm por píxel, quitar 2 px del contorno cuesta unos 5 mm de
   anchura. Medido: con erosión 2 el error de anchura era de −4,86 a −5,55 mm;
   con erosión 0 baja a −0,11 a −0,33 mm.

   La erosión defendía de un píxel de silueta con 97,7 mm de error que EXP-003
   observó una vez. Revisado: el rasterizador de MuJoCo asigna cada píxel a una
   primitiva completa y no interpola profundidad entre ellas, así que no produce
   los píxeles voladores de un sensor real. Ese caso era un desacuerdo subpíxel
   entre la máscara y el rayo de referencia de la auditoría, no un error de
   profundidad. **En hardware real habría que revisarlo**, porque un sensor sí
   genera píxeles voladores en los bordes.

2. **El recorte dejaba 40 mm de holgura por encima de la cara superior, y las
   copas caben ahí.** Las copas ocupan z entre 0,065 y 0,089 m en el marco del
   terminal, y los 150 píxeles falsos que EXP-004 midió sobre ellas entraban en
   la nube y estiraban la altura hasta 30 mm. El límite pasa al propio plano de
   contacto de las copas, con 2 mm de holgura. Es geometría del terminal, no
   ground truth de la caja.

### Percentil de recorte

Recortar medio punto porcentual por extremo, como hacía el prototipo, es
desastroso precisamente porque los extremos tienen poca densidad:

| Percentil | Peor error de anchura |
|---|---:|
| sin recorte | +0,29 mm |
| 0,05 / 99,95 | −0,33 mm |
| 0,5 / 99,5 | −11,75 mm |
| 1,0 / 99,0 | −16,31 mm |

Queda en 0,05 / 99,95.

### Validación de cobertura

Se añade `INSUFFICIENT_FACE_COVERAGE`: cada extremo de cada eje debe tener al
menos 50 puntos en una loncha de 2 mm. El extremo peor sobre el rango de cajas
aporta 121 puntos, así que el umbral deja margen y rechazaría un extremo
sostenido por un puñado de puntos sueltos.

### Incertidumbre

Se suman en cuadratura cinco términos:

- dispersión del bootstrap determinista al remuestrear la nube;
- resolución lateral de la cámara a la distancia de trabajo, 1,58–1,72 mm por
  píxel, repartida entre los dos bordes que definen cada dimensión;
- p95 del residuo del cuboide ajustado;
- residuo de plano por vista, es decir consistencia de registro;
- déficit de cobertura, proporcional a la raíz del soporte que falta.

## Protocolo

```bash
source .venv/bin/activate
python -m pytest -p no:cacheprovider
object-profiling-geometry-audit --output results/geometry-audit.json
```

## Resultados

| Caja | Error longitud | Error anchura | Error altura | Incertidumbre | Confianza |
|---|---:|---:|---:|---:|---:|
| mínima | 0,016 mm | 0,105 mm | 0,218 mm | 1,23 mm | 0,815 |
| nominal | 0,025 mm | 0,218 mm | 0,566 mm | 1,33–1,35 mm | 0,955 |
| máxima | 0,038 mm | 0,328 mm | 0,682 mm | 1,38–1,47 mm | 0,951 |

- Peor error absoluto: **0,682 mm**.
- La incertidumbre cubre el error en los tres casos y en las tres dimensiones.
- Residuo p95 del cuboide: 0,20–0,80 mm. Residuo de plano por vista: ≤ 0,043 mm.

### Comparación de métodos

Peor error absoluto sobre las tres cajas:

| Método | Peor error |
|---|---:|
| extensiones robustas por eje (baseline) | 0,682 mm |
| rectángulo mínimo sobre convex hull | 0,682 mm |
| refinado por plano de soporte | 1,180 mm |
| extensiones sin recorte | 2,119 mm |
| extensiones recortadas al 0,5 % | 4,018 mm |
| componentes principales | 78,808 mm |

El rectángulo mínimo sobre el convex hull empata con el baseline, pero no aporta
nada: la caja ya está alineada con el marco del terminal, así que el rectángulo
encuentra los mismos ejes pagando un convex hull sobre más de cien mil puntos.

Las componentes principales fallan por 78,8 mm. La nube está muy sesgada hacia
las caras que la cámara ve mejor, así que la dirección de máxima varianza no es
un eje de la caja. Es un recordatorio de que un método correcto sobre una nube
completa puede ser inservible sobre una nube parcial.

El refinado por plano de soporte empeora: sustituir el extremo por la mediana de
su loncha de apoyo mete la cara hacia dentro.

## Observaciones y fallos

- El residuo mediano por vista no detecta una vista mal registrada. Al desplazar
  una vista 10 mm, sus puntos de las caras laterales siguen apoyados en el
  cuboide común y la mediana no se mueve. Se usa el percentil 95, que sí lo
  acusa, con umbral de 3 mm frente a los 0,805 mm medidos.
- La incertidumbre de 1,2–1,5 mm es conservadora frente a errores de 0,7 mm. Está
  dominada por el término de resolución lateral. Es un límite físico razonable,
  pero conviene comprobar su calibración sobre muchas seeds antes de afirmar que
  está bien dimensionada.
- El error de altura crece con el tamaño de la caja, de 0,218 a 0,682 mm. La cara
  inferior se ve con más inclinación en cajas grandes.
- La cobertura no detecta una cara **completamente** ausente: el percentil se
  apoyaría en la región densa siguiente y el soporte saldría suficiente. Detecta
  el extremo escasamente sostenido, que es el caso que se da aquí.

## Decisión

Se conservan el baseline de extensiones robustas por eje, el recorte al plano de
las copas, la máscara sin erosionar y la validación de cobertura. El siguiente
paso es el orquestador completo y el benchmark sobre dimensiones aleatorias, que
es donde estas cifras deben sostenerse.
