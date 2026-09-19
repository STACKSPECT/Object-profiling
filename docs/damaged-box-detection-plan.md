# Detección de cajas dañadas: ground truth y plan de implementación

## Estado de este documento

`Implementado`, `ejecutado` y `medido` en el entorno ideal. Los checkpoints
CP0–CP6 están cerrados con pruebas, auditorías headless y hallazgos
EXP-010 a EXP-014. Nada de eso se da por validado con ruido de profundidad
ni con succión física.

## Objetivo

Mientras el UR10e sostiene y rota la caja para medirla, el sistema debe además
decidir si la caja sigue siendo un cuboide apto para apilar. Si no lo es, el
brazo debe llevarla visiblemente a una zona de error en lugar de devolverla al
flujo normal.

### Decisiones de alcance ya tomadas

| Decisión | Valor |
|---|---|
| Qué se rechaza | Solo geometría insegura para apilar. Nada cosmético. |
| Zona de error | Contenedor de rechazo. El brazo lleva la caja y la suelta dentro. |
| Liberación | Se baja hasta el contenedor y se suelta a poca altura. |
| Dimensiones | Se siguen publicando. El daño **no** las invalida. |
| Generación | Generador aleatorio: 50 % dañadas en desarrollo, 10 % al cerrar. |
| Defectos por caja | Uno solo en la versión 1. |
| Cara de agarre | Excluida del daño en la versión 1. |
| Umbral de rechazo | Relativo: 5 % de la arista más corta, con suelo absoluto. |
| Recogida desde cinta | Descartada. Ignorar ese incremento del alcance previo. |

La consecuencia de que las dimensiones se sigan publicando es importante:
`valid` **no** pasa a `false` por daño. Decidir si una caja dañada puede colocarse depende del estado
del palé, que este repositorio no conoce. Este repositorio informa de la
condición; otro módulo decide. La zona de error es la política por defecto
mientras no exista ese módulo.

## Estado del arte

Hay dos familias de trabajo y no son intercambiables.

### A. Modelos de apariencia sobre RGB

Detectan arañazos, agujeros, manchas de humedad, cinta despegada y pliegues.
Son detectores 2D entrenados sobre fotografías reales de cartón:

- [SSD + IFPN para defectos de cartón logístico](https://doi.org/10.2478/eces-2023-0011).
- [TPMN, atención guiada por textura sobre cartón corrugado](https://doi.org/10.14569/IJACSA.2024.0150284),
  con el `cardboard-boxes-dataset` de 1.210 imágenes.
- [Parcels-DNet, deformación frente a penetración](https://doi.org/10.1109/ICASSP48485.2024.10448282).
- [Clasificación de daño en caja corrugada con ANN](https://doi.org/10.1002/pts.2815).
- Datasets: [Carton-Defect-Dataset](https://github.com/HUST-OROP/Carton-Defect-Dataset)
  (Breakage, Color, Crease, Scratch, Normal), y un conjunto de 2026 en
  [Zenodo](https://doi.org/10.5281/zenodo.20124133) con clases face, edge,
  corner, top y no dañada.

**No aplicable aquí todavía.** El RGB de MuJoCo es una caja de color plano sin
impresión, sin flauta y sin cinta. Un detector entrenado sobre fotos reales no
transfiere, y uno entrenado sobre estos renders aprendería la iluminación del
simulador, no el daño.

### B. Inspección geométrica 3D contra un prior de cuboide

Es el método industrial que encaja con este pipeline, que ya produce una nube
fusionada y un cuboide ajustado:

- [US 12.400.320](https://patents.google.com/patent/US12400320B2), inspección 3D
  de paquetes: se ajusta un plano a cada cara y una recta a cada arista, y se
  marca como defectuoso lo que se sale de una banda de tolerancia. Una esquina
  aplastada se declara como «las dos aristas que deberían cortarse en recta no
  lo hacen».
- [Inspección LIDAR de superficies corrugadas](https://doi.org/10.3850/978-981-18-6021-8_or-15-0143.html):
  RANSAC de plano, agrupación euclídea del residuo y caja 3D del defecto.
- CN122089714B y CN122176433A: fusión RGB + nube con prior geométrico para
  localizar hundimientos y protuberancias.

### Qué falla en la industria y por qué importa

La forense de embalaje ([ASTM D642](https://www.astm.org/d0642-20.html),
ISO 12048, procedimientos ISTA de apilado) nombra los modos que hacen insegura
una caja apilada. Son exactamente los que deben modelarse:

| Modo | Aspecto | Por qué es inseguro |
|---|---|---|
| Aplastamiento de esquina | Vértice hundido o ausente | Las esquinas son las columnas de carga |
| Colapso de arista | La arista vertical deja de ser recta | Se pierde el camino de carga del ECT |
| Pandeo de panel | La cara se curva hacia dentro o fuera | La caja deja de ser un cuboide |
| Perforación / solapa abierta | Agujero o solapa levantada | Producto expuesto, pila inestable |

El [palletizer explicable de Doosan](https://github.com/doosan-robotics/explainable-palletizer)
es la versión de producto de la política que se busca: detectar daño y desviar
la caja en lugar de apilarla.

### Conclusión

Para una estación RGB-D que ya ajusta un cuboide, **se empieza por la familia
B**. La familia A queda como evolución, y solo tendría sentido con texturas
fotorrealistas o imágenes reales.

## Viabilidad medida

Antes de escribir el plan se comprobó en esta escena, con `mujoco 3.13.0`,
`scan_rgbd_cam` a 640 × 480 y una caja de 0,30 × 0,20 × 0,15 m situada en el
centro de escaneo. Se comparó la profundidad renderizada contra la de la caja
intacta.

| Variante | Píxeles con cambio > 0,5 mm | Desviación máxima |
|---|---:|---:|
| Malla de cuboide exacta frente a `type="box"` | 1 | — |
| Chaflán de esquina 15 mm | 448 | 16,96 mm |
| Chaflán de esquina 40 mm | 2.186 | 42,71 mm |
| Hundimiento en cara 10 mm | 5.637 | observable |
| Hundimiento en cara 25 mm | 6.798 | observable |

Cuatro conclusiones, todas con consecuencias en el plan:

1. **Sustituir la caja primitiva por una malla de cuboide exacta no cambia la
   imagen**: un solo píxel de diferencia. Por lo tanto se puede usar malla para
   **todas** las cajas, intactas y dañadas, con un único camino de código. Esto
   elimina el sesgo de que la clase «intacta» y la clase «dañada» usen
   representaciones distintas.
2. **El daño es ampliamente observable** a esta resolución y distancia. La
   desviación máxima del chaflán coincide con la profundidad del corte, así que
   la severidad declarada es directamente comparable con la medida.
3. **El sentido de giro de las caras debe verificarse.** Con una malla de
   winding invertido, MuJoCo aplica back-face culling y la profundidad muestrea
   la superficie interior lejana en vez de la exterior cercana. El render sigue
   pareciendo una caja y el error es silencioso. Hay que comprobar que la normal
   de cada triángulo apunta hacia fuera.
4. **El compilador reorienta y recentra la malla** hacia sus ejes principales de
   inercia, y lo compensa en `mesh_quat` / `geom_quat`. Por eso no se puede
   escribir en `model.mesh_vert` con coordenadas de autoría: hay que recompilar,
   o aplicar antes la transformación inversa.

### Restricciones técnicas de MuJoCo detectadas

- `MjSpec.from_file()` **no** abre la ruta Unicode de este repositorio, y
  tampoco la ruta corta que produce `_path_for_mujoco()` en `config.py`, porque
  `GetShortPathNameW` devuelve la extensión en mayúsculas (`.XML`) y el
  despachador de decodificadores distingue mayúsculas. **Solución comprobada**:
  cambiar el directorio de trabajo al del `.xml` y cargar por nombre relativo.
  El mismo `chdir` hace falta en `compile()`, que resuelve `meshdir` contra el
  directorio actual.
- Modificar `uservert` **después** de un `compile()` no tiene efecto. Hay que
  construir un `MjSpec` nuevo por malla.
- `spec.recompile(model, data)` devuelve un **par modelo/datos nuevo**, no
  modifica el objeto en sitio. Un visor pasivo lanzado con el modelo anterior
  se queda con una referencia obsoleta. Esto afecta al showcase, que abre un
  único visor para seis cajas (ver riesgo R1).

## Ground truth: cómo modelamos el daño

### Definición

Una caja está **dañada** cuando su geometría deja de ser un cuboide apto para
apilar. Marcas superficiales que no mueven la nube de puntos están fuera de la
versión 1.

### Propiedad de diseño: el daño no cambia el envolvente

Los tres generadores de la versión 1 **quitan material o hunden hacia dentro**.
Ninguno produce una protuberancia hacia fuera. La consecuencia es que la caja
envolvente real sigue siendo la nominal L × W × H, así que:

- las métricas de error dimensional siguen siendo comparables entre cajas
  intactas y dañadas, contra el mismo ground truth;
- una regresión de precisión causada por el daño se puede medir directamente.

El pandeo hacia fuera queda deliberadamente para después, porque sí cambia el
envolvente y obliga a redefinir qué es la dimensión verdadera.

### Generadores

Cuatro clases, todas con severidad en metros y todas derivadas de una seed.

| Clase | Parámetros | Modo industrial equivalente |
|---|---|---|
| `INTACT` | — | Caja sana |
| `CRUSHED_CORNER` | vértice (1 de 8), profundidad de chaflán | Aplastamiento de esquina |
| `DENTED_FACE` | cara, centro en la cara, profundidad, radio | Perforación / hundimiento local |
| `BUCKLED_PANEL` | cara, flecha máxima hacia dentro | Pandeo de panel |

`CRUSHED_CORNER` corta con un plano oblicuo y proyecta sobre él los vértices que
lo rebasan. `DENTED_FACE` aplica una campana gaussiana hacia dentro.
`BUCKLED_PANEL` aplica una curvatura suave sobre toda la cara. Las tres operan
sobre los vértices de autoría, antes de compilar.

**Un solo defecto por caja en la versión 1.** Con varios defectos simultáneos no
se puede atribuir una detección o un fallo a una causa concreta, y la tabla de
resultados por clase deja de ser interpretable. Combinaciones después.

**La cara de agarre queda excluida del daño.** La succión está abstraída como un
`weld` rígido, que no modela sellado. Si se dañase la cara superior, el
simulador seguiría agarrando con normalidad y estaríamos afirmando
implícitamente que una ventosa sella sobre cartón hundido, que es justo lo que
el principio 9 de `AGENTS.md` prohíbe dar por válido. En la práctica esto
significa: para `DENTED_FACE` y `BUCKLED_PANEL`, excluir la cara `-z` del
terminal; para `CRUSHED_CORNER`, permitir los 8 vértices pero limitar la
profundidad del chaflán a que no alcance el plano de contacto de las copas. Esa
comprobación debe ser una prueba, no una convención.

La idea original de «aristas curvas en lugar de rectas» está cubierta por
`BUCKLED_PANEL`: la arista deja de ser recta porque la cara se curva. No hace
falta modelar la arista como spline; basta con desplazar la malla y **detectar**
la pérdida de rectitud.

### Malla

- Cuboide subdividido, una rejilla por cara, topología **idéntica para todas las
  cajas** (mismo número de vértices y caras). Una cara plana de un solo cuadro
  no puede hundirse; la subdivisión es lo que hace representable el daño.
- Subdivisión inicial propuesta: 16 por lado, es decir 1.734 vértices y 3.072
  triángulos. Debe ajustarse midiendo, no por gusto: suficiente para que el
  defecto más pequeño que queramos detectar abarque varios triángulos.
- Winding hacia fuera, verificado por construcción y por prueba.
- La escala de la rejilla se deriva de L, W y H de la caja, así que la malla
  sustituye por completo al ajuste de `geom_size`.

### Qué no se hace

- **No** se usan modelos de cartón fotorrealistas como ground truth inicial. Se
  pierde el control de la severidad por seed y no se puede afirmar «este
  hundimiento es de 12 mm».
- **No** se usa `flex` ni cuerpo deformable. La caja está soldada al terminal y
  no necesita deformarse durante el ciclo; solo necesita estar ya deformada.
- **No** se entrena ninguna red en la versión 1.

### Frontera de información

`DamageSpec` es generación de escena, igual que `BoxSpec`. Vive junto a él en
`station/environment.py` y **no** puede importarse desde `measure/`. La prueba
`tests/test_information_boundary.py` debe ampliarse con los identificadores
nuevos (`DamageSpec`, `damage_spec`, `DamageKind`, `severity_m` y el nombre de
la malla) para que un futuro cambio no los cuele en el estimador.

## Ground truth: cómo reconocemos el daño

Inspección geométrica sobre la nube ya fusionada, después del ajuste del
cuboide que ya existe en `measure/geometry.py`. Cinco señales, ninguna
suficiente por sí sola.

1. **Planaridad por cara.** Se asignan los puntos a las 6 caras del cuboide
   ajustado y se mide el residuo firmado contra el plano de cada cara. Un
   hundimiento y un pandeo producen residuo negativo concentrado.
2. **Rectitud de arista.** Para cada una de las 12 aristas se toman los puntos
   dentro de un radio, se ajusta una recta y se mide el residuo p95. Es la
   prueba de la patente y la que detecta el colapso de arista.
3. **Ocupación de esquina.** Para cada uno de los 8 vértices se cuenta el
   soporte en una esfera pequeña. **Esta señal es imprescindible y no se puede
   sustituir por el residuo global**: si se corta una esquina, la nube restante
   sigue siendo un cuboide casi perfecto, el residuo se mantiene bajo y un
   detector basado solo en residuo declararía la caja intacta.
4. **Agrupación del residuo firmado.** Distingue hundimiento hacia dentro de
   material ausente, y da la localización y el área del defecto.
5. **`residual_p95_m` global**, que ya se calcula. Sirve de control, no de voto
   único.

**Severidad publicada**: la desviación máxima hacia dentro en milímetros,
comparable con la severidad declarada en `DamageSpec` durante la evaluación.

**Umbrales de detección**: se calibran sobre la distribución de cajas
**intactas**, nunca a ojo. El suelo de ruido conocido del entorno ideal es de
menos de 1 mm de residuo p95 y 0,8 mm de residuo por vista en el peor caso.

**Umbral operativo de «inseguro para apilar»**: relativo al tamaño de la caja,
el 5 % de la arista más corta. Una abolladura de 10 mm no significa lo mismo en
una caja de 80 mm de alto que en una de 250 mm.

Ese criterio necesita un suelo absoluto, y conviene verlo antes de
implementarlo. Con el rango declarado en `BoxRange`, la arista más corta va de
80 a 250 mm, así que el umbral relativo cae entre **4 y 12,5 mm**. En el
extremo bajo, 4 mm deja solo un factor 4 sobre el ruido de medida actual, y ese
margen desaparecerá en cuanto se añada ruido de profundidad. Por tanto:

```text
umbral = max(0.05 * arista_mas_corta, suelo_absoluto)
```

El `suelo_absoluto` **no se fija aquí**: sale de la distribución de intactas de
CP3, como el percentil alto del residuo de una caja sana más un margen
declarado. Ambos valores viven en `AppConfig`.

El inspector solo puede consumir `CameraObservation` y la nube fusionada.

## Contrato

Ampliación de `ObjectDimensions`, subiendo `schema_version` a 4:

```json
{
  "schema_version": 4,
  "dimensions_m": {"length": 0.3435, "width": 0.1988, "height": 0.2273},
  "valid": true,
  "rejection_reason": null,
  "condition": "DAMAGED",
  "routing": "ERROR_ZONE",
  "damage": {
    "kind": "CRUSHED_CORNER",
    "severity_m": 0.031,
    "location": "corner:+x+y-z",
    "evidence": {
      "face_planarity_p95_m": 0.0008,
      "edge_straightness_p95_m": 0.0219,
      "weakest_corner_support": 3
    }
  }
}
```

Reglas:

- `condition` es `INTACT`, `DAMAGED` o `UNKNOWN`. `UNKNOWN` cuando la cobertura
  observada no permite pronunciarse.
- `valid` **no** cambia por daño, según la decisión de alcance. Se mantiene
  ligado a la calidad de la medida.
- `routing` es una pista para el integrador, `NORMAL` o `ERROR_ZONE`.
- `damage` es `null` si `condition` es `INTACT`.

Se añade `INSUFFICIENT_DAMAGE_EVIDENCE` a `RejectionReason` solo si se decide
rechazar la medida cuando no se puede evaluar la condición. Por defecto no: se
publica `condition: UNKNOWN`.

## Plan de implementación

Regla común a todos los checkpoints. Ninguno se cierra sin:

1. `python -m pytest -p no:cacheprovider` en verde, sin pruebas omitidas nuevas;
2. el comando **headless** del checkpoint ejecutado y su JSON guardado;
3. el comando **con visor** ejecutado y comprobado a ojo, con captura guardada;
4. un hallazgo en `docs/findings/` con la plantilla del repositorio, separando
   `propuesto`, `implementado`, `ejecutado` y `medido`;
5. ninguna regresión en la precisión dimensional de cajas intactas.

En macOS el modo visor necesita `mjpython`. En Windows basta con el intérprete
del entorno.

---

### CP0 — La malla como representación única

**Por qué primero.** Cambia la representación de **todas** las cajas, incluidas
las sanas. Si se mete junto al daño, cualquier regresión de precisión queda
confundida entre «la malla» y «el daño». Hay que aislarla.

**Trabajo**

- Nuevo módulo `station/boxmesh.py`: rejilla de cuboide parametrizada por L, W,
  H, con winding hacia fuera garantizado.
- `ProfilingEnvironment` construye el modelo con `MjSpec`, inyecta la malla y
  apunta `box_geom` a ella. Encapsular el `chdir` al directorio de la escena en
  un único ayudante, junto al `_path_for_mujoco()` que ya existe.
- Mantener intactos `geom_rgba`, masa e inercia, que hoy se fijan a mano.

**Criterios de salida**

- Prueba unitaria: para una muestra de dimensiones del rango, todos los
  triángulos tienen normal hacia fuera y la caja envolvente de la malla iguala
  L × W × H con tolerancia de micrómetros.
- Prueba de render: la profundidad de la malla exacta y la de `type="box"`
  difieren en menos de 5 píxeles. Ya medido en 1; sirve de prueba de no
  regresión.
- `object-profiling-benchmark --start 1000 --count 100` reejecutado. El MAE por
  eje no empeora más allá del ruido de la reejecución, y se documenta el delta
  frente a los valores de EXP-007 (0,024 / 0,179 / 0,979 mm).
- `object-profiling-showcase --headless` con 6 de 6 perfiles válidos.
- Showcase con visor abierto y comprobado.

**Riesgo que resuelve**: R1, abajo.

---

### CP1 — Generar cajas dañadas *(obligatorio 1)*

**Trabajo**

- `DamageKind` y `DamageSpec` en `station/environment.py`.
- Generadores `CRUSHED_CORNER`, `DENTED_FACE` y `BUCKLED_PANEL` en
  `station/damage.py`, operando sobre los vértices de `boxmesh.py`.
- `generate_box_spec()` acepta `damage_rate`, con valor 0,5 durante el
  desarrollo. La clase, la cara o el vértice y la severidad salen del mismo
  `rng` de la seed y quedan registrados. Un único defecto por caja.
- La severidad se muestrea **relativa a la arista más corta**, igual que el
  umbral, entre el 2 % y el 25 %. Así hay casos claramente por debajo del
  umbral del 5 %, casos justo en el límite y casos obvios, en cajas de
  cualquier tamaño. Muestrear en milímetros absolutos produciría un conjunto
  trivial para las cajas pequeñas y otro imposible para las grandes.
- Excluir la cara de agarre según la regla de la sección anterior.
- Comando nuevo `object-profiling-damage-audit`: recorre una banda de seeds,
  renderiza cada caja y guarda un mosaico más un JSON con la severidad declarada
  y la desviación de profundidad observada.

**Criterios de salida**

- Prueba: para cada clase y severidad, la desviación máxima de profundidad
  observada crece de forma monótona con la severidad declarada.
- Prueba: con `damage_rate=0.0` el resultado es idéntico a CP0, bit a bit en las
  dimensiones.
- Prueba: la misma seed produce la misma caja dañada.
- Prueba: la caja envolvente real de una malla dañada sigue siendo la nominal,
  que es la propiedad de diseño declarada arriba.
- Prueba: ningún defecto toca la cara de agarre, y ningún chaflán de esquina
  alcanza el plano de contacto de las copas.
- Prueba: cada caja lleva como mucho un defecto.
- Auditoría headless ejecutada, mosaico guardado y revisado a ojo.
- Visor abierto sobre al menos una caja de cada clase.
- Hallazgo `EXP-010`.

---

### CP2 — Recoger cajas dañadas *(obligatorio 2)*

**Por qué es un checkpoint propio.** La colisión de una malla no es la de una
primitiva. MuJoCo colisiona por casco convexo, así que un hundimiento
desaparece para el contacto aunque siga viéndose en la cámara. Además la caja
se apoya en `pickup_support` antes de activar la succión, y una esquina cortada
cambia ese apoyo.

**Trabajo**

- Verificar el reposo sobre `pickup_support` para cada clase de daño: sin
  penetración, sin oscilación residual, sin que la caja se vaya.
- Verificar que el `weld` de succión se establece en la pose esperada y aguanta
  las tres poses de escaneo, incluida `SCAN_TILT_35` con la caja más pesada.
- Revisar `active_cup_names()`: hoy decide qué copas contactan a partir de L y
  W. Con una esquina cortada, una copa exterior podría quedar sobre el vacío.
  Declararlo explícitamente, como exige el principio 9 de `AGENTS.md`.
- Ampliar `object-profiling-checkpoint` para recorrer una banda de seeds con
  daño.

**Criterios de salida**

- Sobre al menos 40 seeds dañadas: 100 % de agarres establecidos, ningún
  `MOTION_TIMEOUT`, ninguna penetración por encima de la tolerancia de contacto.
- El tiempo de ciclo simulado no se degrada respecto a las cajas intactas.
- Checkpoint headless ejecutado y JSON guardado.
- Con visor: comprobado que la caja dañada se agarra y se presenta sin
  deslizarse ni vibrar.
- Hallazgo `EXP-011`.

---

### CP3 — Métricas de daño, todavía sin decidir

**Por qué separado del reconocimiento.** Los umbrales hay que calibrarlos sobre
datos, no inventarlos. Este checkpoint produce los datos; el siguiente decide.

**Trabajo**

- `measure/inspection.py` con las cinco señales: planaridad por cara, rectitud
  de arista, ocupación de esquina, agrupación del residuo firmado y el residuo
  global que ya existe.
- Las métricas se calculan y se registran, pero **no** se emite ninguna
  clasificación ni se toca el contrato.
- Comando `object-profiling-inspection-audit` que recorre una banda mixta y
  vuelca todas las métricas por caja junto a su `DamageSpec`.

**Criterios de salida**

- Distribuciones de intactas frente a dañadas, por clase y por severidad,
  guardadas en JSON y en una figura.
- Se documenta explícitamente qué señal separa cada clase y, sobre todo, **se
  comprueba la predicción de que el residuo global no separa
  `CRUSHED_CORNER`**, y que la ocupación de esquina sí. Si resultara falso, hay
  que decirlo y simplificar el detector.
- Ninguna señal consume `DamageSpec`. Prueba de frontera de información
  ampliada.
- Hallazgo `EXP-012` con las distribuciones.

---

### CP4 — Reconocer cajas dañadas *(obligatorio 3)*

**Trabajo**

- Umbrales derivados de la distribución de **intactas** de CP3, con margen
  declarado, en `AppConfig`.
- `DamageAssessment`: clase, severidad, localización y evidencia.
- Banda de seeds de calibración y banda de evaluación **disjuntas**. No ajustar
  umbrales sobre la banda con la que se reporta.

**Criterios de salida**

- Sobre la banda de evaluación, con 50 % de daño: se reportan tasa de acierto y
  tasa de falso positivo por clase y por tramo de severidad.
- Objetivo inicial propuesto, a revisar con los datos de CP3: ningún falso
  positivo sobre cajas intactas en el entorno ideal, y detección de toda
  severidad por encima del umbral operativo. Es una meta de ingeniería, no un
  resultado.
- Se reporta también la severidad estimada frente a la declarada.
- El MAE dimensional de las cajas intactas no cambia.
- Headless y visor ejecutados.
- Hallazgo `EXP-013`.

---

### CP5 — Descartar cajas dañadas *(obligatorio 4)*

**Trabajo**

- Contrato: `condition`, `routing`, `damage` y `schema_version = 4`.
- **Contenedor de rechazo** en la escena: una caja abierta, con suelo y cuatro
  paredes, dimensionada para la caja mayor del rango con holgura. Dos
  restricciones que hay que respetar al situarlo:
  - **Fuera del encuadre de `scan_rgbd_cam`.** Si entra en el campo de visión
    cambia los fondos por pose y rompe la segmentación por resta de fondo, que
    es la base de todo el pipeline. Hay que comprobarlo, no suponerlo.
  - **Alcanzable sin colisión** desde la pose de retorno, y sin acercarse a
    límites articulares ni a singularidades.
- Pose articular de descarte, resuelta offline igual que `lift_qpos` y
  `tilt_35_qpos`, y declarada en `MotionConfig`. Debe dejar la caja **dentro**
  de la planta del contenedor y a poca altura sobre su suelo.
- Máquina de estados: tras `VALIDATE`, si `routing` es `ERROR_ZONE`, ir a
  `MOVE_TO_ERROR_ZONE`, `RELEASE` y `RETURN_HOME`; si no, el retorno actual.
- `RELEASE` desactiva el `weld` de succión y deja caer la caja desde poca
  altura. Hay que simular esa caída, no teletransportar la caja: la demo
  depende de que se vea caer dentro.

**Criterios de salida**

- La calibración de fondos **no** cambia por añadir el contenedor. Prueba de no
  regresión sobre los fondos por pose, y prueba de que el contenedor no aporta
  ningún píxel al encuadre de `scan_rgbd_cam`.
- El MAE dimensional sobre cajas intactas es idéntico al de CP4. Añadir
  geometría a la escena no puede mover la medida.
- Recorrido completo sobre una banda mixta: cada caja dañada acaba **reposando
  dentro** del contenedor, comprobado por su posición final y por contacto, y
  cada intacta hace el retorno normal, sin intervención.
- Ninguna caja rebota fuera del contenedor ni se queda en el borde.
- Headless con JSON de trazas por caja.
- **Con visor**: grabación o capturas que muestren el brazo aparcando la caja.
  Este es el criterio que pidió el usuario y no se sustituye por una prueba
  headless.
- Hallazgo `EXP-014`.

---

### CP6 — Bajar al 10 % y cerrar

**Trabajo**

- `damage_rate` a 0,1.
- Benchmark completo sobre una banda amplia, con precisión dimensional y
  detección en la misma tabla.
- README actualizado: estado, contrato versión 4 y comandos nuevos.

**Criterios de salida**

- Tasa de perfiles válidos y MAE por eje comparables con EXP-007.
- Métricas de detección con la prevalencia real del 10 %. Hay que reportar
  **precisión además de recall**: con 10 % de prevalencia, una tasa de falso
  positivo que parecía aceptable al 50 % genera muchas más falsas alarmas que
  detecciones correctas.
- Hallazgo de cierre y actualización de `docs/object-profiling-scope.md`.

## Riesgos

**R1. El visor único frente a la recompilación.** `spec.compile()` devuelve un
modelo nuevo, y el showcase abre un visor pasivo para seis cajas sobre un único
modelo. Recompilar por caja deja el visor apuntando a un modelo obsoleto.
Opciones, por orden de preferencia:

1. Compilar un modelo que contenga **las mallas de todas las cajas del recorrido
   como activos separados** y conmutar `geom_dataid` de `box_geom` por episodio.
   Todas las mallas quedan subidas al contexto gráfico al crearlo, así que la
   conmutación no requiere recompilar. Es la opción compatible con el visor.
2. Escribir en `model.mesh_vert` aplicando la transformación inversa de
   `mesh_pos` / `mesh_quat`. Frágil: esa transformación depende de la forma, que
   cambia con el daño, y además habría que recalcular las normales.
3. Recompilar por caja y aceptar reabrir el visor. Es lo más simple, pero choca
   con el límite de un visor por proceso.

Debe resolverse y medirse en **CP0**, antes de introducir daño.

**R2. Colisión por casco convexo.** Un hundimiento no existe para el contacto.
No invalida la percepción, que es visual, pero sí hay que declararlo para no
afirmar que se ha validado el apoyo físico de una caja hundida.

**R3. Coste de la malla.** 3.072 triángulos por caja frente a una primitiva.
Vigilar la latencia de percepción p50, hoy en 29 ms, y el tiempo de ciclo p50,
hoy en 0,28 s. Si el coste de compilar por episodio domina, medirlo y
declararlo por separado del coste de percepción.

**R4. Winding invertido.** Ya observado: produce profundidad silenciosamente
incorrecta con un render que parece plausible. Debe existir una prueba, no solo
cuidado al escribir el generador.

**R5. Confundir ruido con daño.** Hoy no hay ruido de profundidad. Al añadirlo,
los umbrales calibrados en el entorno ideal dejarán de valer. Dejar constancia
en el hallazgo de CP4 de que la calibración es válida solo para el entorno sin
ruido.

**R6. El contenedor de rechazo contamina la escena.** Es geometría nueva en un
pipeline cuya segmentación se basa en restar el fondo. Si acaba en el encuadre
de la cámara de escaneo, o si cambia la iluminación sobre la caja suspendida,
degrada la medición de **todas** las cajas, también las sanas. Por eso CP5 exige
comprobar los fondos y el MAE de intactas, no solo que el descarte funcione.

## Fuera de la versión 1

Registrado para que no se cuele por el camino:

- Protuberancias hacia fuera, que cambian el envolvente y obligan a redefinir
  cuál es la dimensión verdadera.
- Varios defectos en la misma caja.
- Daño en la cara de agarre, que requiere antes un modelo de succión que
  represente el sellado.
- Solapas abiertas y perforaciones.
- Defectos cosméticos y cualquier modelo entrenado sobre RGB.
- Ruido de profundidad, que invalidará los umbrales calibrados en el entorno
  ideal.
