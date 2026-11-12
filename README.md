<img src="viewer/assets/uni_logo.png" alt="Universidad Nacional de Ingeniería" width="96" align="right">

# Manipulador 3R de Ramzy · simulador 3D en Python

**Grupo 3**<br>
**MIA-12 Robótica y Sistemas Autónomos**<br>
**MAESTRIA EN INTELIGENCIA ARTIFICIAL · 2026-2**

Proyecto reproducible de la Universidad Nacional de Ingeniería (UNI).

El [diagrama de arquitectura y trazabilidad](docs/arquitectura_ramzy_3r.png)
resume cómo se conectan el artículo, los módulos Python, las pruebas, los
resultados y el visor. Está disponible también en SVG y PDF dentro de `docs/`.

Reimplementación independiente del modelo nominal de Ramzy, Hammad y Maged
(2024), *Kinematic and Dynamic Modeling of 3DOF Variable Stiffness Links
Manipulator with Experimental Validation*, Applied Sciences 14(12), 5285.
[Artículo y DOI](https://doi.org/10.3390/app14125285).

El proyecto separa la verificación matemática, la comparación parcial con tablas
del artículo y los experimentos propios. El parcial de la UNI exige MATLAB. Los
experimentos Python sirven como preparación técnica, referencia numérica y
material para la discusión con el autor.

## Ejecución en Windows PowerShell

Abre PowerShell dentro de la carpeta del proyecto. Se verificó con Python
3.13.5 y 3.12.14 y con las versiones fijadas en `requirements.txt`. Comprueba
que `python --version` indique 3.13 o 3.12 antes de crear el entorno:

```powershell
python --version
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
.\.venv\Scripts\python.exe run_experiments.py --out "resultados_locales_$(Get-Date -Format yyyyMMdd_HHmmss)"
```

El nombre con fecha y hora conserva corridas anteriores. Si Windows no reconoce
`python`, usa la ruta completa a un ejecutable de Python 3.13 o 3.12 para el
segundo comando. En Linux o macOS, crea el entorno con `python3 -m venv .venv`
y usa `.venv/bin/python` en los comandos restantes. Añade `--sin-dinamica`
si solo quieres regenerar la cinemática, el control y la auditoría de tablas.
La ejecución no descarga los datos ni requiere MATLAB, GPU o credenciales.
La instalación inicial requiere acceso a los paquetes indicados.

## Visor 3D interactivo

Tras generar una carpeta `resultados_locales*`, abre el manipulador animado con:

```powershell
.\.venv\Scripts\python.exe .\launch_viewer.py
```

El comando selecciona la corrida local terminada más reciente y abre una interfaz
3D en el navegador. Allí puedes girar la cámara, reproducir o pausar el brazo,
recorrer el tiempo, cambiar la velocidad y comparar la trayectoria del efector
con la referencia en los seis casos de control. Para usar una corrida concreta,
añade `--results .\nombre_de_carpeta`. Mantén abierta la terminal del visor;
`Ctrl+C` detiene el servidor local. El visor usa `127.0.0.1` y los CSV generados
en este equipo. La biblioteca Three.js está incluida en `viewer/vendor/`, con
su licencia, y el visor no necesita conexión a una CDN.

En **Cinemática inversa → Regular**, el botón **Ver las 4 posturas** anima las
cuatro ternas de `inversa.csv` hacia un mismo punto XYZ. Cada tarjeta muestra
q₁ (base), q₂ (hombro), q₃ (codo), el signo de r y la rama del codo; el panel
de datos muestra los ángulos durante el movimiento, el cambio respecto a la
terna anterior y la distancia instantánea de la punta al objetivo. El paso
entre ternas interpola ángulos para ilustrar el cambio de postura: solo las
posturas finales son soluciones de cinemática inversa. También puedes elegir
una tarjeta o avanzar con **Anterior / Siguiente**.

En la escena y en el panel de la derecha se ven los valores de ρ (distancia
horizontal), h (altura desde el hombro) y d (diagonal hombro–objetivo). El
panel muestra el cálculo numérico de d y de D = cos(q₃), además de r con
signo para la terna elegida. **d y D son magnitudes diferentes**: d se mide
en metros y D no tiene unidad.

La escena anima el modelo cinemático nominal. Los límites de velocidad y el
seguimiento corresponden a la simulación de control; no son una grabación ni
una reproducción física del prototipo publicado.

El modo **Cinemática inversa** muestra cinco casos alcanzables calculados en
`inversa.csv` y dos objetivos inalcanzables definidos en el experimento;
`resumen.json` conserva los conteos de los siete casos. Incluye el caso regular,
codo extendido o plegado, sobre el eje, eje y borde, un objetivo demasiado
lejano y otro demasiado cercano. La pantalla sigue tres pasos: punto XYZ,
combinaciones de base y codo, y comprobación por cinemática directa. El detalle
de ρ, h y D queda en **Ver cálculo geométrico**. Selecciona una terna para mover
el robot a sus ángulos `q₁, q₂, q₃`. En un punto regular hay cuatro ternas,
pero solo dos formas visibles del brazo; la flecha sobre la base señala el
cambio de `q₁`. En el eje, `q₁` es libre: el CSV guarda `0°` como representante
y el deslizador permite ver otros giros. Los objetivos
inalcanzables muestran **0 soluciones**; la postura límite dibujada ayuda a
visualizar la distancia que falta y no es una solución del objetivo.

Debajo de **Cámara**, el botón **Guía de exposición** abre un panel lateral con
el modelo, la derivación de la inversa, la lectura de singularidades, la comparación de controladores del
escenario seleccionado y la evidencia publicada. Sus cifras provienen del
`resumen.json` de la corrida abierta. El panel distingue los experimentos
propios de las mediciones del artículo. La plataforma, los rodamientos y los
acabados del robot son detalles visuales; las posiciones articulares y del
efector siguen los datos numéricos de la simulación.

## Experimentos y evidencias

| Experimento | Qué se comprueba | Archivos principales |
| --- | --- | --- |
| Directa e inversa | DH frente a la expresión cerrada. Cuatro ramas genéricas de posición, soluciones de borde, eje con base libre y objetivos inalcanzables. | `espacio_ideal.png`, `inversa.csv` |
| Diferencial y singularidades | Diferencias finitas centrales, determinante y valores singulares en forma cerrada contrastados con la SVD, rango con tolerancia relativa, barridos hacia ambas familias y mapas de cercanía. | `singularidades.csv`, `barrido_codo.csv`, `barrido_eje.csv`, `singularidad.png`, `mapa_singularidades.png` |
| Validación documental | Comparación por componente con la Tabla 3 y recálculo de errores de la muestra de la Tabla 4. | `tabla3_auditoria.csv`, `tabla3.png`, `tabla4.png` |
| Control cinemático | Pseudoinversa y mínimos cuadrados amortiguados con referencias e inicialización idénticas. | `control_*.csv`, `control_*.png` |
| Dinámica exploratoria | Positividad y simetría de la masa, conservación de energía o balance con disipación. | `dinamica_*.csv`, `dinamica.png` |

`resultados/resumen.json` contiene las métricas y versiones de la ejecución
incluida. `data/` contiene las transcripciones publicadas y `tests/` las
pruebas numéricas. En el código, las posiciones se expresan en metros y los
ángulos en radianes. Las tablas comparativas del artículo mantienen milímetros.

## Singularidades del jacobiano

El jacobiano traslacional `J(q)` (3×3, en m/rad) relaciona la velocidad
articular con la velocidad del efector, `ṗ = J q̇`. Una configuración es
singular cuando `J` pierde rango: existe una dirección cartesiana a la que el
efector no puede moverse, y la inversión de velocidades exige velocidades
articulares que crecen sin cota. Con `r = L2·cos q2 + L3·cos(q2+q3)`, la
distancia horizontal del efector al eje z, el determinante es

```
det J = −L2·L3·r·sin q3
```

**Valores singulares en forma cerrada.** `J` se factoriza como
`Rz(q1)·[[0, −h, −a], [r, 0, 0], [0, r, b]]`, con
`h = L2·sin q2 + L3·sin(q2+q3)`, `a = L3·sin(q2+q3)` y `b = L3·cos(q2+q3)`.
La primera columna es ortogonal a las otras dos. Por eso los tres valores
singulares se separan en dos grupos:

- `σ_eje = |r|`, asociado a la dirección tangencial `(−sin q1, cos q1, 0)`.
- Los dos valores del brazo plano 2R `[[−h, −a], [r, b]]`. Con
  `T = L2² + 2·L3² + 2·L2·L3·cos q3` y `D = L2·L3·|sin q3|`:

  ```
  σ₊ = √((T + √(T² − 4D²)) / 2)        σ_codo = D / σ₊
  ```

Ambos dependen solo de `q3`, de modo que las dos ramas del codo tienen la
misma `σ` mínima y esta es una función de la posición cartesiana. El
producto de los tres es `|det J|`, la manipulabilidad de Yoshikawa. La función
`singular_values(q)` de `ramzy3r/kinematics.py` implementa estas fórmulas, y
`singularity_measures(q)` añade el rango, el número de condición y la familia
que limita. Frente a la SVD de NumPy, las fórmulas coinciden con una
diferencia máxima de 4.4·10⁻¹⁶ m/rad en las 128 configuraciones aleatorias
del experimento. Las pruebas exigen menos de 10⁻¹⁵ m/rad en otras 2000.

**Familias.** Sin límites articulares, las singularidades son:

| Familia | Condición | Lugar en el espacio | Dirección que se pierde | Comportamiento cercano |
| --- | --- | --- | --- | --- |
| Codo extendido | `q3 = 0` | Esfera exterior de radio `L2+L3 = 0.530 m` centrada en el hombro `(0, 0, L1)` | A lo largo del brazo | `σ_min ≈ 0.1196·|q3|` m/rad |
| Codo plegado | `q3 = π` | Esfera interior de radio `L2−L3 = 0.050 m` | A lo largo del brazo | `σ_min ≈ 0.2839·|q3 − π|` m/rad |
| Eje de la base | `r = 0` | Eje z | Tangencial: girar `q1` no mueve el efector | `σ_min = |r|` exactamente |
| Intersección | `r = 0` y `sin q3 = 0` | Puntos del eje z a 0.530 m o 0.050 m del hombro | Dos direcciones (rango 1) | Ambas a la vez |

Las dos familias de codo forman la frontera del espacio alcanzable. No se
evitan cambiando de rama de la inversa. La familia del eje es interior: allí
`q1` queda libre (`base_libre` en `inversa.csv`). Si se pondera la sección
`(r, z)` por `|r|` para obtener el volumen de revolución, cerca del 1.2 % del
volumen alcanzable tiene `σ_min < 0.020 m/rad`. De ese total, el 0.19 %
corresponde a la cercanía del eje y el 1.0 % a la de las esferas del codo.

**Criterio numérico.** El proyecto usa una sola tolerancia relativa,
`SINGULAR_RTOL = 1e-10`: un valor singular con `σ_i ≤ 1e-10·σ_1` cuenta como
cero. La misma tolerancia decide el rango en `singularidades.csv` y trunca la
pseudoinversa del control. El rango es un veredicto binario que depende de esa
tolerancia. Por eso se reporta también `σ_min`, la menor rapidez cartesiana
que produce una velocidad articular de norma 1 rad/s. Además, se marca como
cercana a la singularidad toda pose con `σ_min < λ = 0.020 m/rad`. En esa
zona, el amortiguamiento del control DLS domina la inversión
`σ/(σ² + λ²)`.

**Por qué la forma cerrada.** La SVD numérica tiene un error absoluto del
orden de `ε·σ_1`, por lo que su error relativo en `σ_min` crece como
`ε·σ_1/σ_min`. En el barrido hacia el codo ese error llega al 1.9 % cuando
`q3 = 10⁻¹⁴` rad, mientras que la fórmula cerrada conserva la pendiente
lineal. En el barrido hacia el eje, la SVD se mantiene exacta (3.6·10⁻¹⁶ de
error relativo) porque la columna de `q1` es ortogonal por estructura. La
fila inferior de `singularidad.png` muestra ambos comportamientos.

**Evidencias.**

- `singularidades.csv` lista, para cada caso de `singular_cases()`, los
  valores singulares analíticos y por SVD, el rango con ambos métodos, la
  manipulabilidad, el número de condición y la familia.
- `barrido_codo.csv` y `barrido_eje.csv` acercan la pose a cada familia,
  desde 0.7 y 0.3 rad hasta 10⁻¹⁴ rad.
- `mapa_singularidades.png` dibuja `σ_min` en el plano `(q2, q3)`, válido
  para cualquier `q1`, y en la sección cartesiana `(r, z)`.
- `resumen.json` reúne estas métricas bajo `cinematica`.

## Trazabilidad y decisiones del modelo

- **Cinemática nominal**. Se parte del DH estándar de la Tabla 1 y las longitudes
  de la Tabla 2, en metros, (0.040, 0.290, 0.240). Se reconstruyen las
  transformaciones y se contrastan con las ecuaciones (6)–(8). Esta construcción
  resuelve inconsistencias de las expresiones impresas (1)–(5), como rotaciones
  con una fila nula o la última fila de una transformación con cero en su
  última entrada. En la inversa se usa la altura relativa al hombro y se
  rederiva la ley de cosenos, tomando (9)–(12) como antecedente.
- **Posición y alcance**. La inversa y el control resuelven la tarea XYZ. Las
  ramas analíticas y el espacio ideal admiten todo el dominio angular y
  omiten límites mecánicos, obstáculos y deformación. Las singularidades
  del jacobiano XYZ se tratan en la sección
  [Singularidades del jacobiano](#singularidades-del-jacobiano).
- **Evidencia medida**. Las 26 filas de la Tabla 3 (§4.2) combinan proyecciones.
  Los blancos se conservan. El MAE del modelo nominal frente a las mediciones
  es 1.95, 2.09 y 6.34 mm en X, Y y Z, con 7, 26 y 19 datos por eje.
  La Tabla 4 (p. 18) aporta una muestra de 26 tiempos, cuyos errores se
  recalculan por separado. La muestra carece de las entradas y estados
  completos necesarios para rehacer el ensayo dinámico.
- **Control propio**. La planta ideal integra la velocidad articular por Euler
  con 10 ms, ganancia 6 s⁻¹ y límite de 0.60 rad/s por eje. La referencia
  cartesiana nace de una trayectoria articular alcanzable. Se usa amortiguamiento
  de 0.020 m/rad para DLS. Además del caso regular y el cercano al codo
  extendido, se ensaya un sesgo suave de medición de hasta 1 mm en X,
  concentrado entre 3.6 y 4.4 s. Se mide el error contra la posición real
  simulada. La comparación se repite a 5, 10 y 20 ms para mostrar la
  sensibilidad del pico de velocidad solicitado al paso.
- **Dinámica reconstruida**. Se calcula la masa por energía cinética,
  `M = sum(m*Jv.T@Jv + Jw.T@I_base@Jw)`, y se construyen gravedad, término
  cuadrático en velocidad, rigidez y disipación con las ecuaciones (13)–(18)
  como referencia. La respuesta libre usa cero par externo. Se supone que
  las inercias CAD son diagonales en los ejes DH del eslabón, que los
  desplazamientos de centro de masa parten de la articulación proximal,
  que Z2 es cero y que el reposo elástico es q=0. Se interpretan las
  constantes 1.66 y 1.3 en N·m/rad y N·m·s/rad respectivamente. Esas unidades
  y marcos requieren confirmación. Los factores 0.5 y 1.5 de rigidez son
  perturbaciones paramétricas, sin asociarles valores de presión. Los
  ensayos publicados corresponden a 2 bar.

## Para discutir con el autor

1. **Geometría y marcos.** ¿La Tabla 3 usa un segundo eslabón de 290 mm o
   de 295 mm? En las filas 1–18, la variante de +5 mm baja el MAE frente
   a la teoría impresa de 3.13 a 0.09 mm en Y y de 3.22 a 0.01 mm en Z.
   El modelo principal conserva 290 mm. ¿Las filas 19–25 se proyectaron
   con un radio fijo de 535 mm? El alcance radial ideal nominal máximo es
   530 mm. Las cifras por sí solas no identifican la causa.
2. **Fila 26.** ¿Qué origen de altura y qué fórmula de error corresponden a
   esa medición? La teoría impresa es 29.92 mm y la medición es 30.27 mm
   en Z. La diferencia absoluta es 0.35 mm, mientras que la columna
   de error indica 0.1225. La sensibilidad del MAE al excluir la fila
   se reporta aparte y la auditoría principal mantiene la fila.
3. **Parámetros dinámicos.** ¿En qué marcos y respecto de qué puntos se
   expresaron los centros de masa y las inercias? ¿Cuál es Z2? ¿Qué
   unidades, posición de reposo y protocolo de identificación corresponden
   a la rigidez y el amortiguamiento? ¿Existe una curva presión–rigidez?
4. **Reproducción del ensayo.** Solicitar el `.slx` de la Figura 3, la
   inicialización MATLAB, el transformador ángulo–par, las referencias
   articulares y las series temporales originales con marcos y unidades.

El balance de energía y el residuo directa–inversa comprueban consistencia
interna. El cotejo con las tablas es parcial. Las simulaciones propias sirven
para discutir hipótesis y sensibilidad. La validación física del controlador
y la réplica del ensayo dinámico requieren información adicional del prototipo.

## Bibliografía

- Ramzy, M. R., Hammad, S., & Maged, S. A. (2024). *Kinematic and Dynamic
  Modeling of 3DOF Variable Stiffness Links Manipulator with Experimental
  Validation*. **Applied Sciences, 14**(12), 5285.
  https://doi.org/10.3390/app14125285
