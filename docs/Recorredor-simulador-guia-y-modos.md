# Recorredor — Guía del Simulador Web y Comparativa de Modos

> Documento de apoyo para `sim/live_server.py` (dashboard web en `http://127.0.0.1:8765`).
> La especificación general del robot está en `Recorredor-spec.md`; la matemática del
> perímetro/área, en `Recorredor-matematicas-perimetro.md`.

---

## 1. Qué es el proyecto

**Recorredor** es un robot móvil (objetivo: ESP32 DevKit V1, dos ruedas + rueda loca,
servo de dirección y sensores de proximidad/impacto) que explora una habitación
cerrada de piso plano:

1. Avanza en línea recta hasta detectar una pared.
2. Frena, registra el punto de impacto y gira un rumbo nuevo.
3. Repite hasta completar N *cuerdas* (tramos rectos de pared a pared).
4. Con los puntos de choque **reconstruye el polígono del perímetro** y estima
   **área** y **perímetro** (además de la estimación estocástica por el
   *Método de las Hormigas* sobre los cruces entre cuerdas).
5. Calcula un **camino interior desplazado** `radio_robot × 1.2` hacia adentro
   (~14.4 cm desde el centro) y **recorre el perímetro completo sin chocar**.
   Si choca en la vuelta, los modos con aprendizaje **reaprenden** el mapa
   incorporando el nuevo impacto.

El simulador reproduce toda esa física tick a tick (50 Hz) en `sim/world.py` +
`sim/core/navigator.py`, y la visualización en vivo la sirve
`sim/live_server.py` con el mapa canvas, la telemetría, el historial de runs
guardados y el dashboard comparativo.

### Cómo se estima el mapa

- **Puntos de choque**: proyección del borde del robot en el instante del
  impacto (`project_wall_contact`).
- **Cuerdas**: cada tramo recto entre dos impactos consecutivos, con su
  longitud, duración y rumbo (`ChordSegment`).
- **Ordenación del contorno**: inserción de costo mínimo sobre el convex hull +
  optimización 2-opt (`order_all_wall_points_boundary`) → el camino cerrado más
  corto que pasa por **todos** los puntos de choque, sin picos ni cruces.
- **Reconstrucción del perímetro**: depende del modo (ver §3).
- **Camino interior**: offset de las aristas del polígono reconstruido,
  recortando lazos en nichos angostos (`compute_inward_offset_polygon`) +
  anclaje de la ruta a la posición real del robot
  (`build_anchored_perimeter_route`) → cero diagonales de enganche.

---

## 2. Los 7 modos de navegación

| # | Modo (`valor`) | Exploración | Geometría del mapa | Reaprendizaje |
|---|----------------|-------------|--------------------|---------------|
| 1 | **Clásico** (`classic`) | Rebotes con rumbo variable | Poligonal libre (sirve paredes diagonales) | No |
| 2 | **Recto** (`recto`) | Rebotes con rumbo variable | **Obligatoriamente 90°**: esquinas ortogonales entre puntos | No |
| 3 | **Reaprendizaje** (`reaprendizaje`) | Rebotes con rumbo variable | Poligonal libre | **Sí**: recalcula si choca en la vuelta |
| 4 | **Recto + Reaprendizaje** (`recto_reaprendizaje`) | Rebotes con rumbo variable | Ortogonal 90° | **Sí** |
| 5 | **Bordeador** (`bordeador`) | En cada choque **sondea ambos lados** y elige el más prometedor | Poligonal libre | Sí |
| 6 | **Testeador** (`testeador`) | **Sigue la pared** y tantea perpendicularmente cada `Paso testeo` | Ortogonal | Sí |
| 7 | **Incidente** (`incidente`) | Rebotes con rumbo variable **+ registra la orientación de cada pared** | Ortogonal **por rectas de pared ajustadas** | Sí |

### 1. Modo Clásico
El comportamiento original: rebota, une los puntos de choque en el orden del
contorno y simplifica colineales. Sirve para habitaciones con paredes
diagonales (pentágonos, trapecios, hexágonos, figuras libres dibujadas).

### 2. Modo Recto
Asume que la habitación sólo tiene paredes a 90°. Entre puntos de choque en
paredes distintas infiere la esquina ortogonal eligiendo el codo que no corta
cuerdas ya recorridas (`reconstruct_orthogonal_polygon`). Al dibujar una figura
personalizada con este modo activo, los trazos se **normalizan a 90° en vivo**.

### 3. Modo Reaprendizaje
Igual que el clásico, pero si durante la vuelta perimetral final el robot toca
una pared que no había muestreado (un rincón escondido), agrega el impacto al
mapa, **reconstruye todo el perímetro y el camino interior** y retoma la vuelta,
hasta `Máx Reaprend.` veces.

### 4. Modo Recto + Reaprendizaje
Combina la inferencia ortogonal del modo 2 con el recálculo del modo 3. Es el
modo por defecto para habitaciones de oficina/hogar rectilíneas.

### 5. Modo Bordeador
Exploración distinta: al chocar, **avanza un tramo de sondeo hacia ambos lados**
del impacto, mide despeje y cobertura inexplorada, y continúa por el lado más
prometedor hasta el próximo choque. Recorre el contorno "a tientas" sin
geometría global previa.

### 6. Modo Testeador
Seguimiento de pared con **testeo periódico**: tras doblar la primera esquina
avanza paralelo a la pared, y cada `Paso testeo (m)` gira 90° hacia ella e
intenta avanzar un sondeo corto. Si detecta pared, registra el punto y sigue;
si no, la pared terminó → avanza apenas `Paso esquina (m)` y gira en U para
reencontrar la continuación. Ambos pasos son configurables en la barra.

### 7. Modo Incidente
Rebote clásico, pero en cada impacto **guarda también la orientación de la
pared** (tangente ⊥ al rumbo, ajustada al eje más cercano). La reconstrucción
(`reconstruct_incidence_polygon`):

1. Ordena los hits por el contorno.
2. Agrupa hits consecutivos con **misma orientación y misma recta** (±9 cm):
   cada grupo ajusta UNA recta de pared real.
3. Los vértices son las **intersecciones de rectas de pared consecutivas**
   (entre paredes paralelas, el par de vértices del escalón).
4. Los choques rasantes que votan la orientación equivocada se absorben por
   posición en la recta vecina; y si el polígono inferido deja algún hit fuera
   del contorno, cae a la reconstrucción ortogonal por puntos.

Obligatoriamente 90° (normaliza figuras dibujadas) y con reaprendizaje.

---

## 3. Comparativa entre modos

| Criterio | Clásico | Recto | Reaprend. | Recto+Reap. | Bordeador | Testeador | Incidente |
|----------|---------|-------|-----------|-------------|-----------|-----------|-----------|
| Figuras con diagonales | ✅ | ❌ | ✅ | ❌ | ✅ | ❌ | ❌ |
| Normaliza figura dibujada a 90° | — | ✅ | — | ✅ | — | ✅ | ✅ |
| Robusto a esquinas ocultas | ❌ | ❌ | ✅ | ✅ | ✅ | ✅ | ✅ |
| Precisión típica en rectángulo/L | alta | **muy alta** | alta | **muy alta** | media-alta | alta | **muy alta** |
| Exploración guiada por pared | ❌ | ❌ | ❌ | ❌ | ✅ | ✅ | ❌ |
| Usa orientación del impacto | ❌ | ❌ | ❌ | ❌ | ❌ | implícita | **✅ explícita** |
| Muestreo de paredes largas | según cuerdas | según cuerdas | según cuerdas | según cuerdas | denso | **muy denso** | según cuerdas |
| Riesgo conocido | esquinas sin muestrear | idem | más choques en vuelta | más choques en vuelta | trayectoria irregular | múltiples sondeos lentos | esquinas inferidas fuera de zona muestreada (con fallback ortogonal) |

**Reglas prácticas de elección:**

- Habitación con **paredes diagonales** → `clásico` o `reaprendizaje`.
- Habitación **rectilínea conocida** → `recto_reaprendizaje` (o `incidente`
  para ajuste por rectas de pared).
- Figura **con recovecos ocultos** que las cuerdas pueden no alcanzar →
  `reaprendizaje`, `recto_reaprendizaje`, `bordeador` o `testeador`.
- **Muestreo denso de paredes** (datos de borde casi continuos) → `testeador`.
- **Evaluar mapas por orientación de impacto** → `incidente`.

---

## 4. Parámetros configurables (barra superior)

| Control | Efecto |
|---------|--------|
| `Figura` | Rectángulo 4×3, habitación en L, pentágono, trapecio, hexágono, octógono, figuras guardadas o `✏️ Dibujar` una personalizada |
| `Modo` | Los 7 modos anteriores |
| `Cuerdas` | Cantidad **exacta** de tramos de exploración (8–120) |
| `Máx Reaprend.` | Límite de recálculos del mapa durante la vuelta final |
| `Variación Rumbo` | `Aleatorio inteligente` \| `Ángulo fijo (°)` \| `Alternar lista de ángulos (°)` |
| `Paso testeo (m)` / `Paso esquina (m)` | Sólo en modo Testeador: distancia entre sondeos y avance mínimo para librar la esquina |
| `▶ Iniciar / Nuevo Run` | Nueva simulación con **semilla aleatoria nueva** y todos los controles releídos |

Cada run reproduce: rumbo de salida aleatorio, N cuerdas exactas, dibujo del
perímetro reconstruido al converger, y la vuelta perimetral verde a
`radio × 1.2` de las paredes.

---

## 5. Historial de runs y reportes

- Columna derecha **📋 Historial de Runs**: cada run terminado (o `📸 Guardar
  Reporte ya`) guarda **foto del mapa**, modo, figura, parámetros y resultados
  en `sim/saved_runs.json` (persiste entre sesiones).
- **Click** en un run → se expande con todas las métricas.
- Botón **⬇️** (o `Descargar reporte como imagen PNG` en el detalle) → compone
  una **tarjeta de reporte** (mapa + métricas + tiempos + precisión) y la
  descarga como **PNG** lista para pegar en Word. Flujo típico: correr varias
  runs por modo, elegir la mejor de cada uno y bajar su imagen.
- Botón **🗑️** → borra ese run puntual; `🗑️ Borrar Todo` limpia el historial.

## 6. Dashboard comparativo

`📊 Ver Dashboard Comparativo` abajo de todo:

- **Filtros**: por figura, por modo y por precisión mínima (p. ej. `100` = sólo
  runs perfectas).
- **Tabla ordenable**: click en cualquier encabezado (tiempo, precisión, error,
  rebotes, distancia, …) ordena asc/desc.
- **Gráfico de barras**: compara una métrica a elección entre runs.
- **Scatter X vs Y**: relaciones entre variables (p. ej. distancia vs error de
  área), coloreado por modo.

---

## 7. Comandos

```powershell
# Simulador web interactivo
python -m sim.live_server --port 8765

# Demo por consola (reporte + matplotlib)
python -m sim.run_demo

# Suite de tests
python -m unittest discover -s sim/tests -p "test_*.py"
```

Estado actual de la suite: **16 tests** cubriendo geometría (wrap, hull,
shoelace, offset, ruta anclada sin diagonales), misión completa en
rectángulo/pentágono/L, exactitud de cuerdas, y los 7 modos (incluido el
Incidente: orientación registrada por impacto, reconstrucción por rectas de
pared y robustez ante hits rasantes).
