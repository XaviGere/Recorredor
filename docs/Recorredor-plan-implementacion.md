# Plan Maestro de Implementación — Proyecto `Recorredor`

Este documento detalla la hoja de ruta de ingeniería de principio a fin para llevar `Recorredor` desde los fundamentos matemáticos y la simulación en Python hasta el robot físico operativo con ESP32.

---

## Fase 0 — Arquitectura, Gobernanza y Modelos Matemáticos (Completada)
**Objetivo:** Establecer las reglas del proyecto, los modelos matemáticos formales y el diseño electrónico antes de escribir código de control.

- [x] Creación de `AGENTS.md`, `AgentRules.md` y estructura modular del proyecto.
- [x] Especificación funcional y de máquina de estados (`docs/Recorredor-spec.md`).
- [x] Desarrollo matemático de odometría RK2, Método de las Hormigas (Mallock / Buffon-Crofton), envolvente poligonal e Inward Polygon Offset (`docs/Recorredor-matematicas-perimetro.md`).
- [x] Diseño del esquema electrónico, BOM y pinout seguro para ESP32 (`docs/Recorredor-electronica-y-circuito.md`).
- [x] Registro de Decisiones de Arquitectura (`ADR-0001`, `ADR-0002`, `ADR-0003`).

---

## Fase 1 — Motor Geométrico y Estimadores Estadísticos (Core Python)
**Objetivo:** Implementar las primitivas matemáticas puras sin dependencias externas y validarlas con tests unitarios deterministas.

1. **`sim/core/geometry.py`:**
   - Tipos `Point2D`, `ChordSegment`, `Pose2D`.
   - Intersección exacta de segmentos y conteo incremental de cruces no adyacentes.
   - Algoritmo *Monotone Chain* para envolvente convexa + simplificación de vértices colineales en paredes + validación de visibilidad de cuerdas.
   - Fórmula de la lazada (*Shoelace*) para área y perímetro geométrico.
   - Cálculo de **Inward Offset Polygon** mediante bisectrices interiores a distancia $d_{\text{safe}}$.
2. **`sim/core/estimators.py`:**
   - Estimador de **Mallock (2 pasadas)** y **Estimador Generalizado de Hormigas por Pares de Cuerdas** ($\hat{A}_{\text{ants}}$).
   - Estimador de **Cauchy-Crofton** ($\bar{l} = \pi A / P$) y perímetro combinado $\hat{P}_{\text{ants+crofton}}$.
   - Detector de convergencia multi-criterio (estabilidad de área + cobertura angular + consistencia Hormigas/Polígono).

---

## Fase 2 — Simulador Físico 2D, Odometría y Navegador Perimetral
**Objetivo:** Simular el robot real moviéndose en distintos recintos cerrados, tomando tiempos, velocidades y direcciones, construyendo el mapa virtual y recorriendo el perímetro final sin chocar.

1. **`sim/core/odometry.py`:**
   - Integrador cinemático por punto medio $(v, \omega, \Delta t) \rightarrow (x, y, \theta)$ con soporte de ruido de calibración configurable.
2. **`sim/core/navigator.py`:**
   - Máquina de estados completa (`IDLE` $\rightarrow$ `EXPLORE_FWD` $\rightarrow$ `BRAKE_RECORD` $\rightarrow$ `EVALUATE_MAP` $\rightarrow$ `TURN_HEADING` $\rightarrow$ `BUILD_SAFE_PATH` $\rightarrow$ `FOLLOW_PERIMETER` $\rightarrow$ `COMPLETED`).
   - Estrategia de giro ergódico (basado en ángulo áureo + reflexión hacia el interior) para cubrir paredes y esquinas rápidamente.
   - Controlador *Pure Pursuit* + alineación en esquinas para el recorrido perimetral libre de colisiones.
3. **`sim/world.py` y `sim/visualizer.py`:**
   - Mundo 2D con paredes poligonales arbitrarias, raycasting para el sensor de proximidad frontal y detección de colisión circular del chasis.
   - Generador de gráficos multipanel (PNG/interactivo):
     - Panel 1: Habitación real, rastro de las hormigas (cuerdas), cruces detectados, puntos de impacto, perímetro reconstruido y recorrido final sin choque.
     - Panel 2: Convergencia paso a paso del Área (Real vs. Polígono vs. Método de las Hormigas).
     - Panel 3: Convergencia del Perímetro (Real vs. Polígono vs. Cauchy-Crofton).

---

## Fase 3 — Firmware C++/Arduino para ESP32
**Objetivo:** Traducir el Core validado a C++ embebido de memoria estática para el ESP32, listo para flashear en la placa.

1. **`firmware/recorredor_esp32/config.h`:**
   - Definición de pines (según `docs/Recorredor-electronica-y-circuito.md`) y constantes físicas de calibración.
2. **`firmware/recorredor_esp32/hal_esp32.h`:**
   - ISRs en `IRAM_ATTR` para encoders cuadratura y microswitches de bumper.
   - Lectura no bloqueante de ultrasonido HC-SR04 e integración de giroscopio MPU6050 por I2C.
   - Control PWM (`ledcWrite`) de motores DC (TB6612FNG) y servomotor de dirección.
3. **`firmware/recorredor_esp32/geometry_fixed.h` y `navigator_fsm.h`:**
   - Implementación en arreglos estáticos (`MAX_CHORDS = 128`) de la intersección de cuerdas, estimador de hormigas, Convex Hull, Inward Offset y seguidor de waypoints.
4. **`firmware/recorredor_esp32/recorredor_esp32.ino`:**
   - Loop de control determinista a $50\,\text{Hz}$ ($20\,\text{ms}$) y emisión de telemetría JSON por puerto serie.

---

## Fase 4 — Pruebas en Hardware Real y Calibración en Banco (Siguiente Paso Físico)
**Objetivo:** Puesta en marcha del prototipo electrónico sobre banco y piso real.

1. **Prueba de Sensores en Vacío (Chasis sobre soporte):**
   - Verificar polaridad de encoders (girar rueda hacia adelante debe incrementar `ticks`).
   - Verificar centrado del servomotor en $90^\circ$ mecánicos y sentido de giro.
   - Verificar que al presionar cualquiera de los dos bumpers los motores corten PWM instantáneamente.
2. **Calibración de Odometría Rectilínea y Giro:**
   - Hacer avanzar el robot exactamente $1.00\,\text{m}$ junto a una cinta métrica y ajustar `WHEEL_RADIUS_M` / `TICKS_PER_REV`.
   - Hacer girar el robot $360^\circ$ y verificar la escala del giroscopio MPU6050.
3. **Ejecución en Recinto Cerrado Real:**
   - Colocar una caja/corral rectangular de prueba (ej. $2\,\text{m} \times 1.5\,\text{m}$) o habitación despejada, capturar la salida Serial JSON desde la PC y visualizar el mapa reconstruido.

---

## Fase 5 — Evolución Futura (Obstáculos Internos: Mesas y Sillas)
*(Diferida según especificación inicial, arquitectura ya preparada para soportarla)*
- Clasificación de impactos en **Frontera Exterior** vs **Obstáculos Internos (Islas)** mediante agrupación espacial (*clustering* DBSCAN/distancia) y análisis topológico de cuerdas que rodean un obstáculo.
- Sustitución de *Convex Hull* por **Grilla de Ocupación Probabilística (Log-Odds Occupancy Grid)** + extracción de contornos con grafo de visibilidad.
