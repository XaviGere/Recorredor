# Especificación Funcional y Técnica — Proyecto `Recorredor`

> **Versión:** 1.0  
> **Estado:** Aprobado para implementación MVP (Habitación vacía de piso plano)  
> **Plataformas:** Simulador Python 3.x + Firmware C++/Arduino sobre ESP32

---

## 1. Visión y Alcance del Sistema

`Recorredor` es un robot móvil terrestre autónomo diseñado para descubrir la geometría perimetral de un recinto cerrado desconocido (por ejemplo, una habitación de piso plano sin muebles internos en su Fase 1).

El robot no utiliza LIDAR rotativo costoso ni cámaras de visión artificial; en su lugar, resuelve el problema de mapeo mediante **odometría cinemática + exploración por cuerdas (rebotes controlados)** y **estimación matemática dual**:

1. **Mapeo por puntos de frontera y trazos rectilíneos:** El robot avanza en línea recta tomando muestras de tiempo $\Delta t$, velocidad $v$ y dirección $\theta$ hasta detectar o tocar una pared. En ese instante frena, registra la coordenada $(x_k, y_k)$ del borde y el segmento recorrido (cuerda $S_k$), gira un ángulo determinado $\Delta\theta$ y vuelve a arrancar.
2. **Validación cruzada (Geometría + Método de las Hormigas):** A medida que acumula trazos y cruces entre sus propias trayectorias, calcula tanto la envolvente poligonal de los impactos como el estimador estocástico de **Mallock / Buffon / Cauchy-Crofton** (el mecanismo biológico que usan las hormigas *Leptothorax albipennis* para medir el área de una cavidad según la longitud de sus rastros y el número de intersecciones).
3. **Recorrido perimetral limpio:** Cuando la estimación del mapa converge y las métricas geométrica y estocástica son consistentes, el robot sintetiza un **polígono interior de seguridad** (desplazado hacia adentro una distancia $d_{\text{safe}}$) y recorre todo el perímetro de la habitación de principio a fin **sin chocar con ninguna pared**.

---

## 2. Supuestos del Entorno (Fase 1 — MVP)

1. **Piso plano y horizontal:** El movimiento ocurre en $\mathbb{R}^2$ ($z = 0$, inclinación despreciable).
2. **Recinto cerrado simple:** Las paredes forman un polígono cerrado acotado (inicialmente convexo o cuasi-convexo, ej. rectangular, trapezoidal, pentagonal o salón con esquinas achaflanadas).
3. **Sin obstáculos internos en Fase 1:** No hay patas de mesas o sillas en el interior del salón (el soporte para "islas" internas se documenta como extensión de Fase 2 mediante clasificación de ciclos interiores).
4. **Coeficiente de fricción suficiente:** Las ruedas tienen tracción sobre el piso liso sin patinamiento severo cuando las aceleraciones están limitadas por rampa PWM.

---

## 3. Arquitectura Física del Robot

El diseño soporta dos variantes mecánicas configurables mediante software (ver `ADR-0001`):

- **Modo Híbrido Diferencial + Servo Direccional / Torreta (Recomendado):**
  - Dos ruedas traseras/centrales con motores DC con caja reductora y encoders (tracción diferencial capaz de girar sobre su propio eje o en arco) + un **servomotor SG90/MG90S** que orienta el eje direccional delantero y/o el cabezal del sensor de proximidad para escanear el ángulo libre antes de girar el chasis.
- **Modo Triciclo / Ackermann con Servo de Dirección:**
  - Tracción trasera con motor DC + dirección delantera comandada por servomotor ($\phi \in [-35^\circ, +35^\circ]$), realizando maniobras de retroceso + giro al llegar a una pared.

### Dimensiones de referencia del chasis
- Radio envolvente de seguridad del robot: $R_{\text{robot}} = 0.12\,\text{m}$ ($12\,\text{cm}$).
- Distancia de detección de frenado (proximidad): $d_{\text{stop}} = 0.08\,\text{m}$ ($8\,\text{cm}$ frente al parachoques, o impacto directo $0.00\,\text{m}$ con microswitch).
- Margen de navegación perimetral sin choque: $d_{\text{safe}} = 0.25\,\text{m}$ ($25\,\text{cm}$ hacia el interior de la pared detectada).

---

## 4. Máquina de Estados Finita (FSM) de Misión

```text
                  +------------------+
                  |      0. IDLE     |
                  +--------+---------+
                           | start_mission()
                           v
                  +------------------+
         +------->| 1. EXPLORE_FWD   |<-----------------+
         |        +--------+---------+                  |
         |                 | wall_detected / bumper_hit |
         |                 v                            |
         |        +------------------+                  |
         |        | 2. BRAKE_RECORD  |                  |
         |        +--------+---------+                  |
         |                 | registra P_k, cuerda S_k   |
         |                 | actualiza cruces y mapa    |
         |                 v                            |
         |        +------------------+                  |
         |        | 3. EVALUATE_MAP  |                  |
         |        +---+----------+---+                  |
         |   incompleto|          | mapa convergente     |
         |             v          v                     |
         |  +----------------+  +--------------------+  |
         |  | 4. TURN_HEADING|  | 5. BUILD_SAFE_PATH |  |
         |  +--------+-------+  +---------+----------+  |
         |           | giro completado    | waypoints   |
         +-----------+                    v             |
                                +--------------------+  |
                                | 6. FOLLOW_PERIMETER|--+ (si emergencia,
                                +---------+----------+     corrige)
                                          | vuelta 360° completada
                                          v
                                +--------------------+
                                |   7. COMPLETED     |
                                +--------------------+
```

### Descripción detallada de cada estado

1. **`IDLE`:** Motores detenidos, calibración inicial de offset del giroscopio (IMU en reposo durante $1\,\text{s}$), inicialización de pose $(x_0=0, y_0=0, \theta_0=0)$.
2. **`EXPLORE_FWD`:**
   - El servomotor de dirección/sensor se fija en el centro ($0\,\text{rad}$ relativo al chasis).
   - Los motores avanzan a velocidad crucero constante $v_{\text{ref}}$ (ej. $0.25\,\text{m/s}$) manteniendo el rumbo $\theta_{\text{target}}$ mediante un lazo PID/P sobre el ángulo del giroscopio/encoders.
   - En cada tick ($\Delta t = 20\,\text{ms}$), se actualiza la pose:
     $$x(t+\Delta t) = x(t) + v \cos(\theta)\,\Delta t, \quad y(t+\Delta t) = y(t) + v \sin(\theta)\,\Delta t$$
   - Se monitorea continuamente el sensor de distancia frontal y los bumpers físicos.
3. **`BRAKE_RECORD`:**
   - Al activarse el umbral de proximidad ($d_{\text{sensor}} \le d_{\text{stop}}$) o el microswitch de impacto:
   - Se aplica freno activo inmediato a los motores y se espera asentamiento ($100\,\text{ms}$).
   - Se proyecta la coordenada exacta de la pared sumando el offset desde el centro del robot hasta el punto de sensado/contacto en la dirección $\theta$:
     $$P_{\text{wall}, k} = \left(x + (R_{\text{robot}} + d_{\text{medida}})\cos\theta,\; y + (R_{\text{robot}} + d_{\text{medida}})\sin\theta\right)$$
   - Se almacena el vértice $P_{\text{wall}, k}$ y el segmento de cuerda $S_k = \overline{P_{\text{robot}, k-1} P_{\text{robot}, k}}$.
   - Se calculan todas las intersecciones geométricas de $S_k$ con las cuerdas anteriores $\{S_1, \dots, S_{k-2}\}$.
4. **`EVALUATE_MAP`:**
   - Si el número de impactos $k \ge N_{\min}$ (por defecto $N_{\min} = 14$):
     - Se construye el polígono envolvente $\mathcal{H}_k$ de $\{P_{\text{wall}, 1}, \dots, P_{\text{wall}, k}\}$.
     - Se calcula el área geométrica $A_{\text{poly}}$ y el perímetro $L_{\text{poly}}$.
     - Se calcula la estimación por el **Método de las Hormigas / Buffon-Crofton** ($A_{\text{ants}}, \hat{L}_{\text{crofton}}$).
     - **Criterio de parada / convergencia:**
       1. Variación relativa del área poligonal en los últimos $M=4$ rebotes menor a $\epsilon_{\text{conv}} = 3\%$:
          $$\frac{|A_{\text{poly}}^{(k)} - A_{\text{poly}}^{(k-M)}|}{A_{\text{poly}}^{(k)}} < 0.03$$
       2. Cobertura angular de los impactos desde el centroide en todos los sectores (mínimo 8 sectores de $45^\circ$ visitados).
       3. Consistencia entre el área geométrica y el estimador de hormigas dentro de la banda de tolerancia estadística, o límite máximo de cuerdas alcanzado ($k = N_{\max}$).
5. **`TURN_HEADING`:**
   - Si el robot tocó con el bumper, retrocede $d_{\text{backup}} = 0.05\,\text{m}$ para despegarse de la pared.
   - Usa el servomotor para barrido rápido izquierda/derecha o calcula un giro cuasi-uniforme (usando paso áureo $\Delta\theta \approx 137.5^\circ + \text{ruido}$ o reflexión especular perturbada hacia el interior del recinto) que garantiza exploración ergódica de toda la habitación sin caer en ciclos repetitivos.
6. **`BUILD_SAFE_PATH`:**
   - Simplifica los vértices colineales del polígono perimetral detectado (algoritmo Ramer-Douglas-Peucker o fusión angular).
   - Calcula el **polígono interior desplazado (Inward Offset Polygon)** moviendo cada arista hacia adentro una distancia $d_{\text{safe}} = 0.25\,\text{m}$ e intersectando las rectas adyacentes en las bisectrices de cada esquina.
   - Ordena los waypoints resultantes $\{W_1, W_2, \dots, W_m, W_1\}$ en sentido antihorario (CCW) comenzando por el más cercano a la posición actual del robot.
7. **`FOLLOW_PERIMETER`:**
   - El robot navega secuencialmente hacia cada waypoint $W_j$ utilizando un controlador de seguimiento de camino (*Pure Pursuit* + control de rumbo con servo y motores).
   - Mientras recorre el perímetro, mantiene activo el sensor de proximidad como guarda de seguridad.
   - Al cerrar el lazo de vuelta al primer waypoint $W_1$, pasa a `COMPLETED`.
8. **`COMPLETED`:**
   - Detiene los motores, emite la telemetría final con el mapa completo y permanece en espera.

---

## 5. Contratos de Datos y Telemetría

Tanto el simulador Python como el ESP32 emiten tramas estructuradas en formato JSON por línea (JSONL a $115200\,\text{bps}$ en UART/WiFi) para graficar el estado en tiempo real:

```json
{
  "t_ms": 14520,
  "state": "BRAKE_RECORD",
  "pose": {"x": 2.881, "y": 1.420, "theta": 0.643},
  "chord_idx": 12,
  "wall_hit": {"x": 3.000, "y": 1.500},
  "crossings_total": 29,
  "total_distance_m": 34.65,
  "estimates": {
    "poly_area_m2": 11.84,
    "poly_perimeter_m": 13.92,
    "ants_mallock_area_m2": 11.51,
    "crofton_ratio_area_over_perim": 0.848,
    "converged": false
  }
}
```

---

## 6. Requisitos No Funcionales y Métricas de Éxito

1. **Precisión del Perímetro Estimado:** En simulación con ruido de odometría moderado, el error relativo del perímetro final $L_{\text{poly}}$ frente al perímetro real de la habitación debe ser $< 5\%$.
2. **Precisión del Área (Geométrica vs. Hormigas):** La discrepancia entre el área del polígono reconstruido $A_{\text{poly}}$ y el estimador de las hormigas $A_{\text{ants}}$ tras $N \ge 25$ cuerdas debe situarse dentro del margen teórico $\le 15\%$ (convergiendo como $\mathcal{O}(1/\sqrt{N})$).
3. **Cero Colisiones en Recorrido Final:** Durante el estado `FOLLOW_PERIMETER`, el número de impactos con las paredes debe ser exactamente $0$.
4. **Frecuencia de Control:** Mínimo $50\,\text{Hz}$ ($\Delta t = 20\,\text{ms}$) en el bucle de odometría y seguridad tanto en Python como en ESP32.
