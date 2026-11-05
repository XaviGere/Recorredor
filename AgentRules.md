# Recorredor — Reglas de Ingeniería y Arquitectura (AgentRules)

## 1. Propósito del Proyecto

`Recorredor` es un sistema robótico autónomo de exploración geométrica y mapeo perimetral para interiores planos. Su objetivo es descubrir la forma, el perímetro y el área de un recinto cerrado desconocido mediante rebotes controlados (exploración por cuerdas), validar la reconstrucción mediante geometría computacional y métodos estocásticos bio-inspirados (*Método de las Hormigas* / Cauchy-Crofton), y finalmente recorrer el perímetro completo sin colisionar.

---

## 2. Fuente de Verdad y Jerarquía Documental

1. `docs/Recorredor-spec.md` — Especificación funcional y técnica completa.
2. `docs/Recorredor-matematicas-perimetro.md` — Modelos matemáticos de odometría, intersección de cuerdas, estimadores de área/perímetro y navegación.
3. `docs/Recorredor-electronica-y-circuito.md` — Diseño electrónico, BOM, pinout y protecciones eléctricas.
4. `AgentRules.md` — Reglas arquitectónicas y de implementación.
5. `docs/decisions/ADR-*.md` — Decisiones arquitectónicas puntuales.

---

## 3. Convenciones Físicas y Matemáticas Obligatorias

### 3.1 Unidades del Sistema Internacional (SI)
Todo cálculo interno del Core (tanto en Python como en C++) debe operar exclusivamente en:
- **Posición / Distancia:** metros ($\text{m}$). (Las lecturas de sensores en $\text{cm}$ o $\text{mm}$ se convierten a $\text{m}$ inmediatamente en la capa HAL).
- **Ángulos:** radianes ($\text{rad}$), normalizados siempre al intervalo $(-\pi, \pi]$ mediante `wrap_angle(theta)`.
- **Tiempo:** segundos ($\text{s}$) en punto flotante para integración cinemática, derivados de relojes monotónicos (`micros()` en ESP32, `dt` explícito en simulación).
- **Velocidad lineal:** metros por segundo ($\text{m/s}$).
- **Velocidad angular:** radianes por segundo ($\text{rad/s}$).

### 3.2 Sistema de Coordenadas
- **Marco Global (World Frame $W$):** Origen $(0, 0)$ en la posición inicial del robot al iniciar la misión. Eje $+X$ apunta en la dirección inicial de avance del robot ($\theta_0 = 0$). Eje $+Y$ apunta a la izquierda del robot (regla de la mano derecha en 2D).
- **Marco Local del Robot (Body Frame $B$):** Origen en el centro cinemático del eje motriz. $+x_B$ hacia el frente del chasis, $+y_B$ hacia la izquierda.

---

## 4. Arquitectura de Capas (Simulador y Firmware)

El sistema se divide estrictamente en tres capas para que la inteligencia geométrica sea idéntica en la PC y en el microcontrolador:

1. **HAL (Hardware Abstraction Layer) / Mundo Simulado:**
   - Lee sensores crudos (pulsos de encoder, eco ultrasónico, estado de bumpers, acelerómetro/giroscopio IMU).
   - Aplica control de actuadores (PWM a puente H para motores DC, ángulo PWM para servomotor de dirección).
   - En Python (`sim/world.py`), esta capa simula la física 2D, el raycasting contra las paredes del recinto y el ruido de sensores.
2. **Core (Dominio Robótico y Matemático):**
   - **Odometría (`PoseEstimator`):** Integra $(\Delta s, \Delta \theta)$ o $(v, \omega, \Delta t)$ para mantener la pose estimada $\hat{\mathbf{x}} = (x, y, \theta)$.
   - **Mapeador de Cuerdas (`ChordMapper`):** Registra cada tramo recto entre dos eventos de pared/impacto como una cuerda dirigida $S_k = (P_{k-1}, P_k)$ y calcula en tiempo real los cruces con las cuerdas previas.
   - **Estimador Perimetral (`PerimeterEstimator`):**
     - Calcula la envolvente poligonal de los puntos de contacto $\{P_k\}$ y su área/perímetro geométrico.
     - Calcula el área y perímetro por el **Método de las Hormigas (Mallock / Buffon-Crofton)** usando longitudes de cuerdas e intersecciones.
     - Evalúa el criterio de convergencia para decidir cuándo el mapa está completo.
   - **Planificador y Seguidor (`PerimeterNavigator`):** Genera el polígono interior seguro (*offset polygon* a distancia $d_{\text{safe}}$ de las paredes) y ejecuta el seguimiento de waypoints (*Pure Pursuit* / control proporcional de rumbo) para dar la vuelta completa sin chocar.
3. **Aplicación / Máquina de Estados Finita (FSM):**
   - Coordina las transiciones entre `IDLE`, `EXPLORE_FORWARD`, `BRAKE_AND_RECORD`, `TURN_NEW_HEADING`, `BUILD_MAP`, `FOLLOW_PERIMETER`, `COMPLETED` y `EMERGENCY_STOP`.

---

## 5. Reglas Críticas de Firmware (ESP32)

1. **Prohibición de `delay()` bloqueante en navegación:** Durante el estado `EXPLORE_FORWARD` o `FOLLOW_PERIMETER`, el bucle `loop()` debe correr a $\ge 50\,\text{Hz}$ ($\Delta t \le 20\,\text{ms}$) sin bloqueos para garantizar que un impacto en el bumper detenga los motores en menos de $20\,\text{ms}$.
2. **Interrupciones seguras (`IRAM_ATTR`):** Las rutinas de interrupción (ISR) para encoders ópticos/magnéticos y bumpers en el ESP32 deben usar el atributo `IRAM_ATTR`, modificar variables `volatile` con secciones críticas (`portENTER_CRITICAL_ISR` si aplica) y no invocar nunca `Serial.print` ni funciones lentas dentro de la ISR.
3. **Memoria estática acotada:** En el ESP32 no usar asignación dinámica descontrolada (`new` / `malloc` en bucle). Definir un buffer circular o arreglo estático de tamaño máximo conocido (`MAX_WALL_POINTS = 128`, `MAX_CHORDS = 128`) suficiente para mapear una habitación con holgura en los $520\,\text{KB}$ de SRAM.
4. **Calibración explícita:** Todos los coeficientes físicos (`WHEEL_RADIUS_M`, `TICKS_PER_REV`, `WHEELBASE_M`, `SERVO_CENTER_DEG`, `STOP_DISTANCE_M`, `SAFE_MARGIN_M`) deben residir en un archivo de configuración único (`config.h` en C++, `config.py` en Python).

---

## 6. Reglas de Simulación y Validación (Python)

1. **Cero dependencias pesadas innecesarias:** El Core en `sim/core/` debe usar únicamente la librería estándar de Python (`math`, `dataclasses`, `typing`, `random`). `matplotlib` o `numpy` son opcionales y exclusivos de la capa de visualización (`sim/visualizer.py`).
2. **Reproducibilidad determinista:** Toda simulación con componente aleatorio (ángulos de rebote o ruido de sensores) debe aceptar una semilla (`seed: int`) para que los tests automatizados sean 100 % deterministas.
3. **Verificación de no-colisión final:** El test de aceptación principal de la simulación debe comprobar que, durante la fase `FOLLOW_PERIMETER`, el robot completa el circuito cerrado ($360^\circ$ alrededor del centroide del salón) manteniendo una distancia a las paredes reales mayor al radio del robot ($d_{\min} > R_{\text{robot}}$) y con **cero colisiones**.
