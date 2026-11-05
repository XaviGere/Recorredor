# Recorredor — Guía para Agentes

Estas reglas aplican a todo el repositorio `Recorredor`.

## Lectura obligatoria

Antes de realizar cambios relevantes, leer en este orden:

1. `AgentRules.md` — reglas obligatorias de arquitectura, robótica y trabajo.
2. `docs/Recorredor-spec.md` — especificación funcional, cinemática y de navegación; fuente de verdad del sistema.
3. `docs/Recorredor-matematicas-perimetro.md` — fundamentos matemáticos para reconstrucción del perímetro, método de las hormigas (Mallock/Buffon/Cauchy-Crofton) y validación cruzada área/perímetro.
4. `docs/Recorredor-electronica-y-circuito.md` — esquema electrónico, pinout ESP32, etapas de potencia y acondicionamiento de señales.
5. `docs/Recorredor-plan-implementacion.md` — plan maestro de ejecución por fases.
6. Los ADR aceptados en `docs/decisions/`, cuando existan.

## Fuente de verdad

- `docs/Recorredor-spec.md` define qué debe hacer el robot (físico y simulado) y los contratos de datos.
- `AgentRules.md` define cómo se estructura el código entre el Core matemático, el Simulador en Python y el Firmware en C++/Arduino para ESP32.
- `docs/Recorredor-matematicas-perimetro.md` rige las fórmulas de odometría, intersección de segmentos y estimación de área/perímetro.

Si existe una contradicción o una decisión de hardware no cerrada: no asumir silenciosamente. Declarar el supuesto de calibración o solicitar definición.

## Reglas esenciales del dominio robótico

- **Entorno objetivo (Fase 1 / MVP):** Habitación cerrada, piso plano, sin obstáculos internos (mesas/sillas diferidas a Fase futura).
- **Sistema de coordenadas:** Marco global 2D Cartesiano Derecho $(x, y, \theta)$ expresado en **metros ($\text{m}$)** y **radianes ($\text{rad}$)** en el intervalo $(-\pi, \pi]$, con origen $(0, 0, 0)$ en el punto de encendido del robot.
- **Ciclo de exploración por cuerdas (Billiard / Chord Exploration):**
  1. Avanzar en línea recta midiendo tiempo $\Delta t$, velocidad lineal $v$ (por encoders o estimación calibrada) y rumbo $\theta$.
  2. Detectar pared mediante sensor de proximidad frontal (ultrasonido/ToF) o microswitch de impacto (bumper).
  3. Frenar por completo, registrar el **Punto de Contacto** $P_k = (x_k, y_k)$ y el **Segmento/Cuerda** $S_k = \overline{P_{k-1} P_k}$.
  4. Girar un ángulo controlado $\Delta \theta$ (con servo de dirección + diferencial o giro sobre eje) y repetir.
- **Doble validación geométrica-estadística:**
  - **Geometría computacional:** Reconstrucción del polígono del perímetro mediante envolvente convexa (*Convex Hull* / Monotone Chain) o $\alpha$-shape sobre los puntos de impacto $\{P_k\}$, calculando área $A_{\text{poly}}$ y perímetro $L_{\text{poly}}$ con la fórmula de la lazada (*Shoelace formula*).
  - **Método de las Hormigas (Buffon / Mallock / Cauchy-Crofton):** Estimación estocástica del área y perímetro a partir de la longitud de las cuerdas recorridas $\{L_i\}$ y el número de cruces (intersecciones) $C$ entre los segmentos de trayectoria, verificando consistencia ($A_{\text{ants}} \approx A_{\text{poly}}$) antes de dar por cerrado el mapeo.
- **Fase final de recorrido limpio:** Una vez convergente el mapa del perímetro, el robot genera un camino interior desplazado hacia adentro una distancia de seguridad $d_{\text{safe}} > R_{\text{robot}} + \epsilon_{\text{odom}}$ (*inward polygon offset*) y recorre todo el perímetro cerrado **sin chocar**.
- **Calibración física obligatoria:** El hardware real nunca es ideal; todo parámetro físico (pulsos por vuelta, radio de rueda, distancia entre ejes, sesgo del giroscopio, offset del servo, velocidad de sonido) debe estar parametrizado y expuesto como constante de calibración, nunca hardcodeado en medio de una fórmula.

## Arquitectura obligatoria

```text
+-------------------------------------------------------------------+
|                   CAPA DE PRESENTACIÓN / TELEMETRÍA               |
|  Visualizador 2D Matplotlib/Tk (Python)  |  Monitor Serial JSON   |
+-------------------------------------------------------------------+
                                   |
                                   v
+-------------------------------------------------------------------+
|                   CORE DE NAVEGACIÓN Y MAPEO                      |
|  PoseEstimator (Odometría + IMU) | ChordMapper (Puntos y Cruces)  |
|  PerimeterEstimator (Hull + Hormigas) | PathFollower (PurePursuit)|
+-------------------------------------------------------------------+
                 |                                   |
                 v                                   v
+---------------------------------+ +-------------------------------+
|   HAL SIMULADO (Python)         | |   HAL HARDWARE (ESP32 C++)    |
|  Entorno Poligonal 2D + Raycast | |  Motores DC PWM + Servo Dir   |
|  Ruido Gaussiano + Colisiones   | |  HC-SR04/VL53L0X + Bumpers    |
|                                 | |  MPU6050 IMU + Encoders       |
+---------------------------------+ +-------------------------------+
```

- Mantener paridad algorítmica entre `sim/core/` (Python) y `firmware/recorredor_esp32/` (C++/Arduino).
- El Core matemático no depende de `matplotlib` ni de registros hardware específicos de ESP32 (`ledcWrite`, `digitalRead` van detrás de funciones/clases HAL).
- El loop de control en el ESP32 debe ser **no bloqueante** (basado en `millis()` / `micros()`, prohibido usar `delay()` largos durante el movimiento que impidan leer el bumper de impacto).

## Seguridad electrónica y mecánica

- Nunca alimentar motores DC ni servomotores desde el pin `3V3` del ESP32.
- El pin `ECHO` del HC-SR04 (si se alimenta a $5\,\text{V}$) entrega $5\,\text{V}$: **obligatorio** divisor resistivo ($1\,\text{k}\Omega / 2\,\text{k}\Omega$) o desplazador de nivel hacia el GPIO del ESP32 (tolerante solo a $3.3\,\text{V}$).
- Los microswitches de parachoques (*bumpers*) tienen prioridad máxima de interrupción/frenado sobre cualquier comando de avance.

## Comandos del proyecto

Ejecutar desde la raíz de `Recorredor/`:

```powershell
# Ejecutar suite de tests matemáticos y de simulación
python -m unittest discover -s sim/tests -p "test_*.py" -v

# Ejecutar simulación completa (headless con reporte y generación de gráfica PNG)
python -m sim.run_demo --headless --save-plot sim_output.png

# Ejecutar simulación interactiva / animada
python -m sim.run_demo
```

## Definition of Done

Una tarea está terminada cuando:

- Cumple `docs/Recorredor-spec.md` y `AgentRules.md`.
- Conserva unidades SI ($\text{m}$, $\text{rad}$, $\text{s}$) en todo el Core.
- Pasa todos los tests unitarios y de simulación en `sim/tests/`.
- Si modifica el algoritmo de control o mapeo, mantiene sincronizados el simulador Python y el firmware ESP32.
- Documenta cualquier parámetro físico o supuesto nuevo.
