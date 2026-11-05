# Diseño Electrónico, Circuito de Conexiones y BOM — `Recorredor` (ESP32)

Este documento especifica la arquitectura de hardware, lista de materiales (BOM), asignación de pines (Pinout) del **ESP32 DevKit V1 (30/38 pines)**, esquema eléctrico de interconexión y protecciones electrónicas obligatorias.

---

## 1. Lista de Materiales (BOM)

| Ítem | Componente | Modelo Recomendado | Función en `Recorredor` |
| :--- | :--- | :--- | :--- |
| 1 | Microcontrolador Principal | **ESP32-WROOM-32 DevKit V1** | Procesamiento a $240\,\text{MHz}$ con FPU float, PWM por hardware (`LEDC`), interrupciones en todos los GPIOs y telemetría WiFi/Serial. |
| 2 | Driver de Motores DC (Puente H) | **TB6612FNG** (preferido) o **L298N** | Control bidireccional y de velocidad (PWM) para los 2 motores de tracción. El TB6612FNG usa MOSFETs con caída de tensión mínima ($<0.3\,\text{V}$ vs $2\,\text{V}$ del L298N). |
| 3 | Motores de Tracción (x2) | **Motores DC N20 Metal (6V, 150-200 RPM) con Encoder Magnético Hall** (o TT Amarillos + discos ópticos LM393) | Movimiento lineal y diferencial + medición precisa de distancia recorrida $\Delta s$. |
| 4 | Servomotor de Dirección / Escaneo | **MG90S** (engranajes metálicos) o **SG90** | Orienta el eje direccional delantero y/o el sensor ultrasónico para seleccionar el nuevo vector de salida. |
| 5 | Sensor de Proximidad Frontal | **HC-SR04** (Ultrasonido) o **VL53L0X** (Láser ToF I2C) | Detección anticipada de la pared a $5-15\,\text{cm}$ para frenado suave sin golpe brusco. |
| 6 | Sensores de Impacto / Parachoques (x2) | **Microswitches KW11-3Z con palanca** (Izquierdo y Derecho) | Detección física redundante de contacto con pared (prioridad de interrupción). |
| 7 | Unidad de Medición Inercial (IMU) | **MPU6050** (Giroscopio + Acelerómetro 6-DOF por I2C) | Medición directa de velocidad angular $\omega_z$ e integración precisa del ángulo de giro $\theta$ (evita el error por patinamiento de ruedas). |
| 8 | Alimentación | **2x Baterías Li-Ion 18650 en serie ($7.4\,\text{V}$ nominal, $8.4\,\text{V}$ carga completa) + BMS 2S** | Fuente de energía primaria con capacidad $\ge 2500\,\text{mAh}$. |
| 9 | Regulador Step-Down (Buck) | **LM2596 o MP1584EN ajustado a $5.0\,\text{V}$ ($3\,\text{A}$)** | Alimenta la línea de $5\,\text{V}$ (pin `VIN` del ESP32, servomotor MG90S y sensor HC-SR04) aislando el ruido de los motores. |
| 10 | Pasivos de Protección | **Resistencias $1\,\text{k}\Omega$ y $2\,\text{k}\Omega$** + Capacitor electrolítico $470\,\mu\text{F}/16\text{V}$ | Divisor de tensión $5\,\text{V} \rightarrow 3.3\,\text{V}$ para el pin `ECHO` del HC-SR04 y filtrado de picos de corriente del servo. |

---

## 2. Tabla de Asignación de Pines (Pinout ESP32)

> **Regla de diseño ESP32:** Se evitan los pines de *strapping* conflictivos en el arranque (`GPIO 0`, `GPIO 2`, `GPIO 12`, `GPIO 15`) y los pines internos de la memoria Flash (`GPIO 6` a `GPIO 11`). Los pines `GPIO 34, 35, 36, 39` son **solo entrada** (*Input Only*) y no tienen resistencias pull-up internas, por lo que se destinan a los canales de encoders que ya cuentan con salida activa de los sensores Hall.

| Subsistema | Señal del Componente | Pin ESP32 | Dirección | Notas Eléctricas / Hardware |
| :--- | :--- | :--- | :--- | :--- |
| **Motor Izquierdo (A)** | `PWMA` (Velocidad) | `GPIO 25` | Salida PWM | Canal LEDC 0 ($20\,\text{kHz}$, 8 bits) |
| | `AIN1` (Dirección 1) | `GPIO 26` | Salida Digital | Lógica $3.3\,\text{V}$ hacia TB6612FNG |
| | `AIN2` (Dirección 2) | `GPIO 27` | Salida Digital | Lógica $3.3\,\text{V}$ hacia TB6612FNG |
| **Motor Derecho (B)** | `PWMB` (Velocidad) | `GPIO 32` | Salida PWM | Canal LEDC 1 ($20\,\text{kHz}$, 8 bits) |
| | `BIN1` (Dirección 1) | `GPIO 33` | Salida Digital | Lógica $3.3\,\text{V}$ hacia TB6612FNG |
| | `BIN2` (Dirección 2) | `GPIO 14` | Salida Digital | Lógica $3.3\,\text{V}$ hacia TB6612FNG |
| **Driver Standby** | `STBY` (Habilitación) | `GPIO 13` | Salida Digital | `HIGH` = Motores habilitados, `LOW` = Parada libre |
| **Servomotor Dirección** | `SERVO_PWM` (Señal) | `GPIO 18` | Salida PWM | Canal LEDC 2 ($50\,\text{Hz}$, pulsos $500-2400\,\mu\text{s}$) |
| **Encoders de Rueda** | `ENC_L_A` (Izq Fase A) | `GPIO 34` | Entrada (ISR) | Input-only; pull-up externo $10\,\text{k}\Omega$ a $3.3\,\text{V}$ |
| | `ENC_L_B` (Izq Fase B) | `GPIO 35` | Entrada | Input-only; pull-up externo $10\,\text{k}\Omega$ a $3.3\,\text{V}$ |
| | `ENC_R_A` (Der Fase A) | `GPIO 36` | Entrada (ISR) | Input-only (`VP`); pull-up externo $10\,\text{k}\Omega$ a $3.3\,\text{V}$ |
| | `ENC_R_B` (Der Fase B) | `GPIO 39` | Entrada | Input-only (`VN`); pull-up externo $10\,\text{k}\Omega$ a $3.3\,\text{V}$ |
| **Sensor Ultrasonido** | `TRIG` (Disparo) | `GPIO 19` | Salida Digital | Pulso de $10\,\mu\text{s}$ a $3.3\,\text{V}$ (compatible con entrada TTL de HC-SR04) |
| | `ECHO` (Recepción) | `GPIO 23` | Entrada Digital | **¡OBLIGATORIO divisor resistivo $1\,\text{k}\Omega / 2\,\text{k}\Omega$!** ($5\,\text{V} \rightarrow 3.3\,\text{V}$) |
| **Bumpers de Impacto** | `BUMPER_LEFT` | `GPIO 16` | Entrada (ISR) | Configurado como `INPUT_PULLUP` interno; switch cierra a `GND` |
| | `BUMPER_RIGHT` | `GPIO 17` | Entrada (ISR) | Configurado como `INPUT_PULLUP` interno; switch cierra a `GND` |
| **IMU (MPU6050)** | `I2C_SDA` | `GPIO 21` | Bidireccional | Bus I2C estándar a $400\,\text{kHz}$ ($3.3\,\text{V}$) |
| | `I2C_SCL` | `GPIO 22` | Salida Clock | Bus I2C estándar a $400\,\text{kHz}$ ($3.3\,\text{V}$) |
| **Botón Inicio / LED** | `BTN_START` | `GPIO 4` | Entrada | `INPUT_PULLUP`; inicia exploración o resetea misión |

---

## 3. Diagrama Esquemático de Conexiones (ASCII)

```text
+===================================================================================+
|                           ETAPA DE POTENCIA Y BATERÍA                             |
+===================================================================================+
  [ Batería 2S Li-Ion 7.4V - 8.4V ]
         |                   |
         | (+)               +---------------------------------------+
         v                                                           |
   [Interruptor General]                                             | (GND Común)
         |                                                           |
         +------------------------+---> A Pin VM (TB6612FNG Motores) |
         |                        |                                  |
         v                        v                                  |
  [Buck Step-Down LM2596]   [Cap 470uF]                              |
    Ajustado a 5.0V DC            |                                  |
         |                        +----------------------------------+
         +---> A Pin VIN (ESP32)
         +---> A VCC (Servomotor MG90S) + [Cap 470uF entre 5V y GND]
         +---> A VCC (Sensor Ultrasonido HC-SR04)


+===================================================================================+
|                     INTERCONEXIÓN LÓGICA ESP32 <-> PERIFÉRICOS                    |
+===================================================================================+

                       +---------------------------------+
                       |        ESP32 DevKit V1          |
                       |                                 |
  [MPU6050 IMU]        |                                 |       [Driver TB6612FNG]
  +-----------+        |                                 |       +----------------+
  |       VCC |<-------+ 3V3                     GPIO 25 +------>| PWMA           |
  |       GND |<-------+ GND                     GPIO 26 +------>| AIN1   AO1/AO2 +===> (Motor Izq)
  |       SDA |<------>+ GPIO 21                 GPIO 27 +------>| AIN2           |
  |       SCL |<-------+ GPIO 22                 GPIO 32 +------>| PWMB           |
  +-----------+        |                         GPIO 33 +------>| BIN1   BO1/BO2 +===> (Motor Der)
                       |                         GPIO 14 +------>| BIN2           |
  [Bumpers Impacto]    |                         GPIO 13 +------>| STBY       VCC |<--- 3.3V (ESP32)
  (Switch Izq)---GND   |                                 |       |             VM |<--- 7.4V (Batería)
       +-------------->+ GPIO 16 (INPUT_PULLUP)          |       +----------------+
  (Switch Der)---GND   |                                 |
       +-------------->+ GPIO 17 (INPUT_PULLUP)          |       [Servomotor Dirección]
                       |                                 |       +----------------+
  [Encoders Hall 3.3V] |                         GPIO 18 +------>| PWM (Naranja)  |
  Enc Izq A/B -------->+ GPIO 34 / GPIO 35               |       | VCC (Rojo)     |<--- 5.0V (Buck)
  Enc Der A/B -------->+ GPIO 36 / GPIO 39               |       | GND (Marrón)   |<--- GND Común
                       |                                 |       +----------------+
                       |                                 |
                       |                         GPIO 19 +------> TRIG [HC-SR04 Ultrasonido]
                       |                                 |
                       |                         GPIO 23 +<--+--- Punto Medio (3.3V máx)
                       +---------------------------------+   |
                                                            [R1 = 1 kOhm]
                                                             |
                                                             +--- ECHO [HC-SR04 (Sale 5V)]
                                                             |
                                                            [R2 = 2 kOhm]
                                                             |
                                                            GND Común
```

---

## 4. Detalles Críticos de Ingeniería Electrónica

### 4.1 Divisor de Tensión Obligatorio en `ECHO` (HC-SR04)
El sensor HC-SR04 estándar requiere $5.0\,\text{V}$ en su pin `VCC` para excitar los transductores piezoeléctricos a $40\,\text{kHz}$. Su pin `TRIG` reconoce niveles de $3.3\,\text{V}$ como nivel alto lógico (`HIGH`), por lo que `GPIO 19` se conecta directo. Sin embargo, el pin `ECHO` responde con un pulso de **$5.0\,\text{V}$**. Como los GPIOs del ESP32 no son tolerantes a $5\,\text{V}$, conectarlo directo degrada o quema el diodo de clamping interno.
Con $R_1 = 1\,\text{k}\Omega$ en serie y $R_2 = 2\,\text{k}\Omega$ a `GND`:
$$V_{\text{GPIO23}} = 5.0\,\text{V} \cdot \frac{2\,\text{k}\Omega}{1\,\text{k}\Omega + 2\,\text{k}\Omega} = 3.33\,\text{V} \quad (\text{Seguro})$$
*(Alternativa: Usar el modelo **HC-SR04P / RCWL-1601** que opera nativamente a $3.3\,\text{V}$, o un sensor láser **VL53L0X** en el mismo bus I2C `GPIO 21/22` que el MPU6050).*

### 4.2 Aislamiento de Ruido Conducido por Motores y Servomotor
1. **Tierras unificadas en estrella (Star Ground):** El `GND` de la batería, el `GND` de salida del regulador Buck LM2596, el `GND` del driver TB6612FNG y el `GND` del ESP32 deben unirse en un nodo común cercano al regulador.
2. **Capacitores de desacoplo:** Soldar un capacitor cerámico de $100\,\text{nF}$ entre los terminales de cada motor DC para suprimir el ruido de conmutación de las escobillas que podría inducir pulsos falsos en los cables de los encoders.
3. **Nunca alimentar el servomotor desde el regulador AMS1117 interno del ESP32 (pin `3V3`):** Un servo MG90S al arrancar un giro brusco consume picos de corriente de $500-800\,\text{mA}$, lo que provocaría un *Brownout Reset* inmediato en el ESP32 si no se toma de la salida de $5\,\text{V}$ del regulador Buck externo con su capacitor de $470\,\mu\text{F}$.
