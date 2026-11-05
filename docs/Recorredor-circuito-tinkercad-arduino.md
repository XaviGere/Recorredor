# Recorredor — Circuito en Tinkercad (Arduino UNO / Nano)

> Wiring para la versión **Arduino** del robot, pensada para montarla en
> [Tinkercad Circuits](https://www.tinkercad.com). Hay dos sketches de un solo
> archivo, idénticos en lógica (UNO y Nano comparten el ATmega328P y el mismo
> pinout lógico):
> - `firmware/recorredor_tinkercad_uno/recorredor_tinkercad_uno.ino`
> - `firmware/recorredor_tinkercad_nano/recorredor_tinkercad_nano.ino`
>
> Ambos implementan **únicamente el modo Recto + Reaprendizaje**.
>
> **Diagrama visual de conexiones (Nano + protoboard):**
> `docs/recorredor_tinkercad_nano_circuito.png` — se regenera con
> `python tools/draw_tinkercad_nano_circuit.py`.
>
> El diseño usa sólo componentes que existen en el catálogo de Tinkercad:
> Arduino UNO R3, puente H **L293D**, micro-servo **SG90**, dos
> **HC-SR04** (PING))) y motores DC de caja plástica.

---

## 1. Lista de componentes

| Cant. | Componente | Rol |
|-------|------------|-----|
| 1 | Arduino UNO R3 **o** Nano | Cerebro (FSM + odometría `v·t`) — mismo pinout lógico |
| 1 | Driver L293D (DIP-16 o módulo) | Potencia para motores DC |
| 2 o 4 | Motor DC 3–6 V c/ caja reductora | Tracción (ver §3 según chasis) |
| 1 | Micro-servo SG90 | Cambio de dirección (tren delantero / torreta) |
| 2 | HC-SR04 (PING))) | Sensores ultrasonido frontales izq. y der. |
| 1 | Microswitch / pushbutton (opcional) | Bumper de impacto |
| 1 | Pushbutton (opcional) | Botón de inicio de misión |
| 1 | Portapilas 4×AA o fuente 5–6 V | Alimentación de motores |
| — | Cables jumper, protoboard | Conexiones |

> En Tinkercad todo puede alimentarse desde el propio UNO (5 V) porque es
> simulación; en físico los motores **siempre** por fuente aparte con GND común.

---

## 2. Pinout completo (coincide con el sketch)

> Vale igual para UNO y Nano: en ambos D3/D5/D6/D9/D10/D11 son PWM, la librería
> Servo usa Timer1 y bloquea el PWM de D9/D10, y A0–A5 sirven como digitales
> (en el Nano, A6 y A7 son sólo analógicos — no se usan).

| Señal | Pin UNO/Nano | Destino | Nota |
|-------|---------|---------|------|
| ENA | **D5** (PWM) | L293D pin 1 (EN1,2) | Velocidad canal izquierdo |
| IN1 | **D4** | L293D pin 2 (1A) | Sentido motor izquierdo |
| IN2 | **D7** | L293D pin 7 (2A) | Sentido motor izquierdo |
| ENB | **D6** (PWM) | L293D pin 9 (EN3,4) | Velocidad canal derecho |
| IN3 | **D8** | L293D pin 10 (3A) | Sentido motor derecho |
| IN4 | **D12** | L293D pin 15 (4A) | Sentido motor derecho |
| Servo | **D9** | SG90 cable naranja/señal | ±45° respecto a 90° |
| US-IZQ TRIG | **D10** | HC-SR04 #1 TRIG | Pulso de disparo |
| US-IZQ ECHO | **D11** | HC-SR04 #1 ECHO | `pulseIn` con timeout 11 ms |
| US-DER TRIG | **A2** | HC-SR04 #2 TRIG | Pin analógico usado como digital |
| US-DER ECHO | **A3** | HC-SR04 #2 ECHO | Pin analógico usado como digital |
| Bumper | **D2** | Microswitch a GND | `INPUT_PULLUP`, opcional |
| Botón start | **A0** | Pushbutton a GND | `INPUT_PULLUP`, opcional |

**Pines que quedan libres:** D0/D1 (Serial-USB, no usar), D3, D13, A1, A4/A5
(reservados para un futuro MPU6050 por I²C si se agrega IMU real).

> ⚠️ **Compatibilidad Servo/PWM en UNO:** la librería `Servo` deshabilita
> `analogWrite` en D9 y D10. Por eso el servo va por `Servo.write()` en D9 y
> los PWM de motores están en D5/D6 (no tocar D9/D10 para motores).
> D10 se usa sólo como salida digital de TRIG, que sigue funcionando.

---

## 3. Motores según chasis (3 o 4 ruedas)

El driver L293D tiene **2 canales independientes** (A: pines 3 y 6 / B: pines 11 y 14).

### Opción A — 3 ruedas (2 motores + rueda loca)
```
L293D pin 3  (OUT1) ──► Motor IZQUIERDO (+)
L293D pin 6  (OUT2) ──► Motor IZQUIERDO (−)
L293D pin 11 (OUT3) ──► Motor DERECHO   (+)
L293D pin 14 (OUT4) ──► Motor DERECHO   (−)
```
Tercer apoyo: rueda loca / bola loca (sin conexión). Con esta opción poné
`DIFFERENTIAL_STEER = true` en el sketch: el giro lo hacen los motores a
distinta velocidad y el servo queda para apuntar los sensores.

### Opción B — 4 ruedas (4 motores, 2 en paralelo por lado)
```
L293D pin 3  (OUT1) ──► Motor IZQ-DELANTERO(+) y Motor IZQ-TRASERO(+)
L293D pin 6  (OUT2) ──► Motor IZQ-DELANTERO(−) y Motor IZQ-TRASERO(−)
L293D pin 11 (OUT3) ──► Motor DER-DELANTERO(+) y Motor DER-TRASERO(+)
L293D pin 14 (OUT4) ──► Motor DER-DELANTERO(−) y Motor DER-TRASERO(−)
```
Idem opción A para el código (`DIFFERENTIAL_STEER = true`).

### Opción C — Triciclo/auto (servo dirige, motores traccionan)
Chasis tipo auto: el **servo gira el tren delantero** (dirección Ackermann) y
los motores sólo traccionan (ruedas traseras). Cableado = opción A o B y
`DIFFERENTIAL_STEER = false` (default): el servo dobla y los motores empujan
parejo. Es la opción que mejor aprovecha el "servo para cambiar de dirección".

---

## 4. Servo SG90 (dirección)

| Cable SG90 | Conexión |
|------------|----------|
| Naranja (señal) | **D9** |
| Rojo (Vcc) | 5 V |
| Marrón (GND) | GND |

- Reposo en **90°** (recto). El sketch limita el barrido a **±42°**.
- En Tinkercad el servo se alimenta de los 5 V del UNO; en físico, si el chasis
  es pesado, alimentarlo por fuente separada (5–6 V) con GND común.

---

## 5. Sensores ultrasónicos HC-SR04 ×2

Los dos sensores van montados al frente, abiertos unos **±15–20°** (uno
apuntando adelante-izquierda y otro adelante-derecha). Sirven dos propósitos:

1. **Distancia frontal efectiva** = `min(d_izq, d_der)` → frenado a 5 cm.
2. **Estimación de incidencia de la pared** (reservado para el futuro modo
   Incidente): la diferencia `d_izq − d_der` indica la inclinación de la pared
   frente al robot (sólo usa 2 el modo Incidente — por eso el circuito lleva 2).

| HC-SR04 #1 (izquierdo) | Pin | HC-SR04 #2 (derecho) | Pin |
|------------------------|-----|-----------------------|-----|
| VCC | 5 V | VCC | 5 V |
| TRIG | **D10** | TRIG | **A2** |
| ECHO | **D11** | ECHO | **A3** |
| GND | GND | GND | GND |

> En UNO los ECHO van directos (5 V tolerant en entrada digital). Si más
> adelante se migra a ESP32, **el divisor 1 kΩ/2 kΩ en ECHO es obligatorio**
> (ver `docs/Recorredor-electronica-y-circuito.md`).

---

## 6. Alimentación (L293D)

| Pin L293D | Conexión |
|-----------|----------|
| 8 (VS, motor supply) | **5–6 V fuente motores** (en Tinkercad: 5 V del UNO sirve) |
| 16 (VSS, lógica) | 5 V del UNO |
| 4, 5, 12, 13 (GND) | GND común (UNO + fuente motores unidos) |
| 1 (EN1,2) | D5 (ENA, PWM) |
| 9 (EN3,4) | D6 (ENB, PWM) |
| 2, 7, 10, 15 | D4, D7, D8, D12 (IN1–IN4) |
| 3, 6, 11, 14 | Salidas a motores (ver §3) |

**Reglas:** GND de la fuente de motores **siempre unido** al GND del UNO;
nunca alimentar motores reales desde el pin 5 V del UNO.

---

## 7. Extras opcionales ("poco más")

- **Bumper de impacto** (D2 → microswitch a GND): si el robot toca algo que el
  ultrasonido no vio (pared por debajo del cono, obstáculo bajo), dispara el
  mismo evento de choque que el sensor — y durante la vuelta final dispara el
  **reaprendizaje**.
- **Botón de inicio** (A0 → pushbutton a GND): arranca la misión al presionarlo;
  si no hay botón, arranca solo a los 3 s del reset.

---

## 8. Dos arquitecturas de firmware

### 8a. Autónomo (`recorredor_tinkercad_uno/nano/*.ino`)

Todo corre en la placa: FSM, odometría, mapa y vuelta perimetral.

### 8b. Thin Client + Cerebro en PC (`recorredor_thin_client/` + `tools/recorredor_brain.py`)

La placa queda **solo con movimiento y sensores**:

- Lee los 2 HC-SR04 + bumper y los reporta como JSON por el enlace.
- Ejecuta comandos ASCII: `DRV <v> <w>` (tracción+giro), `STR <deg>` (servo),
  `STP` (frenar), `PNG` (ping). Con watchdog: sin comandos por 700 ms frena solo.
- Enlace: **Serial @115200** en UNO/Nano (en Tinkercad el Serial Monitor es el
  canal); **WiFi UDP :5005** en ESP32 (`-DUSE_WIFI_UDP=1` + credenciales).

La PC corre `tools/recorredor_brain.py`, que implementa el modo
Recto + Reaprendizaje completo reutilizando la geometría del simulador
(`reconstruct_walled_polygon`, `compute_inward_offset_polygon`,
`build_anchored_perimeter_route`): odometría `v·t`, hull ortogonal,
offset interior y reaprendizaje (ignora impactos que caen sobre el
perímetro ya reconstruido: rozar una pared conocida al doblar una esquina
es drift de odometría, no un mapa equivocado).

```powershell
python tools/recorredor_brain.py --udp 5005            # ESP32 WiFi
python tools/recorredor_brain.py --serial COM5         # UNO/Nano por USB
```

---

## 8c. Detalle del sketch autónomo (modo Recto + Reaprendizaje)

```
EXPLORE  → avanza recto hasta detectar pared (min de los 2 HC-SR04 ≤ 5 cm o bumper)
BRAKE    → frena y REGISTRA: punto de impacto (x,y) + cuerda recorrida
EVALUATE → si hay <16 cuerdas: elige rumbo nuevo (preferencia ±90°, modo recto) → TURN
           si llegó a 16: → BUILD_PATH
TURN     → servo (o diferencial) gira hasta el rumbo objetivo → EXPLORE
BUILD_PATH → hull de los puntos de choque → offset interior R×1.2 →
             SNAP ortogonal a 90° (aristas H/V estrictas) → FOLLOW
FOLLOW   → sigue los waypoints de la vuelta a velocidad perimetral;
           si CHOCA → registra el punto nuevo y vuelve a BUILD_PATH
           (reaprendizaje, hasta 8 veces) → si completa la vuelta: DONE
DONE     → motores a 0, telemetría final con Área, Perímetro y reaprendizajes
```

La odometría es `d = v·t` con integración de punto medio (igual que el
simulador Python y el firmware ESP32), sin encoders. La telemetría sale por
**Serial @115200** en JSON por línea — en Tinkercad se ve en el Serial Monitor.

### Ajustes rápidos (arriba del .ino)

| Constante | Default | Qué cambia |
|-----------|---------|------------|
| `DIFFERENTIAL_STEER` | `false` | `true` = giro por motores (chasis diferencial); `false` = servo dirige (triciclo/auto) |
| `STOP_DISTANCE_M` | 0.05 | Distancia de frenado frontal |
| `CHORDS_EXPLORE` | 16 | Cuerdas antes de armar el mapa |
| `SAFE_MARGIN_M` | R×1.2 = 0.144 | Distancia de la ruta interior a las paredes |
| `MAX_RELEARNS` | 8 | Límite de recálculos durante la vuelta |
| `CRUISE_SPEED_MPS` | 0.30 | Velocidad de exploración |

---

## 9. Notas de portabilidad UNO → ESP32

Cuando se lleve al ESP32 real, el pinout ya está definido en
`firmware/recorredor_esp32/config.h` (TB6612FNG, encoders, IMU, WiFi-UDP).
Diferencias a tener en cuenta: el ECHO de 5 V necesita divisor resistivo, el
PWM se configura con `ledc`/API ESP32, y la telemetría puede salir por UDP al
dashboard de Python además de Serial.
