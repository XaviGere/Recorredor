// ============================================================================
// RECORREDOR — THIN CLIENT (firmware "tonto": solo movimiento + sensores)
// ----------------------------------------------------------------------------
// Toda la inteligencia (FSM, mapa, recto+reaprendizaje) corre en la PC con
// tools/recorredor_brain.py. Este sketch solo:
//   1) Lee los 2 HC-SR04 + bumper y los REPORTA por el canal de telemetría.
//   2) Ejecuta comandos de movimiento que llegan por el mismo canal.
//
// CANAL:
//   - UNO / Nano (Tinkercad o físico): Serial USB @115200 — el "cable" es el
//     enlace; en Tinkercad la consola del Serial Monitor hace de link.
//   - ESP32 (robot real): mismo protocolo pero por WiFi UDP puerto 5005
//     (compilar con USE_WIFI_UDP=1, definir WIFI_SSID/WIFI_PASS abajo).
//
// PROTOCOLO (una línea por comando, ASCII):
//   DRV <v> <w>   → tracción v [m/s] y giro w [rad/s] (diferencial) o
//                   tracción v con el servo ya posicionado (triciclo)
//   STR <deg>     → servo de dirección: -45..+45 respecto de 90° (recto=0)
//   STP           → motores a 0 inmediato
//   PNG           → responde {"pong":1}
//
// TELEMETRÍA (una línea JSON cada TELEM_MS):
//   {"d1":<m>,"d2":<m>,"bump":<0|1>,"v":<cmd_v>,"w":<cmd_w>}
//
// Seguridad: si no llegan comandos por CMD_TIMEOUT_MS, los motores frenan solos
// (watchdog por si se corta el enlace con la PC).
// ============================================================================

#include <Arduino.h>
#include <Servo.h>

// ----------------------- 0. PLATAFORMA / ENLACE -----------------------------
// Para ESP32 con WiFi UDP: compilar con -DUSE_WIFI_UDP=1 y completar credenciales.
// En UNO/Nano esto queda en 0 y el enlace es Serial.
#ifndef USE_WIFI_UDP
#define USE_WIFI_UDP 0
#endif

#if USE_WIFI_UDP && defined(ESP32)
  #include <WiFi.h>
  #include <WiFiUdp.h>
  const char* WIFI_SSID = "TU_RED_WIFI";
  const char* WIFI_PASS = "TU_CLAVE_WIFI";
  WiFiUDP udp;
  IPAddress PC_IP(255, 255, 255, 255);      // broadcast hasta conocer la IP de la PC
  const uint16_t UDP_PORT = 5005;
  bool pc_known = false;
#endif

// ------------------------------ 1. PINOUT -----------------------------------
// L293D: canal A = motor(es) izquierdo(s), canal B = derecho(s)
constexpr uint8_t PIN_ENA = 5, PIN_IN1 = 4, PIN_IN2 = 7;
constexpr uint8_t PIN_ENB = 6, PIN_IN3 = 8, PIN_IN4 = 12;
constexpr uint8_t PIN_SERVO = 9;
constexpr uint8_t PIN_US1_TRIG = 10, PIN_US1_ECHO = 11;   // frontal izquierdo
constexpr uint8_t PIN_US2_TRIG = A2, PIN_US2_ECHO = A3;   // frontal derecho
constexpr uint8_t PIN_BUMPER = 2;                         // opcional, a GND

// --------------------------- 2. CONFIGURACIÓN -------------------------------
constexpr bool    DIFFERENTIAL_STEER = true;   // true=2 motores+loca/4x4; false=servo dirige
constexpr float   WHEELBASE_M        = 0.18f;
constexpr float   MAX_STEER_DEG      = 45.0f;
constexpr float   V_REF_MPS          = 0.45f;  // velocidad a PWM=255 (calibrar)
constexpr uint32_t TELEM_MS          = 50;     // 20 Hz
constexpr uint32_t CMD_TIMEOUT_MS    = 700;    // watchdog del enlace
constexpr float   US_MAX_DIST_M      = 2.0f;

Servo steerServo;
float cmd_v = 0.0f, cmd_w = 0.0f;
uint32_t last_cmd_ms = 0;

// ------------------------------- 3. DRIVE -----------------------------------
void motorChannel(uint8_t inA, uint8_t inB, uint8_t pwm, float v) {
  int duty = (int)(constrain(fabsf(v) / V_REF_MPS, 0.0f, 1.0f) * 255.0f);
  if (duty < 20) { digitalWrite(inA, LOW); digitalWrite(inB, LOW); analogWrite(pwm, 0); return; }
  digitalWrite(inA, v >= 0 ? HIGH : LOW);
  digitalWrite(inB, v >= 0 ? LOW : HIGH);
  analogWrite(pwm, duty);
}

void applyDrive() {
  if (DIFFERENTIAL_STEER) {
    float vl = cmd_v - 0.5f * WHEELBASE_M * cmd_w;
    float vr = cmd_v + 0.5f * WHEELBASE_M * cmd_w;
    motorChannel(PIN_IN1, PIN_IN2, PIN_ENA, vl);
    motorChannel(PIN_IN3, PIN_IN4, PIN_ENB, vr);
  } else {
    motorChannel(PIN_IN1, PIN_IN2, PIN_ENA, cmd_v);
    motorChannel(PIN_IN3, PIN_IN4, PIN_ENB, cmd_v);
  }
}

void stopMotors() { cmd_v = cmd_w = 0.0f; applyDrive(); }

// ------------------------------ 4. SENSORES ---------------------------------
float readUltra(uint8_t trig, uint8_t echo) {
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long us = pulseIn(echo, HIGH, 11000UL);
  return us == 0 ? US_MAX_DIST_M : (float)us * 0.000343f * 0.5f;
}

// --------------------------- 5. CANAL DE RED --------------------------------
void linkSend(const char* s) {
#if USE_WIFI_UDP && defined(ESP32)
  if (pc_known) { udp.beginPacket(PC_IP, UDP_PORT); udp.print(s); udp.endPacket(); return; }
#endif
  Serial.println(s);
}

String linkRead() {                         // una línea completa o vacío
#if USE_WIFI_UDP && defined(ESP32)
  int n = udp.parsePacket();
  if (n > 0) {
    char buf[64]; int r = udp.read(buf, min(n, 63)); buf[r] = 0;
    PC_IP = udp.remoteIP(); pc_known = true;
    return String(buf);
  }
#endif
  if (Serial.available()) return Serial.readStringUntil('\n');
  return "";
}

// --------------------------- 6. COMANDOS ------------------------------------
void handleCommand(String c) {
  c.trim();
  if (c.length() == 0) return;
  last_cmd_ms = millis();
  if (c.startsWith("DRV")) {
    int sp = c.indexOf(' ', 3);
    cmd_v = c.substring(4, sp).toFloat();
    cmd_w = c.substring(sp + 1).toFloat();
    applyDrive();
  } else if (c.startsWith("STR")) {
    float deg = constrain(c.substring(4).toFloat(), -MAX_STEER_DEG, MAX_STEER_DEG);
    steerServo.write(90.0f + deg);
  } else if (c.startsWith("STP")) {
    stopMotors();
  } else if (c.startsWith("PNG")) {
    linkSend("{\"pong\":1}");
  }
}

// ------------------------------ 7. SETUP ------------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_ENA, OUTPUT); pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_US1_TRIG, OUTPUT); pinMode(PIN_US1_ECHO, INPUT);
  pinMode(PIN_US2_TRIG, OUTPUT); pinMode(PIN_US2_ECHO, INPUT);
  pinMode(PIN_BUMPER, INPUT_PULLUP);
  steerServo.attach(PIN_SERVO);
  steerServo.write(90);
  stopMotors();

#if USE_WIFI_UDP && defined(ESP32)
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(200); Serial.print('.'); }
  udp.begin(UDP_PORT);
  Serial.print(F("{\"ip\":\"")); Serial.print(WiFi.localIP()); Serial.println(F("\"}"));
#endif
  linkSend("{\"event\":\"THIN_CLIENT_READY\"}");
}

// ------------------------------ 8. LOOP -------------------------------------
void loop() {
  static uint32_t last_telem = 0;
  uint32_t now = millis();

  String c = linkRead();
  if (c.length()) handleCommand(c);

  // Watchdog: sin comandos -> frenar
  if (now - last_cmd_ms > CMD_TIMEOUT_MS && (cmd_v != 0.0f || cmd_w != 0.0f)) stopMotors();

  if (now - last_telem >= TELEM_MS) {
    last_telem = now;
    float d1 = readUltra(PIN_US1_TRIG, PIN_US1_ECHO);
    float d2 = readUltra(PIN_US2_TRIG, PIN_US2_ECHO);
    int bump = (digitalRead(PIN_BUMPER) == LOW) ? 1 : 0;
    char buf[120];
    snprintf(buf, sizeof(buf),
             "{\"d1\":%.3f,\"d2\":%.3f,\"bump\":%d,\"v\":%.3f,\"w\":%.3f}",
             d1, d2, bump, cmd_v, cmd_w);
    linkSend(buf);
  }
}
