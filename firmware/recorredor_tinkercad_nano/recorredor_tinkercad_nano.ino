// ============================================================================
// RECORREDOR — Sketch ÚNICO para Arduino NANO (Tinkercad)
// También compila igual en UNO (mismo ATmega328P y mismo pinout lógico).
// Modo implementado: RECTO + REAPRENDIZAJE (habitación ortogonal a 90° +
// recálculo del mapa si choca durante la vuelta perimetral).
//
// Componentes básicos Tinkercad (ver docs/recorredor_tinkercad_nano_circuito.png
// y docs/Recorredor-circuito-tinkercad-arduino.md):
//   - Placa Arduino Nano sobre protoboard.
//   - Driver L293D (puente H) para los motores DC comunes.
//   - Servo común SG90 de dirección (tren delantero o torreta).
//   - 2 x HC-SR04 comunes (ultrasonido) frontales izq. y der.
//   - Bumper de impacto opcional en D2.
//   - Botón de inicio opcional en A0 (o arranque automático a los 3 s).
//
//   En este modo sólo se usa min(d_izq, d_der) como distancia frontal; la
//   diferencia entre ambos sensores queda para el futuro modo Incidente.
//
// Odometría: d = v * t  (sin encoders, igual que en el simulador Python).
// ============================================================================

#include <Arduino.h>
#include <Servo.h>

// ============================ 1. PINOUT NANO ================================
// L293D (Puente H): Canal A = motor(es) izquierdo(s), Canal B = derecho(s)
constexpr uint8_t PIN_ENA      = 5;   // PWM velocidad izquierda
constexpr uint8_t PIN_IN1      = 4;
constexpr uint8_t PIN_IN2      = 7;
constexpr uint8_t PIN_ENB      = 6;   // PWM velocidad derecha
constexpr uint8_t PIN_IN3      = 8;
constexpr uint8_t PIN_IN4      = 12;

constexpr uint8_t PIN_SERVO    = 9;   // SG90 dirección (Servo lib: no usar PWM en 9/10)

constexpr uint8_t PIN_US1_TRIG = 10;  // Ultrasonido frontal IZQUIERDO
constexpr uint8_t PIN_US1_ECHO = 11;
constexpr uint8_t PIN_US2_TRIG = A2;  // Ultrasonido frontal DERECHO (A2/A3 como digitales)
constexpr uint8_t PIN_US2_ECHO = A3;

constexpr uint8_t PIN_BUMPER   = 2;   // Microswitch de impacto (opcional, a GND)
constexpr uint8_t PIN_BTN      = A0;  // Botón de inicio (opcional, a GND)

// ===================== 2. CALIBRACIÓN FÍSICA (SI) ===========================
constexpr float    ROBOT_RADIUS_M        = 0.12f;  // Radio envolvente (12 cm)
constexpr float    STOP_DISTANCE_M       = 0.05f;  // Frenar a 5 cm de la pared
constexpr float    SAFE_MARGIN_M         = ROBOT_RADIUS_M * 1.2f; // Ruta interior
constexpr float    WHEELBASE_M           = 0.18f;  // Entre ruedas motrices
constexpr float    CRUISE_SPEED_MPS      = 0.30f;  // Velocidad exploración
constexpr float    PERIMETER_SPEED_MPS   = 0.24f;  // Velocidad vuelta final
constexpr float    MAX_STEER_DEG         = 42.0f;  // Máximo ángulo del servo
constexpr float    HEADING_KP            = 2.6f;   // Ganancia de rumbo -> servo
constexpr float    WAYPOINT_TOLERANCE_M  = 0.09f;
constexpr uint32_t LOOP_MS               = 20;     // Lazo 50 Hz

// Capacidad estática (SRAM del UNO = 2 KB — no subir sin medir)
constexpr uint8_t  MAX_CHORDS            = 20;
constexpr uint8_t  MAX_VERTS             = 16;
constexpr uint8_t  MAX_WAYPOINTS         = MAX_VERTS + 3;
constexpr uint8_t  CHORDS_EXPLORE        = 16;     // Cuerdas antes de armar el mapa
constexpr uint8_t  MAX_RELEARNS          = 8;      // Límite de recálculos en la vuelta

// Si el chasis es diferencial (2 motores + rueda loca) poner true:
// el giro se hace con los motores y el servo sólo apunta. Si es triciclo/auto
// (servo dirige las ruedas delanteras) dejar false: el giro es arco servo+motor.
constexpr bool DIFFERENTIAL_STEER = false;

// ========================== 3. GEOMETRÍA 2D =================================
struct Point2D { float x, y; };
struct Pose2D  { float x, y, theta; };
struct Chord   { Point2D a, b; float len; };

float wrapAngle(float a) {
  while (a <= -PI) a += 2.0f * PI;
  while (a >   PI) a -= 2.0f * PI;
  return a;
}
float dist2D(const Point2D& a, const Point2D& b) { return hypotf(a.x - b.x, a.y - b.y); }
float cross2D(const Point2D& o, const Point2D& a, const Point2D& b) {
  return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}
float polygonArea(const Point2D* v, uint8_t n) {
  float s = 0.0f;
  for (uint8_t i = 0; i < n; ++i) s += v[i].x * v[(i + 1) % n].y - v[(i + 1) % n].x * v[i].y;
  return 0.5f * fabsf(s);
}
float polygonPerim(const Point2D* v, uint8_t n) {
  float p = 0.0f;
  for (uint8_t i = 0; i < n; ++i) p += dist2D(v[i], v[(i + 1) % n]);
  return p;
}

// Convex hull simplificado (Monotone Chain) — memoria estática apta para UNO.
uint8_t buildHull(const Point2D* in, uint8_t n, Point2D* out) {
  if (n < 3) { for (uint8_t i = 0; i < n; ++i) out[i] = in[i]; return n; }
  Point2D s[MAX_CHORDS];
  for (uint8_t i = 0; i < n; ++i) s[i] = in[i];
  for (uint8_t i = 1; i < n; ++i) {              // orden lexicográfico por inserción
    Point2D k = s[i]; int8_t j = (int8_t)i - 1;
    while (j >= 0 && (s[j].x > k.x || (fabsf(s[j].x - k.x) < 1e-5f && s[j].y > k.y))) {
      s[j + 1] = s[j]; j--;
    }
    s[j + 1] = k;
  }
  Point2D h[2 * MAX_VERTS]; uint8_t k = 0;
  for (uint8_t i = 0; i < n; ++i) {
    while (k >= 2 && cross2D(h[k - 2], h[k - 1], s[i]) <= 0.0f) k--;
    if (k < 2 * MAX_VERTS) h[k++] = s[i];
  }
  for (int8_t i = (int8_t)n - 2, t = (int8_t)k + 1; i >= 0; --i) {
    while (k >= t && cross2D(h[k - 2], h[k - 1], s[i]) <= 0.0f) k--;
    if (k < 2 * MAX_VERTS) h[k++] = s[i];
  }
  if (k > 1) k--;
  // Quitar vértices casi colineales sobre la misma pared (<= 8°)
  uint8_t m = 0;
  for (uint8_t i = 0; i < k && m < MAX_VERTS; ++i) {
    const Point2D& pv = h[(i + k - 1) % k];
    const Point2D& cu = h[i];
    const Point2D& nx = h[(i + 1) % k];
    float a1 = atan2f(cu.y - pv.y, cu.x - pv.x);
    float a2 = atan2f(nx.y - cu.y, nx.x - cu.x);
    if (fabsf(wrapAngle(a2 - a1)) < 8.0f * DEG_TO_RAD && k > 3) continue;
    out[m++] = cu;
  }
  return m;
}

// Offset interior por bisectrices (camino seguro a R * 1.2 de las paredes).
uint8_t inwardOffset(const Point2D* v, uint8_t n, float margin, Point2D* out) {
  if (n < 3) return 0;
  for (uint8_t i = 0; i < n; ++i) {
    const Point2D& pp = v[(i + n - 1) % n];
    const Point2D& pc = v[i];
    const Point2D& pn = v[(i + 1) % n];
    float e1x = pc.x - pp.x, e1y = pc.y - pp.y, l1 = max(hypotf(e1x, e1y), 1e-6f); e1x /= l1; e1y /= l1;
    float e2x = pn.x - pc.x, e2y = pn.y - pc.y, l2 = max(hypotf(e2x, e2y), 1e-6f); e2x /= l2; e2y /= l2;
    float bx = -e1y - e2y, by = e1x + e2x, bl = hypotf(bx, by);
    float ch = 1.0f;
    if (bl < 1e-5f) { bx = -e1y; by = e1x; }
    else { bx /= bl; by /= bl; ch = max(-e1y * bx + e1x * by, 0.35f); }
    float sh = margin / ch;
    out[i] = {pc.x + bx * sh, pc.y + by * sh};
  }
  return n;
}

// MODO RECTO: ajusta el polígono de ruta a aristas estrictamente ortogonales.
// Cada arista se proyecta al eje dominante; los vértices se recalculan como
// intersección de las rectas H/V resultantes (equivalente a "codos a 90°").
void snapPolyToOrtho(Point2D* v, uint8_t n) {
  if (n < 3) return;
  const float TOL = 0.20f;  // aristas que se desvían <20°/20cm se colapsan al eje
  bool horiz[MAX_VERTS];
  for (uint8_t i = 0; i < n; ++i) {
    float dx = fabsf(v[(i + 1) % n].x - v[i].x);
    float dy = fabsf(v[(i + 1) % n].y - v[i].y);
    horiz[i] = (dx >= dy);
  }
  for (uint8_t i = 0; i < n; ++i) {
    uint8_t ip = (i + n - 1) % n, in = (i + 1) % n;
    if (!horiz[ip] && !horiz[i] && fabsf(v[ip].x - v[in].x) < TOL) v[i].x = (v[ip].x + v[in].x) * 0.5f;
    if (horiz[ip] && horiz[i] && fabsf(v[ip].y - v[in].y) < TOL) v[i].y = (v[ip].y + v[in].y) * 0.5f;
    if (horiz[ip] != horiz[i]) {
      // Vértice entre arista H y arista V: toma y de la H y x de la V
      if (horiz[ip]) { v[i].y = v[ip].y; v[i].x = v[in].x; }
      else           { v[i].x = v[ip].x; v[i].y = v[in].y; }
    }
  }
}

// ========================= 4. HAL (hardware) ================================
Servo steerServo;

void motorChannel(uint8_t inA, uint8_t inB, uint8_t pwm, float v) {
  int duty = (int)(constrain(fabsf(v) / 0.45f, 0.0f, 1.0f) * 255.0f);
  if (duty < 20) { digitalWrite(inA, LOW); digitalWrite(inB, LOW); analogWrite(pwm, 0); return; }
  if (v >= 0.0f) { digitalWrite(inA, HIGH); digitalWrite(inB, LOW); }
  else           { digitalWrite(inA, LOW);  digitalWrite(inB, HIGH); }
  analogWrite(pwm, duty);
}

void setDrive(float v_mps, float omega_radps) {
  if (DIFFERENTIAL_STEER) {
    // Giro diferencial: ruedas a distinta velocidad; el servo queda para apuntar sensores
    float vl = v_mps - 0.5f * WHEELBASE_M * omega_radps;
    float vr = v_mps + 0.5f * WHEELBASE_M * omega_radps;
    motorChannel(PIN_IN1, PIN_IN2, PIN_ENA, vl);
    motorChannel(PIN_IN3, PIN_IN4, PIN_ENB, vr);
  } else {
    // Triciclo/auto: ambos canales igual (tracción), la dirección la da el servo
    motorChannel(PIN_IN1, PIN_IN2, PIN_ENA, v_mps);
    motorChannel(PIN_IN3, PIN_IN4, PIN_ENB, v_mps);
  }
}

void setSteerRad(float err_rad) {
  float deg = constrain(err_rad * RAD_TO_DEG * 1.4f, -MAX_STEER_DEG, MAX_STEER_DEG);
  steerServo.write(90.0f + deg);
}

void stopMotors() {
  motorChannel(PIN_IN1, PIN_IN2, PIN_ENA, 0.0f);
  motorChannel(PIN_IN3, PIN_IN4, PIN_ENB, 0.0f);
}

float readUltra(uint8_t trig, uint8_t echo) {
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long us = pulseIn(echo, HIGH, 11000UL);   // ~1.9 m máx, no bloquea el lazo
  if (us == 0) return 2.0f;
  return (float)us * 0.000343f * 0.5f;
}

// ========================== 5. FSM DEL MODO =================================
enum class State : uint8_t {
  IDLE, EXPLORE, BRAKE, EVALUATE, TURN, BUILD_PATH, FOLLOW, RELEARN_HIT, DONE
};

State     state = State::IDLE;
Pose2D    pose  = {0, 0, 0};
float     t_s = 0.0f, dist_m = 0.0f;
float     target_heading = 0.0f, last_wall_dist = 0.0f;
float     last_v = 0.0f, last_omega = 0.0f;

Point2D   wall_pts[MAX_CHORDS];   uint8_t n_wall = 0;
Chord     chords[MAX_CHORDS];     uint8_t n_chords = 0;
Point2D   chord_start = {0, 0};

Point2D   hull[MAX_VERTS];        uint8_t n_hull = 0;
Point2D   wps[MAX_WAYPOINTS];     uint8_t n_wps = 0, wp_idx = 0;
uint8_t   relearn_count = 0;

// Ordena los waypoints de la ruta interior arrancando por el más cercano y
// cerrando el ciclo (versión simple del "anclaje" del simulador).
void buildSafeRoute() {
  n_hull = buildHull(wall_pts, n_wall, hull);
  Point2D raw[MAX_VERTS];
  uint8_t nr = inwardOffset(hull, n_hull, SAFE_MARGIN_M, raw);
  snapPolyToOrtho(raw, nr);                       // <- MODO RECTO a 90°
  if (nr < 3) { state = State::DONE; return; }
  uint8_t c = 0; float best = dist2D({pose.x, pose.y}, raw[0]);
  for (uint8_t i = 1; i < nr; ++i) {
    float d = dist2D({pose.x, pose.y}, raw[i]);
    if (d < best) { best = d; c = i; }
  }
  n_wps = 0;
  for (uint8_t i = 0; i < nr; ++i) wps[n_wps++] = raw[(c + i) % nr];
  wps[n_wps++] = wps[0];                          // cerrar la vuelta
  wp_idx = 0;
  state = State::FOLLOW;
}

void recordWallHit(float d_wall) {
  float off = ROBOT_RADIUS_M + max(0.0f, d_wall);
  if (n_wall < MAX_CHORDS)
    wall_pts[n_wall++] = {pose.x + off * cosf(pose.theta), pose.y + off * sinf(pose.theta)};
}

void recordChord() {
  Point2D now = {pose.x, pose.y};
  float l = dist2D(chord_start, now);
  if (l >= 0.05f && n_chords < MAX_CHORDS)
    chords[n_chords++] = {chord_start, now, l};
  chord_start = now;
}

float pickNextHeading() {
  // Modo recto: preferir rumbos alineados a 90° respecto al rumbo actual,
  // con variación pseudoaleatoria para muestrear todas las paredes.
  if (n_wall >= 3) {
    float cx = 0, cy = 0;
    for (uint8_t i = 0; i < n_wall; ++i) { cx += wall_pts[i].x; cy += wall_pts[i].y; }
    float to_c = atan2f(cy / n_wall - pose.y, cx / n_wall - pose.x);
    static const float off[8] = {0.0f, 0.5f, -0.5f, 1.0f, -1.0f, 0.35f, -0.35f, 0.8f};
    return wrapAngle(to_c + off[n_chords % 8]);
  }
  return wrapAngle(pose.theta + PI - 0.5f + fmodf(n_chords * 2.4f, 0.8f));
}

void fsmStep(float front_m, bool bumper, float dt) {
  switch (state) {
    case State::IDLE:
    case State::DONE:
      stopMotors(); return;

    case State::EXPLORE:
      if (bumper || front_m <= STOP_DISTANCE_M) {
        last_wall_dist = bumper ? 0.0f : front_m;
        recordWallHit(last_wall_dist);
        recordChord();
        state = State::BRAKE;
        return;
      }
      {
        float err = wrapAngle(target_heading - pose.theta);
        float om = constrain(HEADING_KP * err, -2.2f, 2.2f);
        last_v = CRUISE_SPEED_MPS; last_omega = om;
        setDrive(last_v, om); setSteerRad(err);
      }
      return;

    case State::BRAKE:
      stopMotors(); last_v = 0.0f; last_omega = 0.0f;
      state = State::EVALUATE; return;

    case State::EVALUATE:
      if (n_chords >= CHORDS_EXPLORE) state = State::BUILD_PATH;
      else { target_heading = pickNextHeading(); state = State::TURN; }
      return;

    case State::TURN: {
      float err = wrapAngle(target_heading - pose.theta);
      if (fabsf(err) <= 0.05f) {
        chord_start = {pose.x, pose.y};
        state = State::EXPLORE;
        return;
      }
      // Girar: diferencial -> giro en el lugar; triciclo -> arco lento con servo al máx.
      if (DIFFERENTIAL_STEER) {
        float om = constrain(HEADING_KP * err, -2.0f, 2.0f);
        if (fabsf(om) < 0.4f) om = (err >= 0.0f) ? 0.4f : -0.4f;
        last_v = 0.0f; last_omega = om; setDrive(0.0f, om);
      } else {
        float sgn = (err >= 0.0f) ? 1.0f : -1.0f;
        setSteerRad(sgn * 0.75f);
        last_v = 0.12f; last_omega = 0.12f * tanf(0.75f) / WHEELBASE_M * sgn;
        setDrive(last_v, 0.0f);   // tracción uniforme; el servo ya está girado
      }
      return;
    }

    case State::BUILD_PATH:
      buildSafeRoute(); return;

    case State::FOLLOW: {
      if (wp_idx >= n_wps) { state = State::DONE; return; }
      // REAPRENDIZAJE: un choque en la vuelta = pared no muestreada ->
      // se registra y se reconstruye mapa + ruta completas.
      if (bumper || front_m <= STOP_DISTANCE_M * 0.8f) {
        recordWallHit(bumper ? 0.0f : front_m);
        relearn_count++;
        Serial.print(F("{\"event\":\"RELEARN\",\"n\":"));
        Serial.print(relearn_count); Serial.println(F("}"));
        state = (relearn_count < MAX_RELEARNS) ? State::BUILD_PATH : State::DONE;
        return;
      }
      Point2D tgt = wps[wp_idx];
      Point2D cur = {pose.x, pose.y};
      float d = dist2D(cur, tgt);
      if (d <= WAYPOINT_TOLERANCE_M) {
        wp_idx++;
        if (wp_idx >= n_wps) { state = State::DONE; return; }
        tgt = wps[wp_idx];
      }
      float des = atan2f(tgt.y - cur.y, tgt.x - cur.x);
      float err = wrapAngle(des - pose.theta);
      float om = constrain(HEADING_KP * err, -2.2f, 2.2f);
      // En modo recto no avanzar en diagonal: primero alinear (<12°) y luego avanzar
      if (fabsf(err) > 0.21f && !DIFFERENTIAL_STEER) {
        last_v = 0.10f; last_omega = 0.10f * tanf(0.75f * ((err >= 0) ? 1 : -1)) / WHEELBASE_M;
        setSteerRad(err); setDrive(last_v, 0.0f);
      } else {
        last_v = min(PERIMETER_SPEED_MPS, max(0.10f, 1.6f * d));
        last_omega = om;
        setDrive(last_v, om); setSteerRad(err);
      }
      return;
    }

    case State::RELEARN_HIT:
      state = State::BUILD_PATH; return;
  }
}

// ========================= 6. ODOMETRÍA v*t =================================
void updateOdom(float dt) {
  float ds = last_v * dt;
  float dth;
  if (DIFFERENTIAL_STEER) {
    dth = last_omega * dt;
  } else {
    // Cinemática Ackermann aproximada: dtheta = v * tan(steer) / wheelbase
    float steer = (steerServo.read() - 90.0f) * DEG_TO_RAD;
    dth = (fabsf(last_v) > 1e-4f) ? last_v * tanf(steer) / WHEELBASE_M * dt : 0.0f;
  }
  float mid = pose.theta + 0.5f * dth;
  pose.x += ds * cosf(mid);
  pose.y += ds * sinf(mid);
  pose.theta = wrapAngle(pose.theta + dth);
  t_s += dt;
  dist_m += fabsf(ds);
}

// ========================= 7. TELEMETRÍA ====================================
void telemetry(bool event) {
  static uint32_t last_ms = 0;
  uint32_t now = millis();
  if (!event && now - last_ms < 250) return;
  last_ms = now;
  Serial.print(F("{\"t\":")); Serial.print(t_s, 1);
  Serial.print(F(",\"st\":")); Serial.print((uint8_t)state);
  Serial.print(F(",\"x\":")); Serial.print(pose.x, 3);
  Serial.print(F(",\"y\":")); Serial.print(pose.y, 3);
  Serial.print(F(",\"th\":")); Serial.print(pose.theta, 3);
  Serial.print(F(",\"ch\":")); Serial.print(n_chords);
  Serial.print(F(",\"wp\":")); Serial.print(n_wall);
  Serial.print(F(",\"rel\":")); Serial.print(relearn_count);
  Serial.print(F(",\"A\":")); Serial.print(polygonArea(hull, n_hull), 3);
  Serial.print(F(",\"P\":")); Serial.print(polygonPerim(hull, n_hull), 3);
  Serial.println(F("}"));
}

// ============================= 8. SETUP/LOOP ================================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_ENA, OUTPUT); pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_US1_TRIG, OUTPUT); pinMode(PIN_US1_ECHO, INPUT);
  pinMode(PIN_US2_TRIG, OUTPUT); pinMode(PIN_US2_ECHO, INPUT);
  pinMode(PIN_BUMPER, INPUT_PULLUP);
  pinMode(PIN_BTN, INPUT_PULLUP);
  steerServo.attach(PIN_SERVO);
  steerServo.write(90);
  stopMotors();

  Serial.println(F("{\"event\":\"BOOT\",\"fw\":\"recorredor_uno_tinkercad\",\"mode\":\"recto+reaprendizaje\"}"));

  // Esperar botón en A0 o arrancar solo a los 3 s (cómodo en Tinkercad)
  uint32_t t0 = millis();
  while (millis() - t0 < 3000) {
    if (digitalRead(PIN_BTN) == LOW) break;
    delay(20);
  }
  target_heading = pose.theta;
  state = State::EXPLORE;
  Serial.println(F("{\"event\":\"MISSION_STARTED\"}"));
}

void loop() {
  static uint32_t last_ms = 0;
  uint32_t now = millis();
  if (now - last_ms < LOOP_MS) return;
  float dt = (now - last_ms) * 0.001f;
  last_ms = now;

  float d1 = readUltra(PIN_US1_TRIG, PIN_US1_ECHO);   // frontal izquierdo
  float d2 = readUltra(PIN_US2_TRIG, PIN_US2_ECHO);   // frontal derecho
  float front = min(d1, d2);                        // distancia frontal efectiva
  bool bumper = (digitalRead(PIN_BUMPER) == LOW);

  State prev = state;
  updateOdom(dt);
  fsmStep(front, bumper, dt);
  telemetry(prev != state);

  if (state == State::DONE) {
    stopMotors();
    Serial.print(F("{\"event\":\"COMPLETED\",\"A\":")); Serial.print(polygonArea(hull, n_hull), 3);
    Serial.print(F(",\"P\":")); Serial.print(polygonPerim(hull, n_hull), 3);
    Serial.print(F(",\"rel\":")); Serial.print(relearn_count);
    Serial.print(F(",\"dist\":")); Serial.print(dist_m, 2);
    Serial.println(F("}"));
    while (true) delay(500);                        // fin de misión
  }
}
