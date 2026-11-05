// Sketch Principal para ESP32 — Robot Autónomo Recorredor
// Decisiones confirmadas:
//   1. Chasis de 2 ruedas motrices + rueda loca y servomotor de dirección/escaneo.
//   2. Estimación de distancia recorrida por cinemática directa (d = v * t).
//   3. Telemetría en tiempo real vía WiFi UDP (además de Serial) hacia el visor en vivo de Python.

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "config.h"
#include "hal_esp32.h"
#include "geometry_fixed.h"
#include "navigator_fsm.h"

RecorredorHAL hal;
NavigatorFSM  navigator;
WiFiUDP       udp;
bool          wifi_connected = false;
uint32_t      last_loop_ms = 0;
uint32_t      last_udp_stream_ms = 0;

// Velocidades comandadas en el ciclo previo para integrar d = v * dt
float         last_cmd_v_mps = 0.0f;
float         last_cmd_omega_radps = 0.0f;

void setupWiFiTelemetry() {
    if (!ENABLE_WIFI_TELEMETRY) return;
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 4000) {
        delay(100);
    }
    if (WiFi.status() == WL_CONNECTED) {
        wifi_connected = true;
        udp.begin(TELEMETRY_UDP_PORT);
        Serial.print("{\"event\":\"WIFI_CONNECTED\",\"ip\":\"");
        Serial.print(WiFi.localIP());
        Serial.println("\"}");
    } else {
        Serial.println("{\"event\":\"WIFI_TIMEOUT_FALLBACK_SERIAL_ONLY\"}");
    }
}

void broadcastTelemetry() {
    char json_buf[320];
    size_t len = navigator.formatTelemetryJSON(json_buf, sizeof(json_buf));
    Serial.write((const uint8_t*)json_buf, len);
    if (wifi_connected && len > 0) {
        udp.beginPacket(TELEMETRY_UDP_IP, TELEMETRY_UDP_PORT);
        udp.write((const uint8_t*)json_buf, len);
        udp.endPacket();
    }
}

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("{\"event\":\"BOOT\",\"project\":\"Recorredor_ESP32\",\"odom\":\"v*t\",\"chassis\":\"2WD_CASTER_SERVO\"}");

    hal.begin();
    setupWiFiTelemetry();
    last_loop_ms = millis();

    navigator.startMission();
    Serial.println("{\"event\":\"MISSION_STARTED\"}");
}

void loop() {
    uint32_t now_ms = millis();
    uint32_t elapsed_ms = now_ms - last_loop_ms;

    // Lazo de control no bloqueante a 50 Hz (cada 20 ms)
    if (elapsed_ms < CONTROL_LOOP_PERIOD_MS) {
        return;
    }
    last_loop_ms = now_ms;
    float dt_s = elapsed_ms * 0.001f;

    // 1. Leer sensores (Ultrasonido, Bumpers e IMU opcional)
    float gyro_z = hal.readCalibratedGyroZRadps();
    bool bumper_hit = hal.isBumperTriggered();
    float front_dist_m = hal.readFrontDistanceMeters();

    // 2. Actualizar coordenadas (x, y, theta) mediante d = v * dt (o encoders si se desactiva USE_OPEN_LOOP_VT_ODOM)
    if (USE_OPEN_LOOP_VT_ODOM) {
        navigator.updateOdometryVT(last_cmd_v_mps, last_cmd_omega_radps, gyro_z, hal.mpu_available, dt_s);
    } else {
        int32_t dl_ticks = 0, dr_ticks = 0;
        hal.consumeEncoderDeltas(dl_ticks, dr_ticks);
        navigator.updateOdometryEncoders(dl_ticks, dr_ticks, gyro_z, dt_s);
    }

    // 3. Ejecutar un paso de la máquina de estados (FSM)
    MissionState prev_state = navigator.state;
    float cmd_v = 0.0f, cmd_omega = 0.0f, cmd_servo = 0.0f;
    navigator.step(front_dist_m, bumper_hit, dt_s, cmd_v, cmd_omega, cmd_servo);

    // 4. Comandar motores DC y servomotor de dirección
    if (navigator.state == MissionState::BRAKE_RECORD || navigator.state == MissionState::COMPLETED) {
        hal.stopMotors();
        last_cmd_v_mps = 0.0f;
        last_cmd_omega_radps = 0.0f;
    } else {
        hal.setDifferentialVelocity(cmd_v, cmd_omega);
        hal.setServoAngleRad(cmd_servo);
        last_cmd_v_mps = cmd_v;
        last_cmd_omega_radps = cmd_omega;
    }

    // 5. Emitir telemetría en vivo por WiFi UDP (cada 100 ms o en cada impacto de pared)
    bool wall_event = (prev_state == MissionState::EVALUATE_MAP) ||
                      (prev_state != MissionState::COMPLETED && navigator.state == MissionState::COMPLETED);
    if (wall_event || (now_ms - last_udp_stream_ms >= 100)) {
        last_udp_stream_ms = now_ms;
        broadcastTelemetry();
    }
}
