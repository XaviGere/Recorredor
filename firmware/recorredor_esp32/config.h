// Configuración de Pines y Calibración Física — Firmware Recorredor ESP32
#ifndef RECORREDOR_CONFIG_H
#define RECORREDOR_CONFIG_H

#include <Arduino.h>

// ============================================================================
// 1. PINOUT ESP32 DEVKIT V1 (Coincide con docs/Recorredor-electronica-y-circuito.md)
// ============================================================================

// Puente H TB6612FNG - Motor Izquierdo (Canal A)
constexpr uint8_t PIN_MOTOR_L_PWM  = 25;
constexpr uint8_t PIN_MOTOR_L_IN1  = 26;
constexpr uint8_t PIN_MOTOR_L_IN2  = 27;

// Puente H TB6612FNG - Motor Derecho (Canal B)
constexpr uint8_t PIN_MOTOR_R_PWM  = 32;
constexpr uint8_t PIN_MOTOR_R_IN1  = 33;
constexpr uint8_t PIN_MOTOR_R_IN2  = 14;

// Habilitación del Driver TB6612FNG
constexpr uint8_t PIN_MOTOR_STBY   = 13;

// Servomotor de Dirección / Escaneo Frontal
constexpr uint8_t PIN_SERVO_STEER  = 18;

// Encoders Cuadratura (Pines Input-Only del ESP32 con pull-up externo de 10k a 3.3V)
constexpr uint8_t PIN_ENC_L_A      = 34;
constexpr uint8_t PIN_ENC_L_B      = 35;
constexpr uint8_t PIN_ENC_R_A      = 36; // VP
constexpr uint8_t PIN_ENC_R_B      = 39; // VN

// Sensor de Proximidad Ultrasónico HC-SR04 (¡ECHO con divisor resistivo 1k/2k a 3.3V!)
constexpr uint8_t PIN_ULTRA_TRIG   = 19;
constexpr uint8_t PIN_ULTRA_ECHO   = 23;

// Microswitches de Impacto (Bumpers físicos con INPUT_PULLUP interno)
constexpr uint8_t PIN_BUMPER_LEFT  = 16;
constexpr uint8_t PIN_BUMPER_RIGHT = 17;

// Bus I2C para IMU MPU6050 (Giroscopio + Acelerómetro)
constexpr uint8_t PIN_I2C_SDA      = 21;
constexpr uint8_t PIN_I2C_SCL      = 22;

// Botón de Inicio de Misión
constexpr uint8_t PIN_BTN_START    = 4;

// ============================================================================
// 2. CONFIGURACIÓN WI-FI Y TELEMETRÍA EN VIVO (UDP -> PC Dashboard)
// ============================================================================

constexpr bool ENABLE_WIFI_TELEMETRY    = true;
constexpr const char* WIFI_SSID         = "TU_RED_WIFI";
constexpr const char* WIFI_PASSWORD     = "TU_CLAVE_WIFI";
constexpr const char* TELEMETRY_UDP_IP  = "255.255.255.255"; // Broadcast en la red local (o IP de tu PC)
constexpr uint16_t TELEMETRY_UDP_PORT   = 5005;              // Puerto que escucha `python -m sim.live_server`

// ============================================================================
// 3. CONSTANTES FÍSICAS Y DE CALIBRACIÓN (Unidades SI: metros, radianes, segundos)
// ============================================================================
// Decisión confirmada: Chasis de 2 ruedas motrices + rueda loca + servomotor
// Estimación de distancia por cinemática directa (d = v * dt) y ángulo por omega * dt (o IMU)

constexpr bool USE_OPEN_LOOP_VT_ODOM    = true;    // true = estima distancia con v_cmd * dt sin requerir encoders
constexpr float ROBOT_RADIUS_M          = 0.12f;   // Radio envolvente del chasis (12 cm)
constexpr float WHEEL_RADIUS_M          = 0.0325f; // Radio de rueda motriz (32.5 mm)
constexpr float WHEELBASE_M             = 0.18f;   // Distancia entre ruedas motrices (18 cm)
constexpr float TICKS_PER_REV           = 720.0f;  // Pulsos de encoder (opcional si USE_OPEN_LOOP_VT_ODOM = false)
constexpr float METERS_PER_TICK         = (2.0f * PI * WHEEL_RADIUS_M) / TICKS_PER_REV;
constexpr float LINEAR_SPEED_CALIB_GAIN = 1.00f;   // Factor de ajuste empírico para calibrar d = (k * v) * t

// Parámetros de frenado y margen perimetral
constexpr float STOP_DISTANCE_M         = 0.04f;   // Frenar a 4 cm de la pared
constexpr float BACKUP_DISTANCE_M       = 0.04f;   // Retroceso si toca el bumper
constexpr float SAFE_PERIMETER_MARGIN_M = ROBOT_RADIUS_M * 1.2f; // Radio del círculo * 1.2 (0.144 m)

// Lazo de control en tiempo real (50 Hz = 20 ms)
constexpr uint32_t CONTROL_LOOP_PERIOD_MS = 20;
constexpr float CRUISE_SPEED_MPS          = 0.35f;
constexpr float PERIMETER_SPEED_MPS       = 0.28f;
constexpr float MAX_ANGULAR_SPEED_RADPS   = 2.2f;
constexpr float HEADING_KP                = 3.8f;
constexpr float WAYPOINT_TOLERANCE_M      = 0.08f;
constexpr float GYRO_COMPLEMENTARY_ALPHA  = 0.95f; // 95% Giroscopio + 5% Odometría diferencial

// Capacidad estática de memoria para mapeo en SRAM del ESP32
constexpr uint16_t MAX_CHORDS           = 128;
constexpr uint16_t MAX_HULL_VERTICES    = 32;
constexpr uint16_t MIN_CHORDS_FOR_MAP   = 16;
constexpr uint16_t MAX_CHORDS_EXPLORE   = 32;
constexpr float ANTS_BOUNDARY_FACTOR    = 1.18f;
constexpr float AREA_STABILITY_TOL      = 0.03f;

#endif // RECORREDOR_CONFIG_H
