// Capa de Abstracción de Hardware (HAL) para ESP32: Encoders ISR, Ultrasonido, Bumpers, IMU y PWM
#ifndef RECORREDOR_HAL_ESP32_H
#define RECORREDOR_HAL_ESP32_H

#include <Arduino.h>
#include <Wire.h>
#include "config.h"

// Contadores volátiles modificados dentro de interrupciones en IRAM
volatile int32_t g_enc_left_ticks  = 0;
volatile int32_t g_enc_right_ticks = 0;
volatile bool    g_bumper_hit_flag = false;

void IRAM_ATTR isrEncoderLeft() {
    if (digitalRead(PIN_ENC_L_B) == HIGH) g_enc_left_ticks++;
    else g_enc_left_ticks--;
}

void IRAM_ATTR isrEncoderRight() {
    if (digitalRead(PIN_ENC_R_B) == HIGH) g_enc_right_ticks++;
    else g_enc_right_ticks--;
}

void IRAM_ATTR isrBumperHit() {
    g_bumper_hit_flag = true;
}

class RecorredorHAL {
public:
    float gyro_bias_z_radps = 0.0f;
    bool mpu_available = false;

    void begin() {
        // Configurar pines de puente H TB6612FNG
        pinMode(PIN_MOTOR_L_IN1, OUTPUT);
        pinMode(PIN_MOTOR_L_IN2, OUTPUT);
        pinMode(PIN_MOTOR_R_IN1, OUTPUT);
        pinMode(PIN_MOTOR_R_IN2, OUTPUT);
        pinMode(PIN_MOTOR_STBY, OUTPUT);
        digitalWrite(PIN_MOTOR_STBY, HIGH);

        // Configurar PWM para motores y servomotor (API compatible ESP32 Arduino Core)
        pinMode(PIN_MOTOR_L_PWM, OUTPUT);
        pinMode(PIN_MOTOR_R_PWM, OUTPUT);
        pinMode(PIN_SERVO_STEER, OUTPUT);

        // Configurar Encoders
        pinMode(PIN_ENC_L_A, INPUT);
        pinMode(PIN_ENC_L_B, INPUT);
        pinMode(PIN_ENC_R_A, INPUT);
        pinMode(PIN_ENC_R_B, INPUT);
        attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrEncoderLeft, RISING);
        attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrEncoderRight, RISING);

        // Configurar Bumpers de impacto con pull-up interno
        pinMode(PIN_BUMPER_LEFT, INPUT_PULLUP);
        pinMode(PIN_BUMPER_RIGHT, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(PIN_BUMPER_LEFT), isrBumperHit, FALLING);
        attachInterrupt(digitalPinToInterrupt(PIN_BUMPER_RIGHT), isrBumperHit, FALLING);

        // Configurar Ultrasonido HC-SR04
        pinMode(PIN_ULTRA_TRIG, OUTPUT);
        pinMode(PIN_ULTRA_ECHO, INPUT);
        digitalWrite(PIN_ULTRA_TRIG, LOW);

        // Configurar Botón de inicio
        pinMode(PIN_BTN_START, INPUT_PULLUP);

        // Inicializar I2C y MPU6050 (dirección 0x68)
        Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
        initMPU6050();
        stopMotors();
        setServoAngleRad(0.0f);
    }

    void initMPU6050() {
        Wire.beginTransmission(0x68);
        Wire.write(0x6B); // Registro PWR_MGMT_1
        Wire.write(0x00); // Despertar MPU6050
        if (Wire.endTransmission() == 0) {
            mpu_available = true;
            // Calibrar bias del giroscopio Z en reposo (100 muestras)
            float sum = 0.0f;
            for (int i = 0; i < 100; ++i) {
                sum += readRawGyroZRadps();
                delay(5);
            }
            gyro_bias_z_radps = sum / 100.0f;
        }
    }

    float readRawGyroZRadps() {
        if (!mpu_available) return 0.0f;
        Wire.beginTransmission(0x68);
        Wire.write(0x47); // GYRO_ZOUT_H
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)0x68, (uint8_t)2, (uint8_t)true);
        if (Wire.available() < 2) return 0.0f;
        int16_t raw_gz = (Wire.read() << 8) | Wire.read();
        // Escala por defecto +-250 deg/s -> 131 LSB/(deg/s)
        float deg_per_s = (float)raw_gz / 131.0f;
        return deg_per_s * DEG_TO_RAD;
    }

    float readCalibratedGyroZRadps() {
        return readRawGyroZRadps() - gyro_bias_z_radps;
    }

    void consumeEncoderDeltas(int32_t& out_dl, int32_t& out_dr) {
        static int32_t prev_l = 0;
        static int32_t prev_r = 0;
        noInterrupts();
        int32_t curr_l = g_enc_left_ticks;
        int32_t curr_r = g_enc_right_ticks;
        interrupts();
        out_dl = curr_l - prev_l;
        out_dr = curr_r - prev_r;
        prev_l = curr_l;
        prev_r = curr_r;
    }

    bool isBumperTriggered() {
        bool hit = g_bumper_hit_flag || (digitalRead(PIN_BUMPER_LEFT) == LOW) || (digitalRead(PIN_BUMPER_RIGHT) == LOW);
        g_bumper_hit_flag = false;
        return hit;
    }

    float readFrontDistanceMeters() {
        digitalWrite(PIN_ULTRA_TRIG, LOW);
        delayMicroseconds(2);
        digitalWrite(PIN_ULTRA_TRIG, HIGH);
        delayMicroseconds(10);
        digitalWrite(PIN_ULTRA_TRIG, LOW);

        // Timeout corto de 12 ms (~2 metros máx) para no bloquear el lazo de 50 Hz
        unsigned long duration_us = pulseIn(PIN_ULTRA_ECHO, HIGH, 12000UL);
        if (duration_us == 0) return 2.0f;
        return (duration_us * 0.000343f) * 0.5f;
    }

    void stopMotors() {
        digitalWrite(PIN_MOTOR_L_IN1, HIGH);
        digitalWrite(PIN_MOTOR_L_IN2, HIGH);
        digitalWrite(PIN_MOTOR_R_IN1, HIGH);
        digitalWrite(PIN_MOTOR_R_IN2, HIGH);
        analogWrite(PIN_MOTOR_L_PWM, 0);
        analogWrite(PIN_MOTOR_R_PWM, 0);
    }

    void setDifferentialVelocity(float linear_mps, float angular_radps) {
        float v_left  = linear_mps - 0.5f * WHEELBASE_M * angular_radps;
        float v_right = linear_mps + 0.5f * WHEELBASE_M * angular_radps;
        setSingleMotor(PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2, PIN_MOTOR_L_PWM, v_left);
        setSingleMotor(PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2, PIN_MOTOR_R_PWM, v_right);
    }

    void setServoAngleRad(float angle_rad) {
        // Limitar entre -45° y +45° respecto al centro (90°)
        float clamped_deg = constrain(angle_rad * RAD_TO_DEG, -45.0f, 45.0f);
        float servo_deg = 90.0f + clamped_deg;
        // Mapear a ciclo útil de 8 bits si se usa PWM o pulso estándar
        int duty = (int)map((long)servo_deg, 0, 180, 26, 128);
        analogWrite(PIN_SERVO_STEER, duty);
    }

private:
    void setSingleMotor(uint8_t in1, uint8_t in2, uint8_t pwm_pin, float speed_mps) {
        const float max_speed = 0.50f;
        int pwm_val = (int)(constrain(fabsf(speed_mps) / max_speed, 0.0f, 1.0f) * 255.0f);
        if (pwm_val < 25) {
            digitalWrite(in1, LOW);
            digitalWrite(in2, LOW);
            analogWrite(pwm_pin, 0);
            return;
        }
        if (speed_mps >= 0.0f) {
            digitalWrite(in1, HIGH);
            digitalWrite(in2, LOW);
        } else {
            digitalWrite(in1, LOW);
            digitalWrite(in2, HIGH);
        }
        analogWrite(pwm_pin, pwm_val);
    }
};

#endif // RECORREDOR_HAL_ESP32_H
