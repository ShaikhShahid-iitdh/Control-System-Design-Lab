/*
  Furuta Pendulum Hardware Controller (Arduino)

  Features:
  - Quadrature encoder readout (base + pendulum) using Encoder library
  - Fixed-rate control loop (2 ms)
  - LQR feedback: u = -Kx
  - Voltage saturation and PWM mapping
  - Safety interlocks (angle limit, startup disabled, serial arm/disarm)
  - CSV telemetry over serial for logging and report plots

  Required library:
  - Encoder by Paul Stoffregen

  IMPORTANT:
  1) Verify pin mapping for your hardware.
  2) Verify encoder counts-per-rev and angle sign conventions.
  3) Verify motor driver interface (PWM + DIR).
  4) Set pendulum upright offset before enabling control.
*/

#include <Arduino.h>
#include <Encoder.h>

// ----------------------------
// User hardware configuration
// ----------------------------

// Arduino Mega + Arduino Motor Shield Rev3 (L298P)
// Common shield pinout:
//   Motor A: PWM=D3,  DIR=D12, BRAKE=D9,  CURRENT_SENSE=A0
//   Motor B: PWM=D11, DIR=D13, BRAKE=D8,  CURRENT_SENSE=A1
// Select one motor channel below.
#define USE_MOTOR_CHANNEL_A 1

// Use encoder pins that do not conflict with shield control pins.
const int BASE_ENC_PIN_A = 20;
const int BASE_ENC_PIN_B = 21;
const int PEND_ENC_PIN_A = 18;
const int PEND_ENC_PIN_B = 19;

#if USE_MOTOR_CHANNEL_A
const int MOTOR_PWM_PIN = 3;
const int MOTOR_DIR_PIN = 12;
const int MOTOR_BRAKE_PIN = 9;
const int MOTOR_CURRENT_SENSE_PIN = A0;
#else
const int MOTOR_PWM_PIN = 11;
const int MOTOR_DIR_PIN = 13;
const int MOTOR_BRAKE_PIN = 8;
const int MOTOR_CURRENT_SENSE_PIN = A1;
#endif

// Supply and command limits
const float SUPPLY_VOLTAGE = 12.0f;
const float COMMAND_VOLTAGE_LIMIT = 10.0f;
const int PWM_MAX = 255;

// Encoder scaling: counts per mechanical revolution
const float BASE_COUNTS_PER_REV = 4096.0f;
const float PEND_COUNTS_PER_REV = 4096.0f;

// If your encoder sign is opposite, flip with -1.0f
const float BASE_SIGN = 1.0f;
const float PEND_SIGN = 1.0f;

// Pendulum encoder offset [rad] to make upright = 0 rad
// Set this after calibration, or use serial command 'z' for zero-at-current.
volatile float pendulum_upright_offset_rad = 0.0f;

// Control rate
const float TS = 0.002f;                     // 2 ms
const unsigned long TS_US = 2000UL;

// Safety
const float MAX_PEND_ANGLE_FOR_CONTROL = 1.2f;   // rad (~68.8 deg), disarm if exceeded
const float ARM_WINDOW = 0.35f;                  // rad (~20 deg), must be near upright to arm

// LQR gain from report: K = [-1.0000, 33.0576, -0.8563, 4.5040]
// State x = [theta_b, theta_p, theta_b_dot, theta_p_dot]
const float K1 = -1.0000f;
const float K2 = 33.0576f;
const float K3 = -0.8563f;
const float K4 = 4.5040f;

// Velocity estimation filter (first-order LPF on backward difference)
const float VEL_ALPHA = 0.25f;  // larger = faster, noisier

// ----------------------------
// Internal state
// ----------------------------

Encoder encBase(BASE_ENC_PIN_A, BASE_ENC_PIN_B);
Encoder encPend(PEND_ENC_PIN_A, PEND_ENC_PIN_B);

bool control_enabled = false;
unsigned long next_control_us = 0;
unsigned long last_log_ms = 0;

float theta_b = 0.0f;
float theta_p = 0.0f;
float theta_b_dot = 0.0f;
float theta_p_dot = 0.0f;

float theta_b_prev = 0.0f;
float theta_p_prev = 0.0f;

long base_count_zero = 0;
long pend_count_zero = 0;

// ----------------------------
// Helpers
// ----------------------------

float wrapToPi(float a) {
  while (a > PI) a -= 2.0f * PI;
  while (a < -PI) a += 2.0f * PI;
  return a;
}

float countsToRad(long counts, float counts_per_rev, float sign) {
  return sign * (2.0f * PI) * ((float)counts / counts_per_rev);
}

float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

void setMotorVoltage(float v_cmd) {
  float v = clampf(v_cmd, -COMMAND_VOLTAGE_LIMIT, COMMAND_VOLTAGE_LIMIT);

  bool dir = (v >= 0.0f);
  float duty = fabs(v) / SUPPLY_VOLTAGE;
  duty = clampf(duty, 0.0f, 1.0f);

  int pwm = (int)(duty * (float)PWM_MAX);

  digitalWrite(MOTOR_DIR_PIN, dir ? HIGH : LOW);
  digitalWrite(MOTOR_BRAKE_PIN, LOW);
  analogWrite(MOTOR_PWM_PIN, pwm);
}

void stopMotor() {
  digitalWrite(MOTOR_BRAKE_PIN, HIGH);
  analogWrite(MOTOR_PWM_PIN, 0);
}

void disarmControl(const char* reason) {
  control_enabled = false;
  stopMotor();
  Serial.print("INFO,DISARM,");
  Serial.println(reason);
}

void armControlIfSafe() {
  if (fabs(theta_p) <= ARM_WINDOW) {
    control_enabled = true;
    Serial.println("INFO,ARMED");
  } else {
    Serial.print("INFO,ARM_REJECT,pend_angle_rad=");
    Serial.println(theta_p, 6);
  }
}

void zeroEncodersAtCurrent() {
  base_count_zero = encBase.read();
  pend_count_zero = encPend.read();
  theta_b_prev = 0.0f;
  theta_p_prev = 0.0f;
  theta_b_dot = 0.0f;
  theta_p_dot = 0.0f;
  Serial.println("INFO,ZERO_DONE");
}

void zeroPendulumUprightAtCurrent() {
  long pend_rel = encPend.read() - pend_count_zero;
  float pend_now = countsToRad(pend_rel, PEND_COUNTS_PER_REV, PEND_SIGN);
  pendulum_upright_offset_rad = pend_now;
  Serial.print("INFO,UPRIGHT_OFFSET_SET,");
  Serial.println(pendulum_upright_offset_rad, 6);
}

void updateStateEstimate() {
  long base_rel = encBase.read() - base_count_zero;
  long pend_rel = encPend.read() - pend_count_zero;

  float base_angle_raw = countsToRad(base_rel, BASE_COUNTS_PER_REV, BASE_SIGN);
  float pend_angle_raw = countsToRad(pend_rel, PEND_COUNTS_PER_REV, PEND_SIGN);

  theta_b = wrapToPi(base_angle_raw);
  theta_p = wrapToPi(pend_angle_raw - pendulum_upright_offset_rad);

  float db = (theta_b - theta_b_prev) / TS;
  float dp = (theta_p - theta_p_prev) / TS;

  theta_b_dot = (1.0f - VEL_ALPHA) * theta_b_dot + VEL_ALPHA * db;
  theta_p_dot = (1.0f - VEL_ALPHA) * theta_p_dot + VEL_ALPHA * dp;

  theta_b_prev = theta_b;
  theta_p_prev = theta_p;
}

void processSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == 'e') {
      armControlIfSafe();
    } else if (c == 'd') {
      disarmControl("USER");
    } else if (c == 'x') {
      disarmControl("ESTOP");
    } else if (c == 'z') {
      zeroEncodersAtCurrent();
    } else if (c == 'u') {
      zeroPendulumUprightAtCurrent();
    } else if (c == 'h') {
      Serial.println("INFO,CMDS,e=arm,d=disarm,x=estop,z=zero_enc,u=set_upright_offset,h=help");
    }
  }
}

void setup() {
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  pinMode(MOTOR_BRAKE_PIN, OUTPUT);
  pinMode(MOTOR_CURRENT_SENSE_PIN, INPUT);

  digitalWrite(MOTOR_BRAKE_PIN, HIGH);
  stopMotor();

  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  delay(300);

  zeroEncodersAtCurrent();
  next_control_us = micros() + TS_US;

  Serial.println("INFO,BOOT_OK");
#if USE_MOTOR_CHANNEL_A
  Serial.println("INFO,Shield=Arduino Motor Shield Rev3 (L298P), channel=A");
#else
  Serial.println("INFO,Shield=Arduino Motor Shield Rev3 (L298P), channel=B");
#endif
  Serial.println("INFO,Controller disabled by default. Put pendulum upright and send 'u' then 'e'.");
  Serial.println("t_ms,theta_b,theta_p,theta_b_dot,theta_p_dot,u_cmd,enabled");
}

void loop() {
  processSerialCommands();

  unsigned long now_us = micros();
  if ((long)(now_us - next_control_us) >= 0) {
    next_control_us += TS_US;

    updateStateEstimate();

    // LQR law: u = -Kx
    float u_cmd = -(K1 * theta_b + K2 * theta_p + K3 * theta_b_dot + K4 * theta_p_dot);

    if (control_enabled) {
      if (fabs(theta_p) > MAX_PEND_ANGLE_FOR_CONTROL) {
        disarmControl("ANGLE_LIMIT");
      } else {
        setMotorVoltage(u_cmd);
      }
    } else {
      stopMotor();
      u_cmd = 0.0f;
    }

    // Telemetry at ~100 Hz
    unsigned long now_ms = millis();
    if (now_ms - last_log_ms >= 10) {
      last_log_ms = now_ms;
      Serial.print(now_ms);
      Serial.print(',');
      Serial.print(theta_b, 6);
      Serial.print(',');
      Serial.print(theta_p, 6);
      Serial.print(',');
      Serial.print(theta_b_dot, 6);
      Serial.print(',');
      Serial.print(theta_p_dot, 6);
      Serial.print(',');
      Serial.print(u_cmd, 6);
      Serial.print(',');
      Serial.println(control_enabled ? 1 : 0);
    }
  }
}
