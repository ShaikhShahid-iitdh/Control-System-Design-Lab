/*
 * ============================================================
 *  Furuta Pendulum — IMPULSE RESPONSE EXPERIMENT
 *  Hardware : Arduino MEGA 2560 + L298P Motor Driver Shield
 * ============================================================
 *
 * PURPOSE:
 *   Apply a short-duration high-amplitude torque pulse (impulse
 *   approximation) to the arm motor and record the free response
 *   of both the arm (θ) and pendulum (α).
 *
 * EXPERIMENT PROCEDURE:
 *   1. Place pendulum in DOWN position (hanging naturally).
 *   2. Upload this sketch, open Serial Monitor at 115200 baud.
 *   3. Send 'g' to trigger one impulse shot.
 *   4. The sketch will:
 *        a. Record 0.5 s baseline (motor off)
 *        b. Fire motor at IMPULSE_PWM for IMPULSE_MS
 *        c. Brake immediately, record 3 s of free decay
 *   5. Repeat with different IMPULSE_PWM / IMPULSE_MS values.
 *
 * SUGGESTED PARAMETER SWEEPS:
 *   IMPULSE_MS  : 30, 50, 80, 100 ms
 *   IMPULSE_PWM : 150, 200, 255
 *
 * WIRING: Same as furuta_main.ino
 * ─────────────────────────────────────────────────────────────
 */

// ─────────────────────────────────────────────────────────────
//  PINS
// ─────────────────────────────────────────────────────────────
#define MOTOR_ENA   10
#define MOTOR_IN1    8
#define MOTOR_IN2    7

#define ENC_ARM_A    2
#define ENC_ARM_B    3
#define ENC_PEN_A   18
#define ENC_PEN_B   19

// ─────────────────────────────────────────────────────────────
//  EXPERIMENT PARAMETERS — EDIT THESE
// ─────────────────────────────────────────────────────────────
const int   IMPULSE_PWM  = 255;   // Peak PWM amplitude (0–255)
const unsigned long IMPULSE_MS  = 50;    // Pulse width (ms) — keep short!
const unsigned long PRE_MS      = 500;   // Baseline before pulse (ms)
const unsigned long POST_MS     = 3000;  // Free-decay recording after pulse (ms)
const unsigned long SAMPLE_MS   = 5;     // Sampling period → 200 Hz

// ─────────────────────────────────────────────────────────────
//  ENCODER CONSTANTS
// ─────────────────────────────────────────────────────────────
const float ARM_CPR      = 4096.0;
const float PENDULUM_CPR = 4096.0;

// ─────────────────────────────────────────────────────────────
//  GLOBALS
// ─────────────────────────────────────────────────────────────
volatile long enc_arm = 0;
volatile long enc_pen = 0;
volatile bool arm_B   = false;
volatile bool pen_B   = false;

// ─────────────────────────────────────────────────────────────
//  ISRs
// ─────────────────────────────────────────────────────────────
void ISR_arm_A() { bool a = digitalRead(ENC_ARM_A); (a == arm_B) ? enc_arm-- : enc_arm++; }
void ISR_arm_B() { arm_B = digitalRead(ENC_ARM_B);  bool a = digitalRead(ENC_ARM_A); (a == arm_B) ? enc_arm++ : enc_arm--; }
void ISR_pen_A() { bool a = digitalRead(ENC_PEN_A); (a == pen_B) ? enc_pen-- : enc_pen++; }
void ISR_pen_B() { pen_B = digitalRead(ENC_PEN_B);  bool a = digitalRead(ENC_PEN_A); (a == pen_B) ? enc_pen++ : enc_pen--; }

// ─────────────────────────────────────────────────────────────
//  MOTOR HELPERS
// ─────────────────────────────────────────────────────────────
void setMotor(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0)      { digitalWrite(MOTOR_IN1, HIGH); digitalWrite(MOTOR_IN2, LOW);  analogWrite(MOTOR_ENA, pwm);  }
  else if (pwm < 0) { digitalWrite(MOTOR_IN1, LOW);  digitalWrite(MOTOR_IN2, HIGH); analogWrite(MOTOR_ENA, -pwm); }
  else              { digitalWrite(MOTOR_IN1, HIGH);  digitalWrite(MOTOR_IN2, HIGH); analogWrite(MOTOR_ENA, 0);    }
}

void brakeMotor() {
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, HIGH);
  analogWrite(MOTOR_ENA, 255);
}

// ─────────────────────────────────────────────────────────────
//  LOG ONE DATA POINT
// ─────────────────────────────────────────────────────────────
void logData(unsigned long t_ms, int pwm_val) {
  noInterrupts();
  long a = enc_arm;
  long p = enc_pen;
  interrupts();

  float theta = (float)a * (2.0f * PI) / ARM_CPR;
  float alpha = (float)p * (2.0f * PI) / PENDULUM_CPR;

  Serial.print(t_ms);      Serial.print(',');
  Serial.print(theta, 5);  Serial.print(',');
  Serial.print(alpha, 5);  Serial.print(',');
  Serial.println(pwm_val);
}

// ─────────────────────────────────────────────────────────────
//  RUN ONE IMPULSE EXPERIMENT
// ─────────────────────────────────────────────────────────────
void runImpulseExperiment() {
  Serial.println(F("# === IMPULSE RESPONSE EXPERIMENT ==="));
  Serial.print(F("# IMPULSE_PWM=")); Serial.println(IMPULSE_PWM);
  Serial.print(F("# IMPULSE_MS="));  Serial.println(IMPULSE_MS);
  Serial.print(F("# PRE_MS="));      Serial.println(PRE_MS);
  Serial.print(F("# POST_MS="));     Serial.println(POST_MS);
  Serial.println(F("# ---"));
  Serial.println(F("time_ms,theta_rad,alpha_rad,motor_pwm"));

  // Zero encoders
  noInterrupts();
  enc_arm = 0;
  enc_pen = 0;
  interrupts();

  unsigned long t0 = millis();

  // PHASE 1: Pre-impulse baseline
  unsigned long phase_end = t0 + PRE_MS;
  while (millis() < phase_end) {
    logData(millis() - t0, 0);
    delay(SAMPLE_MS);
  }

  // PHASE 2: Impulse — fire and time precisely
  unsigned long t_impulse_start = millis();
  setMotor(IMPULSE_PWM);

  // Use busy-wait for accurate short pulse timing
  while ((millis() - t_impulse_start) < IMPULSE_MS) {
    // Tight loop — do NOT add delay() here
    // Log at high speed during impulse
    logData(millis() - t0, IMPULSE_PWM);
  }

  // PHASE 3: Brake immediately & record free decay
  brakeMotor();
  phase_end = millis() + POST_MS;
  while (millis() < phase_end) {
    logData(millis() - t0, 0);
    delay(SAMPLE_MS);
  }

  Serial.println(F("# EXPERIMENT COMPLETE"));
  Serial.println(F("# Send 'g' to run again"));
}

// ─────────────────────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  brakeMotor();

  pinMode(ENC_ARM_A, INPUT_PULLUP);
  pinMode(ENC_ARM_B, INPUT_PULLUP);
  pinMode(ENC_PEN_A, INPUT_PULLUP);
  pinMode(ENC_PEN_B, INPUT_PULLUP);

  arm_B = digitalRead(ENC_ARM_B);
  pen_B = digitalRead(ENC_PEN_B);

  attachInterrupt(digitalPinToInterrupt(ENC_ARM_A), ISR_arm_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_ARM_B), ISR_arm_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_PEN_A), ISR_pen_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_PEN_B), ISR_pen_B, CHANGE);

  Serial.println(F("=== Impulse Response Experiment ==="));
  Serial.println(F("Place pendulum in DOWN position, then send 'g' to fire."));
}

// ─────────────────────────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────────────────────────
void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'g') runImpulseExperiment();
  }
}
