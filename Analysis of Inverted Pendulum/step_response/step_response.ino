/*
 * ============================================================
 *  Furuta Pendulum — STEP RESPONSE EXPERIMENT
 *  Hardware : Arduino MEGA 2560 + L298P Motor Driver Shield
 * ============================================================
 *
 * PURPOSE:
 *   Apply a step input (constant motor voltage) to the arm motor
 *   and record the angular response of BOTH the arm (θ) and
 *   pendulum (α) over time. Repeat for multiple PWM levels.
 *
 * EXPERIMENT PROCEDURE:
 *   1. Place pendulum in DOWN position (hanging naturally).
 *   2. Upload this sketch and open Serial Monitor at 115200 baud.
 *   3. Type 'g' + Enter to start a single step experiment.
 *   4. The sketch will:
 *        a. Log 0.5 s of baseline (motor off)
 *        b. Apply STEP_PWM for STEP_DURATION_MS
 *        c. Brake and log 1 s of decay
 *   5. Copy the CSV output into a file for MATLAB/Python analysis.
 *   6. Repeat for different STEP_PWM values (suggested: 50, 100, 150, 200).
 *
 * STATE-SPACE IDENTIFICATION FROM STEP DATA:
 *   States  : x = [θ, θ̇, α, α̇]
 *   Input   : u = motor PWM (proxy for voltage/torque)
 *   Output  : y = [θ, α]
 *   Use MATLAB System Identification Toolbox (ssest) or
 *   scipy.signal in Python to fit a state-space model.
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

#define ENC_ARM_A    2   // INT0
#define ENC_ARM_B    3   // INT1
#define ENC_PEN_A   18   // INT5
#define ENC_PEN_B   19   // INT4

// ─────────────────────────────────────────────────────────────
//  EXPERIMENT PARAMETERS — EDIT THESE
// ─────────────────────────────────────────────────────────────
const int   STEP_PWM          = 120;    // Step amplitude (0–255)
const unsigned long PRE_MS    = 500;    // Baseline recording before step (ms)
const unsigned long STEP_MS   = 1500;   // Duration motor is ON (ms)
const unsigned long POST_MS   = 2000;   // Recording after motor is OFF (ms)
const unsigned long SAMPLE_MS = 5;      // Sampling period → 200 Hz

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

bool  experiment_done = false;
bool  waiting         = true;

// ─────────────────────────────────────────────────────────────
//  ISRs
// ─────────────────────────────────────────────────────────────
void ISR_arm_A() { bool a = digitalRead(ENC_ARM_A); (a == arm_B) ? enc_arm-- : enc_arm++; }
void ISR_arm_B() { arm_B  = digitalRead(ENC_ARM_B); bool a = digitalRead(ENC_ARM_A); (a == arm_B) ? enc_arm++ : enc_arm--; }
void ISR_pen_A() { bool a = digitalRead(ENC_PEN_A); (a == pen_B) ? enc_pen-- : enc_pen++; }
void ISR_pen_B() { pen_B  = digitalRead(ENC_PEN_B); bool a = digitalRead(ENC_PEN_A); (a == pen_B) ? enc_pen++ : enc_pen--; }

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
//  READ ENCODERS — returns angles in radians
// ─────────────────────────────────────────────────────────────
void readAngles(float &theta, float &alpha) {
  noInterrupts();
  long a = enc_arm;
  long p = enc_pen;
  interrupts();
  theta = (float)a * (2.0f * PI) / ARM_CPR;
  alpha = (float)p * (2.0f * PI) / PENDULUM_CPR;
}

// ─────────────────────────────────────────────────────────────
//  LOG ONE DATA POINT
// ─────────────────────────────────────────────────────────────
void logData(unsigned long t_ms, int pwm_val) {
  float theta, alpha;
  readAngles(theta, alpha);
  Serial.print(t_ms);       Serial.print(',');
  Serial.print(theta, 5);   Serial.print(',');
  Serial.print(alpha, 5);   Serial.print(',');
  Serial.println(pwm_val);
}

// ─────────────────────────────────────────────────────────────
//  RUN ONE STEP EXPERIMENT
// ─────────────────────────────────────────────────────────────
void runStepExperiment() {
  Serial.println(F("# === STEP RESPONSE EXPERIMENT ==="));
  Serial.print(F("# STEP_PWM=")); Serial.println(STEP_PWM);
  Serial.print(F("# PRE_MS="));  Serial.println(PRE_MS);
  Serial.print(F("# STEP_MS=")); Serial.println(STEP_MS);
  Serial.print(F("# POST_MS=")); Serial.println(POST_MS);
  Serial.println(F("# ---"));
  Serial.println(F("time_ms,theta_rad,alpha_rad,motor_pwm"));

  // — Zero encoders at start —
  noInterrupts();
  enc_arm = 0;
  enc_pen = 0;
  interrupts();

  unsigned long t0    = millis();
  unsigned long phase_end;

  // PHASE 1: Pre-step baseline (motor OFF)
  phase_end = t0 + PRE_MS;
  while (millis() < phase_end) {
    unsigned long now = millis();
    logData(now - t0, 0);
    delay(SAMPLE_MS);
  }

  // PHASE 2: Step (motor ON)
  setMotor(STEP_PWM);
  phase_end = millis() + STEP_MS;
  while (millis() < phase_end) {
    unsigned long now = millis();
    logData(now - t0, STEP_PWM);
    delay(SAMPLE_MS);
  }

  // PHASE 3: Post-step decay (motor OFF)
  brakeMotor();
  phase_end = millis() + POST_MS;
  while (millis() < phase_end) {
    unsigned long now = millis();
    logData(now - t0, 0);
    delay(SAMPLE_MS);
  }

  Serial.println(F("# EXPERIMENT COMPLETE"));
  Serial.println(F("# Send 'g' to run again"));
  experiment_done = true;
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

  Serial.println(F("=== Step Response Experiment ==="));
  Serial.println(F("Place pendulum in DOWN position, then send 'g' to start."));
}

// ─────────────────────────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────────────────────────
void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'g') {
      experiment_done = false;
      runStepExperiment();
    }
  }
}
