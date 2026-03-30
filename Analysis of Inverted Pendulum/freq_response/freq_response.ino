/*
 * ============================================================
 *  Furuta Pendulum — FREQUENCY RESPONSE EXPERIMENT
 *  Hardware : Arduino MEGA 2560 + L298P Motor Driver Shield
 * ============================================================
 *
 * PURPOSE:
 *   Sweep sinusoidal motor inputs at increasing frequencies to
 *   map out the system's frequency response (Bode plot).
 *   Two modes are available:
 *     MODE 1 — Single-frequency sine test  (send 'f')
 *     MODE 2 — Chirp sweep (auto-increasing freq)  (send 'c')
 *
 * HOW FREQUENCY RESPONSE IS USED FOR STATE-SPACE ID:
 *   At each frequency ω:
 *     |G(jω)| = output amplitude / input amplitude
 *     ∠G(jω) = phase lag between input and output
 *   Fit these to a transfer function → convert to state space.
 *   In MATLAB: use 'invfreqs()' or System ID toolbox.
 *
 * EXPERIMENT PROCEDURE (Chirp sweep):
 *   1. Place pendulum in DOWN position.
 *   2. Send 'c' — the sweep runs automatically through
 *      FREQ_START_HZ to FREQ_END_HZ.
 *   3. Each frequency runs for DWELL_CYCLES sine cycles.
 *   4. Copy full CSV output to file for offline analysis.
 *
 * EXPERIMENT PROCEDURE (Single frequency):
 *   1. Set TEST_FREQ_HZ to desired frequency.
 *   2. Send 'f' — runs SINGLE_DURATION_MS of data.
 *   3. Measure amplitude and phase from the time-domain data.
 *
 * SAFETY: Motor oscillates continuously during test.
 *   Set SINE_AMPLITUDE conservatively (80–150).
 *   Be ready to power off if pendulum swings too wildly.
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
// Shared
const int   SINE_AMPLITUDE    = 120;    // Peak PWM amplitude (0–255)
const unsigned long SAMPLE_MS = 5;      // 200 Hz sampling rate

// MODE 1: Single-frequency sine test
const float SINGLE_FREQ_HZ       = 1.0;    // Test frequency (Hz)
const unsigned long SINGLE_DUR_MS = 5000;  // How long to run (ms)

// MODE 2: Chirp sweep
const float FREQ_START_HZ  = 0.2;   // Start frequency (Hz)
const float FREQ_END_HZ    = 5.0;   // End frequency (Hz)  — stay below 8 Hz
const float FREQ_STEP_HZ   = 0.2;   // Step between frequencies (Hz)
const int   DWELL_CYCLES   = 5;     // Number of sine cycles to record per frequency

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
//  READ ANGLES
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
//  SINE WAVE MOTOR DRIVE + DATA LOGGING
//   freq_hz   : input frequency
//   duration_ms : how long to run
//   t_offset  : time offset so time column is continuous in chirp
// ─────────────────────────────────────────────────────────────
void runSineBlock(float freq_hz, unsigned long duration_ms, unsigned long t_offset) {
  unsigned long t_start = millis();
  unsigned long t_end   = t_start + duration_ms;

  while (millis() < t_end) {
    unsigned long now     = millis();
    float elapsed_s       = (now - t_start) * 1e-3f;

    // Sinusoidal input
    float sine_val = sinf(2.0f * PI * freq_hz * elapsed_s);
    int   pwm      = (int)(SINE_AMPLITUDE * sine_val);
    setMotor(pwm);

    // Log data
    float theta, alpha;
    readAngles(theta, alpha);

    Serial.print(t_offset + (now - t_start));  Serial.print(',');
    Serial.print(freq_hz, 3);   Serial.print(',');
    Serial.print(theta, 5);     Serial.print(',');
    Serial.print(alpha, 5);     Serial.print(',');
    Serial.print(sine_val, 4);  Serial.print(',');
    Serial.println(pwm);

    // Busy-wait for next sample
    while ((millis() - now) < SAMPLE_MS);
  }
}

// ─────────────────────────────────────────────────────────────
//  MODE 1: SINGLE FREQUENCY TEST
// ─────────────────────────────────────────────────────────────
void runSingleFreq() {
  Serial.println(F("# === SINGLE FREQUENCY SINE TEST ==="));
  Serial.print(F("# FREQ_HZ="));      Serial.println(SINGLE_FREQ_HZ);
  Serial.print(F("# AMPLITUDE_PWM=")); Serial.println(SINE_AMPLITUDE);
  Serial.print(F("# DURATION_MS="));  Serial.println(SINGLE_DUR_MS);
  Serial.println(F("# ---"));
  Serial.println(F("time_ms,freq_hz,theta_rad,alpha_rad,sine_normalized,motor_pwm"));

  noInterrupts(); enc_arm = 0; enc_pen = 0; interrupts();

  runSineBlock(SINGLE_FREQ_HZ, SINGLE_DUR_MS, 0);

  brakeMotor();
  Serial.println(F("# DONE — send 'f' to repeat, 'c' for chirp sweep"));
}

// ─────────────────────────────────────────────────────────────
//  MODE 2: CHIRP SWEEP
// ─────────────────────────────────────────────────────────────
void runChirpSweep() {
  Serial.println(F("# === CHIRP FREQUENCY SWEEP ==="));
  Serial.print(F("# FREQ_START_HZ="));  Serial.println(FREQ_START_HZ);
  Serial.print(F("# FREQ_END_HZ="));    Serial.println(FREQ_END_HZ);
  Serial.print(F("# FREQ_STEP_HZ="));   Serial.println(FREQ_STEP_HZ);
  Serial.print(F("# DWELL_CYCLES="));   Serial.println(DWELL_CYCLES);
  Serial.print(F("# AMPLITUDE_PWM="));  Serial.println(SINE_AMPLITUDE);
  Serial.println(F("# ---"));
  Serial.println(F("time_ms,freq_hz,theta_rad,alpha_rad,sine_normalized,motor_pwm"));

  noInterrupts(); enc_arm = 0; enc_pen = 0; interrupts();

  unsigned long global_offset = 0;

  for (float f = FREQ_START_HZ; f <= FREQ_END_HZ + 0.001f; f += FREQ_STEP_HZ) {
    // Calculate dwell time: enough for DWELL_CYCLES complete sine periods
    unsigned long dwell_ms = (unsigned long)((DWELL_CYCLES / f) * 1000.0f);

    Serial.print(F("# Starting freq: ")); Serial.println(f);

    runSineBlock(f, dwell_ms, global_offset);
    global_offset += dwell_ms;

    // Brief brake between frequencies to allow transients to die
    brakeMotor();
    delay(300);
    global_offset += 300;
  }

  brakeMotor();
  Serial.println(F("# CHIRP SWEEP COMPLETE"));
  Serial.println(F("# Send 'c' to repeat, 'f' for single frequency"));
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

  Serial.println(F("=== Frequency Response Experiment ==="));
  Serial.println(F("Place pendulum in DOWN position."));
  Serial.println(F("  'f' → Single-frequency sine test"));
  Serial.println(F("  'c' → Chirp sweep (full Bode data)"));
}

// ─────────────────────────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────────────────────────
void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    if      (cmd == 'f') runSingleFreq();
    else if (cmd == 'c') runChirpSweep();
  }
}
