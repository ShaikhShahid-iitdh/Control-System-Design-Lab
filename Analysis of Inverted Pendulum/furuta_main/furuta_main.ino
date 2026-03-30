/*
 * ============================================================
 *  Furuta Pendulum — Main Data Acquisition & Motor Control
 *  Hardware : Arduino MEGA 2560 + L298P Motor Driver Shield
 *  Author   : Control System Design Lab
 * ============================================================
 *
 * WIRING GUIDE
 * ─────────────────────────────────────────────────────────────
 * L298P Motor Driver Shield (sits on top of MEGA):
 *   The shield maps directly — just stack it on the MEGA.
 *   Motor A output terminals → Furuta arm DC motor
 *
 *   Shield pin (as labelled) | Function
 *   ─────────────────────────|─────────────────────────────────
 *   ENA  (Pin 10 on shield)  | PWM speed control — do NOT wire, auto-connected on shield
 *   IN1  (Pin 8  on shield)  | Motor direction bit 1
 *   IN2  (Pin 7  on shield)  | Motor direction bit 2
 *   Motor A screw terminal   | Connect DC motor wires here
 *   12V / GND screw terminal | External 12V power supply for motor
 *   IMPORTANT: Never power motor from Arduino 5V/Vin!
 *
 * Encoder 1 — Rotary Arm (θ, theta):
 *   Channel A → Pin 2  (INT0)
 *   Channel B → Pin 3  (INT1)
 *   Vcc → 5V,  GND → GND
 *
 * Encoder 2 — Pendulum (α, alpha):
 *   Channel A → Pin 18 (INT5)
 *   Channel B → Pin 19 (INT4)
 *   Vcc → 5V,  GND → GND
 *
 * Serial output: 115200 baud → open Serial Monitor or log via Python
 * ─────────────────────────────────────────────────────────────
 *
 * COMMANDS (send via Serial Monitor):
 *   's'  → Set motor speed (prompts for value -255 to 255)
 *   'z'  → Zero / reset encoder counts
 *   'r'  → Toggle data reporting (on/off)
 *   'b'  → Brake motor (stop immediately)
 *   'h'  → Print this help message
 */

// ─────────────────────────────────────────────────────────────
//  PIN DEFINITIONS
// ─────────────────────────────────────────────────────────────
// L298P Motor Driver (Motor A channel)
#define MOTOR_ENA   10   // PWM speed (0–255)
#define MOTOR_IN1    8   // Direction bit 1
#define MOTOR_IN2    7   // Direction bit 2

// Encoder 1 — Arm angle θ (theta)  [interrupt pins]
#define ENC_ARM_A    2   // Interrupt 0
#define ENC_ARM_B    3   // Interrupt 1

// Encoder 2 — Pendulum angle α (alpha)  [interrupt pins]
#define ENC_PEN_A   18   // Interrupt 5
#define ENC_PEN_B   19   // Interrupt 4

// ─────────────────────────────────────────────────────────────
//  ENCODER SPECIFICATIONS — adjust to match your hardware
// ─────────────────────────────────────────────────────────────
// Counts per revolution in 4× quadrature decoding mode
// Common values: 600, 1024, 2000, 4000 CPR
const float ARM_CPR     = 4096.0;   // Arm encoder counts/rev
const float PENDULUM_CPR = 4096.0;  // Pendulum encoder counts/rev

// ─────────────────────────────────────────────────────────────
//  GLOBAL VARIABLES
// ─────────────────────────────────────────────────────────────
volatile long  enc_arm = 0;        // Raw arm encoder count
volatile long  enc_pen = 0;        // Raw pendulum encoder count
volatile bool  arm_B_state = false;
volatile bool  pen_B_state = false;

// Angular position and velocity
float theta       = 0.0;  // Arm angle (rad)
float alpha       = 0.0;  // Pendulum angle (rad)
float theta_dot   = 0.0;  // Arm angular velocity (rad/s)
float alpha_dot   = 0.0;  // Pendulum angular velocity (rad/s)

// Velocity computation (finite difference)
long  prev_arm    = 0;
long  prev_pen    = 0;
unsigned long prev_time_us = 0;

// Control
int   motor_pwm   = 0;    // -255 to 255 (sign = direction)
bool  reporting   = true; // Serial output on/off

// Timing
const unsigned long SAMPLE_PERIOD_MS = 10;  // 100 Hz sample rate
unsigned long last_sample = 0;

// ─────────────────────────────────────────────────────────────
//  INTERRUPT SERVICE ROUTINES — Encoder decoding (4× quadrature)
// ─────────────────────────────────────────────────────────────

// Arm encoder — Channel A triggers interrupt
void ISR_arm_A() {
  bool a = digitalRead(ENC_ARM_A);
  if (a == arm_B_state) enc_arm--;
  else                  enc_arm++;
}

// Arm encoder — Channel B triggers interrupt
void ISR_arm_B() {
  arm_B_state = digitalRead(ENC_ARM_B);
  bool a = digitalRead(ENC_ARM_A);
  if (a == arm_B_state) enc_arm++;
  else                  enc_arm--;
}

// Pendulum encoder — Channel A triggers interrupt
void ISR_pen_A() {
  bool a = digitalRead(ENC_PEN_A);
  if (a == pen_B_state) enc_pen--;
  else                  enc_pen++;
}

// Pendulum encoder — Channel B triggers interrupt
void ISR_pen_B() {
  pen_B_state = digitalRead(ENC_PEN_B);
  bool a = digitalRead(ENC_PEN_A);
  if (a == pen_B_state) enc_pen++;
  else                  enc_pen--;
}

// ─────────────────────────────────────────────────────────────
//  MOTOR CONTROL FUNCTIONS
// ─────────────────────────────────────────────────────────────

/*
 * setMotor(pwm)
 *   pwm: -255 to +255
 *   Positive = one direction, Negative = reverse, 0 = coast stop
 */
void setMotor(int pwm) {
  motor_pwm = constrain(pwm, -255, 255);

  if (motor_pwm > 0) {
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
    analogWrite(MOTOR_ENA, motor_pwm);
  } else if (motor_pwm < 0) {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
    analogWrite(MOTOR_ENA, -motor_pwm);
  } else {
    // Brake: both IN pins HIGH → short-circuit brake in L298P
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, HIGH);
    analogWrite(MOTOR_ENA, 0);
  }
}

// Active brake (stops motor immediately)
void brakeMotor() {
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, HIGH);
  analogWrite(MOTOR_ENA, 255);  // max brake force
  motor_pwm = 0;
}

// ─────────────────────────────────────────────────────────────
//  ANGLE & VELOCITY COMPUTATION
// ─────────────────────────────────────────────────────────────
void computeAngles() {
  // Read encoder counts atomically (disable interrupts briefly)
  noInterrupts();
  long arm_count = enc_arm;
  long pen_count = enc_pen;
  interrupts();

  unsigned long now_us = micros();
  float dt = (now_us - prev_time_us) * 1e-6f;  // seconds

  // Convert counts to radians  (2π per revolution)
  theta = (float)arm_count * (2.0f * PI) / ARM_CPR;
  alpha = (float)pen_count * (2.0f * PI) / PENDULUM_CPR;

  // Angular velocity via finite difference (rad/s)
  if (dt > 0) {
    theta_dot = (float)(arm_count - prev_arm) * (2.0f * PI) / (ARM_CPR * dt);
    alpha_dot = (float)(pen_count - prev_pen) * (2.0f * PI) / (PENDULUM_CPR * dt);
  }

  prev_arm     = arm_count;
  prev_pen     = pen_count;
  prev_time_us = now_us;
}

// ─────────────────────────────────────────────────────────────
//  SERIAL COMMAND HANDLER
// ─────────────────────────────────────────────────────────────
void printHelp() {
  Serial.println(F("=== Furuta Pendulum Commands ==="));
  Serial.println(F("  s  : Set motor PWM (-255 to 255)"));
  Serial.println(F("  b  : Brake motor"));
  Serial.println(F("  z  : Zero encoder counts"));
  Serial.println(F("  r  : Toggle data reporting"));
  Serial.println(F("  h  : Show this help"));
  Serial.println(F("================================"));
}

void handleSerial() {
  if (!Serial.available()) return;

  char cmd = Serial.read();
  switch (cmd) {
    case 's': {
      Serial.print(F("Enter PWM (-255 to 255): "));
      while (!Serial.available());
      int val = Serial.parseInt();
      setMotor(val);
      Serial.print(F("Motor PWM set to: "));
      Serial.println(val);
      break;
    }
    case 'b':
      brakeMotor();
      Serial.println(F("Motor braked."));
      break;
    case 'z':
      noInterrupts();
      enc_arm = 0;
      enc_pen = 0;
      interrupts();
      prev_arm = 0;
      prev_pen = 0;
      Serial.println(F("Encoders zeroed."));
      break;
    case 'r':
      reporting = !reporting;
      Serial.print(F("Reporting: "));
      Serial.println(reporting ? F("ON") : F("OFF"));
      break;
    case 'h':
      printHelp();
      break;
    default:
      break;
  }
}

// ─────────────────────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println(F("Furuta Pendulum — Starting up..."));

  // Motor driver pins
  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  brakeMotor();  // Safe start — motor stopped

  // Encoder pins with pull-up resistors
  pinMode(ENC_ARM_A, INPUT_PULLUP);
  pinMode(ENC_ARM_B, INPUT_PULLUP);
  pinMode(ENC_PEN_A, INPUT_PULLUP);
  pinMode(ENC_PEN_B, INPUT_PULLUP);

  // Initialise B-channel states before attaching interrupts
  arm_B_state = digitalRead(ENC_ARM_B);
  pen_B_state = digitalRead(ENC_PEN_B);

  // Attach interrupts — CHANGE triggers on both rising and falling edges (4× decoding)
  attachInterrupt(digitalPinToInterrupt(ENC_ARM_A), ISR_arm_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_ARM_B), ISR_arm_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_PEN_A), ISR_pen_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_PEN_B), ISR_pen_B, CHANGE);

  prev_time_us = micros();

  // CSV header for Serial Plotter / logging
  Serial.println(F("time_ms,theta_rad,alpha_rad,theta_dot_rads,alpha_dot_rads,arm_raw,pen_raw,motor_pwm"));

  printHelp();
}

// ─────────────────────────────────────────────────────────────
//  MAIN LOOP
// ─────────────────────────────────────────────────────────────
void loop() {
  handleSerial();

  unsigned long now = millis();
  if (now - last_sample >= SAMPLE_PERIOD_MS) {
    last_sample = now;

    computeAngles();

    if (reporting) {
      // CSV format: time, theta, alpha, theta_dot, alpha_dot, raw counts, motor pwm
      Serial.print(now);          Serial.print(',');
      Serial.print(theta, 4);     Serial.print(',');
      Serial.print(alpha, 4);     Serial.print(',');
      Serial.print(theta_dot, 4); Serial.print(',');
      Serial.print(alpha_dot, 4); Serial.print(',');

      noInterrupts();
      Serial.print(enc_arm);  Serial.print(',');
      Serial.print(enc_pen);
      interrupts();

      Serial.print(',');
      Serial.println(motor_pwm);
    }
  }
}
