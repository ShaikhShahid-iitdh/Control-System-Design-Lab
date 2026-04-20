/*
 * Furuta Pendulum (Rotary Inverted Pendulum) State-Feedback Controller
 * Arduino Mega 2560 + L298P Motor Driver Shield
 * 
 * Purpose: Implement state-feedback control u = -K*x to stabilize pendulum at upright
 * 
 * Hardware Connections:
 * - Encoder 1 (arm/base angle): Digital pins 2,3 (interrupt)
 * - Encoder 2 (pendulum angle): Digital pins 20,21 (interrupt)
 * - L298N Motor Driver:
 *   - IN1: Pin 8  (direction)
 *   - IN2: Pin 9  (direction)
 *   - ENA: Pin 5  (PWM speed)
 *   - OUT1, OUT2: Motor terminals
 * - Serial: USB for data logging/monitoring
 */

#include <PinChangeInterrupt.h>

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

// Encoder pins (interrupt capable)
const int ENCODER_ARM_A = 2;
const int ENCODER_ARM_B = 3;
const int ENCODER_PEND_A = 20;
const int ENCODER_PEND_B = 21;

// Motor driver pins (L298N)
const int MOTOR_IN1 = 8;
const int MOTOR_IN2 = 9;
const int MOTOR_PWM = 5;  // PWM pin for speed control

// System parameters
const float MOTOR_MAX_VOLTAGE = 12.0;  // V (supply voltage)
const float PWM_MAX = 255.0;            // Max PWM value
const float SUPPLY_VOLTAGE = 12.0;      // Supply voltage for scaling
const float VOLTAGE_TO_PWM = PWM_MAX / SUPPLY_VOLTAGE;  // Conversion factor

// Encoder resolution
const int ENCODER_PPR_ARM = 360;        // Pulses per revolution (adjust based on your encoder)
const int ENCODER_PPR_PEND = 360;
const float COUNTS_TO_RAD_ARM = 2.0 * PI / ENCODER_PPR_ARM;
const float COUNTS_TO_RAD_PEND = 2.0 * PI / ENCODER_PPR_PEND;

// ============================================================================
// STATE-FEEDBACK CONTROL PARAMETERS
// ============================================================================

// State feedback gain matrix K (computed from pole placement)
// u = -K * x where x = [theta, alpha, theta_dot, alpha_dot]
// K = [-2.3241, 29.3636, -1.0588, 3.6805]
const float K[4] = {-2.3241, 29.3636, -1.0588, 3.6805};

// Desired equilibrium point (upright)
const float ALPHA_DESIRED = 0.0;  // radians

// Control sampling rate
const unsigned long CONTROL_PERIOD_MS = 20;  // 50 Hz (20 ms)
unsigned long last_control_time = 0;

// ============================================================================
// STATE VARIABLES
// ============================================================================

// Encoder positions (in counts)
volatile long encoder_arm_count = 0;
volatile long encoder_pend_count = 0;

// Previous encoder counts for velocity estimation
long prev_encoder_arm_count = 0;
long prev_encoder_pend_count = 0;

// System state vector: x = [theta, alpha, theta_dot, alpha_dot]
float state[4] = {0.0, 0.0, 0.0, 0.0};

// Control input
float control_voltage = 0.0;

// Data logging
unsigned long sample_count = 0;
unsigned long last_log_time = 0;
const unsigned long LOG_PERIOD_MS = 100;  // Log every 100 ms

// ============================================================================
// INTERRUPT SERVICE ROUTINES (ISRs)
// ============================================================================

void interrupt_arm_a() {
  // Encoder A interrupt for arm
  if (digitalRead(ENCODER_ARM_B) == HIGH) {
    encoder_arm_count++;
  } else {
    encoder_arm_count--;
  }
}

void interrupt_arm_b() {
  // Encoder B interrupt for arm
  if (digitalRead(ENCODER_ARM_A) == LOW) {
    encoder_arm_count++;
  } else {
    encoder_arm_count--;
  }
}

void interrupt_pend_a() {
  // Encoder A interrupt for pendulum
  if (digitalRead(ENCODER_PEND_B) == HIGH) {
    encoder_pend_count++;
  } else {
    encoder_pend_count--;
  }
}

void interrupt_pend_b() {
  // Encoder B interrupt for pendulum
  if (digitalRead(ENCODER_PEND_A) == LOW) {
    encoder_pend_count++;
  } else {
    encoder_pend_count--;
  }
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(500);
  
  Serial.println("====================================");
  Serial.println("Furuta Pendulum State-Feedback Controller");
  Serial.println("====================================");
  Serial.print("Control Period: ");
  Serial.print(CONTROL_PERIOD_MS);
  Serial.println(" ms");
  Serial.print("Frequency: ");
  Serial.print(1000.0 / CONTROL_PERIOD_MS);
  Serial.println(" Hz");
  Serial.println();

  // Configure motor driver pins
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);
  
  // Stop motor initially
  set_motor_voltage(0.0);
  
  // Configure encoder pins as inputs
  pinMode(ENCODER_ARM_A, INPUT_PULLUP);
  pinMode(ENCODER_ARM_B, INPUT_PULLUP);
  pinMode(ENCODER_PEND_A, INPUT_PULLUP);
  pinMode(ENCODER_PEND_B, INPUT_PULLUP);
  
  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(ENCODER_ARM_A), interrupt_arm_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_ARM_B), interrupt_arm_b, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PEND_A), interrupt_pend_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PEND_B), interrupt_pend_b, CHANGE);
  
  Serial.println("Hardware initialized. Waiting for start signal...");
  Serial.println("Send 's' to start control, 'e' to stop emergency.");
  Serial.println();
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Check for serial commands
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 's' || cmd == 'S') {
      Serial.println("Starting control...");
      last_control_time = millis();
      control_loop_active();
    } else if (cmd == 'e' || cmd == 'E') {
      Serial.println("Emergency stop!");
      set_motor_voltage(0.0);
      return;
    }
  }
  
  delay(100);
}

// ============================================================================
// CONTROL LOOP
// ============================================================================

void control_loop_active() {
  // Active control loop - runs until stopped by user
  unsigned long start_time = millis();
  
  Serial.println("--- CONTROL ACTIVE ---");
  Serial.println("Time(ms), Theta(rad), Alpha(rad), Theta_dot(rad/s), Alpha_dot(rad/s), Voltage(V)");
  
  while (true) {
    unsigned long now = millis();
    
    // Check for stop command
    if (Serial.available() > 0) {
      char cmd = Serial.read();
      if (cmd == 'e' || cmd == 'E') {
        Serial.println("--- CONTROL STOPPED ---");
        set_motor_voltage(0.0);
        break;
      }
    }
    
    // Run control loop at fixed rate
    if ((now - last_control_time) >= CONTROL_PERIOD_MS) {
      last_control_time = now;
      
      // Update state from encoders
      update_state();
      
      // Compute control input: u = -K * x
      compute_control();
      
      // Apply control with saturation
      apply_control_saturated();
      
      // Log data at lower rate
      if ((now - last_log_time) >= LOG_PERIOD_MS) {
        last_log_time = now;
        log_data(now - start_time);
        sample_count++;
      }
    }
  }
}

// ============================================================================
// STATE ESTIMATION FROM ENCODERS
// ============================================================================

void update_state() {
  // Disable interrupts to safely read encoder counts
  noInterrupts();
  long arm_count = encoder_arm_count;
  long pend_count = encoder_pend_count;
  interrupts();
  
  // Convert encoder counts to radians
  float theta = arm_count * COUNTS_TO_RAD_ARM;
  float alpha = pend_count * COUNTS_TO_RAD_PEND;
  
  // Compute velocities by numerical differentiation
  // Using backward difference: x_dot = (x[n] - x[n-1]) / dt
  static float dt = CONTROL_PERIOD_MS / 1000.0;  // Convert to seconds
  
  float theta_dot = (theta - state[0]) / dt;
  float alpha_dot = (alpha - state[1]) / dt;
  
  // Low-pass filter on velocities to reduce noise (optional but recommended)
  const float alpha_filter = 0.3;  // Smoothing factor (0.0-1.0)
  theta_dot = alpha_filter * theta_dot + (1.0 - alpha_filter) * state[2];
  alpha_dot = alpha_filter * alpha_dot + (1.0 - alpha_filter) * state[3];
  
  // Update state vector
  state[0] = theta;        // Arm angle
  state[1] = alpha;        // Pendulum angle
  state[2] = theta_dot;    // Arm velocity
  state[3] = alpha_dot;    // Pendulum velocity
  
  // Store for next iteration
  prev_encoder_arm_count = arm_count;
  prev_encoder_pend_count = pend_count;
}

// ============================================================================
// CONTROL COMPUTATION
// ============================================================================

void compute_control() {
  // State-feedback control law: u = -K * x
  // K = [k0, k1, k2, k3]
  // u = -(k0*theta + k1*alpha + k2*theta_dot + k3*alpha_dot)
  
  control_voltage = 0.0;
  for (int i = 0; i < 4; i++) {
    control_voltage -= K[i] * state[i];
  }
}

// ============================================================================
// MOTOR CONTROL WITH SATURATION
// ============================================================================

void apply_control_saturated() {
  // Saturate voltage to safe limits
  float voltage_sat = constrain(control_voltage, -MOTOR_MAX_VOLTAGE, MOTOR_MAX_VOLTAGE);
  
  // Convert voltage to PWM value
  float pwm_value = (voltage_sat / SUPPLY_VOLTAGE) * PWM_MAX;
  pwm_value = constrain(pwm_value, -PWM_MAX, PWM_MAX);
  
  // Set motor direction and speed
  set_motor_voltage(voltage_sat);
}

void set_motor_voltage(float voltage) {
  // Saturate to physical limits
  voltage = constrain(voltage, -MOTOR_MAX_VOLTAGE, MOTOR_MAX_VOLTAGE);
  
  // Compute PWM duty cycle
  int pwm_value = abs(voltage) * (255.0 / MOTOR_MAX_VOLTAGE);
  pwm_value = constrain(pwm_value, 0, 255);
  
  // Set direction based on voltage polarity
  if (voltage > 0.5) {
    // Clockwise (or forward direction)
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
    analogWrite(MOTOR_PWM, pwm_value);
  } else if (voltage < -0.5) {
    // Counter-clockwise (or reverse direction)
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
    analogWrite(MOTOR_PWM, pwm_value);
  } else {
    // Stop motor
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
    analogWrite(MOTOR_PWM, 0);
  }
}

// ============================================================================
// DATA LOGGING
// ============================================================================

void log_data(unsigned long elapsed_ms) {
  // Print data in CSV format
  Serial.print(elapsed_ms);
  Serial.print(", ");
  Serial.print(state[0], 6);  // theta
  Serial.print(", ");
  Serial.print(state[1], 6);  // alpha
  Serial.print(", ");
  Serial.print(state[2], 6);  // theta_dot
  Serial.print(", ");
  Serial.print(state[3], 6);  // alpha_dot
  Serial.print(", ");
  Serial.println(control_voltage, 4);  // control voltage
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

void print_state() {
  Serial.println("\n--- Current State ---");
  Serial.print("Arm angle (theta): ");
  Serial.print(state[0], 6);
  Serial.println(" rad");
  
  Serial.print("Pendulum angle (alpha): ");
  Serial.print(state[1], 6);
  Serial.println(" rad");
  
  Serial.print("Arm velocity (theta_dot): ");
  Serial.print(state[2], 6);
  Serial.println(" rad/s");
  
  Serial.print("Pendulum velocity (alpha_dot): ");
  Serial.print(state[3], 6);
  Serial.println(" rad/s");
  
  Serial.print("Control voltage: ");
  Serial.print(control_voltage, 4);
  Serial.println(" V");
  Serial.println();
}

void print_encoder_raw() {
  Serial.println("\n--- Raw Encoder Counts ---");
  Serial.print("Arm encoder count: ");
  Serial.println(encoder_arm_count);
  
  Serial.print("Pendulum encoder count: ");
  Serial.println(encoder_pend_count);
  Serial.println();
}
