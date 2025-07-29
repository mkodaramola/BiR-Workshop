//#############################################################################
//
// FILE:   pid_robot_bluetooth.ino
//
// TITLE:  PID-Controlled Robot Car with Bluetooth Tuning
//
//! \author Daramola Oluwafemi
//! \date   30th July, 2025
//!
//! This sketch implements a robot car controlled using a PID algorithm,
//! where the car tries to maintain a desired distance (setpoint) from an
//! object using an ultrasonic sensor (HC-SR04).
//!
//! The PID gains (Kp, Ki, Kd) and setpoint can be adjusted in real-time
//! via Bluetooth using commands sent from a mobile app or terminal.
//! The robot can also be turned ON or OFF remotely using Bluetooth commands.
//!
//! \b External \b Connections \n
//!  - HC-SR04 Ultrasonic Sensor:
//!     - VCC   -> Arduino 5V
//!     - GND   -> Arduino GND
//!     - TRIG  -> Arduino pin 2
//!     - ECHO  -> Arduino pin 3
//!
//!  - L298N Motor Driver:
//!     - IN1   -> Arduino pin 4
//!     - IN2   -> Arduino pin 5
//!     - IN3   -> Arduino pin 6
//!     - IN4   -> Arduino pin 7
//!     - ENA   -> Arduino pin 9 (PWM)
//!     - ENB   -> Arduino pin 10 (PWM)
//!
//!  - HC-05 Bluetooth Module:
//!     - VCC   -> Arduino 5V
//!     - GND   -> Arduino GND
//!     - TXD   -> Arduino pin 0 (RX)
//!     - RXD   -> Arduino pin 1 (TX) [Use voltage divider to step down 5V]
//!
//! \b Bluetooth \b Commands \n
//!  - "ON"     : Enable robot movement and PID control
//!  - "OFF"    : Disable robot movement
//!  - "SPxx"   : Set distance setpoint (e.g., SP50 sets setpoint to 50cm)
//!  - "KPx.x"  : Set proportional gain (e.g., KP1.2)
//!  - "KIx.x"  : Set integral gain (e.g., KI0.05)
//!  - "KDx.x"  : Set derivative gain (e.g., KD0.8)
//!
//! \b Watch \b Variables \n
//!  - \b distance : Current measured distance from ultrasonic sensor
//!  - \b error    : Difference between setpoint and distance
//!  - \b output   : PID output (used to compute motor speed)
//!  - \b speed    : PWM value sent to motors
//!
//#############################################################################



// Pin definitions for ultrasonic sensor
#define TRIGGER_PIN 2   // Trigger pin for ultrasonic sensor
#define ECHO_PIN    3   // Echo pin for ultrasonic sensor

// Motor direction pins
#define in1 4
#define in2 5
#define in3 6
#define in4 7

// PWM enable pins (for motor speed control)
#define enA 9
#define enB 10

// PID timing
unsigned long lastMillis = 0;
unsigned long samplingTime = 100;  // PID update interval in milliseconds

// PID control variables
float setpoint = 50;       // Desired distance in cm
float kp = 0.5, ki = 0.1, kd = 1.0;  // PID gains
float error = 0, P = 0, I = 0, D = 0;
float lastInput = 0, output = 0;
float minOutput = -255, maxOutput = 255;
float speed = 0;           // Final PWM value to motors

bool robotEnabled = false;  // Flag to start/stop robot

// Bluetooth input handler
String inputString = "";     // Holds incoming characters
bool inputComplete = false; // True when full command is received

// Arduino setup runs once
void setup() {
  Serial.begin(9600);  // Start serial communication (Bluetooth module uses this)

  // Set ultrasonic sensor pins
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Set motor direction pins as outputs
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Set PWM pins as outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

  // Initially stop the motors
  stopMotors();

  // Reserve memory for Bluetooth input to avoid fragmentation
  inputString.reserve(50);
}

// Arduino main loop runs repeatedly
void loop() {
  handleBluetooth();  // Check for Bluetooth commands

  // If robot is off, stop the motors and skip control
  if (!robotEnabled) {
    stopMotors();
    return;
  }

  // Run PID control at fixed intervals
  if (millis() - lastMillis >= samplingTime) {
    lastMillis = millis();

    float distance = readDistance();  // Get distance from ultrasonic sensor
    computePID(distance);             // Run PID computation

    // Apply speed via PWM
    analogWrite(enA, constrain(speed, 0, 255));
    analogWrite(enB, constrain(speed, 0, 255));

    // Control motor direction based on sign of error
    if (error > 0) {
      forward();
    } else if (error < 0) {
      backward();
    } else {
      stopMotors();
    }
  }
}

// Handle Bluetooth messages (e.g., ON, SP50, KP1.2)
void handleBluetooth() {
  while (Serial.available()) {
    char c = Serial.read();   // Read next character
    if (c == '\n') {          // End of message (sent by app)
      inputComplete = true;
      break;
    } else {
      inputString += c;       // Append char to buffer
    }
  }

  // Parse the complete input string
  if (inputComplete) {
    if (inputString.startsWith("ON")) robotEnabled = true;
    else if (inputString.startsWith("OFF")) robotEnabled = false;
    else if (inputString.startsWith("SP")) setpoint = inputString.substring(2).toFloat();  // e.g. SP50
    else if (inputString.startsWith("KP")) kp = inputString.substring(2).toFloat();        // e.g. KP1.5
    else if (inputString.startsWith("KI")) ki = inputString.substring(2).toFloat();        // e.g. KI0.02
    else if (inputString.startsWith("KD")) kd = inputString.substring(2).toFloat();        // e.g. KD0.5

    inputString = "";         // Clear buffer
    inputComplete = false;    // Reset flag
  }
}

// Read distance from HC-SR04 ultrasonic sensor in cm
float readDistance() {
  // Send 10us pulse to trigger
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  // Measure time of echo
  long duration = pulseIn(ECHO_PIN, HIGH);

  // Convert time to distance in cm
  return duration / 58.0;
}

// PID logic to adjust motor speed
void computePID(float input) {
  float dt = samplingTime / 1000.0;  // Convert ms to seconds

  error = setpoint - input;          // How far we are from the goal

  P = kp * error;                    // Proportional term
  I += ki * error * dt;              // Accumulated error (integral)
  D = -kd * (input - lastInput) / dt;  // Rate of change of input (derivative)

  output = P + I + D;                // Total control output
  output = constrain(output, minOutput, maxOutput);  // Keep within allowed limits

  // Convert output to speed PWM value
  float baseSpeed = 100;
  float extra = abs(output);
  extra = min(extra, 255 - baseSpeed);
  speed = baseSpeed + extra;

  lastInput = input;  // Save for next derivative calculation
}

// Make robot go forward
void forward() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

// Make robot go backward
void backward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

// Stop all motors
void stopMotors() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}
