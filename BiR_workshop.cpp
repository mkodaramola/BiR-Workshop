#define TRIGGER_PIN 2
#define ECHO_PIN 3

#define in1 4
#define in2 5
#define in3 6
#define in4 7
#define enA 9
#define enB 10

unsigned long lastMillis = 0;
unsigned long samplingTime = 100;  // ms

float setpoint = 50;
float kp = 0.5, ki = 0.1, kd = 1.0;
float error = 0, P = 0, I = 0, D = 0;
float lastInput = 0, output = 0;
float minOutput = -255, maxOutput = 255;
float speed = 0;

bool robotEnabled = false;

String inputString = "";
bool inputComplete = false;

void setup() {
  Serial.begin(9600);

  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

  stopMotors();

  inputString.reserve(50);
}

void loop() {
  handleBluetooth();

  if (!robotEnabled) {
    stopMotors();
    return;
  }

  if (millis() - lastMillis >= samplingTime) {
    lastMillis = millis();
    float distance = readDistance();
    computePID(distance);

    analogWrite(enA, constrain(speed, 0, 255));
    analogWrite(enB, constrain(speed, 0, 255));

    if (error > 0) {
      forward();
    } else if (error < 0) {
      backward();
    } else {
      stopMotors();
    }
  }
}

void handleBluetooth() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      inputComplete = true;
      break;
    } else {
      inputString += c;
    }
  }

  if (inputComplete) {
    if (inputString.startsWith("ON")) robotEnabled = true;
    else if (inputString.startsWith("OFF")) robotEnabled = false;
    else if (inputString.startsWith("SP")) setpoint = inputString.substring(2).toFloat();
    else if (inputString.startsWith("KP")) kp = inputString.substring(2).toFloat();
    else if (inputString.startsWith("KI")) ki = inputString.substring(2).toFloat();
    else if (inputString.startsWith("KD")) kd = inputString.substring(2).toFloat();

    inputString = "";
    inputComplete = false;
  }
}

float readDistance() {
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  return duration / 58.0;  // cm
}

void computePID(float input) {
  float dt = samplingTime / 1000.0;
  error = setpoint - input;
  P = kp * error;
  I += ki * error * dt;
  D = -kd * (input - lastInput) / dt;
  output = P + I + D;
  output = constrain(output, minOutput, maxOutput);

  float baseSpeed = 100;
  float extra = abs(output);
  extra = min(extra, 255 - baseSpeed);
  speed = baseSpeed + extra;

  lastInput = input;
}

void forward() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void backward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void stopMotors() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}
