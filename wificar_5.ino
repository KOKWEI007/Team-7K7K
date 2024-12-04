// === GLOBAL VARIABLES ===
// Wi-Fi credentials
const char* ssid = "UPMWiFi";
const char* password = "040729abc";

// Web server instance
#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
WebServer server(80);

// Motor pins
int motor1Pin1 = 27;
int motor1Pin2 = 26;
int enable1Pin = 14;
int motor2Pin1 = 33;
int motor2Pin2 = 25;
int enable2Pin = 32;

// Infrared sensors
int irSensorLeft = 34;
int irSensorRight = 35;

// Servo motors
static const int servo1Pin = 13;
static const int servo2Pin = 12;
Servo servo1;
Servo servo2;
int servo1Angle = 90; // Default angle for servo1
int servo2Angle = 90; // Default angle for servo2

// Mode and motor speed
bool isManualMode = true;    // Start in manual mode
int dutyCycle = 200;         // Default PWM value (0-255)
bool isControllingServo = false; // Prevent servo movement during motor speed control

// === FUNCTION DECLARATIONS ===
void handleRoot();
void handleServo1();
void handleServo2();
void handleSpeed();
void handleForward();
void handleLeft();
void handleStop();
void handleRight();
void handleReverse();
void handleManualMode();
void handleLineMode();
void lineFollowerMode();

// === MAIN EXECUTION ===
void setup() {
  Serial.begin(115200);

  // Initialize motor pins
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  pinMode(enable2Pin, OUTPUT);

  // Initialize IR sensors
  pinMode(irSensorLeft, INPUT);
  pinMode(irSensorRight, INPUT);

  // Initialize motor speed to 0
  analogWrite(enable1Pin, 0);
  analogWrite(enable2Pin, 0);
  
  
  // Attach servos
  servo1.attach(servo1Pin);
  servo1.write(servo1Angle);
  servo2.attach(servo2Pin);
  servo2.write(servo2Angle);

  // Initialize Wi-Fi
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi. IP: " + WiFi.localIP().toString());

  // Setup routes
  server.on("/", handleRoot);
  server.on("/servo1", handleServo1);
  server.on("/servo2", handleServo2);
  server.on("/manual", handleManualMode);
  server.on("/line", handleLineMode);
  server.on("/forward", handleForward);
  server.on("/left", handleLeft);
  server.on("/stop", handleStop);
  server.on("/right", handleRight);
  server.on("/reverse", handleReverse);
  server.on("/speed", handleSpeed);

  // Start server
  server.begin();
}

void loop() {
  server.handleClient();
  if (!isManualMode) {
    lineFollowerMode();
  }
}

// === HANDLER FUNCTIONS ===
// Root web page handler
void handleRoot() {
  const char html[] PROGMEM = R"rawliteral(
  <!DOCTYPE HTML><html>
  <head>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <link rel="icon" href="data:,">
    <style>
      html { font-family: Helvetica; text-align: center; margin: 0; }
      .button { background-color: #4CAF50; border: none; color: white; padding: 15px 30px; text-decoration: none; font-size: 20px; cursor: pointer; margin: 5px; }
      .button2 {background-color: #555555;}
      input[type=range] { width: 80%; }
    </style>
    <script>
      function switchToManual() { fetch('/manual'); }
      function switchToLine() { fetch('/line'); }
      function moveForward() { fetch('/forward'); }
      function moveLeft() { fetch('/left'); }
      function stopRobot() { fetch('/stop'); }
      function moveRight() { fetch('/right'); }
      function moveReverse() { fetch('/reverse'); }
      function updateMotorSpeed(value) { fetch(`/speed?value=${value}`); }
      function updateServo1Angle(value) { fetch(`/servo1?angle=${value}`); }
      function updateServo2Angle(value) { fetch(`/servo2?angle=${value}`); }
    </script>
  </head>
  <body>
    <h1>ESP32 Robot Control</h1>
    <p><button class="button" onclick="switchToManual()">MANUAL</button>
       <button class="button" onclick="switchToLine()">LINE FOLLOWER</button></p>
    <p><button class="button" onclick="moveForward()">FORWARD</button></p>
    <p>
      <button class="button" onclick="moveLeft()">LEFT</button>
      <button class="button button2" onclick="stopRobot()">STOP</button>
      <button class="button" onclick="moveRight()">RIGHT</button>
    </p>
    <p><button class="button" onclick="moveReverse()">REVERSE</button></p>
    <p>Motor Speed: <input type="range" min="0" max="255" step="1" onchange="updateMotorSpeed(this.value)"></p>
    <p>Servo1 Angle: <input type="range" min="0" max="180" step="1" value="90" onchange="updateServo1Angle(this.value)"></p>
    <p>Servo2 Angle: <input type="range" min="0" max="180" step="1" value="90" onchange="updateServo2Angle(this.value)"></p>
  </body>
  </html>)rawliteral";
  server.send(200, "text/html", html);
}

// Servo1 and Servo2 handlers
void handleServo1() {
  if (server.hasArg("angle")) {
    isControllingServo = true;
    servo1Angle = server.arg("angle").toInt();
    servo1.write(servo1Angle);
    Serial.println("Servo1 angle set to: " + String(servo1Angle));
    isControllingServo = false;
  }
  server.send(200, "text/plain", "OK");
}

void handleServo2() {
  if (server.hasArg("angle")) {
    isControllingServo = true;
    servo2Angle = server.arg("angle").toInt();
    servo2.write(servo2Angle);
    Serial.println("Servo2 angle set to: " + String(servo2Angle));
    isControllingServo = false;
  }
  server.send(200, "text/plain", "OK");
}

void handleSpeed() {
  if (!isControllingServo && server.hasArg("value")) {
    dutyCycle = server.arg("value").toInt(); // Get the motor speed from the client
    dutyCycle = constrain(dutyCycle, 0, 255); // Ensure the duty cycle is within PWM limits (0-255)
    analogWrite(enable1Pin, dutyCycle); // Apply the speed to motor 1
    analogWrite(enable2Pin, dutyCycle); // Apply the speed to motor 2
    Serial.println("Motor speed set to: " + String(dutyCycle));
  }
  server.send(200);
}


void handleManualMode() {
  isManualMode = true;
  Serial.println("Switched to Manual Mode");
  handleStop();
  server.send(200);
}

void handleLineMode() {
  isManualMode = false;
  Serial.println("Switched to Line Follower Mode");
  server.send(200);
}

// Manual mode handlers
void handleForward() {
  if (isManualMode) {
    // Move motors forward by controlling direction pins
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
  }
  server.send(200);
}

void handleLeft() {
  if (isManualMode) {
    // Turn left by controlling direction pins
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW);
  }
  server.send(200);
}

void handleStop() {
  // Stop motors by setting direction pins to LOW
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
  server.send(200);
}

void handleRight() {
  if (isManualMode) {
    // Turn right by controlling direction pins
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
  }
  server.send(200);
}

void handleReverse() {
  if (isManualMode) {
    // Move motors in reverse by changing direction pins
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
  }
  server.send(200);
}
//line follow mode
void lineFollowerMode() {
  int leftValue = digitalRead(irSensorLeft);
  int rightValue = digitalRead(irSensorRight);
  int baseSpeed = 100;        // Base speed for line-following
  int turnSpeed = 255; // Speed difference for turns 
  
   if (leftValue == LOW && rightValue == LOW) {
    // Move forward
    analogWrite(enable1Pin, baseSpeed);
    analogWrite(enable2Pin, baseSpeed);
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
  } else if (leftValue == LOW && rightValue == HIGH) {
    // Turn right
    analogWrite(enable1Pin, turnSpeed);
    analogWrite(enable2Pin, turnSpeed );
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
  } else if (leftValue == HIGH && rightValue == LOW) {
    // Turn left
    analogWrite(enable1Pin, turnSpeed );
    analogWrite(enable2Pin, turnSpeed);
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
    delay(20);
  } else if (leftValue == HIGH && rightValue == HIGH) {
    // Stop
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW);
    analogWrite(enable1Pin, 0);
    analogWrite(enable2Pin, 0);
    delay(20);
  }
}
