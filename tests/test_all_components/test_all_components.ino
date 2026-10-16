#include <Servo.h>

// 1. Define Pin Connections
const int joystickXPin = A0;
const int joystickYPin = A1;
const int joystickBtnPin = A2;

const int trigPin = 4;
const int echoPin = 3;

const int servo1Pin = 6;
const int servo2Pin = 5;

// 2. Create Servo Objects
Servo servo1; //pitch
Servo servo2; //yaw

void setup() {
  // Start the Serial Monitor at 9600 baud rate for debugging
  Serial.begin(9600);

  // Configure Joystick Button with internal pull-up resistor
  pinMode(joystickBtnPin, INPUT_PULLUP);

  // Configure Ultrasonic Pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Attach the servos to their respective pins
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
}

void loop() {
  // --- JOYSTICK & SERVO LOGIC ---
  
  // Read analog values from joystick (ranges from 0 to 1023)
  int joyX = analogRead(joystickXPin);
  int joyY = analogRead(joystickYPin);
  
  // Read the button state (LOW means pressed because of INPUT_PULLUP)
  int btnState = digitalRead(joystickBtnPin);

  // Map the joystick values to servo angles (0 to 180 degrees)
  int servo1Angle = map(joyX, 0, 1023, 0, 180);
  int servo2Angle = map(joyY, 0, 1023, 0, 180);

  // Command servos to move to the calculated angles
  servo1.write(servo1Angle);
  servo2.write(servo2Angle);

  // --- ULTRASONIC SENSOR LOGIC ---
  
  // Ensure the trigger pin is clear
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Send a 10-microsecond HIGH pulse to trigger the sensor
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure the time it takes for the echo to return
  long duration = pulseIn(echoPin, HIGH);

  // Calculate distance in centimeters (Speed of sound is ~0.034 cm/microsecond)
  int distance = duration * 0.034 / 2;

  // --- PRINT TO SERIAL MONITOR ---
  
  Serial.print("X: ");
  Serial.print(joyX);
  Serial.print(" | Y: ");
  Serial.print(joyY);
  Serial.print(" | Btn: ");
  Serial.print(btnState == LOW ? "PRESSED  " : "UNPRESSED");
  Serial.print(" | S1: ");
  Serial.print(servo1Angle);
  Serial.print("° | S2: ");
  Serial.print(servo2Angle);
  Serial.print("° | Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Small delay to keep the servos stable and the Serial Monitor readable
  delay(50);
}