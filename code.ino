#include <Servo.h>      // Include Servo Library
#include <NewPing.h>    // Include NewPing Library

// L298N Motor Control Pins
const int LeftMotorForward = 4;
const int LeftMotorBackward = 5;
const int RightMotorForward = 6;
const int RightMotorBackward = 7;
const int LeftMotorEnable = 9;  // PWM pin for left motor speed control
const int RightMotorEnable = 10; // PWM pin for right motor speed control

#define TRIGGER_PIN A1  // Arduino pin tied to trigger pin on the ultrasonic sensor
#define ECHO_PIN A2     // Arduino pin tied to echo pin on the ultrasonic sensor
#define MAX_DISTANCE 250 // Maximum distance we want to ping for (in cm)

// Object Initialization
Servo servo_motor;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// Variables
boolean goesForward = false;
int distance = 100;
int obstacleCount = 0;  // Count the number of obstacles encountered
int motorSpeed = 180;   // Default motor speed (0 to 255)

// Setup Function
void setup() {
  // Motor pins setup
  pinMode(LeftMotorForward, OUTPUT);
  pinMode(LeftMotorBackward, OUTPUT);
  pinMode(RightMotorForward, OUTPUT);
  pinMode(RightMotorBackward, OUTPUT);
  pinMode(LeftMotorEnable, OUTPUT);
  pinMode(RightMotorEnable, OUTPUT);
  
  // Attach servo motor
  servo_motor.attach(3);
  servo_motor.write(90);  // Start servo at center position
  delay(2000);            // Allow servo to stabilize

  // Initialize distance
  calibrateSensor();

  // Begin Serial Communication for Debugging
  Serial.begin(9600);
  Serial.println("Obstacle Avoiding Robot Initialized");
}

// Loop Function
void loop() {
  int distanceRight = 0;
  int distanceLeft = 0;

  delay(50); // Small delay to avoid sensor overloading

  if (distance <= 20) { // Obstacle detected
    obstacleCount++; // Increment obstacle count
    Serial.println("Obstacle detected!");

    moveStop();
    delay(300);
    moveBackward();
    delay(400);
    moveStop();
    delay(300);

    // Look around to decide the best direction
    distanceRight = lookRight();
    distanceLeft = lookLeft();

    Serial.print("Distance Right: ");
    Serial.print(distanceRight);
    Serial.print(" cm, Distance Left: ");
    Serial.print(distanceLeft);
    Serial.println(" cm");

    if (distanceRight > distanceLeft) {
      turnRight();
      Serial.println("Turning Right");
    } else if (distanceLeft > distanceRight) {
      turnLeft();
      Serial.println("Turning Left");
    } else {
      // Randomize decision when distances are equal
      if (random(0, 2) == 0) {
        turnRight();
        Serial.println("Turning Right (Random Choice)");
      } else {
        turnLeft();
        Serial.println("Turning Left (Random Choice)");
      }
    }
    delay(300);
    moveStop();
  } else {
    moveForward();
    Serial.println("Moving Forward");
  }

  // Update distance
  distance = readPing();

  // Log current state
  logStatus();
}

// Sensor Calibration Function
void calibrateSensor() {
  Serial.println("Calibrating Sensor...");
  for (int i = 0; i < 4; i++) {
    distance = readPing();
    delay(100);
  }
  Serial.println("Sensor Calibration Complete");
}

// Look Right Function
int lookRight() {
  servo_motor.write(45); // Turn servo to the right
  delay(500);
  int distance = readPing();
  delay(100);
  servo_motor.write(90); // Reset servo to center
  return distance;
}

// Look Left Function
int lookLeft() {
  servo_motor.write(135); // Turn servo to the left
  delay(500);
  int distance = readPing();
  delay(100);
  servo_motor.write(90); // Reset servo to center
  return distance;
}

// Read Ping Function
int readPing() {
  delay(100); // Wait between pings
  int cm = sonar.ping_cm();
  if (cm == 0) {
    cm = MAX_DISTANCE; // Set to max distance if no object detected
  }
  return cm;
}

// Movement Functions
void moveStop() {
  analogWrite(LeftMotorEnable, 0);
  analogWrite(RightMotorEnable, 0);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);
}

void moveForward() {
  analogWrite(LeftMotorEnable, motorSpeed);
  analogWrite(RightMotorEnable, motorSpeed);
  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorForward, HIGH);
  digitalWrite(RightMotorBackward, LOW);
}

void moveBackward() {
  analogWrite(LeftMotorEnable, motorSpeed);
  analogWrite(RightMotorEnable, motorSpeed);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(RightMotorBackward, HIGH);
}

void turnRight() {
  analogWrite(LeftMotorEnable, motorSpeed);
  analogWrite(RightMotorEnable, motorSpeed);
  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(RightMotorBackward, HIGH);
}

void turnLeft() {
  analogWrite(LeftMotorEnable, motorSpeed);
  analogWrite(RightMotorEnable, motorSpeed);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorForward, HIGH);
  digitalWrite(RightMotorBackward, LOW);
}

// Log Status Function
void logStatus() {
  Serial.print("Current Distance: ");
  Serial.print(distance);
  Serial.print(" cm, Obstacles Encountered: ");
  Serial.println(obstacleCount);
}
// Shri Harivansh, Hope you like it !
