#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// MPU6050
Adafruit_MPU6050 mpu;

// Fall/Tilt Detection Thresholds
const float accelZFallThreshold = 5.0;  // Acceleration below this indicates a fall
const float gyroZTiltThreshold = 2.0;   // Angular velocity above this indicates tilting

// Flags for detection
bool fallDetected = false;
bool tiltDetected = false;

// Motor A
int motor1Pin1 = 26; 
int motor1Pin2 = 27; 
int enablePinA = 25; 
// Motor B 
int motor2Pin1 = 14; 
int motor2Pin2 = 12; 
int enablePinB = 13; 
// Ultrasonic Sensor 
const int trigPin = 32;
const int echoPin = 33;
// Buzzer 
const int buzzer = 19;
//default command
String currentCommand = "STOP"; 

void setup() {
  // Initialize Serial Communication
  Serial.begin(115200);

  // MPU6050 Initialization
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  Serial.println("MPU6050 Initialized!");

  //setup motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  Serial.println("");
  delay(100);

  // Initialize Motor Pins
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enablePinA, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enablePinB, OUTPUT);

  // Initialize Ultrasonic Sensor Pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Initialize Buzzer Pin
  pinMode(buzzer, OUTPUT);

  // Stop Motors Initially
  stopMotors();
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readString();
    command.trim();
    Serial.print("Command: ");
    Serial.println(command);
    //check command 
    if (command.equalsIgnoreCase("STOP")) {
      currentCommand = "STOP"; 
      stopMotors();
    } else {
      currentCommand = command; 
    }
  }
  //check for obstacles 
  int distance = getDistance();
  if (distance != -1 && distance < 20) { 
    Serial.println("OBSTACLE DETECTED!");
    stopMotors();
    currentCommand = "STOP";
    for (int i = 0; i < 4; i++){
    tone(buzzer, 400);
    delay(500);
    noTone(buzzer);
    } 
  } 
  // Execute the current command
  if (!currentCommand.equalsIgnoreCase("STOP")) {
    directMotors(currentCommand);
  }
  delay(100); 

  // Check MPU6050 for fall or tilt detection
  if (checkFallOrTilt()) {
    triggerEmergencyStop();
    return; // Skip further processing
  }
}
//Detect obstacles using Ultrasonic Sensor 
int getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  int duration = pulseIn(echoPin, HIGH);
  if (duration == 0) {
    Serial.println("Sensor timeout. Invalid distance.");
    return -1; 
  }
  return duration * 0.034 / 2;
}

void directMotors(String command) {
  command.toLowerCase();
  if (command == "forward") {
    Serial.println("Moving Forward");
    setMotors(HIGH, LOW, HIGH, LOW);
  } else if (command == "backward") {
    Serial.println("Moving Backward");
    setMotors(LOW, HIGH, LOW, HIGH);
  } else if (command == "left") {
    Serial.println("Turning Left");
    setMotors(LOW, LOW, LOW, HIGH);
  } else if (command == "right") {
    Serial.println("Turning Right");
    setMotors(LOW, HIGH, LOW, LOW);
  } else if (command == "stop") {
    Serial.println("Stopping Motors");
    stopMotors();
  } else {
    Serial.println("Unknown Command");
  }
}
bool checkFallOrTilt() {
  if (mpu.getMotionInterruptStatus()) {
    // Get new sensor events with the readings
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Print out the values for debugging
    Serial.print("AccelX:");
    Serial.print(a.acceleration.x);
    Serial.print(", ");
    Serial.print("AccelY:");
    Serial.print(a.acceleration.y);
    Serial.print(", ");
    Serial.print("AccelZ:");
    Serial.print(a.acceleration.z);
    Serial.print(", ");
    Serial.print("GyroX:");
    Serial.print(g.gyro.x);
    Serial.print(", ");
    Serial.print("GyroY:");
    Serial.print(g.gyro.y);
    Serial.print(", ");
    Serial.print("GyroZ:");
    Serial.print(g.gyro.z);
    Serial.println("");

    // Fall Detection
    if (a.acceleration.z < accelZFallThreshold && !fallDetected) {
      Serial.println("FALL DETECTED!");
      fallDetected = true;
      return true;
    } else if (a.acceleration.z >= accelZFallThreshold) {
      fallDetected = false;
    }

    // Tilt Detection
    if (abs(g.gyro.z) > gyroZTiltThreshold && !tiltDetected) {
      Serial.println("TILT DETECTED!");
      tiltDetected = true;
      return true;
    } else if (abs(g.gyro.z) <= gyroZTiltThreshold) {
      tiltDetected = false;
    }
  }

  delay(10); // Optional, may want to minimize for real-time responsiveness
  return false;
}

void setMotors(int m1Pin1State, int m1Pin2State, int m2Pin1State, int m2Pin2State) {
  digitalWrite(enablePinA, HIGH);
  digitalWrite(enablePinB, HIGH);
  digitalWrite(motor1Pin1, m1Pin1State);
  digitalWrite(motor1Pin2, m1Pin2State);
  digitalWrite(motor2Pin1, m2Pin1State);
  digitalWrite(motor2Pin2, m2Pin2State);
}

void stopMotors() {
  digitalWrite(enablePinA, LOW);
  digitalWrite(enablePinB, LOW);
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
}

void triggerEmergencyStop() {
  Serial.println("Emergency Stop Triggered!");
    for (int i = 0; i < 4; i++){
    tone(buzzer, 400);
    delay(500);
    noTone(buzzer);
    }
    stopMotors(); 
}


