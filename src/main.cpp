
#include "thingProperties.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// MPU6050
Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;

// Motion Detection Thresholds
const float accelZFallThreshold = 5.0;
const float gyroZTiltThreshold = 2.0;

// Flags for detection
bool fallDetected = false;
bool tiltDetected = false;

// Motor Pins
int motor1Pin1 = 26; 
int motor1Pin2 = 27; 
int enablePinA = 25; 
int motor2Pin1 = 14; 
int motor2Pin2 = 12; 
int enablePinB = 13; 

// Ultrasonic Sensor
const int trigPin = 32;
const int echoPin = 33;

// Buzzer
const int buzzer = 19;

// Cloud Variables
float acc_x, acc_y, acc_z;
float gyro_x, gyro_y, gyro_z;
CloudSwitch backward, forward, left, right, halt;
CloudLocation gps;
bool obstacle = false;


void setup() {
   // Initialize Serial Communication
  Serial.begin(115200);

  // MPU6050 Initialization
 if (!mpu.begin()) {
    Serial.println("Failed to initialize MPU6050 chip");
    delay(5000); 
}
  Serial.println("MPU6050 Initialized!");

  // Initialize Cloud Variables
  initProperties();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();
  
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

  // Initialize CloudSwitch variables to false
  backward = forward = left = right = false;
  halt = true;

  // Stop Motors Initially
  stopMotors();
}

void loop() {
  ArduinoCloud.update();

  //Motor Controls synced with cloudswitch variables 
    onObstacleChange();
    if (!obstacle) { // Only process commands if no obstacle is detected
        onForwardChange();
        onBackwardChange();
        onLeftChange();
        onRightChange();
        onHaltChange();
    }

   // Handle motion detection
    if (mpu.getMotionInterruptStatus()) {
      mpu.getEvent(&a, &g, &temp);

      acc_x = a.acceleration.x;
      acc_y = a.acceleration.y;
      acc_z = a.acceleration.z;
      gyro_x = g.gyro.x;
      gyro_y = g.gyro.y;
      gyro_z = g.gyro.z;

      onAccZChange();
      onGyroZChange();
    }

    
}

// Movement Functions
void moveForward() {
  Serial.println("Moving Forward");
  setMotors(HIGH, LOW, HIGH, LOW);
}

void moveBackward() {
  Serial.println("Moving Backward");
  setMotors(LOW, HIGH, LOW, HIGH);
}

void turnLeft() {
  Serial.println("Turning Left");
  setMotors(LOW, LOW, LOW, HIGH);
}

void turnRight() {
  Serial.println("Turning Right");
  setMotors(LOW, HIGH, LOW, LOW);
}

// Motor Control
void setMotors(int m1Pin1State, int m1Pin2State, int m2Pin1State, int m2Pin2State) {
  digitalWrite(enablePinA, HIGH);
  digitalWrite(enablePinB, HIGH);
  digitalWrite(motor1Pin1, m1Pin1State);
  digitalWrite(motor1Pin2, m1Pin2State);
  digitalWrite(motor2Pin1, m2Pin1State);
  digitalWrite(motor2Pin2, m2Pin2State);
}

void stopMotors() {
  Serial.println("Stopping Motors");
  digitalWrite(enablePinA, LOW);
  digitalWrite(enablePinB, LOW);
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
}

// Obstacle Detection
int getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  int duration = pulseIn(echoPin, HIGH);

  if (duration == 0) {
    Serial.println("Sensor timeout. Invalid distance.");
    return -1;  // Invalid distance
  }
  return duration * 0.034 / 2;  // Calculate distance
}

void onObstacleChange() {
  int distance = getDistance();
  if (distance != -1 && distance <= 20) { 
    Serial.println("Obstacle detected!");
    stopMotors();
    soundBuzzer(4);
    obstacle = true;
  } else {
    obstacle = false;
  }
}

// Motion Detection
void onAccZChange() {
  if (a.acceleration.z < accelZFallThreshold && !fallDetected) {
    Serial.println("FALL DETECTED!");
    triggerEmergencyStop();
    fallDetected = true;
  } else if (a.acceleration.z >= accelZFallThreshold) {
    fallDetected = false;
  }
}

void onGyroZChange() {
  if (abs(g.gyro.z) > gyroZTiltThreshold && !tiltDetected) {
    Serial.println("TILT DETECTED!");
    triggerEmergencyStop();
    tiltDetected = true;
  } else if (abs(g.gyro.z) <= gyroZTiltThreshold) {
    tiltDetected = false;
  }
}

// Emergency Stop
void triggerEmergencyStop() {
  Serial.println("Emergency Stop Triggered!");
  soundBuzzer(4);
  stopMotors();
}

// Buzzer Function
void soundBuzzer(int count) {
  for (int i = 0; i < count; i++) {
    tone(buzzer, 1000);
    delay(500);
    noTone(buzzer);
    delay(500);
  }
}

// CloudSwitch Handlers
void onForwardChange() {
  if (forward) {
    moveForward();
    forward = false;
  }
}

void onBackwardChange() {
  if (backward) {
    moveBackward();
    backward = false;
  }
}

void onLeftChange() {
  if (left) {
    turnLeft();
    left = false;
  }
}

void onRightChange() {
  if (right) {
    turnRight();
    right = false;
  }
}

void onHaltChange() {
  if (halt) {
    stopMotors();
    halt = false;
  }
}








