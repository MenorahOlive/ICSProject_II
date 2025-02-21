/* Fill-in information from Blynk Device Info here */
#define BLYNK_TEMPLATE_ID           "TMPL2clZJy-2l"
#define BLYNK_TEMPLATE_NAME         "Quickstart Template"
#define BLYNK_AUTH_TOKEN            "Q2RiM-RcElz72kZBLSuErWH9GNIc_PrP"

#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// WiFi credentials
char ssid[] = "";
char pass[] = "";

// Pin definitions
const int motor1Pin1 = 26, motor1Pin2 = 27, enablePinA = 25; //Motor A
const int motor2Pin1 = 14, motor2Pin2 = 12, enablePinB = 13; //Motor B
const int trigPin = 32, echoPin = 33, buzzer = 19;

const int freq = 30000;
const int pwmChannelA = 2;
const int pwmChannelB = 3;
const int resolution = 8;

String command;

Adafruit_MPU6050 mpu;

BlynkTimer timer;

//set up ESP32 PMW channels to control speed
void setupPWM() {
  ledcSetup(pwmChannelA, freq, resolution);
  ledcAttachPin(enablePinA, pwmChannelA);
  
  ledcSetup(pwmChannelB, freq, resolution);
  ledcAttachPin(enablePinB, pwmChannelB);
}

// Stop all motors
void stopMotors() {
  setMotorSpeeds(0, 0);
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
}

//set motor speeds using PMW from 0-255
void setMotorSpeeds(int speedA, int speedB) {
  ledcWrite(pwmChannelA, speedA);
  ledcWrite(pwmChannelB, speedB);
}

//set motor values to control the speed and direction of the motors
void setMotors(int m1Pin1State, int m1Pin2State, int m2Pin1State, int m2Pin2State, int speedRight= 255, int speedLeft = 255) {
  setMotorSpeeds(speedRight, speedLeft);
  digitalWrite(motor1Pin1, m1Pin1State);
  digitalWrite(motor1Pin2, m1Pin2State);
  digitalWrite(motor2Pin1, m2Pin1State);
  digitalWrite(motor2Pin2, m2Pin2State);
}

const int LEFT_SPEED = 255;
const int RIGHT_SPEED = 240;

//direct the motors using voice commands 
void directMotors(String command) {
  command.toLowerCase();
  if (command == "forward") {
    setMotors(HIGH, LOW, HIGH, LOW, RIGHT_SPEED, LEFT_SPEED); 
    Blynk.virtualWrite(V0, "Moving forward");  
  } else if (command == "backward") {
    setMotors(LOW, HIGH, LOW, HIGH, RIGHT_SPEED, LEFT_SPEED); 
    timer.setTimeout(1000L, stopMotors);   
    Blynk.virtualWrite(V0, "Moving backward");  
  } else if (command == "left") {
    setMotors(HIGH, LOW, LOW, LOW, RIGHT_SPEED, 0);  
    timer.setTimeout(200L, stopMotors);  
    Blynk.virtualWrite(V0, "Turning left");  
  } else if (command == "right") {
    setMotors(LOW, LOW, HIGH, LOW, 0, LEFT_SPEED);  
    timer.setTimeout(200L, stopMotors); 
    Blynk.virtualWrite(V0, "Turning right");  
  } else if (command == "stop") {
    stopMotors(); 
    Blynk.virtualWrite(V0, "Motors stopped");  
  } else {
    Blynk.virtualWrite(V0, "Invalid command");  
  }
}

// Uses Blynk to visualize the commands and IFTTT to execute them 
BLYNK_WRITE(V0) {
  command = param.asStr(); 
  Serial.println("Received command: " + command);

  directMotors(command);
}

//Detect fall events using acceleration on Z
void checkMPU6050() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Blynk.virtualWrite(V3, a.acceleration.z);  // Send Accel Z to V3
  Blynk.virtualWrite(V4, g.gyro.z);  // Send Gyro Z to V4

  if (a.acceleration.z < 5.0) {
    Blynk.virtualWrite(V1, "Fall Detected");
    Blynk.logEvent("falldetected");
  }
}

//Uses ultrasonic sensor to detect obstacles 
void measureDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  int distance = pulseIn(echoPin, HIGH) * 0.034 / 2;
  Blynk.virtualWrite(V2, distance);  // Send distance to V2

  if (distance < 40) {
    tone(buzzer, 500);
    stopMotors();
    timer.setTimeout(800L, []() { noTone(buzzer); }); // Stop buzzer after 800ms
    Blynk.virtualWrite(V1, "Obstacle Detected");
  } else {
    noTone(buzzer);
    Blynk.virtualWrite(V1, "Clear path");
  }
}

//Initial hardware and software configurations 
void setup() {
  Serial.begin(115200);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  // Initialize Motor pins
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);

  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
 

  // Initialize Ultrasonic sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Initialize buzzer
  pinMode(buzzer, OUTPUT);

  // Initialize PWM
  setupPWM();
  
  //setupt motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  //Initialize mpu
  if (!mpu.begin()) {
  Serial.println("Failed to initialize MPU6050!");
  while (1);
  }
  Serial.println("MPU6050 initialized successfully!");
  //stop motors initially
  stopMotors();
  //periodically read values from these sensors 
  timer.setInterval(200L, checkMPU6050);
  timer.setInterval(200L, measureDistance);
}

void loop() {
  Blynk.run();
  timer.run();
}
