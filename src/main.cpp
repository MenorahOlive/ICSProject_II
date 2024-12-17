/*************************************************************
  Integrating MPU6050, DC Motors, Ultrasonic Sensor, and Blynk
 *************************************************************/

/* Fill-in information from Blynk Device Info here */
#define BLYNK_TEMPLATE_ID           " "
#define BLYNK_TEMPLATE_NAME         " "
#define BLYNK_AUTH_TOKEN            " "

#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// WiFi credentials
char ssid[] = " ";
char pass[] = " ";

// Pin definitions
const int motor1Pin1 = 26, motor1Pin2 = 27, enablePinA = 25; //Motor A
const int motor2Pin1 = 14, motor2Pin2 = 12, enablePinB = 13; //Motor B
const int trigPin = 32, echoPin = 33, buzzer = 19;

String command;

Adafruit_MPU6050 mpu;

BlynkTimer timer;

// Stop all motors
void stopMotors() {
  digitalWrite(enablePinA, LOW);
  digitalWrite(enablePinB, LOW);
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
}

// Move motors in specific directions
void setMotors(int m1Pin1State, int m1Pin2State, int m2Pin1State, int m2Pin2State) {
  digitalWrite(enablePinA, HIGH);
  digitalWrite(enablePinB, HIGH);
  digitalWrite(motor1Pin1, m1Pin1State);
  digitalWrite(motor1Pin2, m1Pin2State);
  digitalWrite(motor2Pin1, m2Pin1State);
  digitalWrite(motor2Pin2, m2Pin2State);
}

void directMotors(String command) {
  command.toLowerCase();
  if (command == "forward") {
    setMotors(HIGH, LOW, HIGH, LOW); 
    Blynk.virtualWrite(V0, "Moving forward");  
  } else if (command == "backward") {
    setMotors(LOW, HIGH, LOW, HIGH);  
    Blynk.virtualWrite(V0, "Moving backward");  
  } else if (command == "left") {
    setMotors(HIGH, LOW, LOW, LOW);  
    timer.setTimeout(800L, stopMotors);  
    Blynk.virtualWrite(V0, "Turning left");  
  } else if (command == "right") {
    setMotors(LOW, LOW, HIGH, LOW);  
    timer.setTimeout(700L, stopMotors); 
    Blynk.virtualWrite(V0, "Turning right");  
  } else if (command == "stop") {
    stopMotors(); 
    Blynk.virtualWrite(V0, "Motors stopped");  
  } else {
    Blynk.virtualWrite(V0, "Invalid command");  
  }
}

BLYNK_WRITE(V0) {
  command = param.asStr(); 
  Serial.println("Received command: " + command);

  directMotors(command);
}

void setup() {
  Serial.begin(115200);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  // Initialize Motor pins
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enablePinA, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enablePinB, OUTPUT);

  // Initialize Ultrasonic sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Initialize buzzer
  pinMode(buzzer, OUTPUT);

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

  stopMotors();
}

void loop() {
  Blynk.run();
  timer.run();

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  int distance = pulseIn(echoPin, HIGH) * 0.034 / 2;
  Blynk.virtualWrite(V2, distance);  // Send distance to V2

  if(distance<20){
    tone(buzzer, 500);
    timer.setTimeout(700L, stopMotors);
    noTone(buzzer);
    Blynk.virtualWrite(V1, "Obstacle Detetcted");
    }
    else{
    noTone(buzzer);
    }

  // MPU6050 Sensor
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  Blynk.virtualWrite(V3, a.acceleration.z);  // Send Accel Z to V3
  Blynk.virtualWrite(V4, g.gyro.z);  // Send Gyro Z to V4

  if (a.acceleration.z < 5.0){
    Blynk.virtualWrite(V1, "Fall Detected");
    Blynk.logEvent("falldetected");
  }

   
}
