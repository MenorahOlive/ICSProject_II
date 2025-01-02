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
char ssid[] = "Beracah Faiba";
char pass[] = "Africa@2023";

// Pin definitions
const int motor1Pin1 = 26, motor1Pin2 = 27, enablePinA = 25; //Motor A
const int motor2Pin1 = 14, motor2Pin2 = 12, enablePinB = 13; //Motor B
const int trigPin = 32, echoPin = 33, buzzer = 19;

String command;

Adafruit_MPU6050 mpu;

BlynkTimer timer;

float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ; //used in void loop()
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

const int maxSpeed = 255; //max PWM value written to motor speed pin. It is typically 255.
const int minSpeed = 160; //min PWM value at which motor moves
float angle; //due to how I orientated my MPU6050 on my car, angle = roll
float targetAngle = 0;
int equilibriumSpeed = 248; //rough estimate of PWM at the speed pin of the stronger motor, while driving straight 
//and weaker motor at maxSpeed
int leftSpeedVal;
int rightSpeedVal;
bool isDriving = false; //it the car driving forward OR rotate/stationary
bool prevIsDriving = true; //equals isDriving in the previous iteration of void loop()
bool paused = false;
static int count;
static int countStraight;

// Stop all motors
void stopMotors() {
  ledcWrite(0, leftSpeedVal); // Channel 0
  ledcWrite(1, rightSpeedVal); // Channel 1
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
}

// Move motors in specific directions
void setMotors(int m1Pin1State, int m1Pin2State, int m2Pin1State, int m2Pin2State) {
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
    isDriving = true;
  } else if (command == "backward") {
    setMotors(LOW, HIGH, LOW, HIGH);  
    Blynk.virtualWrite(V0, "Moving backward");  
  } else if (command == "left") {
    setMotors(HIGH, LOW, LOW, LOW);    
    Blynk.virtualWrite(V0, "Turning left");  
    targetAngle += 90;
      if (targetAngle > 180){
        targetAngle -= 360;
      }
      isDriving = false;
  } else if (command == "right") {
    setMotors(LOW, LOW, HIGH, LOW);   
    Blynk.virtualWrite(V0, "Turning right");  
     targetAngle -= 90;
      if (targetAngle <= -180){
        targetAngle += 360;
      }
      isDriving = false;
  } else if (command == "stop") {
    stopMotors(); 
    Blynk.virtualWrite(V0, "Motors stopped"); 
    isDriving = false; 
  } else {
    Blynk.virtualWrite(V0, "Invalid command");  
    paused = !paused;
    stopMotors();
    isDriving = false;
  }
}

BLYNK_WRITE(V0) {
  command = param.asStr(); 
  Serial.println("Received command: " + command);

  directMotors(command);
}

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

void measureDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  int distance = pulseIn(echoPin, HIGH) * 0.034 / 2;
  Blynk.virtualWrite(V2, distance);  // Send distance to V2

  if (distance < 20) {
    tone(buzzer, 500);
    stopMotors();
    timer.setTimeout(800L, []() { noTone(buzzer); }); // Stop buzzer after 500ms
    Blynk.virtualWrite(V1, "Obstacle Detected");
  } else {
    noTone(buzzer);
    Blynk.virtualWrite(V1, "Clear path");
  }
}

void readAcceleration() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
}

void readGyro() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
}

void driving (){//called by void loop(), which isDriving = true
  int deltaAngle = round(targetAngle - angle); //rounding is neccessary, since you never get exact values in reality
  command = "forward";
  if (deltaAngle != 0){
    controlSpeed ();
    rightSpeedVal = maxSpeed;
    ledcWrite(0, leftSpeedVal); // Channel 0
    ledcWrite(1, rightSpeedVal); // Channel 1

    
  }
}

void controlSpeed (){//this function is called by driving ()
  int deltaAngle = round(targetAngle - angle);
  int targetGyroX;
  
  if (deltaAngle > 30){
      targetGyroX = 60;
  } else if (deltaAngle < -30){
    targetGyroX = -60;
  } else {
    targetGyroX = 2 * deltaAngle;
  }
  
  if (round(targetGyroX - g.gyro.x) == 0){
    ;
  } else if (targetGyroX > g.gyro.x){
    leftSpeedVal = changeSpeed(leftSpeedVal, -1); //would increase GyroX
  } else {
    leftSpeedVal = changeSpeed(leftSpeedVal, +1);
  }
}

void rotate (){//called by void loop(), which isDriving = false
  int deltaAngle = round(targetAngle - angle);
  int targetGyroX;
  if (abs(deltaAngle) <= 1){
    stopMotors();
  } else {
    if (angle > targetAngle) { //turn left
      command = "left";
    } else if (angle < targetAngle) {//turn right
      command = "right";
    }

    //setting up propoertional control, see Step 3 on the website
    if (abs(deltaAngle) > 30){
      targetGyroX = 60;
    } else {
      targetGyroX = 2 * abs(deltaAngle);
    }
    
    if (round(targetGyroX - abs(g.gyro.x)) == 0){
      ;
    } else if (targetGyroX > abs(g.gyro.x)){
      leftSpeedVal = changeSpeed(leftSpeedVal, +1); //would increase abs(GyroX)
    } else {
      leftSpeedVal = changeSpeed(leftSpeedVal, -1);
    }
    rightSpeedVal = leftSpeedVal;
    
    ledcWrite(0, leftSpeedVal); // Channel 0
    ledcWrite(1, rightSpeedVal); // Channel 1

  }
}   

int changeSpeed (int motorSpeed, int increment){
  motorSpeed += increment;
  if (motorSpeed > maxSpeed){ //to prevent motorSpeed from exceeding 255, which is a problem when using analogWrite
    motorSpeed = maxSpeed;
  } else if (motorSpeed < minSpeed){
    motorSpeed = minSpeed;
  }
  return motorSpeed;
}

void calculateError() {
  //When this function is called, ensure the car is stationary
  
  // Read accelerometer values 200 times
  c = 0;
  while (c < 200) {
    readAcceleration();
    // Sum all readings
    AccErrorX += (atan((a.acceleration.y) / sqrt(pow((a.acceleration.x), 2) + pow((a.acceleration.z), 2))) * 180 / PI);
    AccErrorY += (atan(-1 * (a.acceleration.x) / sqrt(pow((a.acceleration.y), 2) + pow((a.acceleration.z), 2))) * 180 / PI);
    c++;
  }
  //Divide the sum by 200 to get the error value, since expected value of reading is zero
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  
  // Read gyro values 200 times
  while (c < 200) {
    readGyro();
    // Sum all readings
    GyroErrorX += g.gyro.x;
    GyroErrorY += g.gyro.y;
    GyroErrorZ += g.gyro.z;
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  Serial.println("The the gryoscope setting in MPU6050 has been calibrated");
}
void correctMotion(){
  // === Read accelerometer (on the MPU6050) data === //
  readAcceleration();
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(a.acceleration.y / sqrt(pow(a.acceleration.x, 2) + pow(a.acceleration.z, 2))) * 180 / PI) - AccErrorX; //AccErrorX is calculated in the calculateError() function
  accAngleY = (atan(-1 * a.acceleration.x / sqrt(pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2))) * 180 / PI) - AccErrorY;
  
   // === Read gyroscope (on the MPU6050) data === //
  previousTime = currentTime;
  currentTime = micros();
  elapsedTime = (currentTime - previousTime) / 1000000; // Divide by 1000 to get seconds
  readGyro();
  // Correct the outputs with the calculated error values
  g.gyro.x -= GyroErrorX; //GyroErrorX is calculated in the calculateError() function
  g.gyro.y -= GyroErrorY;
  g.gyro.z -= GyroErrorZ;

  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX += g.gyro.x * elapsedTime; // deg/s * s = deg
  gyroAngleY += g.gyro.y * elapsedTime;
  yaw += g.gyro.z * elapsedTime;

   //combine accelerometer- and gyro-estimated angle values. 0.96 and 0.04 values are determined through trial and error by other people
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
  angle = roll; //if you mounted MPU6050 in a different orientation to me, angle may not = roll. It can roll, pitch, yaw or minus version of the three
  //for me, turning right reduces angle. Turning left increases angle.

  if (count < 6){  
    count ++;
  } else { //runs once after void loop() runs 7 times. void loop runs about every 2.8ms, so this else condition runs every 19.6ms or 50 times/second
    count = 0;
    if (!paused){
      if (isDriving != prevIsDriving){
          leftSpeedVal = equilibriumSpeed;
          countStraight = 0;
          Serial.print("mode changed, isDriving: ");
          Serial.println(isDriving);
      }
      if (isDriving) {
        if (abs(targetAngle - angle) < 3){
          if (countStraight < 20){
            countStraight ++;
          } else {
            countStraight = 0;
            equilibriumSpeed = leftSpeedVal; //to find equilibrium speed, 20 consecutive readings need to indicate car is going straight
            Serial.print("EQUILIBRIUM reached, equilibriumSpeed: ");
            Serial.println(equilibriumSpeed);
          }
        } else {
          countStraight = 0;
        }
        driving();
      } else {
        rotate();
      }
      prevIsDriving = isDriving;
    }
  }
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

  // Configure PWM properties
const int freq = 5000;  // Frequency in Hz
const int resolution = 8;  // Resolution in bits (0-255 for 8 bits)

// Set up PWM channels
ledcSetup(0, freq, resolution); // Channel 0 for enablePinA
ledcSetup(1, freq, resolution); // Channel 1 for enablePinB

// Attach channels to motor enable pins
ledcAttachPin(enablePinA, 0); // Attach Channel 0 to enablePinA
ledcAttachPin(enablePinB, 1); // Attach Channel 1 to enablePinB

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
  timer.setInterval(20L, checkMPU6050);
  timer.setInterval(50L, measureDistance);
  timer.setInterval(20L, calculateError);
  timer.setInterval(20L, calculateMotion);
  currentTime = micros();
}

void loop() {
  Blynk.run();
  timer.run();
}






