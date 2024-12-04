#include <Arduino.h>
// Motor A
int motor1Pin1 = 26; 
int motor1Pin2 = 27; 
int enablePinA = 25; 
// Motor B 
int motor2Pin1 = 14; 
int motor2Pin2 = 12; 
int enablePinB = 13; 
//Ultrasonic sensor 
const int trigPin = 32;
const int echoPin = 33;
//buzzer
const int buzzer = 19;

void stopMotors() {
  digitalWrite(enablePinA, LOW);
  digitalWrite(enablePinB, LOW);
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
}

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
    Serial.println("Moving Forward");
    setMotors(HIGH, LOW, HIGH, LOW);
  } else if (command == "backward") {
    Serial.println("Moving Backward");
    setMotors(LOW, HIGH, LOW, HIGH);
  } else if (command == "left") {
    Serial.println("Turning Left");
    setMotors(HIGH, LOW, LOW, LOW);
  } else if (command == "right") {
    Serial.println("Turning Right");
    setMotors(LOW, LOW, HIGH, LOW);
  } else if (command == "stop") {
    Serial.println("Stopping Motors");
    stopMotors();
  } else {
    Serial.println("Unknown Command");
  }
}


void setup() {
  // Initialize Motor Pins
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enablePinA, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enablePinB, OUTPUT);
  //Initialize Ultrasonic sensor 
  pinMode(trigPin, OUTPUT);//trigger
  pinMode(echoPin, INPUT);//echo
  
  Serial.begin(115200);

  // testing
  Serial.print("Testing DC Motor...");
  stopMotors();
}

void loop() {
  if (Serial.available() > 0) {

    String input = Serial.readString();
    Serial.print("Command: ");
    Serial.println(input);
    input.trim();
    digitalWrite(trigPin, LOW);
    delay(2);
    digitalWrite(trigPin, HIGH);
    delay(10);
    digitalWrite(trigPin, LOW);
    int timetaken=pulseIn(echoPin,HIGH);
    int distance = timetaken*0.034/2;
    Serial.print("Distance=");
    Serial.print(distance);
    Serial.print("cm");
    Serial.println("");
    if(distance<20){
    tone(buzzer, 500);
    delay(1000);
    noTone(buzzer);
    delay(10);  
    stopMotors();
    }
    else{
    noTone(buzzer);
    directMotors(input);
    }

  }

  
 
 
}





