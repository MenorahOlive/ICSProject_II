#include <Arduino.h>

// Motor A
int motor1Pin1 = 26; 
int motor1Pin2 = 27; 
int enablePinA = 25; 
// Motor B 
int motor2Pin1 = 14; 
int motor2Pin2 = 12; 
int enablePinB = 13; 


void setup() {
  // Initialize Motor Pins
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enablePinA, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enablePinB, OUTPUT);
  
  Serial.begin(115200);

  // testing
  Serial.print("Testing DC Motor...");
}

void loop() {
  // Move the DC motor forward at maximum speed
  Serial.println("Moving Forward");
  digitalWrite(enablePinA, HIGH);
  digitalWrite(enablePinB, HIGH); 
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW); 
   digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW); 
  delay(2000);

  // Stop the DC motor
  Serial.println("Motor stopped");
  digitalWrite(enablePinA, LOW);
  digitalWrite(enablePinB, LOW); 
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW); 
  delay(1000);

  // Move DC motor backwards at maximum speed
  Serial.println("Moving Backwards");
  digitalWrite(enablePinA, HIGH);
  digitalWrite(enablePinB, HIGH); 
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH); 
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH); 
  delay(2000);

  // Stop the DC motor
  Serial.println("Motor stopped");
  digitalWrite(enablePinA, LOW);
  digitalWrite(enablePinB, LOW); 
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW); 
  delay(1000);

    // Move DC motor right at maximum speed
  Serial.println("Moving Right");
  digitalWrite(enablePinA, HIGH);
  digitalWrite(enablePinB, HIGH); 
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW); 
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW); 
  delay(2000);

  // Stop the DC motor
  Serial.println("Motor stopped");
  digitalWrite(enablePinA, LOW);
  digitalWrite(enablePinB, LOW); 
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW); 
  delay(1000);

    // Move DC motor left at maximum speed
  Serial.println("Moving Left");
  digitalWrite(enablePinA, HIGH);
  digitalWrite(enablePinB, HIGH); 
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW); 
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW); 
  delay(2000);

  // Stop the DC motor
  Serial.println("Motor stopped");
  digitalWrite(enablePinA, HIGH);
  digitalWrite(enablePinB, HIGH); 
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW); 
  delay(1000);

}