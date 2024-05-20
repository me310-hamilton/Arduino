#include <SparkFun_TB6612.h>

// Define motor driver pins
#define AIN1 2
#define AIN2 4
#define PWMA 5
#define BIN1 7
#define BIN2 8
#define PWMB 6
#define STBY 9

// Define motor offsets
const int offsetA = 1;
const int offsetB = 1;

// Initialize motors
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

void setup() {
  // Initialize the standby pin
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH); // Disable standby mode
}

void loop() {
  // Example: Motor 1 forward, Motor 2 backward
  motor1.drive(255);  // Motor 1: forward
  motor2.drive(-255); // Motor 2: backward
  delay(2000);        // Run motors for 2 seconds

  // Stop both motors
  motor1.brake();
  motor2.brake();
  delay(2000);        // Wait for 2 seconds

  // Example: Motor 1 backward, Motor 2 forward
  motor1.drive(-255); // Motor 1: backward
  motor2.drive(255);  // Motor 2: forward
  delay(2000);        // Run motors for 2 seconds

  // Stop both motors
  motor1.brake();
  motor2.brake();
  delay(2000);        // Wait for 2 seconds

  // Put motor driver in standby mode
  digitalWrite(STBY, LOW);
  delay(2000);        // Wait for 2 seconds

  // Disable standby mode
  digitalWrite(STBY, HIGH);
  delay(2000);        // Wait for 2 seconds
}