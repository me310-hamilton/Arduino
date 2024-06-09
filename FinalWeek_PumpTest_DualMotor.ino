#include <SparkFun_TB6612.h>

// Pins for all inputs, keep in mind the PWM defines must be on PWM pins
// the default pins listed are the ones used on the Redbot (ROB-12097) with
// the exception of STBY which the Redbot controls with a physical switch
#define PWMA 17
#define AIN2 15
#define AIN1 14
#define STBY 16
#define BIN1 4
#define BIN2 5
#define PWMB 6

// Define motor driver pins for bottom hardware
#define PWMA_BOTTOM 7
#define AIN2_BOTTOM 8
#define AIN1_BOTTOM 9
#define STBY_BOTTOM 10
#define BIN1_BOTTOM 11
#define BIN2_BOTTOM 12
#define PWMB_BOTTOM 13


char incomingbyte='0';
// these constants are used to allow you to make your motor configuration 
// line up with function names like forward.  Value can be 1 or -1
// Define motor offsets
const int offsetA = 1;
const int offsetB = 1;

// Initialize motors for top hardware
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

// Initialize motors for bottom hardware
Motor motor1Bottom = Motor(AIN1_BOTTOM, AIN2_BOTTOM, PWMA_BOTTOM, offsetA, STBY_BOTTOM);
Motor motor2Bottom = Motor(BIN1_BOTTOM, BIN2_BOTTOM, PWMB_BOTTOM, offsetB, STBY_BOTTOM);

#define DRV8871_PIN 30
#define DRV8871_PIN_R 32

#define DRV8871_PIN2 34
#define DRV8871_PIN_R2 36

void setup() {
  // Initialize the standby pin

  Serial.begin(9600);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1_BOTTOM, OUTPUT);
  pinMode(AIN2_BOTTOM , OUTPUT);
  pinMode(PWMA, OUTPUT);
  
  // Initialize DRV8871 motor pins
  pinMode(DRV8871_PIN, OUTPUT);
  pinMode(DRV8871_PIN_R, OUTPUT);
  pinMode(DRV8871_PIN2, OUTPUT);
  pinMode(DRV8871_PIN_R2, OUTPUT);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(PWMA, LOW);
  digitalWrite(DRV8871_PIN, LOW);
  digitalWrite(DRV8871_PIN_R, LOW);
  digitalWrite(DRV8871_PIN2, LOW);
  digitalWrite(DRV8871_PIN_R2, LOW);
   
}

void loop() {
  if(Serial.available()>0){
    incomingbyte=Serial.read();
    if(incomingbyte=='1'){
      motor1.drive(255,1000);
      //motor1.drive(-255,1000);
      motor1.brake();
      delay(1000);
      }
  
  
    if(incomingbyte=='2'){
      Serial.print("2");
      //motor1.drive(255,1000);
      motor1.drive(-255,1000);
      motor1.brake();
      delay(1000);
      }
     
  
  
    if(incomingbyte=='3'){
      digitalWrite(DRV8871_PIN, HIGH);
      digitalWrite(DRV8871_PIN_R, LOW);
      delay(10000);
      // Stop both motors
      digitalWrite(DRV8871_PIN, LOW);
      //motor2.brake();
      //delay(1000);        // Wait for 2 seconds
      //motor1.drive(-255); // Motor 1: backward
      //motor2.drive(255);  // Motor 2: forward
      //delay(1000);        // Run motors for 2 seconds
      }
      if(incomingbyte=='4'){
      digitalWrite(DRV8871_PIN, LOW);
      digitalWrite(DRV8871_PIN_R, HIGH);
      delay(10000);
      // Stop both motors
      digitalWrite(DRV8871_PIN_R, LOW);
      //motor2.brake();
      //delay(1000);        // Wait for 2 seconds
      //motor1.drive(-255); // Motor 1: backward
      //motor2.drive(255);  // Motor 2: forward
      //delay(1000);        // Run motors for 2 seconds
      }


      if(incomingbyte=='5'){
      motor2.drive(255,1000);
      //motor1.drive(-255,1000);
      motor2.brake();
      delay(1000);
      }
      if(incomingbyte=='6'){
      Serial.print("6");
   
      motor2.drive(-255,1000);
      motor2.brake();
      delay(1000);
      }
      
      if(incomingbyte=='7'){
      digitalWrite(DRV8871_PIN2, HIGH);
      digitalWrite(DRV8871_PIN_R2, LOW);
      delay(5000);
      // Stop both motors
      digitalWrite(DRV8871_PIN2, LOW);
      }
      if(incomingbyte=='8'){
      digitalWrite(DRV8871_PIN2, LOW);
      digitalWrite(DRV8871_PIN_R2, HIGH);
      delay(5000);
     
      digitalWrite(DRV8871_PIN_R2, LOW);
     
      }
      
      if(incomingbyte=='a'){
      motor1Bottom.drive(255,1000);
      //motor1.drive(-255,1000);
      motor1Bottom.brake();
      delay(1000);
      }
      if(incomingbyte=='b'){
      motor1Bottom.drive(-255,1000);
      //motor1.drive(-255,1000);
      motor1Bottom.brake();
      delay(1000);
      }
      if(incomingbyte=='c'){
      motor2Bottom.drive(255,1000);
      //motor1.drive(-255,1000);
      motor2Bottom.brake();
      delay(1000);
      }
      if(incomingbyte=='d'){
      motor2Bottom.drive(-255,1000);
      //motor1.drive(-255,1000);
      motor2Bottom.brake();
      delay(1000);
      }
      }
  
  
  

  // Put motor driver in standby mode
  
}
