#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SparkFun_TB6612.h>
#include <Servo.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define ONE_WIRE_BUS 46
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);
int i=0;
const int ledTop=A11;
const int ledB=A12;
// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

namespace TrayStates {
  const String IDLE = "IDLE";
  const String CONFIGURED = "CONFIGURED";
  const String FILLING_SPIRAL_TUBING = "FILLING_SPIRAL_TUBING";
  const String MEDIA_HEATING = "MEDIA_HEATING";
  const String EMPTYING_FLASK = "EMPTYING_FLASK";
  const String FILLING_FLASK = "FILLING_FLASK";
  const String STOPPED = "STOPPED";
}

namespace CoolingStates {
  const String COOLING = "COOLING";
  const String IDLE = "IDLE";
}

namespace Events {
  const String EXCHANGE = "E";
  const String SETUP = "S";
  const String FINISH = "F";
  const String LED_ON = "LED_ON";
  const String LED_OFF = "LED_OFF";
}

int device = 5;
String message = "Device On";

// Display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// DONT CHANGE THIS DIRECTLY, ALWAYS USE SET STATE
JsonDocument state;
// Define motor driver pins for top hardware
#define THERMAL1 48
#define THERMAL2 50

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
String sessionName="";
// Define motor offsets
const int offsetA = 1;
const int offsetB = 1;

// Initialize motors for top hardware
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

// Initialize motors for bottom hardware
Motor motor1Bottom = Motor(AIN1_BOTTOM, AIN2_BOTTOM, PWMA_BOTTOM, offsetA, STBY_BOTTOM);
Motor motor2Bottom = Motor(BIN1_BOTTOM, BIN2_BOTTOM, PWMB_BOTTOM, offsetB, STBY_BOTTOM);

// Define servo pins
#define SERVO_PIN 44
#define SERVO_PIN_BOTTOM 45


// Initialize servos
Servo servo1;
Servo servo2Bottom;

// Define push button pins
#define BUTTON_PIN 3
#define BUTTON_PIN_BOTTOM 2

// Define DRV8871 motor pins
#define DRV8871_PIN 30
#define DRV8871_PIN_R 32

#define DRV8871_PIN2 34
#define DRV8871_PIN_R2 36

volatile bool buttonPressed = false;
volatile bool buttonPressedBottom = false;

// Define time variables
const int motorDriveTime = 1000; // in milliseconds
const int drv8871MotorTime = 5000; // in milliseconds
const int FingerEngage = 1000; // in milliseconds
const int FillTubingTime = 14000; // in milliseconds
const int EmptyFlaskTime = 15000; // in milliseconds

const int FingerEngage1 = 3000; // in milliseconds
const int FillTubingTime1 = 5000; // in milliseconds
const int EmptyFlaskTime1 = 5000; // in milliseconds

unsigned long startTime=0;
unsigned endTime=0;

void setTrayState(String newState, String trayId) {
  state[trayId] = newState;
  serializeJson(state, Serial);
  Serial.println("");
}

void setCoolingState(String newState) {
  state["cooling"] = newState;
  serializeJson(state, Serial);
  Serial.println("");
}

void setTempState(int newTemp) {
  state["chamberTemp"] = newTemp;
  serializeJson(state, Serial);
  Serial.println("");
}
void setLedState(bool newState, String trayId) {
  String ledStatePropName = "led_" + trayId;
  state[ledStatePropName] = newState;
  serializeJson(state, Serial);
  Serial.println("");
}

void TCA9548A(uint8_t bus) {
  Wire.beginTransmission(0x70);  // TCA9548A address
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}

void Display_LCD_Middle(int device, String text) {
  TCA9548A(device);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(5, 20);
  display.println(text);
  
  display.display(); 
}
void Display_LCD_Top(int device, String text,String sN,String T) {
  TCA9548A(device);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(5, 17);
  display.println(text);
  display.setCursor(1, 1);
  display.setTextSize(1.5);
  display.print("Session: ");
  display.print(sN);
  display.print("  T=4C");
  //display.println(sN);
  display.setCursor(1, 55);
  display.setTextSize(1);
  display.print("Progress: ");
  display.print("90% ");
   display.print(T);
  display.println(" min");
  display.display(); 
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  JsonDocument initialState;
  initialState["top"] = TrayStates::IDLE;
  initialState["bottom"] = TrayStates::IDLE;
  initialState["cooling"] = CoolingStates::IDLE;
  initialState["chamberTemp"] = 37;
  initialState["led_top"] = false;
  initialState["led_bottom"] = false;

  pinMode(ledTop, OUTPUT);
  pinMode(ledB, OUTPUT);
  pinMode(THERMAL1, OUTPUT);
  pinMode(THERMAL2, OUTPUT);
  digitalWrite(THERMAL1, HIGH); // Disable standby mode
  digitalWrite(THERMAL2, LOW); // Disable standby mode
  state = initialState;
  serializeJson(state, Serial);
  Serial.println("");

  // Start I2C communication with the Multiplexer
  Wire.begin();

  // Init OLED display on bus number 0
  TCA9548A(5);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  } 
  // Clear the buffer
  display.clearDisplay();

  // Init OLED display on bus number 1
  TCA9548A(7);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  } 
  // Clear the buffer
  display.clearDisplay();

  Display_LCD_Middle(5, message); 
  Display_LCD_Middle(7, message);
  
  // Initialize motor driver
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH); // Disable standby mode

  pinMode(STBY_BOTTOM, OUTPUT);
  digitalWrite(STBY_BOTTOM, HIGH); // Disable standby mode for bottom hardware

  // Initialize servos
  servo1.attach(SERVO_PIN);
  servo1.write(5); // Initial position
  servo2Bottom.attach(SERVO_PIN_BOTTOM);
  servo2Bottom.write(6); // Initial position for bottom hardware

  // Initialize push button with interrupt
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButtonPress, FALLING);

  pinMode(BUTTON_PIN_BOTTOM, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN_BOTTOM), handleButtonPressBottom, FALLING);

  // Initialize DRV8871 motor pins
  pinMode(DRV8871_PIN, OUTPUT);
  pinMode(DRV8871_PIN_R, OUTPUT);
  pinMode(DRV8871_PIN2, OUTPUT);
  pinMode(DRV8871_PIN_R2, OUTPUT);
  digitalWrite(DRV8871_PIN, LOW);
  digitalWrite(DRV8871_PIN_R, LOW);
  digitalWrite(DRV8871_PIN2, LOW);
  digitalWrite(DRV8871_PIN_R2, LOW);
}

void loop() {
  updateTemperature();
  String serialMessage = readNextLineIfAvailable();
  if(serialMessage != "") {
  handleMessage(serialMessage);
  }
  if (buttonPressed) {
    buttonPressed = false;
    stopAllMotorsAndFinish();
  }
  if (buttonPressedBottom) {
    buttonPressedBottom = false;
    stopBottomMotorsAndFinish();
  }
  delay(500);
}

void handleButtonPress() {
  buttonPressed = true;
}
void handleButtonPressBottom() {
  buttonPressedBottom = true;
}

void stopAllMotorsAndFinish() {
  motor1.brake();
  motor2.brake();
  servo1.write(0); // Reset servo position
  setTrayState(TrayStates::STOPPED, "top");
  //setTrayState(TrayStates::STOPPED, "bottom");
  setCoolingState(CoolingStates::IDLE);
  Display_LCD_Middle(5, "STOPPED");
  //Display_LCD_Middle(7, "STOPPED");
}
void stopBottomMotorsAndFinish() {
  motor1Bottom.brake();
  motor2Bottom.brake();
  servo2Bottom.write(8); // Reset bottom servo position
  
  setTrayState(TrayStates::STOPPED, "bottom");
  setCoolingState(CoolingStates::IDLE);
 
  Display_LCD_Middle(7, "STOPPED");
}

void handleMessage(String serialMessage) {
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, serialMessage);
  if (error) {
    Serial.println("ERROR deserializing message");
    Serial.println(error.c_str());
    return;
  }
  //document must contain event property to be a valid event
  if (!doc.containsKey("e")) {
    Serial.println("event prop 'e' missing");
    return;
  }

  if (!doc.containsKey("Id")) {
    Serial.println("trayId prop 'Id' missing");
    return;
  }

  String eventType = doc["e"];
  String trayId = doc["Id"];

  if (!isEventAllowed(eventType, trayId)) {
    Serial.println("Event forbidden");
    return;
  }

  if (eventType == Events::SETUP) {
    /* COMMENTED OUT, because Mannheim Arduino Uno ran out of memory and couldn't process a message containing all of these keys (can't find the MEGA atm)
  if(!eventDoc.containsKey("sessionName") || !eventDoc.containsKey("cellType") || !eventDoc.containsKey("startTime") || !eventDoc.containsKey("endTime")) {
    Serial.println("sessionName, cellType, startTime or endTime missing in Exchange event");
    return; 
  }
  
  String sessionName = eventDoc["sessionName"];
  String cellType = eventDoc["cellType"];
  //startTime and endTime are UTC times (milliseconds since Jan 1, 1970)
  int startTimeUTC = eventDoc["startTime"];
  int endTimeUTC = eventDoc["endTime"];
  */

    if (!doc.containsKey("sN") || !doc.containsKey("T")) {
      Serial.println("sessionName 'sN' or endTime 'T' missing in Exchange event");
      return;
    }
    startTime=millis();
    String sessionName = doc["sN"];
    endTime = doc["T"];
    handleSetupEvent(trayId,sessionName,endTime);
  }

  if (eventType == Events::EXCHANGE) {
    if (!doc.containsKey("mMl")) {
      Serial.println("mediaAmountInMl 'mMl' missing in Exchange event");
      return;
    }
    int mediaAmountInMl = doc["mMl"];
    handleExchangeEvent(trayId, mediaAmountInMl,sessionName,endTime);
  }

  if (eventType == Events::FINISH) {
    handleFinishEvent(trayId);
  }

  if (eventType == Events::LED_ON) {
    handleLedOnEvent(trayId);
  }

  if (eventType == Events::LED_OFF) {
    handleLedOffEvent(trayId);
  }
}

void handleExchangeEvent(String trayId, int mediaAmountInMl,String sessionName,unsigned long Time) {
  if (trayId=="top"){
    device=5;
    
  }
  else{
    device=7;
  }
  JsonDocument newState = state;
  
  setTrayState(TrayStates::FILLING_SPIRAL_TUBING, trayId);
  message=" Starting";
  Display_LCD_Top(device,message,sessionName,String(Time));
  fillSpiralTubing(device);
    //fill spiral tubing
  //Display_LCD_Middle(device,"FILLING SPIRAL TUBING");
  delay(1000);

  setTrayState(TrayStates::MEDIA_HEATING, trayId);
    //heating the media
  message=" Heating    Media";
  Display_LCD_Top(device,message,sessionName,String(Time));
  delay(4000);

//Tilting the flask
  if (device==5){
    tiltFlask(servo1, true);}
  else if (device==7){
    tiltFlask(servo2Bottom, true);}
  //servo1.write(90);
  delay(4000);

  setTrayState(TrayStates::EMPTYING_FLASK, trayId);
  message=" Emptying   Flask";
  Display_LCD_Top(device,message,sessionName,String(Time));
  EmptyFlask(device);

//Tilting back the flask
  if (device==5){
    tiltFlask(servo1, false);}
  else if (device==7){
    tiltFlask(servo2Bottom, false);}
  delay(4000);

 //fill flask
  setTrayState(TrayStates::FILLING_FLASK,trayId);
  message=" Filling    Flask";
  
  Display_LCD_Top(device,message,sessionName,String(Time));
  fillFlask(device);
   
  delay(1000);

  setTrayState(TrayStates::CONFIGURED, trayId);
  message="  Cycle   Completed";
  Display_LCD_Top(device,message,sessionName,String(Time));
  delay(5000);
  message=" Running";
  Display_LCD_Top(device,message,sessionName,String(Time));

}
void fillSpiralTubing(int device) {
  if (device==5){
    for(int i = 0; i <3; i++){
      motor2.drive(255); // Motor 2 backward
      delay(FingerEngage);
      motor2.brake();
      delay(500);
      }
    for(int i = 0; i <1; i++){
      digitalWrite(DRV8871_PIN_R, HIGH);
      delay(FillTubingTime);
      digitalWrite(DRV8871_PIN_R, LOW);}
    
  
    i=0;
    for(int i = 0; i <3; i++){
      motor2.drive(-255); // Motor 2 Disengage
      delay(FingerEngage);
      motor2.brake();
      delay(500);
      }
     i=0;
    }
  else if (device==7){
    for(int i = 0; i <15; i++){
      motor2Bottom.drive(-255,FingerEngage);
      //motor1.drive(-255,1000);
      motor2Bottom.brake();
      
      delay(500);
      }
    delay(2000);
    for(int i = 0; i <7; i++){
      digitalWrite(DRV8871_PIN2, LOW);
      digitalWrite(DRV8871_PIN_R2, HIGH);
      delay(FillTubingTime);
      digitalWrite(DRV8871_PIN_R2, LOW);}
    
    delay(2000);
    for(int i = 0; i <15; i++){
      motor2Bottom.drive(255,FingerEngage);
      //motor1.drive(-255,1000);
      motor2Bottom.brake();
      
      delay(500);
      }
     i=0;}
  }

void EmptyFlask(int device) {
  if (device==5){
    for(int i = 0; i <3; i++){
      motor1.drive(255); // Motor 2 backward
      delay(FingerEngage);
      motor1.brake();
      delay(500);
      }
    i=0;
    delay(2000);
    for(int i = 0; i <1; i++){
      digitalWrite(DRV8871_PIN, HIGH);
      delay(EmptyFlaskTime);
      digitalWrite(DRV8871_PIN, LOW);}
    delay(2000);
    for(int i = 0; i <3; i++){
      motor1.drive(-255); // Motor 2 backward
      delay(FingerEngage);
      motor1.brake();
      delay(500);
      }
    i=0;
    }
  else if (device==7){
    for(int i = 0; i <15; i++){
      motor1Bottom.drive(-255,FingerEngage);
      //motor1.drive(-255,1000);
      motor1Bottom.brake();
      
      delay(500);
      }
    for(int i = 0; i <7; i++){
      digitalWrite(DRV8871_PIN2, LOW);
      digitalWrite(DRV8871_PIN_R2, HIGH);
      delay(EmptyFlaskTime);
      digitalWrite(DRV8871_PIN_R2, LOW);}
    
    for(int i = 0; i <15; i++){
      motor1Bottom.drive(255,FingerEngage);
      //motor1.drive(-255,1000);
      motor1Bottom.brake();
      
      delay(500);
      }
    }
  }

void driveMotorForTime(Motor motor, int driveTime) {
  motor.drive(255); // Drive motor at full speed
  delay(driveTime);
  motor.brake();
}

void fillFlask(int device) {
  if (device==5){
    for(int i = 0; i <3; i++){
      motor2.drive(255); // Motor 2 backward
      delay(FingerEngage);
      motor2.brake();
      delay(500);
      }
     
    for(int i = 0; i <1; i++){
    digitalWrite(DRV8871_PIN_R, HIGH);
    delay(FillTubingTime);
    digitalWrite(DRV8871_PIN_R, LOW);}

    //Display_LCD_Middle(device,"Heating   Media");
    //delay(6000);

    //digitalWrite(DRV8871_PIN, HIGH);
    //delay(FillTubingTime);
    //digitalWrite(DRV8871_PIN, LOW);
    //delay(500);
    //Pumping back media
    //Display_LCD_Middle(device,"Filling   Flask");
    delay(2000);
    for(int i = 0; i <1; i++){
    digitalWrite(DRV8871_PIN, HIGH);
    delay(FillTubingTime);
    digitalWrite(DRV8871_PIN, LOW);}
    for(int i = 0; i <3; i++){
      motor2.drive(-255); // Motor 2 Disengage
      delay(FingerEngage);
      motor2.brake();
      delay(500);
      }
    
  }
  else if (device==7){
    for(int i = 0; i <15; i++){
      motor2Bottom.drive(-255,FingerEngage);
      //motor1.drive(-255,1000);
      motor2Bottom.brake();
      
      delay(500);
      }
    delay(2000);
    for(int i = 0; i <7; i++){
      digitalWrite(DRV8871_PIN2, LOW);
      digitalWrite(DRV8871_PIN_R2, HIGH);
      delay(FillTubingTime);
      digitalWrite(DRV8871_PIN_R2, LOW);}
    delay(5000);

    for(int i = 0; i <7; i++){
      digitalWrite(DRV8871_PIN2, HIGH);
      delay(FillTubingTime);
      digitalWrite(DRV8871_PIN2, LOW);}
    delay(2000);
    
    for(int i = 0; i <15; i++){
      motor2Bottom.drive(255,FingerEngage);
      //motor1.drive(-255,1000);
      motor2Bottom.brake();
      
      delay(500);
      }
  }
  
}
void tiltFlask(Servo servo, bool tilt) {
  if (tilt) {
    for (int pos = 8; pos <= 48; pos += 2) {
      servo.write(pos);
      delay(200);
    }
  } else {
    for (int pos =48; pos >= 8; pos -= 2) {
      servo.write(pos);
      delay(200);
    }
  }
}

void handleSetupEvent(String trayId,String sessionName,unsigned long Time) {
  
  if (trayId=="top"){
    device=5;
  }
  else{
    device=7;
  }
  //start cooling system if not running already
  if(state["cooling"] == CoolingStates::IDLE) {
    startCoolingSystem();
  }
  setTrayState(TrayStates::CONFIGURED,trayId);
  message=" Running";
  Display_LCD_Top(device,message,sessionName,String(Time));
}

void handleFinishEvent(String trayId) {
  if (trayId=="top"){
    device=5;
  }
  else{
    device=7;
  }
  Display_LCD_Middle(device,"Finish"); 
  delay(250);
  setTrayState(TrayStates::IDLE, trayId);
  //stop cooling system, if all trays are IDLE
  if(state["top"] == TrayStates::IDLE && state["bottom"] == TrayStates::IDLE) {
    stopCoolingSystem();
  }
}

void startCoolingSystem() {
  //start cooling system
  digitalWrite(THERMAL2, HIGH);
  setCoolingState(CoolingStates::COOLING);
}

void stopCoolingSystem() {
  //stop cooling system
  digitalWrite(THERMAL2, LOW);
  setCoolingState(CoolingStates::IDLE);
}

void handleLedOnEvent(String trayId) {
  //turn on led
  if(trayId=="top"){
    analogWrite(ledTop,255);}
  else if(trayId=="bottom"){
    analogWrite(ledB,255);}
  delay(250);
  setLedState(true, trayId);
}

void handleLedOffEvent(String trayId) {
  //turn off led
  if(trayId=="top"){
    analogWrite(ledTop,0);}
  else if(trayId=="bottom"){
    analogWrite(ledB,0);}
  delay(250);
  analogWrite(ledTop,0);
  setLedState(false, trayId);
}


bool isEventAllowed(String eventType, String trayId) {
  String trayState = state[trayId];
  bool ledState = state["led_" + trayId];
  if (trayState == TrayStates::IDLE && eventType == Events::SETUP) {
    return true;
  }
  if (trayState == TrayStates::CONFIGURED && (eventType == Events::EXCHANGE || eventType == Events::FINISH)) {
    return true;
  }

  if ((ledState == false && eventType == Events::LED_ON) || (ledState == true && eventType == Events::LED_OFF)) {
    return true;
  }

  return false;
}

int updateTemperature() {
  int temperature;
  sensors.requestTemperatures();
  if(state["cooling"] == CoolingStates::IDLE) {
    temperature = sensors.getTempCByIndex(0);
  } else {
    temperature = sensors.getTempCByIndex(0);
  }
  setTempState(temperature);
}

String readNextLineIfAvailable() {
  /*
  while (Serial.available() == 0) {

    //wait for Serial message to arrive
  }
  */
  String input = "";
  while (Serial.available()) {

    delay(5);
    char c = Serial.read();  //gets one byte from serial buffer
    input += c;              //makes the string readString
  }
  return input;
}
