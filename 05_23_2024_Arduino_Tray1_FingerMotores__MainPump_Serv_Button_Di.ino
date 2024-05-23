#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SparkFun_TB6612.h>
#include <Servo.h>
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

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
  const String EXCHANGE = "EXCHANGE";
  const String SETUP = "SETUP";
  const String FINISH = "FINISH";
}

int device = 5;
String message = "Device On";

// Display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// DONT CHANGE THIS DIRECTLY, ALWAYS USE SET STATE
JsonDocument state;
// Define motor driver pins for top hardware
#define AIN1 2
#define AIN2 4
#define PWMA 5
#define BIN1 7
#define BIN2 8
#define PWMB 6
#define STBY 9

// Define motor driver pins for bottom hardware
#define AIN1_BOTTOM 12
#define AIN2_BOTTOM 13
#define PWMA_BOTTOM 14
#define BIN1_BOTTOM 15
#define BIN2_BOTTOM 16
#define PWMB_BOTTOM 17
#define STBY_BOTTOM 18

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
#define SERVO_PIN 10
#define SERVO_PIN_BOTTOM 19


// Initialize servos
Servo servo1;
Servo servo2Bottom;

// Define push button pins
#define BUTTON_PIN 3
#define BUTTON_PIN_BOTTOM 2

// Define DRV8871 motor pins
#define DRV8871_PIN 11
#define DRV8871_PIN_R 53
#define DRV8871_PIN2 49
#define DRV8871_PIN_R2 51

volatile bool buttonPressed = false;
volatile bool buttonPressedBottom = false;

// Define time variables
const int motorDriveTime = 1000; // in milliseconds
const int drv8871MotorTime = 5000; // in milliseconds

void setTrayState(String newState, String trayId) {
  state[trayId] = newState;
  serializeJsonPretty(state, Serial);
}

void setCoolingState(String newState) {
  state["cooling"] = newState;
  serializeJsonPretty(state, Serial);
}

void setTempState(int newTemp) {
  state["chamberTemp"] = newTemp;
  serializeJsonPretty(state, Serial);
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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  JsonDocument initialState;
  initialState["top"] = TrayStates::IDLE;
  initialState["bottom"] = TrayStates::IDLE;
  initialState["cooling"] = CoolingStates::IDLE;
  initialState["chamberTemp"] = 37;

  state = initialState;
  serializeJsonPretty(state, Serial);

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
  servo1.write(8); // Initial position
  servo2Bottom.attach(SERVO_PIN_BOTTOM);
  servo2Bottom.write(8); // Initial position for bottom hardware

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
  delay(1000);
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
  Serial.setTimeout(10000);
  DeserializationError error = deserializeJson(doc, serialMessage);
  if (error) {
    Serial.println("ERROR deserializing message");
    Serial.println(error.c_str());
    return;
  }
  //document must contain event property to be a valid event
  if (!doc.containsKey("event")) {
    Serial.println("event prop missing");
    return;
  }
  if (!doc.containsKey("Id")) {
  //if (!doc.containsKey("trayId")) {
    Serial.println("trayId prop missing");
    return;
  }

  String eventType = doc["event"];
  //String trayId = doc["trayId"];
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
    handleSetupEvent(trayId);
    
    
  }

  if (eventType == Events::EXCHANGE) {
    //if(!doc.containsKey("mediaAmountInMl")) {
    if(!doc.containsKey("mMl")) {
    Serial.println("mediaAmountInMl missing in Exchange event");
    return; 
  }
  int mediaAmountInMl = doc["mMl"];
  //int mediaAmountInMl = doc["mediaAmountInMl"];
    handleExchangeEvent(trayId, mediaAmountInMl);
  }
  
  if (eventType == Events::FINISH) {
    handleFinishEvent(trayId);
  }
}

void handleExchangeEvent(String trayId, int mediaAmountInMl) {
  if (trayId=="top"){
    device=5;
  }
  else{
    device=7;
  }
  JsonDocument newState = state;
  
  setTrayState(TrayStates::FILLING_SPIRAL_TUBING, trayId);
  
  fillSpiralTubing();
    //fill spiral tubing
  //Display_LCD_Middle(device,"FILLING SPIRAL TUBING");
  delay(1000);

  setTrayState(TrayStates::MEDIA_HEATING, trayId);
    //heating the media
  message="Heating   Media";
  Display_LCD_Middle(device,message);
  delay(1000);

//Tilting the flask
  tiltFlask(servo1, true);
  //servo1.write(90);
  delay(1000);

  setTrayState(TrayStates::EMPTYING_FLASK, trayId);
  message="Emptying Flask";
  Display_LCD_Middle(device,"Emptying  Flask");
    //tilt, empty, detilt flask
  digitalWrite(DRV8871_PIN, HIGH);
  delay(5000);

  digitalWrite(DRV8871_PIN, LOW);

//Tilting back the flask
  tiltFlask(servo1, false);
  delay(1000);

  setTrayState(TrayStates::FILLING_FLASK,trayId);
  message="Filling Flask";
  fillFlask();
  Display_LCD_Middle(device,"Filling   Flask");
    //fill flask
  delay(1000);

  setTrayState(TrayStates::CONFIGURED, trayId);
  message="Running";
  Display_LCD_Middle(device,message);

}
void fillSpiralTubing() {
  motor1.drive(-255); // Motor 1 backward
  delay(1000);
  motor1.brake();
  digitalWrite(DRV8871_PIN, HIGH);
  delay(5000);

  digitalWrite(DRV8871_PIN, LOW);
  motor1.drive(255);  // Motor 1 forward
  delay(1000);
  motor1.brake();
}
void driveMotorForTime(Motor motor, int driveTime) {
  motor.drive(255); // Drive motor at full speed
  delay(driveTime);
  motor.brake();
}

void fillFlask() {
  motor2.drive(-255); // Motor 2 backward
  delay(1000);
  motor2.brake();
  digitalWrite(DRV8871_PIN, HIGH);
  delay(5000);

  digitalWrite(DRV8871_PIN, LOW);
  
  motor2.drive(255);  // Motor 2 forward
  delay(1000);
  motor2.brake();
}
void tiltFlask(Servo servo, bool tilt) {
  if (tilt) {
    for (int pos = 8; pos <= 47; pos += 2) {
      servo.write(pos);
      delay(200);
    }
  } else {
    for (int pos =47; pos >= 8; pos -= 2) {
      servo.write(pos);
      delay(200);
    }
  }
}

void handleSetupEvent(String trayId) {
  
  
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
  message="Running";
  Display_LCD_Middle(device,message); 
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
  setCoolingState(CoolingStates::COOLING);
}

void stopCoolingSystem() {
  //stop cooling system
  setCoolingState(CoolingStates::IDLE);
}


bool isEventAllowed(String eventType, String trayId) {
  String trayState = state[trayId];
  if (trayState == TrayStates::IDLE && eventType == Events::SETUP) {
    return true;
  }
  if (trayState == TrayStates::CONFIGURED && (eventType == Events::EXCHANGE || eventType == Events::FINISH)) {
    return true;
  }
  return false;
}

int updateTemperature() {
  int temperature;
  if(state["cooling"] == CoolingStates::IDLE) {
    temperature = random(33,37);
  } else {
    temperature = random(4,7);
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
