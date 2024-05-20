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

int device = 0;
String message = "Device On";

// Display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// DONT CHANGE THIS DIRECTLY, ALWAYS USE SET STATE
JsonDocument state;

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

// Define servo pin
#define SERVO_PIN 10

// Initialize servo
Servo servo1;

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
  TCA9548A(0);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  } 
  // Clear the buffer
  display.clearDisplay();

  // Init OLED display on bus number 1
  TCA9548A(1);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  } 
  // Clear the buffer
  display.clearDisplay();

  Display_LCD_Middle(0, message); 
  Display_LCD_Middle(1, message);
  
  // Initialize motor driver
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH); // Disable standby mode

  // Initialize servo
  servo1.attach(SERVO_PIN);
  servo1.write(0); // Initial position
}

void loop() {
  updateTemperature();
  String serialMessage = readNextLineIfAvailable();
  if(serialMessage != "") {
  handleMessage(serialMessage);
  }
  delay(1000);
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
    device=0;
  }
  else{
    device=1;
  }
  JsonDocument newState = state;
  
  setTrayState(TrayStates::FILLING_SPIRAL_TUBING, trayId);
  fillSpiralTubing();
    //fill spiral tubing
  //Display_LCD_Middle(device,"FILLING SPIRAL TUBING");
  delay(1000);

  setTrayState(TrayStates::MEDIA_HEATING, trayId);
    //heating the media
  message="Heating Media";
  Display_LCD_Middle(device,message);
  delay(1000);

//Tilting the flask
  servo1.write(90);
  delay(1000);

  setTrayState(TrayStates::EMPTYING_FLASK, trayId);
  message="Emptying Flask";
  Display_LCD_Middle(device,"Emptying Flask");
    //tilt, empty, detilt flask
  delay(1000);

//Tilting back the flask
  servo1.write(0);
  delay(1000);

  setTrayState(TrayStates::FILLING_FLASK,trayId);
  message="Filling Flask";
  fillFlask();
  Display_LCD_Middle(device,"Filling Flask");
    //fill flask
  delay(1000);

  setTrayState(TrayStates::CONFIGURED, trayId);

}
void fillSpiralTubing() {
  motor1.drive(-255); // Motor 1 backward
  delay(1000);
  motor1.brake();
  delay(5000);
  motor1.drive(255);  // Motor 1 forward
  delay(1000);
  motor1.brake();
}

void fillFlask() {
  motor2.drive(-255); // Motor 2 backward
  delay(1000);
  motor2.brake();
  delay(5000);
  motor2.drive(255);  // Motor 2 forward
  delay(1000);
  motor2.brake();
}

void handleSetupEvent(String trayId) {
  
  
  if (trayId=="top"){
    device=0;
  }
  else{
    device=1;
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
    device=0;
  }
  else{
    device=1;
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
