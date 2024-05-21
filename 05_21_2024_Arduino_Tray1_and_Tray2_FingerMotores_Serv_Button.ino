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
#define BUTTON_PIN_BOTTOM 21

// Define DRV8871 motor pins
#define DRV8871_PIN 11
#define DRV8871_PIN_BOTTOM 20

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
  pinMode(DRV8871_PIN_BOTTOM, OUTPUT);
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
  motor1Bottom.brake();
  motor2Bottom.brake();
  servo1.write(8); // Reset servo position
  //servo2Bottom.write(0); // Reset bottom servo position
  setTrayState(TrayStates::STOPPED, "top");
  
  setCoolingState(CoolingStates::IDLE);
  Display_LCD_Middle(5, "STOPPED");
  
}

void stopBottomMotorsAndFinish() {
  motor1Bottom.brake();
  motor2Bottom.brake();
  servo2Bottom.write(8); // Reset bottom servo position
  
  setTrayState(TrayStates::STOPPED, "bottom");
  setCoolingState(CoolingStates::IDLE);
  servo2Bottom.write(8); // Reset bottom servo position
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
    Serial.println("trayId prop missing");
    return;
  }

  String eventType = doc["event"];
  String trayId = doc["Id"];

  if (!isEventAllowed(eventType, trayId)) {
    Serial.println("Event forbidden");
    return;
  }

  if (eventType == Events::SETUP) {
    handleSetupEvent(trayId);
  }

  if (eventType == Events::EXCHANGE) {
    if(!doc.containsKey("mMl")) {
      Serial.println("mediaAmountInMl missing in Exchange event");
      return; 
    }
    int mediaAmountInMl = doc["mMl"];
    handleExchangeEvent(trayId, mediaAmountInMl);
  }
  
  if (eventType == Events::FINISH) {
    handleFinishEvent(trayId);
  }
}

void handleExchangeEvent(String trayId, int mediaAmountInMl) {
  if (trayId == "top") {
    device = 5;
    handleExchangeTop(mediaAmountInMl);
  } else if (trayId == "bottom") {
    device = 7;
    handleExchangeBottom(mediaAmountInMl);
  }
}

void handleExchangeTop(int mediaAmountInMl) {
  setTrayState(TrayStates::FILLING_SPIRAL_TUBING, "top");
  fillSpiralTubing(motor1, DRV8871_PIN);
  delay(1000);

  setTrayState(TrayStates::MEDIA_HEATING, "top");
  message = "Heating Media";
  Display_LCD_Middle(device, message);
  delay(1000);

  tiltFlask(servo1, true);

  setTrayState(TrayStates::EMPTYING_FLASK, "top");
  message = "Emptying Flask";
  Display_LCD_Middle(device, message);
  driveMotorForTime(motor1, motorDriveTime);
  driveMotorForTime(motor2, motorDriveTime);

  setTrayState(TrayStates::FILLING_FLASK, "top");
  message = "Filling Flask";
  Display_LCD_Middle(device, message);
  delay(1000);
  driveMotorForTime(motor1, motorDriveTime);
  driveMotorForTime(motor2, motorDriveTime);
}

void handleExchangeBottom(int mediaAmountInMl) {
  setTrayState(TrayStates::FILLING_SPIRAL_TUBING, "bottom");
  fillSpiralTubing(motor1Bottom, DRV8871_PIN_BOTTOM);
  delay(1000);

  setTrayState(TrayStates::MEDIA_HEATING, "bottom");
  message = "Heating Media";
  Display_LCD_Middle(device, message);
  delay(1000);

  tiltFlask(servo2Bottom, true);

  setTrayState(TrayStates::EMPTYING_FLASK, "bottom");
  message = "Emptying Flask";
  Display_LCD_Middle(device, message);
  driveMotorForTime(motor1Bottom, motorDriveTime);
  driveMotorForTime(motor2Bottom, motorDriveTime);

  setTrayState(TrayStates::FILLING_FLASK, "bottom");
  message = "Filling Flask";
  Display_LCD_Middle(device, message);
  delay(1000);
  driveMotorForTime(motor1Bottom, motorDriveTime);
  driveMotorForTime(motor2Bottom, motorDriveTime);
}

void fillSpiralTubing(Motor motor, int drvPin) {
  message = "Filling Spiral Tubing";
  Display_LCD_Middle(device, message);
  motor.drive(255); // Drive motor at full speed
  delay(motorDriveTime);
  motor.brake();
  digitalWrite(drvPin, HIGH);
  delay(drv8871MotorTime);
  digitalWrite(drvPin, LOW);
}

void driveMotorForTime(Motor motor, int driveTime) {
  motor.drive(255); // Drive motor at full speed
  delay(driveTime);
  motor.brake();
}

void tiltFlask(Servo servo, bool tilt) {
  if (tilt) {
    for (int pos = 0; pos <= 90; pos += 2) {
      servo.write(pos);
      delay(200);
    }
  } else {
    for (int pos = 90; pos >= 0; pos -= 2) {
      servo.write(pos);
      delay(200);
    }
  }
}

void handleSetupEvent(String trayId) {
  setTrayState(TrayStates::CONFIGURED, trayId);
  if (trayId == "top") {
    device = 5;
  } else {
    device = 7;
  }
  message = "Configuration";
  Display_LCD_Middle(device, message);
}

void handleFinishEvent(String trayId) {
  if (trayId == "top") {
    device = 5;
  } else {
    device = 7;
  }
  setTrayState(TrayStates::IDLE, trayId);
  message = "Process Complete";
  Display_LCD_Middle(device, message);
}

void updateTemperature() {
  // Placeholder: Replace with your temperature reading code
  int temperature = random(35, 38); // Random temperature between 35 and 37 for simulation
  setTempState(temperature);
}

String readNextLineIfAvailable() {
  String serialMessage = "";
  while (Serial.available() > 0) {
    serialMessage = Serial.readStringUntil('\n');
  }
  return serialMessage;
}

bool isEventAllowed(String eventType, String trayId) {
  // event is not allowed if the tray is stopped
  if (state[trayId] == TrayStates::STOPPED) {
    return false;
  }
  // finish events are only allowed if the tray is configured
  if (eventType == Events::FINISH && state[trayId] == TrayStates::CONFIGURED) {
    return true;
  }
  // setup and exchange events are not allowed if the tray is not idle or configured
  if ((eventType == Events::SETUP || eventType == Events::EXCHANGE) && 
      !(state[trayId] == TrayStates::IDLE || state[trayId] == TrayStates::CONFIGURED)) {
    return false;
  }
  return true;
}
