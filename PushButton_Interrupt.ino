// Constants
const int buttonPin = 18;    // Pin connected to the button
const int ledPin = 13;       // Pin connected to the internal LED

// Variables
volatile bool ledState = false;  // Variable to hold the state of the LED

// Interrupt Service Routine (ISR)
void handleButtonPress() {
  ledState = !ledState;  // Toggle the LED state
  digitalWrite(ledPin, ledState);  // Update the LED
}

void setup() {
  // Set the LED pin as output
  pinMode(ledPin, OUTPUT);
  
  // Set the button pin as input with an internal pull-up resistor
  pinMode(buttonPin, INPUT_PULLUP);
  
  // Attach interrupt to the button pin
  attachInterrupt(digitalPinToInterrupt(buttonPin), handleButtonPress, FALLING);
}

void loop() {
  // Nothing to do here, everything is handled by the interrupt
}
