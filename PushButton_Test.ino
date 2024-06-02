const int buttonPin = 19;  // Pin where the button is connected
const int ledPin = 13;     // Internal LED pin

int buttonState = 0;       // Variable for reading the button status

void setup() {
  pinMode(buttonPin, INPUT_PULLUP); // Initialize the button pin as an input with internal pull-up resistor
  pinMode(ledPin, OUTPUT);          // Initialize the LED pin as an output
}

void loop() {
  buttonState = digitalRead(buttonPin); // Read the state of the button

  // Check if the button is pressed
  // Since the button is connected to ground, the input will be LOW when pressed
  if (buttonState == LOW) {
    digitalWrite(ledPin, HIGH); // Turn on the LED
  } else {
    digitalWrite(ledPin, LOW);  // Turn off the LED
  }
}
