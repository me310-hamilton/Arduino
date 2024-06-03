const int led1Pin = A11; // LED 1 connected to analog pin A0
const int led2Pin = A12; // LED 2 connected to analog pin A1

void setup() {
  // Initialize the analog pins as outputs
  pinMode(led1Pin, OUTPUT);
  pinMode(led2Pin, OUTPUT);
  
  // Start the serial communication
  Serial.begin(9600);
}

void loop() {
  // Check if data is available to read
  if (Serial.available() > 0) {
    // Read the incoming byte
    char incomingByte = Serial.read();
    
    // Check the received byte and turn on the corresponding LED
    if (incomingByte == '1') {
      analogWrite(led1Pin, 255); // Turn on LED 1 at full brightness
    } else if (incomingByte == '2') {
      analogWrite(led2Pin, 255); // Turn on LED 2 at full brightness
    }
    else if (incomingByte == '3') {
      analogWrite(led1Pin, 255); // Turn on LED 1 at full brightness
    } else if (incomingByte == '4') {
      analogWrite(led2Pin, 255); // Turn on LED 2 at full brightness
    }
  }
}
