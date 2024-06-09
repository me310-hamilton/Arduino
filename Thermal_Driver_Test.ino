int loadPin=48;// load pin

  
void setup() {
  
  pinMode(loadPin,OUTPUT);


}

void loop() {
  

  digitalWrite(loadPin, HIGH);
  delay(50000);
  digitalWrite(loadPin, LOW);
  delay(10000);


} 
