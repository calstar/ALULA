int RelayPins[] = {22, 23, 19, 21, 5, 18, 16, 17};
int pinCount = 8;

void setup() {
	// Set RelayPin as an output pin
  for (int pinNum = 0; pinNum < pinCount; pinNum++) {
    pinMode(RelayPins[pinNum], OUTPUT); 
    digitalWrite(RelayPins[pinNum], HIGH);
    Serial.begin(115200);
  }
}

void loop() {
  Serial.println("Running");
  for (int pinNum = 0; pinNum < pinCount; pinNum++) {
    // Serial.print("Pin Num: ");
    // Serial.println(RelayPins[pinNum]);

    // Let's turn on the relay...
    digitalWrite(RelayPins[pinNum], LOW);
    Serial.print(pinNum);
    }
    delay(5000);
	
  for (int pinNum = 0; pinNum < pinCount; pinNum++) {
    // Let's turn off the relay...
    digitalWrite(RelayPins[pinNum], HIGH);
  }
  delay(1500);
}

// TO DO: test out with double relay
// pin 23/relay 1 works
// pin 22/relay 2 untested 
// pin 21/relay 2 works
// pin 19/relay 2 untested
//