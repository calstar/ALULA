int RelayPin = 23;

void setup() {
	// Set RelayPin as an output pin
	pinMode(RelayPin, OUTPUT);
}

void loop() {
	// Let's turn on the relay...
	digitalWrite(RelayPin, LOW);
	delay(3000);
	
	// Let's turn off the relay...
	digitalWrite(RelayPin, HIGH);
	delay(3000);
}

// TO DO: test out with double relay
// pin 23/relay 1 works
// pin 22/relay 2 untested 
// pin 21/relay 2 works
// pin 19/relay 2 untested
//