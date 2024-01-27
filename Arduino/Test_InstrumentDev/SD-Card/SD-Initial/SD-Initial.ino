/*
  SD card test

  This example shows how use the utility libraries on which the'
  SD library is based in order to get info about your SD card.
  Very useful for testing a card when you're not sure whether its working or not.

  The circuit:
    FOR ESP32 DEVKIT C
    SD card attached to SPI bus as follows:
 ** MOSI - 23
 ** MISO - 19
 ** CLK - 18
 ** CS - 5.
 		MAKE SURE TO REFER TO PINOUT OF THE BOARD TO FIND DEFAULT SPI PINS. 


*/
// include the SD library:
#include <SPI.h>
#include <SD.h>


// change this to match your SD shield or module;
const int chipSelect = 5;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  pinMode(chipSelect, OUTPUT);


  Serial.print("\nInitializing SD card...");

  // we'll use the initialization code from the utility libraries
  // since we're just testing if the card is working!
  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed. Things to check:");
    Serial.println("* is a card inserted?");
    Serial.println("* is your wiring correct?");
    Serial.println("* did you change the chipSelect pin to match your shield or module?");
    while (1);
  } else {
    Serial.println("Wiring is correct and a card is present.");
  }
}

void loop(void) {
}
