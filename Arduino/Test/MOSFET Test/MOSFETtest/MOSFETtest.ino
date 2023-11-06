#include <Arduino.h>
#include <Wire.h>
#include <EasyPCF8575.h>
EasyPCF8575 pcf;

int dt = 250;

void setup() {
  Serial.begin(115200);
  pcf.startI2C(22, 21, SEARCH); //Only SEARCH, if using normal pins in Arduino
  if (!pcf.check(SEARCH)) {
    Serial.println("Device not found. Try to specify the address");
    Serial.println(pcf.whichAddr());
    while (true);
  }
  pcf.setAllBitsUp(); // make sure everything is off by default (Up = Off, Down = On)
  delay(500); // startup time to make sure its good for personal testing
}

void loop() {
  int pins[] = {1, 0, 8, 9, 10, 11, 12, 13, 14, 15}; // Array of pins
  int numPins = 10; // Number of pins in the array
  cycle_led(pins, numPins); // Call the function to set all pins high
//  toggle(0);
//  toggle(1);
//  toggle(8);
//  toggle(9);
//  toggle(10);
//  toggle(11);
//  toggle(12);
//  toggle(13);
//  toggle(14);
//  toggle(15);
}

void cycle_led(int pins[], int numPins) {
  for (int i = 0; i < numPins; i++) {
    pcf.setBitUp(pins[i]);
    Serial.print("pin high: ");
    Serial.println(pins[i]);
    delay(dt);
  }

  for (int i = 0; i < numPins; i++) {
    pcf.setBitDown(pins[i]);
    Serial.print("pin low: ");
    Serial.println(pins[i]);
    delay(dt);
  }
}


void toggle(int i) {
  delay(dt);
  pcf.setBitUp(i);
  delay(dt);
  pcf.setBitDown(i);
}
