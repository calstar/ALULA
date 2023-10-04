#include <Arduino.h>
#include <Wire.h>
#include <EasyPCF8575.h>

EasyPCF8575 pcf;

void setup(){
    Serial.begin(112500);

    pcf.startI2C(21,22,0x20); //Only SEARCH, if using normal pins in Arduino

    if (!pcf.check(0x20)){
        Serial.println("Device not found. Try to specify the address");
        Serial.println(pcf.whichAddr());
        while (true);
    } 
    pcf.setAllBitsUp();
}

// MOSFET 1-10 correspond to the pins
int expected_pins[] = {0, 1, 2, 3, 4, 15, 14, 13, 12, 11};
void loop(){
  if (Serial.available() > 0) {
    // Serial.read reads a single character as ASCII. Number 1 is 49 in ASCII.
    // Serial sends character and new line character "\n", which is 10 in ASCII.
    int num = 0;
    while (Serial.available() > 0) {
      int c = Serial.read();
      if (isDigit(c)) {
          num = num * 10 + (c - 48);
      }
    }
    pcf.setAllBitsDown();
    Serial.println(num);
    pcf.setBitUp(num);
  }
}
