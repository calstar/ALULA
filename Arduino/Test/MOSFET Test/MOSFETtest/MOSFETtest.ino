#include <Arduino.h>
#include <Wire.h>
#include <EasyPCF8575.h>

EasyPCF8575 pcf;

int dt = 50;

void setup() {
  Serial.begin(115200);
  pcf.startI2C(21, 22, SEARCH); //Only SEARCH, if using normal pins in Arduino
  if (!pcf.check(SEARCH)) {
    Serial.println("Device not found. Try to specify the address");
    Serial.println(pcf.whichAddr());
    while (true);
  }
  pcf.setAllBitsUp(); // make sure everything is off by default (Up = Off, Down = On)
  delay(500); // startup time to make sure its good for personal testing
}

void loop() {
  int num_pins = 8;
  cycle_led(num_pins);
//  toggle_0_7(0);
//  toggle_0_7(1);
//  toggle_0_7(2);
//  toggle_0_7(3);
//  toggle_0_7(4);
//  toggle_0_7(5);
//  toggle_0_7(6);
//  toggle_0_7(7);
}

void cycle_led(int num_pins) {
  for (int i = 0; i < num_pins; i++) {
    pcf.setLeftBitUp(i);
    Serial.print("pin high: ");
    Serial.println(i);
    delay(dt);
  }

  for (int i = 0; i < num_pins; i++) {
    pcf.setLeftBitDown(i);
    Serial.print("pin low: ");
    Serial.println(i);
    delay(dt);
  }
  
}

void toggle_0_7(int i) {
  delay(dt);
  pcf.setLeftBitUp(i);
  delay(dt);
  pcf.setLeftBitDown(i);
}

void toggle_8_15(int i) {
  delay(dt);
  pcf.setRightBitUp(i);
  delay(dt);
  pcf.setRightBitDown(i);
}
