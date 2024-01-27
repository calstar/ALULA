#include <Arduino.h>
#include <Wire.h>
#include "PCF8575.h"

PCF8575 pcf8575(0x20);

void setup(){
  Serial.begin(112500);

  for(int i=0; i<16; i++) {
    pcf8575.pinMode(i, OUTPUT);
  }
  pcf8575.begin();

  mosfetCloseAllValves();
}

// Expected pins
#define MOSFET_ETH_MAIN    7  // P07
#define MOSFET_ETH_PRESS   6  // P06
#define MOSFET_VENT_ETH    5  // P05
#define MOSFET_EXTRA       4  // CAN USE THIS PIN FOR ANYTHING JUST CHANGE ASSIGNMENT AND HARNESS
#define MOSFET_QD_LOX      3  // P03
#define MOSFET_IGNITER     8  // P10
#define MOSFET_LOX_MAIN    9  // P11
#define MOSFET_LOX_PRESS  10  // P12
#define MOSFET_VENT_LOX   11  // P13
#define MOSFET_QD_ETH     12  // P14

void loop(){
  if (Serial.available() > 0) {
  // Serial.parseInt return 0 for non-numeric characters.
    int num = Serial.parseInt();
    if (num > 0) {
      mosfetCloseAllValves();
      Serial.println(num);
      mosfetOpenValve(num);
    }
  }
}

void mosfetCloseAllValves(){
  for(int i=0; i < 16; i++) {
    pcf8575.digitalWrite(i, LOW);
  }
}

void mosfetCloseValve(int num){
  pcf8575.digitalWrite(num, LOW);
}

void mosfetOpenValve(int num){
  pcf8575.digitalWrite(num, HIGH);
}