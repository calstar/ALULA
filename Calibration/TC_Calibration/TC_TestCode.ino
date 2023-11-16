#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include "Adafruit_MAX31855.h"

//DEFINE THERMOCOUPLE PINS
#define TC_DO 13
#define TC_CLK 14
#define TC1_CS 17
#define TC2_CS 16
#define TC3_CS 4
#define TC4_CS 15
#define TC4_DO 23
#define TC4_CLK 18

// Initialize thermocouples.
// Set 2nd parameter to TC1_CS or TC2_CS
Adafruit_MAX31855 thermocouple1(TC_CLK, TC1_CS, TC_DO);
Adafruit_MAX31855 thermocouple2(TC_CLK, TC2_CS, TC_DO);
Adafruit_MAX31855 thermocouple3(TC_CLK, TC3_CS, TC_DO);
Adafruit_MAX31855 thermocouple4(TC4_CLK, TC4_CS, TC4_DO);

double c;
double c2;
double c3;
double c4;

void setup() {
  Serial.begin(112500);
  Serial.println("MAX31855 test");
  // wait for MAX chip to stabilize
  delay(500);
  if (!thermocouple1.begin()) {
    Serial.println("ERROR.");
    while (1) delay(10);
  }
  Serial.println("DONE.");
}

void loop() {
  // basic readout test, just print the current temp
//   Serial.print("TC1= ");
//   Serial.println(thermocouple1.readInternal());

   c = thermocouple1.readCelsius();
   Serial.print("TC1= ");
   Serial.print(c);
   Serial.print("   ");

   c2 = thermocouple2.readCelsius();
   Serial.print("TC2= ");
   Serial.print(c2);
   Serial.print("   ");

   c3 = thermocouple3.readCelsius();
   Serial.print("TC3= ");
   Serial.print(c3);
   Serial.print("   ");

   c4 = thermocouple4.readCelsius();
   Serial.print("TC4= ");
   Serial.print(c4);
   Serial.println("   ");
   
   delay(500);
}
