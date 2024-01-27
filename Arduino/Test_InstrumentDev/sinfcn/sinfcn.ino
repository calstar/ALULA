#include <esp_now.h>
#include <WiFi.h>

//setup instrument inputs and arduino
#include <Wire.h>
#include <Arduino.h>
#include "HX711.h"


float t;
float out;
float counter;

void setup() {
  // put your setup code here, to run once:
counter = 0;
//setup comms
Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
out = sin(counter);
counter = counter +.1 ;
Serial.println(out);
delay(100);
}
