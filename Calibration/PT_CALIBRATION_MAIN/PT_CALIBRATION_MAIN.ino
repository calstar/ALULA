/*
This code runs on the DAQ ESP32 and has a couple of main functions.
1. Read sensor data
2. Send sensor data to COM ESP32
3. Recieve servo commands from COM ESP32
4. Send PWM signals to servos
*/

#include <esp_now.h>
#include <WiFi.h>
// #include <ESP32Servo.h>
#include <Wire.h>
#include <Arduino.h>
#include "HX711.h"


//define pins to use for the various sensors and connections. define takes up less space on the chip
#define ONBOARD_LED  13
#define PTDOUT1 4 //PT-O1
#define CLKPT1 17 
#define PTDOUT2 16 //PT-O2
#define CLKPT2 17 
#define PTDOUT3 6 //PT-E1
#define CLKPT3 17
#define PTDOUT4 7 //PT-E2
#define CLKPT4 17
#define PTDOUT5 15 //PT-C1
#define CLKPT5 17
#define PTDOUT6 5 //LC1
#define CLKPT6 17
// #define PTDOUT7 25
// #define CLKPT7 27
// #define PTDOUT8 26
// #define CLKPT8 27



float pt1=-1;
float pt2=-1;
float pt3=-1;
float pt4=-1;
float pt5=-1;
float pt6=-1;
float pt7=-1;
float pt8=-1;
// String serialMessage = "";
String serialMessage = "";

//define servo min and max values
// #define SERVO_MIN_USEC (900)
// #define SERVO_MAX_USEC (2100)

//Initialize the PT and LC sensor objects which use the HX711 breakout board
HX711 scale1;
HX711 scale2;
HX711 scale3;
HX711 scale4;
HX711 scale5;
HX711 scale6;
//Initialize the servo objects
// Servo servo1;
// Servo servo2;

//define servo necessary values
int ADC_Max = 4096;


void setup() {
  
 
  //set gains for pt pins
  scale1.begin(PTDOUT1, CLKPT1); //LOX tank
  scale1.set_gain(64);
  //Sets the pin as an input

  // set gains for pt pins
  scale2.begin(PTDOUT2, CLKPT2);
  scale2.set_gain(64);

  //set gains for pt pins
  scale3.begin(PTDOUT3, CLKPT3); //ethanol tank
  scale3.set_gain(64);

  //set gains for pt pins
  scale4.begin(PTDOUT4, CLKPT4); //ethanol injector
  scale4.set_gain(64);

  //set gains for pt pins
  scale5.begin(PTDOUT5, CLKPT5); //chamber
  scale5.set_gain(64);


  //set gains for pt pins
  scale6.begin(PTDOUT6, CLKPT6); //chamber
  scale6.set_gain(64);

  // //set gains for pt pins
  // scale7.begin(PTDOUT7, CLKPT7); //chamber
  // scale7.set_gain(128);

  // //set gains for pt pins
  // scale8.begin(PTDOUT8, CLKPT8); //chamber
  // scale8.set_gain(128);
  Serial.begin(115200);
  Serial.println("SETUP");
  
}

void loop() {


  pt1 = scale1.read();
  Serial.print(pt1);
  Serial.print(" ");
  pt2 = scale2.read();
  Serial.print(pt2);
  Serial.print(" ");
  pt3 = scale3.read();
  Serial.print(pt3);
  Serial.print(" ");
  pt4 = scale4.read();
  Serial.print(pt4);
  Serial.print(" ");
  pt5 = scale5.read();
  Serial.print(pt5);
  Serial.print(" ");
  pt6 = scale6.read();
  Serial.println(pt6);


  delay(100);

}

