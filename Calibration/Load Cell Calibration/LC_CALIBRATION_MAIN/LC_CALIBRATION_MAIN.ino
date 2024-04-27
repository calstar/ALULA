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
//#define ONBOARD_LED  13
//#define PTDOUT1 36 //PT-O1
//#define CLKPT1 27 
//#define PTDOUT2 39 //PT-O2
//#define CLKPT2 27 
//#define PTDOUT3 34 //PT-E1
//#define CLKPT3 27
//#define PTDOUT4 35 //PT-E2
//#define CLKPT4 27
#define PTDOUT5 32 //PT-C1
#define CLKPT5 27
#define PTDOUT6 33 //LC1
#define CLKPT6 27
#define PTDOUT7 25 //
#define CLKPT7 27
#define PTDOUT8 26 //
#define CLKPT8 27





float pt1=-1;
float pt2=-1;
float pt3=-1;
float pt4=-1;
float pt5=-1;
float pt6=-1;
float pt7=-1;
float pt8=-1;
// String serialMessage = "";
//float LC_ALL=1.0;
String serialMessage = "";

 
//define servo min and max values
// #define SERVO_MIN_USEC (900)
// #define SERVO_MAX_USEC (2100)

//Initialize the PT and LC sensor objects which use the HX711 breakout board
//HX711 scale1;
//HX711 scale2;
//HX711 scale3;
//HX711 scale4;
//HX711 scale5;
HX711 scale6;
HX711 scale7;
HX711 scale8;
//Initialize the servo objects
// Servo servo1;
// Servo servo2;

//define servo necessary values
int ADC_Max = 4096;


void setup() {


  //set gains for pt pins
  scale6.begin(PTDOUT6, CLKPT6); //chamber
  scale6.set_gain(128);

  //set gains for pt pins
  scale7.begin(PTDOUT7, CLKPT7); //chamber
  scale7.set_gain(128);

  //set gains for pt pins
  scale8.begin(PTDOUT8, CLKPT8); //chamber
  scale8.set_gain(128);
  Serial.begin(115200);
}

void loop() {

  getReadings();

  delay(2);

}

void getReadings(){

//
// pt1 = scale1.read();
//       // Serial.print("pt1: ");
//
//       // Serial.print(" p2: ");
//
//  pt2 = scale2.read();
//
//  pt3 = scale3.read();
//       // Serial.print(" pt3: ");
//
//  pt4 = scale4.read();
//       // Serial.print(" pt4: ");
//
//  pt5 = scale5.read();
//       // Serial.print(" pt5: ");

   pt6 = scale6.read();
        // Serial.print(" pt6: ");
  
   pt7 = scale7.read();
//        Serial.print(" pt7: ");

   pt8 = scale8.read();
//      Serial.print(" pt8: ");      
  
//   LC_ALL = pt6 + pt7 + pt8;


  serialMessage = "";
//  serialMessage.concat(pt6+pt7+pt8);
//  serialMessage.concat(" ");
//  serialMessage.concat(pt2);
//  serialMessage.concat(" ");
//  serialMessage.concat(pt3);
//  serialMessage.concat(" ");
//  serialMessage.concat(pt4);
//  serialMessage.concat(" ");
//  serialMessage.concat(pt5);
  serialMessage.concat(" ");
  serialMessage.concat(pt6);
  serialMessage.concat(" ");
  serialMessage.concat(pt7);
  serialMessage.concat(" ");
  serialMessage.concat(pt8);
//  serialMessage.concat(" ");
//  serialMessage.concat(LC_ALL);

  Serial.println(serialMessage);


}
