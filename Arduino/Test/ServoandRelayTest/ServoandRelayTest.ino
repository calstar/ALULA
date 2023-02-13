// Identical to DAQBoard code, except meant for flowmeter calibration

/*
This code runs on the DAQ ESP32 and has a couple of main functions.
1. Read sensor data
2. Send sensor data to COM ESP32
3. Recieve servo commands from COM ESP32
4. Send PWM signals to servos
*/

#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Arduino.h>
#include "HX711.h"

//define pins to use for the various sensors and connections. define takes up less space on the chip
#define ONBOARD_LED  13

#define CLK 19
#define CLK2 25
#define FM 4

#define S1S 26
#define S2S 25

//define servo min and max values
#define SERVO_MIN_USEC (900)
#define SERVO_MAX_USEC (2100)


int servo1_curr = 0;
int servo2_curr = 0;
int S1=0;
const int RELAYPIN = 27; 
const int RELAYPIN2 = 14; 




//Initialize the servo objects
Servo servo1;
Servo servo2;


//define servo necessary values
int ADC_Max = 4096;



int count=3;

float endTime;
float timeDiff;




void setup() {
   pinMode(RELAYPIN, OUTPUT);
   pinMode(RELAYPIN2, OUTPUT);

// This statement will declare pin 22 as digital output 
digitalWrite(RELAYPIN, HIGH); 
digitalWrite(RELAYPIN2, HIGH); 
  
  //attach servo pins
  servo1.attach(S1S,SERVO_MIN_USEC,SERVO_MAX_USEC);
  servo2.attach(S2S,SERVO_MIN_USEC,SERVO_MAX_USEC);
  
  // attach onboard LED
  pinMode(ONBOARD_LED,OUTPUT);


 Serial.begin(115200);

  // Set device as a Wi-Fi Station

}

void loop() {


  servo1.write(90);
  servo2.write(90);
  delay(50);

  digitalWrite(ONBOARD_LED, HIGH); 
  digitalWrite(RELAYPIN, HIGH); 
  delay(200);
  digitalWrite(RELAYPIN2, HIGH); 
Serial.print("loop");

  delay(2000);
  
  servo1.write(0);
  servo2.write(0);
  delay(50);
  digitalWrite(ONBOARD_LED, LOW); 
  digitalWrite(RELAYPIN, LOW); 

  delay(50);
  digitalWrite(RELAYPIN2, LOW); 
Serial.print("loop");
  delay(2000);

}
