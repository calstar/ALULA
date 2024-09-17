/*
This code runs on the DAQ ESP32 and has a couple of main functions.
1. Read sensor data
2. Send sensor data to COM ESP32
3. Receive servo commands from COM ESP32
4. Send PWM signals to servos
*/

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Arduino.h>

// Define pins for various sensors and connections
#define ONBOARD_LED  2
#define PTDOUT1 27 // PT-O1
//#define PTDOUT2 38 // PT-O2
//#define PTDOUT3 6 // PT-E1
//#define PTDOUT4 7 // PT-E2
//#define PTDOUT5 15 // PT-C1
//#define PTDOUT6 39 // LC1

// Define servo min and max values (commented out since servos are not currently used)
// #define SERVO_MIN_USEC (900)
// #define SERVO_MAX_USEC (2100)

// Initialize pressure sensor data variables
float pt1 = -1;
float pt2 = -1;
float pt3 = -1;
float pt4 = -1;
float pt5 = -1;
float pt6 = -1;

// String to store incoming serial messages
String serialMessage = "";

// Define servo necessary values (commented out as servos are not implemented)
// int ADC_Max = 4096;

void setup() {
  // Set up serial communication for debugging
  Serial.begin(115200);
  Serial.println("SETUP");
  
  // Set up sensor pins (as inputs, assuming they are being used to read some kind of data)
  pinMode(PTDOUT1, INPUT);
  //pinMode(PTDOUT2, INPUT);
  //pinMode(PTDOUT3, INPUT);
  //pinMode(PTDOUT4, INPUT);
  //pinMode(PTDOUT5, INPUT);
  //pinMode(PTDOUT6, INPUT);
  
  // Additional setup code for WiFi, ESP-NOW, etc. can be added here
}

void loop() {
  // Read sensor data from pins
  pt1 = analogRead(PTDOUT1); // Replace this with actual sensor read code as needed
  //pt2 = analogRead(PTDOUT2); // Replace this with actual sensor read code as needed
  //pt3 = analogRead(PTDOUT3); // Replace this with actual sensor read code as needed
  //pt4 = analogRead(PTDOUT4); // Replace this with actual sensor read code as needed
  //pt5 = analogRead(PTDOUT5); // Replace this with actual sensor read code as needed
  //pt6 = analogRead(PTDOUT6); // Replace this with actual sensor read code as needed
  
  // Print the sensor data for debugging
  Serial.print(pt1);
  Serial.println(" ");
  //Serial.print(pt2);
  //Serial.print(" ");
  //Serial.print(pt3);
  //Serial.print(" ");
  //Serial.print(pt4);
  //Serial.print(" ");
  //Serial.print(pt5);
  //Serial.print(" ");
  //Serial.println(pt6);

  delay(100);
}