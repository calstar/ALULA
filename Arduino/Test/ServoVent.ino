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
#define ONBOARD_LED  12
#define PT1DOUT 26
#define PT2DOUT 16

#define CLK 19
#define CLK2 25
#define FM 4

#define S1S 23

//define servo min and max values
#define SERVO_MIN_USEC (900)
#define SERVO_MAX_USEC (2100)

//bitton stuff
const int buttonpin1 = 27;
bool pressed = false;
bool prevPressed = false;
bool valveOpened = false;
int incomingS1 = 0;
int angle = 180;
float pressTime = 0;
int servo1_curr = 0;
int servo2_curr = 0;
int S1=0;


//Initialize flow meter variables for how it computes the flow amount
float currentMillis = 0;
float currentTime;
float goalTime = 100;
float currReading1;
float currReading2;
float loopTime=100;
float programStart;
float calibrationTime = 10000;  //ms

//FM counter
float fmcount;
float flowRate;
boolean currentState;
boolean lastState = false;

//Initialize the PT and LC sensor objects which use the HX711 breakout board
HX711 scale1;
HX711 scale2;

//Initialize the servo objects
Servo servo1;

//define servo necessary values
int ADC_Max = 4096;



int count=3;

float endTime;
float timeDiff;


typedef struct struct_message {
    float pt1;
    float pt2;
    float pt3;
    float pt4;
    float pt5;
    float lc1;
    float lc2;
    float lc3;
    float fm;
    int S1;
    int S2;
    int S1S2;
    boolean I;
} struct_message;

// Create a struct_message called Readings to recieve sensor readings remotely
struct_message incomingReadings;

// Create a struct_message to send commands
struct_message Commands;




void setup() {
  //attach servo pins
  servo1.attach(S1S,SERVO_MIN_USEC,SERVO_MAX_USEC );

  // attach onboard LED
  pinMode(ONBOARD_LED,OUTPUT);

//set gains for pt pins
  scale1.begin(PT1DOUT, CLK);
  scale1.set_gain(64);
  scale2.begin(PT2DOUT, CLK2);
  scale2.set_gain(64);
//Flowmeter untreupt
 pinMode(FM, INPUT);           //Sets the pin as an input

 Serial.begin(115200);

  // Set device as a Wi-Fi Station

  programStart = millis();
}

void loop() {

  pressed = digitalRead(buttonpin1); //push button to send servo signals


if (valveOpened) {
  digitalWrite( ONBOARD_LED,HIGH);
} else {
  digitalWrite( ONBOARD_LED,LOW);
}

if (prevPressed && (millis() - pressTime > 5000)) {
  prevPressed = false;
  Commands.S1 = 180 - servo1_curr;
  servo1_curr = 180 - servo1_curr;
}

  if (pressed && !prevPressed) {
    Commands.S1 = 180 - servo1_curr;
    servo1_curr = 180 - servo1_curr;
    pressTime = millis();
    Serial.print("button");
    //remove the following line with code that detects the status of the valve
  //  valveOpened = !valveOpened;
  // ADDED
  prevPressed = pressed;
  }

  S1=Commands.S1;

  servo1.write(S1);


//  }
}

void getReadings(){
  currentMillis = millis();
  fmcount = 0;
  while (millis() - currentMillis < goalTime) {
    currentState = digitalRead(FM);
    if (!(currentState == lastState)) {
      lastState = currentState;
      Serial.print(1);
      Serial.print(" ");
      Serial.print(1);
      Serial.print(" ");
      Serial.print(1);        
      Serial.print(" ");
      Serial.println(millis());

      
    }
  }

      Serial.print(millis());
      Serial.print(" ");
      Serial.print(scale1.read());
      Serial.print(" ");
      Serial.print(scale2.read());        
      Serial.print(" ");
      Serial.println(1);
}
