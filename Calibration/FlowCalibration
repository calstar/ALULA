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

#define CLK 25
#define FM 4

#define S1S 21

//define servo min and max values
#define SERVO_MIN_USEC (900)
#define SERVO_MAX_USEC (2100)

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
Servo servo2;

//define servo necessary values
int ADC_Max = 4096;

///////////////
//IMPORTANT
//////////////
// REPLACE WITH THE MAC Address of your receiver
uint8_t broadcastAddress[] = {0xC4, 0xDD, 0x57, 0x9E, 0x91, 0x6C};

int count=3;

// Define variables to store readings to be sent
float pt1=1;
float pt2=1;
float pt3=1;
float pt4=1;
float pt5=1;
float lc1=1;
float lc2=1;
float lc3=1;
float fm=2;
//the following are only used in the oposite diretcion, they are included because it may be necessary for the structure to be the same in both directions
int S1; int S2; int S1S2; int I;

// Define variables to store incoming commands, servos and igniter
int incomingS1;
int incomingS2;
int incomingS1S2;
bool incomingI;

float endTime;
float timeDiff;

// Variable to store if sending data was successful
String success;

//Structure example to send data
//Must match the receiver structure
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
    bool I;
} struct_message;

// Create a struct_message called Readings to hold sensor readings
struct_message Readings;

// Create a struct_message to hold incoming commands
struct_message Commands;

void calibrationISR() {
  currentTime = millis();
  float difference = currentTime - programStart;

  float pt1_now = scale1.read();
  float pt2_now = scale2.read();
  Serial.println("Pulse time: " + str(difference) + " PT1: " + str(pt1_now) + " PT2: " + str(pt2_now));
}

void setup() {
  //attach servo pins
  servo1.attach(S1S,SERVO_MIN_USEC,SERVO_MAX_USEC );

  // attach onboard LED
  pinMode(ONBOARD_LED,OUTPUT);

  attachInterrupt(digitalPinToInterrupt(FM), calibrationISR, CHANGE);
  boolean attached = true;

//attach flowmeter pin
  //pinMode(FM, INPUT_PULLUP);

//set gains for pt pins
  scale1.begin(PT1DOUT, CLK);
  scale1.set_gain(64);
  scale2.begin(PT2DOUT, CLK);
  scale2.set_gain(64);
//Flowmeter untreupt
 pinMode(FM, INPUT);           //Sets the pin as an input
 attachInterrupt(FM, Flow, RISING);

 Serial.begin(115200);

  // Set device as a Wi-Fi Station

  programStart = millis();
}

void loop() {
  //Set LED back to low
  digitalWrite(ONBOARD_LED,LOW);

  if ((millis() - programStart) >= calibrationTime && attached) {
    detachInterrupt(digitalPinToInterrupt(FM));
    attached = false;
  }
   //ADD PRINT STATEMENTS FOR DEBUGGING HERE IF NCESSARY
 // printSerial();

//UPDATE SERVO POSITIONS
  //Check new data for servo status updates
  //switch (S1) {
    //case 0:
        //servo1.write(0);
        //break;
     //case 45:
        //servo1.write(45);
        //break;
    //case 90:
        //servo1.write(90);
        //break;
    //case 130:
        //servo1.write(135);
        //break;
  //}
  servo1.write(S1);

  getReadings();

  // Set values to send
  Readings.pt1 = pt1;
  Readings.pt2 = pt2;
  Readings.pt3 = pt3;
  Readings.pt4 = pt4;
  Readings.pt5 = pt5;
  Readings.lc1 = lc1;
  Readings.lc2 = lc2;
  Readings.lc3 = lc3;
  Readings.fm  = fm;

  endTime=millis();
  timeDiff=endTime-programStart;
//  if (timeDiff<loopTime) {
//    delay(timeDiff);
//  }
}

void getReadings(){
  currentMillis = millis();
  fmcount = 0;
  while (millis() - currentMillis < goalTime) {
    currentState = digitalRead(FM);
    if (!(currentState == lastState)) {
      lastState = currentState;
      fmcount += 1;
    }
  }
  flowRate = fmcount * 1000 / goalTime;
  fm =int(flowRate*10000+1);  // Print the integer part of the variable

  pt1 = scale1.read();
  pt2 = scale2.read();
}
