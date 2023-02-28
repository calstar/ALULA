/*
This code runs on the COM ESP32 and has a couple of main tasks.
1. Receive sensor data from DAQ ESP32
2. Send servo commands to DAQ ESP32
*/

#include <Wire.h>
#include <Arduino.h>
#include "HX711.h"
#include <ezButton.h>

// Set pinouts. Currently set arbitrarily
ezButton SWITCH_IDLE(14);
ezButton SWITCH_ARMED(34);
ezButton SWITCH_PRESS(26);
ezButton SWITCH_QD(21);
ezButton SWITCH_IGNITION(23);
ezButton SWITCH_HOTFIRE(18); 
ezButton SWITCH_ABORT(17);

//#define LED_IDLE 13
#define LED_ARMED 13
#define LED_PRESSETH 35
#define LED_PRESSLOX 33
#define LED_PRESS 25
#define LED_QD 27
#define LED_IGNITION 19
#define LED_HOTFIRE 22
#define LED_ABORT 5

float pressTime = 0;

String success;
String message;
int commandedState;

int incomingMessageTime;
int incomingPT1 = 4; //PT errors when initialized to zero
int incomingPT2 = 4;
int incomingPT3 = 4;
int incomingPT4 = 4;
int incomingPT5 = 4;
int incomingLC1 = 4;
int incomingLC2 = 4;
int incomingLC3 = 4;
int incomingTC1 = 4;
int incomingTC2 = 4;
float incomingCap1 = 0;
float incomingCap2 = 0;
bool pressComplete = false;
bool ethComplete = false;
bool oxComplete = false;
short int queueSize = 0;


//TIMING VARIABLES
int state;
int serialState;
int manualState;
int DAQState;
int loopStartTime;
int sendDelay = 50;   //Measured in ms
enum STATES {IDLE, ARMED, PRESS, QD, IGNITION, HOTFIRE, ABORT, DEBUG=99};

double lastPrintTime = 0;

float currTime = 0;
float loopTime = 0;

float sendTime = 0;
float receiveTime = 0;

//ENSURE IP ADDRESS IS CORRECT FOR DEVICE IN USE!!!
//DAQ Breadboard {0x24, 0x62, 0xAB, 0xD2, 0x85, 0xDC}
//DAQ Protoboard {0x0C, 0xDC, 0x7E, 0xCB, 0x05, 0xC4}
//NON BUSTED DAQ {0x7C, 0x9E, 0xBD, 0xD8, 0xFC, 0x14}
// uint8_t broadcastAddress[] = {0x7C, 0x9E, 0xBD, 0xD8, 0xFC, 0x14}; //change to new Mac Address
// {0xC4, 0xDD, 0x57, 0x9E, 0x96, 0x34};
uint8_t broadcastAddress[] = {0x30, 0xC6, 0xF7, 0x2A, 0x28, 0x04};
//{0x30, 0xC6, 0xF7, 0x2A, 0x28, 0x04}

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    int messageTime;
    int pt1;
    int pt2;
    int pt3;
    int pt4;
    int pt5;
    int lc1;
    int lc2;
    int lc3;
    int tc1;
    int tc2;
    float cap1;
    float cap2;
    int commandedState;
    int DAQState;
    short int queueSize;
    bool pressComplete;
    bool ethComplete;
    bool oxComplete;
} struct_message; 

// Create a struct_message called Readings to recieve sensor readings remotely
struct_message incomingReadings;

// Create a struct_message to send commands
struct_message Commands;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
   // Serial.println("Start of Setup");

  // Use switches for moving between states
  SWITCH_IDLE.setDebounceTime(50);
  SWITCH_ARMED.setDebounceTime(50);
  SWITCH_PRESS.setDebounceTime(50);
  SWITCH_QD.setDebounceTime(50);
  SWITCH_IGNITION.setDebounceTime(50);
  SWITCH_HOTFIRE.setDebounceTime(50);
  SWITCH_ABORT.setDebounceTime(50);

  // setup LEDs and set to LOW
  //pinMode(LED_IDLE, OUTPUT);
  pinMode(LED_ARMED, OUTPUT);
  pinMode(LED_PRESSETH, OUTPUT);
  pinMode(LED_PRESSLOX, OUTPUT);
  pinMode(LED_PRESS, OUTPUT);
  pinMode(LED_QD, OUTPUT);
  pinMode(LED_IGNITION, OUTPUT);
  pinMode(LED_HOTFIRE, OUTPUT);
  pinMode(LED_ABORT, OUTPUT);

  //digitalWrite(LED_IDLE, LOW);
  digitalWrite(LED_ARMED, LOW);
  digitalWrite(LED_PRESSETH, LOW);
  digitalWrite(LED_PRESSLOX, LOW);
  digitalWrite(LED_PRESS, LOW);
  digitalWrite(LED_QD, LOW);
  digitalWrite(LED_IGNITION, LOW);
  digitalWrite(LED_HOTFIRE, LOW);
  digitalWrite(LED_ABORT, LOW);

  // Used for debugging
  // Serial.println(WiFi.macAddress());
  state = IDLE;
  serialState = IDLE;
}

void loop() {
  Serial.println(serialState);

  loopStartTime=millis();
  SerialRead();
  SWITCH_IDLE.loop();
  SWITCH_ARMED.loop();
  SWITCH_PRESS.loop();
  SWITCH_QD.loop();
  SWITCH_IGNITION.loop();
  SWITCH_HOTFIRE.loop();
  SWITCH_ABORT.loop();

  switch (state) {

  case (IDLE): //Includes polling
    idle();
    if (SWITCH_ABORT.getState()) {serialState=ABORT;}
    if (SWITCH_ARMED.getState()) {serialState=ARMED;}
    state = serialState;
    break;

  case (ARMED):
    armed();
    if (DAQState == ARMED) {digitalWrite(LED_ARMED, HIGH);}
    if (SWITCH_ABORT.getState()) {serialState=ABORT;}
    if (SWITCH_PRESS.getState()) {serialState=PRESS;}
    if(!SWITCH_ARMED.getState()) {serialState=IDLE;}
   
    state = serialState;
    break;

  case (PRESS): 
    press();
    if (SWITCH_ABORT.getState()) {serialState=ABORT;}
    if (DAQState == PRESS) {digitalWrite(LED_PRESS, HIGH);}
    if (ethComplete) {digitalWrite(LED_PRESSETH, HIGH);}
    if (oxComplete) {digitalWrite(LED_PRESSLOX, HIGH);}
    if (pressComplete && SWITCH_QD.getState()) {serialState=QD;}
    if(!SWITCH_PRESS.getState() && !SWITCH_ARMED.getState()) {serialState=IDLE;}
    

  case (QD):
    quick_disconnect();
    if (SWITCH_ABORT.getState()) {serialState=ABORT;}
    if (DAQState == QD) {digitalWrite(LED_QD, HIGH);}
    if (SWITCH_IGNITION.getState()) {serialState=IGNITION;}
    state = serialState;
    if(!SWITCH_QD.getState() && !SWITCH_PRESS.getState() && !SWITCH_ARMED.getState()) {serialState=IDLE;}
    break;

  case (IGNITION):
    ignition();
    if (SWITCH_ABORT.getState()) {serialState=ABORT;}
    if (DAQState == IGNITION) {digitalWrite(LED_IGNITION, HIGH);}
    if (SWITCH_HOTFIRE.getState()) {serialState=HOTFIRE;}
    state = serialState;
    if(!SWITCH_QD.getState() && !SWITCH_PRESS.getState() && !SWITCH_ARMED.getState() && !SWITCH_IGNITION.getState()) {serialState=IDLE;}
    break;

  case (HOTFIRE):
    hotfire();

    if (SWITCH_ABORT.getState()) {serialState=ABORT;}
    if (DAQState == HOTFIRE) {digitalWrite(LED_HOTFIRE, HIGH);}
    if(!SWITCH_QD.getState() && !SWITCH_PRESS.getState() && !SWITCH_ARMED.getState() && !SWITCH_IGNITION.getState() && !SWITCH_HOTFIRE.getState()) {serialState=IDLE;}
    
    state = serialState;
    break;
  
  case (ABORT):
    if (DAQState == ABORT) {digitalWrite(LED_ABORT, HIGH);} 
    abort_sequence();
    break;

  case (DEBUG):
    debug();
    if (SWITCH_IDLE.getState()) {serialState=IDLE;}
    state = serialState;
    break;
  }
}

void idle() {
  commandedState = IDLE;
}

void armed() {
  commandedState = ARMED;
}

void press() {
  commandedState = PRESS;
}

void quick_disconnect() {
  commandedState = QD;
}

void ignition() {
  commandedState = IGNITION;
}

void hotfire() {
  commandedState = HOTFIRE;
}

void abort_sequence() {
  commandedState = ABORT;
}

void debug() {
  commandedState = DEBUG;
}


void receiveDataPrint() {
  message = "";
  message.concat(millis());
  message.concat(" ");
  message.concat(incomingPT1);
  message.concat(" ");
  message.concat(incomingPT2);
  message.concat(" ");
  message.concat(incomingPT3);
  message.concat(" ");
  message.concat(incomingPT4);
  message.concat(" ");
  message.concat(incomingPT5);
  message.concat(" ");
  message.concat(incomingLC1);
  message.concat(" ");
  message.concat(incomingLC2);
  message.concat(" ");
  message.concat(incomingLC3);
  message.concat(" ");
  // message.concat(incomingCap1);
  // message.concat(" ");
  // message.concat(incomingCap2);
  // message.concat(" ");
  message.concat(Commands.commandedState);
  message.concat(" ");
  message.concat(queueSize);

  Serial.println(message);
}

void SerialRead() {
  if (Serial.available() > 0) {
    manualState = Serial.read() - 48;
    if (manualState == 0) {
      serialState = IDLE;
    } else if (manualState == 1) {
      serialState = ARMED;
    } else if (manualState == 2) {
      serialState = PRESS;
    } else if (manualState == 3) {
      serialState = QD;
    } else if (manualState == 4) {
      serialState = IGNITION;
    } else if (manualState == 5) {
      serialState = HOTFIRE;
    } else if (manualState == 6) {
      serialState = ABORT;
    } else if (manualState = 99) {
      serialState = DEBUG;
    }
  }
}