/*
This code runs on the COM ESP32 and has a couple of main tasks.
1. Receive sensor data from DAQ ESP32
2. Send servo commands to DAQ ESP32
*/

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Arduino.h>
#include "HX711.h"
#include <ezButton.h>
#include "avdweb_Switch.h"
#include "freertos/FreeRTOS.h"
#include "freertos/Task.h"

//IF YOU WANT TO DEBUG, SET THIS TO 1. IF NOT SET ZERO
int DEBUG = 0;

Switch SWITCH_ARMED = Switch(14);  //correct
Switch SWITCH_PRESS = Switch(12);  //correct
Switch SWITCH_QD = Switch(26);   //  correct
Switch SWITCH_IGNITION = Switch(21);
Switch SWITCH_HOTFIRE = Switch(23);
Switch SWITCH_ABORT = Switch(18);
#define LED_ARMED 13   //correct
#define LED_PRESSETH 25 //correct
#define LED_PRESSLOX 33 //corect
#define LED_PRESS 32 //correct
#define LED_QD 27 //correct
#define LED_IGNITION 19 //
#define LED_HOTFIRE 22  //correct
#define LED_ABORT 5

float pressTime = 0;

 String success;
String message;
 int COMState;
int incomingByte = 0;
int incomingMessageTime;
 float incomingPT1 = 4; //PT errors when initialized to zero
float incomingPT2 = 4;
float incomingPT3 = 4;
float incomingPT4 = 4;
 float incomingPT5 = 4;
 float incomingLC1 = 4;
 float incomingLC2 = 4;
 float incomingLC3 = 4;
 float incomingTC1 = 4;
 float incomingTC2 = 4;
 float incomingTC3 = 4;
 float incomingTC4 = 4;
 float incomingCap1 = 0;
 float incomingCap2 = 0;
 bool pressComplete = false;
 bool ethComplete = false;
 bool oxComplete = false;
 short int queueSize = 0;

esp_now_peer_info_t peerInfo;

//TIMING VARIABLES
 int state;
int serialState;
 int manualState;
 int DAQState;
 int loopStartTime;
int sendDelay = 50;   //Measured in ms
enum STATES {IDLE, ARMED, PRESS, QD, IGNITION, HOTFIRE, ABORT};
//#define DEBUG_IDLE 90
//#define DEBUG_ARMED 91
//#define DEBUG_PRESS 92
//#define DEBUG_QD 93
//#define DEBUG_IGNITION 94
//#define DEBUG_HOTFIRE 95
//#define DEBUG_ABORT 96
double lastPrintTime = 0;
int debug_state = 0;

float currTime = 0;
float loopTime = 0;

float sendTime = 0;
float receiveTime = 0;

//ENSURE IP ADDRESS IS CORRECT FOR DEVICE IN USE!!!
//DAQ Breadboard {0x24, 0x62, 0xAB, 0xD2, 0x85, 0xDC}
//DAQ Protoboard {0x0C, 0xDC, 0x7E, 0xCB, 0x05, 0xC4}
//NON BUSTED DAQ {0x7C, 0x9E, 0xBD, 0xD8, 0xFC, 0x14}
// {0xC4, 0xDD, 0x57, 0x9E, 0x96, 0x34};
// uint8_t broadcastAddress[] = {0xC8, 0xF0, 0x9E, 0x50, 0x23, 0x34}; //Core board 1
uint8_t broadcastAddress[] = {0xB0, 0xA7, 0x32, 0xDE, 0xD3, 0x1C}; //Core board 2
//uint8_t broadcastAddress[] = {0x08, 0x3A, 0xF2, 0xB7, 0xEE, 0x00}; //TEST
//{0x30, 0xC6, 0xF7, 0x2A, 0x28, 0x04}

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
     int messageTime;
     float PT_O1;
     float PT_O2;
     float PT_E1;
     float PT_E2;
     float PT_C1;
     float LC_1;
     float LC_2;
     float LC_3;
     float TC_1;
     float TC_2;
     float TC_3;
     float TC_4;
     int COMState;
     int DAQState;
     short int queueLength;
    //  bool pressComplete;
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
  Serial.println("Start of Setup");

  // setup LEDs and set to LOW
  pinMode(LED_ARMED, OUTPUT);
  pinMode(LED_PRESSETH, OUTPUT);
  pinMode(LED_PRESSLOX, OUTPUT);
  pinMode(LED_PRESS, OUTPUT);
  pinMode(LED_QD, OUTPUT);
  pinMode(LED_IGNITION, OUTPUT);
  pinMode(LED_HOTFIRE, OUTPUT);
  pinMode(LED_ABORT, OUTPUT);

  digitalWrite(LED_ARMED, LOW);
  digitalWrite(LED_PRESSETH, LOW);
  digitalWrite(LED_PRESSLOX, LOW);
  digitalWrite(LED_PRESS, LOW);
  digitalWrite(LED_QD, LOW);
  digitalWrite(LED_IGNITION, LOW);
  digitalWrite(LED_HOTFIRE, LOW);
  digitalWrite(LED_ABORT, LOW);


//
//  while(SWITCH_ABORT.on()){digitalWrite(LED_ABORT, HIGH);}
  Serial.println(WiFi.macAddress());
  //set device as WiFi station
  WiFi.mode(WIFI_STA);

  //initialize ESP32
   if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }


  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

  // Used for debugging
  // Serial.println(WiFi.macAddress());
  state = IDLE;
  serialState = IDLE;
}


void loop(){
    loopStartTime=millis();
    if (Serial.available() > 0) {
    // read the incoming byte:
    state = (int)Serial.parseInt();
  }
  // Serial.print("I received: ");
  SWITCH_ARMED.poll();
  SWITCH_PRESS.poll();
  SWITCH_QD.poll();
  SWITCH_IGNITION.poll();
  SWITCH_HOTFIRE.poll();
  SWITCH_ABORT.poll();
  if (DEBUG ==1) {
  Serial.print("COM State: ");
  Serial.print(state);
  Serial.print(";      DAQ State: ");
  Serial.println(DAQState);
  if(!SWITCH_PRESS.on() && !SWITCH_ARMED.on() && !SWITCH_ABORT.on() && !SWITCH_QD.on() && !SWITCH_IGNITION.on() && !SWITCH_HOTFIRE.on()) {serialState=IDLE;}
  }

  switch (state) {
//
// if (DEBUG ==1) {
//  Serial.println(state);
// }

  case (IDLE): //Includes polling
    idle();
    if (SWITCH_ABORT.on()) {serialState=ABORT;}
    if (SWITCH_ARMED.on()) {serialState=ARMED;}
    state = serialState;
    break;

  case (ARMED):
    dataSendCheck();
    if (DAQState == ARMED) {digitalWrite(LED_ARMED, HIGH);}
    if (SWITCH_ABORT.on()) {serialState=ABORT;}
    if (SWITCH_PRESS.on()) {serialState=PRESS;}
    if(!SWITCH_ARMED.on()) {serialState=IDLE;}

    state = serialState;
    break;

  case (PRESS):
   // press();
    if (SWITCH_ABORT.on()) {serialState=ABORT;}
    if (DAQState == PRESS) {digitalWrite(LED_PRESS, HIGH);}
    if (ethComplete) {digitalWrite(LED_PRESSETH, HIGH);}
    if (oxComplete) {digitalWrite(LED_PRESSLOX, HIGH);}
    if (!ethComplete) {digitalWrite(LED_PRESSETH, LOW);}
    if (!oxComplete) {digitalWrite(LED_PRESSLOX, LOW);}
    if (SWITCH_QD.on()) {serialState=QD;}  //add pressComplete && later
    //add return to idle functionality hyer
    dataSendCheck();
    state = serialState;
    if(!SWITCH_PRESS.on() && !SWITCH_ARMED.on()) {serialState=IDLE;}
    break;

  case (QD):
    //quick_disconnect();
    if (SWITCH_ABORT.on()) {serialState=ABORT;}
    if (DAQState == QD) {digitalWrite(LED_QD, HIGH);}
    if (SWITCH_IGNITION.on()) {serialState=IGNITION;}
    state = serialState;
    if(!SWITCH_QD.on() && !SWITCH_PRESS.on() && !SWITCH_ARMED.on()) {serialState=IDLE;}
    dataSendCheck();
    break;


  case (IGNITION):
    //ignition();
    if (SWITCH_ABORT.on()) {serialState=ABORT;}
    if (DAQState == IGNITION) {digitalWrite(LED_IGNITION, HIGH);}
    if (SWITCH_HOTFIRE.on()) {serialState=HOTFIRE;}
    if(!SWITCH_QD.on() && !SWITCH_PRESS.on() && !SWITCH_ARMED.on() && !SWITCH_IGNITION.on()) {serialState=IDLE;}
    dataSendCheck();
    state = serialState;
    break;

  case (HOTFIRE):
    // hotfire();

    if (SWITCH_ABORT.on()) {serialState=ABORT;}
    if (DAQState == HOTFIRE) {digitalWrite(LED_HOTFIRE, HIGH);}
    dataSendCheck();

    state = serialState;
    break;

  case (ABORT):
    if (DAQState == ABORT) {digitalWrite(LED_ABORT, HIGH);}
    digitalWrite(LED_ABORT, HIGH);
    digitalWrite(LED_IGNITION,LOW);
    digitalWrite(LED_QD, LOW);
    digitalWrite(LED_ARMED, LOW);
    digitalWrite(LED_PRESS, LOW);
    digitalWrite(LED_PRESSETH, LOW);
    digitalWrite(LED_PRESSLOX, LOW);
    digitalWrite(LED_HOTFIRE, LOW);
    state = ABORT;
    dataSendCheck();
    if(!SWITCH_QD.on() && !SWITCH_PRESS.on() && !SWITCH_ARMED.on() && !SWITCH_IGNITION.on() && !SWITCH_HOTFIRE.on() && !SWITCH_ABORT.on()) {serialState=IDLE;}
    state = serialState;
    break;

//  case (DEBUG):
//   // debug();
//    serialState=IDLE;
//    state = serialState;
//    debug();
//    break;
  }

  }

 void turnoffLEDs() {
    digitalWrite(LED_ARMED, LOW);
    digitalWrite(LED_PRESS, LOW);
    digitalWrite(LED_PRESSETH, LOW);
    digitalWrite(LED_PRESSLOX, LOW);
    digitalWrite(LED_QD, LOW);
    digitalWrite(LED_IGNITION, LOW);
    digitalWrite(LED_HOTFIRE, LOW);
    digitalWrite(LED_ABORT, LOW);
 }

void idle() {
  dataSendCheck();
  turnoffLEDs();
}



void dataSendCheck() {
    dataSend();
}

void dataSend() {
  // Set values to send
  Commands.COMState = state;
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &Commands, sizeof(Commands));
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  //Serial.print("Bytes received: ");
//  Serial.println(len);
  DAQState = incomingReadings.DAQState;
  incomingPT1 = incomingReadings.PT_O1; //LOX Tank PT
  incomingPT2 = incomingReadings.PT_O2; //LOX Injector PT
  incomingPT3 = incomingReadings.PT_E1; //ETH Tank PT
  incomingPT4 = incomingReadings.PT_E2; //ETH Injector PT
  incomingPT5 = incomingReadings.PT_C1; //COMBUSTION CHAMBER PT
  incomingLC1 = incomingReadings.LC_1;
  incomingLC2 = incomingReadings.LC_2;
  incomingLC3 = incomingReadings.LC_3;
  incomingTC1 = incomingReadings.TC_1; //Phenolic-Interface Thermocouple
  incomingTC2 = incomingReadings.TC_2;
  incomingTC3 = incomingReadings.TC_3;
  incomingTC4 = incomingReadings.TC_4;

  // pressComplete = incomingReadings.pressComplete;
  oxComplete = incomingReadings.oxComplete;
  ethComplete = incomingReadings.ethComplete;
  queueSize = incomingReadings.queueLength;
  receiveDataPrint();
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("\r\nLast Packet Send Status:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
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
  message.concat(incomingTC1);
  message.concat(" ");
  message.concat(incomingTC2);
  message.concat(" ");
  message.concat(incomingTC3);
  message.concat(" ");
  message.concat(incomingTC4);
  message.concat(" ");
  // message.concat(incomingCap1);
  // message.concat(" ");
  // message.concat(incomingCap2);
  // message.concat(" ");
  message.concat(Commands.COMState);
  message.concat(" ");
  message.concat(DAQState);
  message.concat(" ");
  message.concat(queueSize);

  Serial.println(message);
}
