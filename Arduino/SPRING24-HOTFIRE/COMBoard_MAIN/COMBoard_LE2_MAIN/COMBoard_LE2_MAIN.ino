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
#include "avdweb_Switch.h" //https://github.com/avandalen/avdweb_Switch
#include "freertos/FreeRTOS.h"
#include "freertos/Task.h"

//IF YOU WANT TO DEBUG, SET THIS TO 1. IF NOT SET ZERO
int DEBUG = 0;
bool SWITCHES = false;

#define COM_ID 1
#define DAQ_POWER_ID 2
#define DAQ_SENSE_ID 3

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
String serialMessage;
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
// uint8_t broadcastAddress[] = {0x08, 0x3A, 0xF2, 0xB7, 0x29, 0xBC}; //DAQ 1
//uint8_t broadcastAddress[] = {0xC8, 0xF0, 0x9E, 0x4F, 0x3C, 0xA4}; //Core board 2
//uint8_t broadcastAddress[] = {0x08, 0x3A, 0xF2, 0xB7, 0x29, 0xBC}; //TEST
uint8_t broadcastAddress[] = {0xe4, 0x65, 0xB8, 0x27, 0x62, 0x64}; //POWER

//{0x30, 0xC6, 0xF7, 0x2A, 0x28, 0x04}

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
     int id;
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

struct_message myData;
//struct_message for incoming SENSE Board Readings
struct_message SENSE;
// Create a struct_message for POWER DAQ data.
struct_message POWER;
// Create a struct_message to hold outgoing commands
struct_message Commands;
// Callback when data is received, should we add this to the daq_sense board?
esp_now_peer_info_t peerInfo;


// Callback when data is received, should we add this to the daq_sense board?
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  if (myData.id == DAQ_SENSE_ID) {
    SENSE = myData;
    // Serial.println("received sense");
    // Serial.println("dskljfhlksdj");
  }
  else if (myData.id == DAQ_POWER_ID) {
    POWER = myData;
    // Serial.print(POWER.DAQState);
  }
//  Serial.printf("Board ID %u: %u bytes\n", myData.id, len);
}


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
  receiveDataPrint();
  loopStartTime=millis();
  if (Serial.available() > 0) {
  // read the incoming byte:
    serialState = Serial.read()-48; //serial monitor input must be set to "No Line Ending"
  }
  if (POWER.DAQState != serialState) {
    dataSendCheck(); 
    // Serial.print("sdhgklsdhfljksf");
  }
  // Serial.print("I received: "); 
  SWITCH_ARMED.poll();
  SWITCH_PRESS.poll();
  SWITCH_QD.poll();
  SWITCH_IGNITION.poll();
  SWITCH_HOTFIRE.poll();
  SWITCH_ABORT.poll();
  if (SWITCH_ABORT.on()) {Serial.println("abort switch on");}
  if (SWITCH_ARMED.on()) {Serial.println("armed switch on");}
  if (SWITCH_PRESS.on()) {Serial.println("press switch on");}
  if (SWITCH_QD.on()) {Serial.println("qd switch on");}
  if (SWITCH_IGNITION.on()) {Serial.println("ignitie switch on");}
  if (SWITCH_HOTFIRE.on()) {Serial.println("hf switch on");}
  if (DEBUG ==1) {
  Serial.print("COM State: ");
  Serial.print(state);
  Serial.print(";      DAQ State: ");
  Serial.println(POWER.DAQState);
  }
  switch (state) {
//
// if (DEBUG ==1) {
//  Serial.println(state);
// }

  case (IDLE): //Includes polling
    turnoffLEDs();
    if (SWITCH_ABORT.on()) {serialState=ABORT;}
    if (SWITCH_ARMED.on()) {serialState=ARMED;}
    state = serialState;
    break;

  case (ARMED):
    if (POWER.DAQState == ARMED) {digitalWrite(LED_ARMED, HIGH);}
    if (SWITCH_ABORT.on()) {serialState=ABORT;}
    if (SWITCH_PRESS.on()) {serialState=PRESS;}
    if(!SWITCH_ARMED.on() && SWITCHES) {serialState=IDLE;}
    state = serialState;
    break;

  case (PRESS):
   // press();
    if (SWITCH_ABORT.on()) {serialState=ABORT;}
    if (POWER.DAQState == PRESS) {digitalWrite(LED_PRESS, HIGH);}
    if (ethComplete) {digitalWrite(LED_PRESSETH, HIGH);}
    if (oxComplete) {digitalWrite(LED_PRESSLOX, HIGH);}
    if (!ethComplete) {digitalWrite(LED_PRESSETH, LOW);}
    if (!oxComplete) {digitalWrite(LED_PRESSLOX, LOW);}
    if (SWITCH_QD.on()) {serialState=QD;}  //add pressComplete && later
    //add return to idle functionality hyer
    if(!SWITCH_PRESS.on() && !SWITCH_ARMED.on() && SWITCHES) {serialState=IDLE;}
    state = serialState;
    break;

  case (QD):
    //quick_disconnect();
    if (SWITCH_ABORT.on()) {serialState=ABORT;}
    if (POWER.DAQState == QD) {digitalWrite(LED_QD, HIGH);}
    if (SWITCH_IGNITION.on()) {serialState=IGNITION;}
    if(!SWITCH_QD.on() && !SWITCH_ARMED.on() && SWITCHES) {serialState=IDLE;}
    dataSendCheck();
    state = serialState;
    break;


  case (IGNITION):
    //ignition();
    if (SWITCH_ABORT.on()) {serialState=ABORT;}
    if (POWER.DAQState == IGNITION) {digitalWrite(LED_IGNITION, HIGH);}
    if (SWITCH_HOTFIRE.on()) {serialState=HOTFIRE;}
    if(!SWITCH_ARMED.on() && !SWITCH_IGNITION.on() && SWITCHES) {
      serialState=IDLE;}
    // Serial.print("ksjdgksldjf");
    dataSendCheck();
    state = serialState;
    break;

  case (HOTFIRE):
    // hotfire();
    if (SWITCH_ABORT.on()) {serialState=ABORT;}
    if (POWER.DAQState == HOTFIRE) {digitalWrite(LED_HOTFIRE, HIGH);}
    if (!SWITCH_ARMED.on() && !SWITCH_HOTFIRE.on() && SWITCHES) {
      serialState=IDLE;}
    dataSendCheck();
    state = serialState;
    break;

  case (ABORT):
    if (POWER.DAQState == ABORT) {digitalWrite(LED_ABORT, HIGH);}
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
    if(!SWITCH_ARMED.on() && !SWITCH_ABORT.on() && SWITCHES) {serialState=IDLE;}
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



void dataSendCheck() {
    dataSend();
}

void dataSend() {
  // Set values to send
  Commands.COMState = state;
  Commands.id = COM_ID;
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &Commands, sizeof(Commands));
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
  serialMessage.clear();
  serialMessage.concat(" ");
  serialMessage.concat(SENSE.messageTime);
  serialMessage.concat(" ");
  serialMessage.concat(SENSE.messageTime);
  serialMessage.concat(" ");
  serialMessage.concat(POWER.messageTime);
  serialMessage.concat(" ");
  serialMessage.concat(SENSE.PT_O1);
  serialMessage.concat(" ");
  serialMessage.concat(SENSE.PT_O2);
  serialMessage.concat(" ");
  serialMessage.concat(SENSE.PT_E1);
  serialMessage.concat(" ");
  serialMessage.concat(SENSE.PT_E2);
  serialMessage.concat(" ");
  serialMessage.concat(SENSE.PT_C1);
  serialMessage.concat(" ");
  serialMessage.concat(SENSE.LC_1);
  serialMessage.concat(" ");
  serialMessage.concat(SENSE.LC_2);
  serialMessage.concat(" ");
  serialMessage.concat(SENSE.LC_3);
  serialMessage.concat(" ");
  serialMessage.concat(SENSE.TC_1);
  serialMessage.concat(" ");
  serialMessage.concat(SENSE.TC_2);
  serialMessage.concat(" ");
  serialMessage.concat(SENSE.TC_3);
  serialMessage.concat(" ");
  serialMessage.concat(SENSE.TC_4);
  serialMessage.concat(" ");
  serialMessage.concat(POWER.ethComplete);
  serialMessage.concat(" ");
  serialMessage.concat(POWER.oxComplete);
  serialMessage.concat(" ");
  serialMessage.concat(Commands.COMState);
  serialMessage.concat(" ");
  serialMessage.concat(POWER.DAQState);
  serialMessage.concat(" ");
  serialMessage.concat(SENSE.queueLength);
  Serial.println(serialMessage);
  serialMessage = "";
}


////////BANDAID CODE FOR SENSE-COM COMMS ISSUE

// void receiveDataPrint() {
//   serialMessage.clear();
//   serialMessage.concat(" ");
//   serialMessage.concat(millis());
//   serialMessage.concat(" ");
//   serialMessage.concat(SENSE.messageTime);
//   serialMessage.concat(" ");
//   serialMessage.concat(POWER.messageTime);
//   serialMessage.concat(" ");
//   serialMessage.concat(POWER.PT_O1);
//   serialMessage.concat(" ");
//   serialMessage.concat(POWER.PT_O2);
//   serialMessage.concat(" ");
//   serialMessage.concat(POWER.PT_E1);
//   serialMessage.concat(" ");
//   serialMessage.concat(POWER.PT_E2);
//   serialMessage.concat(" ");
//   serialMessage.concat(POWER.PT_C1);
//   serialMessage.concat(" ");
//   serialMessage.concat(POWER.LC_1);
//   serialMessage.concat(" ");
//   serialMessage.concat(POWER.LC_2);
//   serialMessage.concat(" ");
//   serialMessage.concat(POWER.LC_3);
//   serialMessage.concat(" ");
//   serialMessage.concat(POWER.TC_1);
//   serialMessage.concat(" ");
//   serialMessage.concat(POWER.TC_2);
//   serialMessage.concat(" ");
//   serialMessage.concat(POWER.TC_3);
//   serialMessage.concat(" ");
//   serialMessage.concat(POWER.TC_4);
//   serialMessage.concat(" ");
//   serialMessage.concat(POWER.ethComplete);
//   serialMessage.concat(" ");
//   serialMessage.concat(POWER.oxComplete);
//   serialMessage.concat(" ");
//   serialMessage.concat(Commands.COMState);
//   serialMessage.concat(" ");
//   serialMessage.concat(POWER.DAQState);
//   serialMessage.concat(" ");
//   serialMessage.concat(SENSE.queueLength);
//   Serial.println(serialMessage);
// }
