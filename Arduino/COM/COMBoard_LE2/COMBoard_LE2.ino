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
 int commandedState;
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
#define DEBUG 99
#define DEBUG_IDLE 90
#define DEBUG_ARMED 91
#define DEBUG_PRESS 92
#define DEBUG_QD 93
#define DEBUG_IGNITION 94
#define DEBUG_HOTFIRE 95
#define DEBUG_ABORT 96
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
// uint8_t broadcastAddress[] = {0x7C, 0x9E, 0xBD, 0xD8, 0xFC, 0x14}; //change to new Mac Address
// {0xC4, 0xDD, 0x57, 0x9E, 0x96, 0x34};
uint8_t broadcastAddress[] = {0xC8, 0xF0, 0x9E, 0x4F, 0xAF, 0x40};
//{0x30, 0xC6, 0xF7, 0x2A, 0x28, 0x04}

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
     float messageTime;
     float pt1;
     float pt2;
     float pt3;
     float pt4;
     float pt5;
     float lc1;
     float lc2;
     float lc3;
     float tc1;
     float tc2;
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

 

  while(SWITCH_ABORT.on()){digitalWrite(LED_ABORT, HIGH);}

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
 //   Serial.println("Failed to add peer");
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
  Serial.print("COM State: ");
  Serial.print(state);
  Serial.print(";      DAQ State: ");
  Serial.println(DAQState);
  if(!SWITCH_PRESS.on() && !SWITCH_ARMED.on() && !SWITCH_ABORT.on() && !SWITCH_QD.on() && !SWITCH_IGNITION.on() && !SWITCH_HOTFIRE.on()) {serialState=IDLE;}


  switch (state) {
  Serial.println(state);

  case (IDLE): //Includes polling
    idle();
    if (SWITCH_ABORT.on()) {serialState=ABORT;}
    if (SWITCH_ARMED.on()) {serialState=ARMED;}
    state = serialState;
    break;

  case (ARMED):
    armed();
    if (DAQState == ARMED) {digitalWrite(LED_ARMED, HIGH);}
    if (SWITCH_ABORT.on()) {serialState=ABORT;}
    if (SWITCH_PRESS.on()) {serialState=PRESS;}
    if(!SWITCH_ARMED.on()) {serialState=IDLE;}
   
    state = serialState;
    armed();

    break;

  case (PRESS): 
   // press();
    if (SWITCH_ABORT.on()) {serialState=ABORT;}
    if (DAQState == PRESS) {digitalWrite(LED_PRESS, HIGH);}
    if (ethComplete) {digitalWrite(LED_PRESSETH, HIGH);}
    if (oxComplete) {digitalWrite(LED_PRESSLOX, HIGH);}
    if (SWITCH_QD.on()) {serialState=QD;}  //add pressComplete && later
    state = serialState;
    press();
    
    

  case (QD):
    //quick_disconnect();
    if (SWITCH_ABORT.on()) {serialState=ABORT;}
    if (DAQState == QD) {digitalWrite(LED_QD, HIGH);}
    if (SWITCH_IGNITION.on()) {serialState=IGNITION;}
    state = serialState;
    if(!SWITCH_QD.on() && !SWITCH_PRESS.on() && !SWITCH_ARMED.on()) {serialState=IDLE;}
        quick_disconnect();
    break;


  case (IGNITION):
    //ignition();
    if (SWITCH_ABORT.on()) {serialState=ABORT;}
    if (DAQState == IGNITION) {digitalWrite(LED_IGNITION, HIGH);}
    if (SWITCH_HOTFIRE.on()) {serialState=HOTFIRE;}
    state = serialState;
    if(!SWITCH_QD.on() && !SWITCH_PRESS.on() && !SWITCH_ARMED.on() && !SWITCH_IGNITION.on()) {serialState=IDLE;}
    ignition();
    break;

  case (HOTFIRE):
    // hotfire();

    if (SWITCH_ABORT.on()) {serialState=ABORT;}
    if (DAQState == HOTFIRE) {digitalWrite(LED_HOTFIRE, HIGH);}
    hotfire();
   
    
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
    abort_sequence();
    if(!SWITCH_QD.on() && !SWITCH_PRESS.on() && !SWITCH_ARMED.on() && !SWITCH_IGNITION.on() && !SWITCH_HOTFIRE.on() && !SWITCH_ABORT.on()) {serialState=IDLE;}
    state = serialState;     
    break;

  case (DEBUG):
   // debug();
    serialState=IDLE;
    state = serialState;
    debug();
    break;
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

void armed() {
 
  dataSendCheck();
}

void press() {

  dataSendCheck();
}

void quick_disconnect() {

  dataSendCheck();
}

void ignition() {
  
  dataSendCheck();
}

void hotfire() {
 
  dataSendCheck();
}

void abort_sequence() {
  dataSendCheck();
}

void debug() {
  while(state == DEBUG){
    debug_state = Serial.parseInt();
  switch (debug_state) {
  
  case (DEBUG_IDLE): //Includes polling
    //idle();
  break;    
    
    

  case (DEBUG_ARMED):
    digitalWrite(LED_ARMED, HIGH);
    break;    

  case (DEBUG_PRESS): 
  
    digitalWrite(LED_PRESS, HIGH);
    digitalWrite(LED_PRESSETH, HIGH);
    digitalWrite(LED_PRESSLOX, HIGH);
    
    
    
  case (DEBUG_QD):
    digitalWrite(LED_QD, HIGH);

    break;


  case (DEBUG_IGNITION):
    
   digitalWrite(LED_IGNITION, HIGH);
   
    break;

  case (DEBUG_HOTFIRE):
    // hotfire();

    digitalWrite(LED_HOTFIRE, HIGH);
  
    break;
  
  case (DEBUG_ABORT):
   digitalWrite(LED_ABORT, HIGH);
    digitalWrite(LED_IGNITION,LOW);
    digitalWrite(LED_QD, LOW);
    digitalWrite(LED_ARMED, LOW);
    digitalWrite(LED_PRESS, LOW);
    digitalWrite(LED_PRESSETH, LOW);
    digitalWrite(LED_PRESSLOX, LOW);
    digitalWrite(LED_HOTFIRE, LOW);
  
    break;
  default :
  state = IDLE;    

  }

   
  }
  
}

void dataSendCheck() {
 
    dataSend(); 
  
}

void dataSend() {
  // Set values to send
  Commands.commandedState = state;

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &Commands, sizeof(Commands));
}
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  Serial.print("Bytes received: ");
  Serial.println(len);
  DAQState = incomingReadings.DAQState;
  incomingPT1 = incomingReadings.pt1;
  incomingPT2 = incomingReadings.pt2;
  incomingPT3 = incomingReadings.pt3;
  incomingPT4 = incomingReadings.pt4;
  incomingPT5 = incomingReadings.pt5;
  incomingLC1 = incomingReadings.lc1;
  incomingLC2 = incomingReadings.lc2;
  incomingLC3 = incomingReadings.lc3;
  incomingTC1 = incomingReadings.tc1;
  incomingTC2 = incomingReadings.tc2;
  pressComplete = incomingReadings.pressComplete;
  oxComplete = incomingReadings.oxComplete;
  ethComplete = incomingReadings.ethComplete;
  queueSize = incomingReadings.queueSize;
  Serial.print("PT1: "); 
  Serial.print(incomingPT1);
  Serial.print("   PT2: "); 
  Serial.print(incomingPT2);
  Serial.print("   PT3: "); 
  Serial.print(incomingPT3);
  Serial.print("   PT4: "); 
  Serial.print(incomingPT4);
  Serial.print("   PT5: "); 
  Serial.println(incomingPT5);
  

  

  
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
  // message.concat(incomingCap1);
  // message.concat(" ");
  // message.concat(incomingCap2);
  // message.concat(" ");
  message.concat(Commands.commandedState);
  message.concat(" ");
  message.concat(queueSize);

  Serial.println(message);
}

