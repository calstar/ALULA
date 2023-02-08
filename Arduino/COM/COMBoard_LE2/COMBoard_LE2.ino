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

// Set pinouts. Currently set arbitrarily
#define BUTTON_IDLE 19
#define BUTTON_ARMED 22
#define BUTTON_FILL 16
#define BUTTON_PRESS 17
#define BUTTON_QD 5
#define BUTTON_IGNITION 1
#define BUTTON_HOTFIRE 27 
#define BUTTON_ABORT 12

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
short int queueSize = 0;
bool fillComplete = false;
bool pressEthComplete = false;
bool pressLOXComplete = false;

esp_now_peer_info_t peerInfo;

//TIMING VARIABLES
int state;
int serialState;
int manualState;
int loopStartTime;
int sendDelay = 50;   //Measured in ms
enum STATES {IDLE, ARMED, FILL, PRESS_ETH, PRESS_LOX, QD, IGNITION, HOTFIRE, ABORT, DEBUG=99};

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
    short int queueSize;
    bool fillComplete;
    bool pressEthComplete;
    bool pressLOXComplete;
} struct_message; 

// Create a struct_message called Readings to recieve sensor readings remotely
struct_message incomingReadings;

// Create a struct_message to send commands
struct_message Commands;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == 0) {
    sendTime = millis();
  }
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));

  incomingMessageTime= incomingReadings.messageTime;
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
  incomingCap1 = incomingReadings.cap1;
  incomingCap2 = incomingReadings.cap2;
  queueSize = incomingReadings.queueSize;

  receiveTime = millis();
  receiveDataPrint();
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
   // Serial.println("Start of Setup");

  // Use buttons for moving between states
  pinMode(BUTTON_IDLE,INPUT);
  pinMode(BUTTON_ARMED,INPUT);
  pinMode(BUTTON_FILL,INPUT);
  pinMode(BUTTON_PRESS,INPUT);
  pinMode(BUTTON_QD,INPUT);
  pinMode(BUTTON_IGNITION,INPUT);
  pinMode(BUTTON_HOTFIRE,INPUT);
  pinMode(BUTTON_ABORT,INPUT);

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

void loop() {
  loopStartTime=millis();
  SerialRead();

  switch (state) {

  case (IDLE): //Includes polling
    idle();
    if (digitalRead(BUTTON_ARMED)==1) {serialState=ARMED;}
    if (digitalRead(BUTTON_ABORT)==1) {serialState=ABORT;}
    state = serialState;
    break;

  case (ARMED):
    armed();
    if (digitalRead(BUTTON_FILL)==1) {serialState=FILL;}
    if (digitalRead(BUTTON_ABORT)==1) {serialState=ABORT;}
    state = serialState;
    break;

  case (FILL): //Fills LOX tank
    fill();
    if (digitalRead(BUTTON_PRESS)==1 && fillComplete) {serialState=PRESS_ETH;} 
    if (digitalRead(BUTTON_IDLE)==1) {serialState=IDLE;}
    if (digitalRead(BUTTON_ABORT)==1) {serialState=ABORT;}
    state = serialState;  
    break;

  case (PRESS_ETH): //Pressurizes ethanol tank
    press_eth();
    if (digitalRead(BUTTON_IDLE)==1) {serialState=IDLE;}
    if (pressEthComplete) {serialState=PRESS_LOX;}
    if (digitalRead(BUTTON_ABORT)==1) {serialState=ABORT;}
    state = serialState;
    break;

  case (PRESS_LOX): //Pressurizes LOX tank
    press_lox();
    if (digitalRead(BUTTON_IDLE)==1) {serialState=IDLE;}
    if (digitalRead(BUTTON_QD)==1 && pressLOXComplete) {serialState=QD;}
    if (digitalRead(BUTTON_ABORT)==1) {serialState=ABORT;}
    state = serialState;
    break;

  case (QD):
    quick_disconnect();
    if (digitalRead(BUTTON_IDLE)==1) {serialState=IDLE;}
    if (digitalRead(BUTTON_IGNITION)==1) {serialState=IGNITION;}
    if (digitalRead(BUTTON_ABORT)==1) {serialState=ABORT;}
    state = serialState;
    break;

  case (IGNITION):
    ignition();
    if (digitalRead(BUTTON_IDLE)==1) {serialState=IDLE;}
    if (digitalRead(BUTTON_HOTFIRE)==1) {serialState=HOTFIRE;}
    if (digitalRead(BUTTON_ABORT)==1) {serialState=ABORT;}
    state = serialState;
    break;

  case (HOTFIRE):
    hotfire();
    if (digitalRead(BUTTON_IDLE)==1) {serialState=IDLE;}
    if (digitalRead(BUTTON_ABORT)==1) {serialState=ABORT;}
    state = serialState;
    break;
  
  case (ABORT): 
    abort_sequence();
    if (digitalRead(BUTTON_IDLE)==1) {serialState=IDLE;}
    state = serialState;
    break;

  case (DEBUG):
    debug();
    if (digitalRead(BUTTON_IDLE)==1) {serialState=IDLE;}
    state = serialState;
    break;

  }
}

void idle() {
  commandedState = IDLE;
  dataSendCheck();
}

void armed() {
  commandedState = ARMED;
  dataSendCheck();
}

void fill() {
  commandedState = FILL;
  dataSendCheck();
}

void press_eth() {
  commandedState = PRESS_ETH;
  dataSendCheck();
}


void press_lox() {
  commandedState = PRESS_LOX;
  dataSendCheck();
}

void quick_disconnect() {
  commandedState = QD;
  dataSendCheck();
}

void ignition() {
  commandedState = IGNITION;
  dataSendCheck();
}

void hotfire() {
  commandedState = HOTFIRE;
  dataSendCheck();
}

void abort_sequence() {
  commandedState = ABORT;
  dataSendCheck();
}

void debug() {
  commandedState = DEBUG;
  dataSendCheck();
}

void dataSendCheck() {
  if ((loopStartTime-sendTime) > sendDelay) {
    dataSend(); 
  }
}

void dataSend() {
  // Set values to send
  Commands.commandedState = commandedState;

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &Commands, sizeof(Commands));
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
  message.concat(incomingCap1);
  message.concat(" ");
  message.concat(incomingCap2);
  message.concat(" ");
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
      serialState = FILL;
    } else if (manualState == 3) {
      serialState = PRESS_ETH;
    } else if (manualState == 4) {
      serialState = PRESS_LOX;
    } else if (manualState == 5) {
      serialState = QD;
    } else if (manualState == 6) {
      serialState = IGNITION;
    } else if (manualState == 7) {
      serialState = HOTFIRE;
    } else if (manualState == 8) {
      serialState = ABORT;
    } else if (manualState = 99) {
      serialState = DEBUG;
    }
  }
}