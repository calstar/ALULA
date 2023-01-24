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

// *** START - WILL BE REPLACED BY LABVIEW STATE DETERMINATION *** //
// Set pinouts
#define BUTTON1 19
#define BUTTON2 17
#define BUTTON3 16
#define BUTTON4 21 
#define BUTTON5 27
#define PRESS_BUTTON 1 
#define QD_BUTTON 2

#define INDICATOR1  4 // State 2 light
#define INDICATOR2 23 // Armed indicator
#define INDICATOR3 22 // Servo1 indicator
#define INDICATOR4 14 // Servo 2 
#define INDICATOR5 25 // DAQ indicaor
#define INDICATOR6 5 // COM indicator
// *** END *** //

float pressTime = 0;

String success;
String message;
int incomingMessageTime;
int incomingPT1 = 4; //PT errors when initialized to zero
int incomingPT2 = 4;
int incomingPT3 = 4;
int incomingPT4 = 4;
int incomingFM = 0;
int incomingPT5 = 4;
int incomingPT6 = 4;
int incomingPT7 = 4;
short int incomingI = 0;
int incomingDebug = 0;
int actualState = -5;
short int queueSize = 0;
esp_now_peer_info_t peerInfo;
bool pressed1 = false;
bool pressed2 = false;
bool pressed3 = false;
int commandstate = 0;

//TIMING VARIABLES
String state = "idle";
int loopStartTime;
int dataCollectionDelay = 10;
int sendDelay = 50;

String commandedState;

int lastPrintTime = 0;
int PrintDelay = 1000;

float button1Time = 0;
float currTime = 0;
float loopTime = 0;

float sendTime = 0;
float receiveTime = 0;

//for blinking LED during Data Collection
int x = 1;
unsigned long t1;
unsigned long t2;

//

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
    int pt1val;
    int pt2val;
    int pt3val;
    int pt4val;
    int pt5val;
    int pt6val;
    int pt7val;
    int fmval;
    String commandedState = "idle";
    unsigned char I;
    short int queueSize;
    int Debug;
} struct_message;

// Create a struct_message called Readings to recieve sensor readings remotely
struct_message incomingReadings;

// Create a struct_message to send commands
struct_message Commands;

//

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
  incomingPT6 = incomingReadings.pt6;
  incomingPT7 = incomingReadings.pt7;
  incomingFM = incomingReadings.fmval;
  queueSize = incomingReadings.queueSize;
  incomingI = incomingReadings.I;
  incomingDebug = incomingReadings.Debug;

  receiveTimeCOM = millis();
  receiveDataPrint();
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
   // Serial.println("Start of Setup");

  // Use buttons for moving between states
  pinMode(BUTTON1,INPUT);
  pinMode(BUTTON2, INPUT);
  pinMode(BUTTON3,INPUT);
  pinMode(BUTTON4, INPUT);
  pinMode(BUTTON5, INPUT);

  pinMode(INDICATOR1, OUTPUT);
  pinMode(INDICATOR2, OUTPUT);
  pinMode(INDICATOR3, OUTPUT);
  pinMode(INDICATOR4, OUTPUT);
  pinMode(INDICATOR5, OUTPUT);
  pinMode(INDICATOR6, OUTPUT);

  pinMode(PRESS_BUTTON, OUTPUT);
  pinMode(QD_BUTTON, OUTPUT);

  digitalWrite(INDICATOR2,LOW);
  digitalWrite(INDICATOR3, LOW);
  digitalWrite(INDICATOR4, LOW);
  digitalWrite(INDICATOR5, LOW);
  digitalWrite(INDICATOR6, LOW);
  digitalWrite(INDICATOR1, LOW);

  digitalWrite(PRESS_BUTTON, LOW);
  digitalWrite(QD_BUTTON, LOW);

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

  Serial.println(WiFi.macAddress());
}

void printLine(String string) {
  Serial.println(string);
}

// States: Idle, Fill LOX, Press LOX, Press Eth, Vent LOX, Vent Eth, Hotfire, Ignition
void loop() {
  loopStartTime=millis();
  SerialRead();

  switch (state) {

  case ("idle"): //Includes polling
    idle();
    if (digitalRead(BUTTON2) == 1) { 
      state = "press_eth";}
    break;

  case ("press_eth"): //Pressurizes ethanol tank
    press_eth();
    if (digitalRead(BUTTON3) == 1) { state = "fill";}
    if (digitalRead(BUTTON1) == 1) { state = "vent_eth";}
    break;

  case ("fill"): //Fills LOX tank
    fill();
    if (digitalRead(BUTTON4)==1) {state="press_lox";} 
    if (digitalRead(BUTTON1)==1) {state="vent";}  
    break;

  case ("press_lox"): //Pressurizes LOX tank
    press_lox();
    if (digitalRead(BUTTON5)==1) {state="vent"};
    break;

  case ("hotfire"):
    hotfire();
    if (digitalRead(BUTTON1)==1) {state="idle"};
    state = "ignition"

  case ("ignition"):
    ignition();
    state = "idle";

  case ("vent_eth"): //Vents ethanol
    vent_eth();
    if (digitalRead(BUTTON1)==1) {state="idle";}
  
  case ("vent"): //Vents both LOX and ethanol
    vent();
    if (digitalRead(BUTTON1)==1) {state=="idle";}
  }
}

void idle() {
  commandedState= "idle";
  dataSendCheck();
}

void press_eth() {
  commandedState = "press_eth";
  dataSendCheck();
}

void fill() {
  commandedState = "fill";
  dataSendCheck();
}

void press_lox() {
  commandedState = "press_lox";
  dataSendCheck();
}

void hotfire() {
  commandedState = "hotfire";
  dataSendCheck();
}

void ignition() {
  commandedState = "ignition";
  dataSendCheck();
}

void vent_eth() {
  commandedState = "vent_eth";
  dataSendCheck();
}

void vent() {
  commandedState = "vent";
  dataSendCheck();
}

void dataSendCheck() {
  if ((loopStartTime-lastSendTime) > sendDelay) dataSend(); 
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
  message.concat(incomingPT6);
  message.concat(" ");
  message.concat(incomingPT7);
  message.concat(" ");
  message.concat(incomingFM);
  message.concat(" ");
  message.concat(Commands.commandedState);
  message.concat(" ");
  message.concat(queueSize);

  printLine(message);
}