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
//#include <ezButton.h>
#include "avdweb_Switch.h"
#include "freertos/FreeRTOS.h"
#include "freertos/Task.h"

#define COM_ID 0
#define DAQ_ID 1
#define FLIGHT_ID 2

//IF YOU WANT TO DEBUG, SET THIS TO True, if not, set False
bool DEBUG = true;
bool WIFIDEBUG = false;
bool SWITCHES = false; // If we are using switches

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


esp_now_peer_info_t peerInfo;

enum STATES {IDLE, ARMED, PRESS, QD, IGNITION, HOTFIRE, ABORT};
String stateNames[] = { "Idle", "Armed", "Press", "QD", "Ignition", "HOTFIRE", "Abort" };

//#define DEBUG_IDLE 90
//#define DEBUG_ARMED 91
//#define DEBUG_PRESS 92
//#define DEBUG_QD 93
//#define DEBUG_IGNITION 94
//#define DEBUG_HOTFIRE 95
//#define DEBUG_ABORT 96

//ENSURE MAC ADDRESS IS CORRECT FOR DEVICE IN USE!!!
//DAQ Breadboard {0x24, 0x62, 0xAB, 0xD2, 0x85, 0xDC}
//DAQ Protoboard {0x0C, 0xDC, 0x7E, 0xCB, 0x05, 0xC4}
//NON BUSTED DAQ {0x7C, 0x9E, 0xBD, 0xD8, 0xFC, 0x14}
// {0xC4, 0xDD, 0x57, 0x9E, 0x96, 0x34};
// uint8_t broadcastAddress[] = {0xC8, 0xF0, 0x9E, 0x50, 0x23, 0x34}; //Core board 1
//uint8_t broadcastAddress[] = {0x08, 0x3A, 0xF2, 0xB7, 0xEE, 0x00}; //TEST
//{0x30, 0xC6, 0xF7, 0x2A, 0x28, 0x04}

uint8_t DAQBroadcastAddress[] = {0xE8, 0x6B, 0xEA, 0xD3, 0x93, 0x88};
uint8_t FlightBroadcastAddress[] = {0x48, 0x27, 0xE2, 0x2F, 0x22, 0x08};

//Structure example to send data
//Must match the receiver structure
struct struct_readings {
  float PT_O1;
  float PT_O2;
  float PT_E1;
  float PT_E2;
  float PT_C1;
  float TC_1;
  float TC_2;
  float TC_3;
  float TC_4;
};

struct struct_message {
  int messageTime;
  int sender;
  
  int COMState;
  int DAQState;
  int FlightState;
  bool AUTOABORT;
  short int FlightQueueLength;
  bool ethComplete;
  bool oxComplete;
  bool oxVentComplete;
  bool ethVentComplete;
  bool sdCardInitialized;

  struct_readings filteredReadings;
  
};

// Create a struct_message called Readings to recieve sensor readings remotely
struct_message incomingDAQReadings;
struct_message incomingFlightReadings;
// Create a struct_message to send commands
struct_message sendCommands;

int COMState = IDLE;
int DAQState = IDLE;
int FlightState = IDLE;

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


  Serial.println(WiFi.macAddress());
  //set device as WiFi station
  WiFi.mode(WIFI_STA);

  //initialize ESP32
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register peer
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  // Add peer
  memcpy(peerInfo.peer_addr, DAQBroadcastAddress, 6);

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
  }

  memcpy(peerInfo.peer_addr, FlightBroadcastAddress, 6);
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  Serial.print("My MAC Address ");
  Serial.println(WiFi.macAddress());

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

  COMState = IDLE;
}


void loop() {
  // Get the state from Serial input
  if (Serial.available() > 0) {
    COMState = Serial.read() - '0';
    Serial.println(COMState);
  }

  SWITCH_ARMED.poll();
  SWITCH_PRESS.poll();
  SWITCH_QD.poll();
  SWITCH_IGNITION.poll();
  SWITCH_HOTFIRE.poll();
  SWITCH_ABORT.poll();
    
  if (DEBUG) {
    Serial.print("COM State: ");
    Serial.print(COMState);
    Serial.print(" \tDAQ State: ");
    Serial.print(DAQState);
    Serial.print(" \tFlight State: ");
    Serial.println(FlightState);
    // if(!SWITCH_PRESS.on() && !SWITCH_ARMED.on() && !SWITCH_ABORT.on() && !SWITCH_QD.on() && !SWITCH_IGNITION.on() && !SWITCH_HOTFIRE.on()) {
    //   Serial.println("GOING BACK TO IDLE");
    //    COMState = IDLE; }
  }

  switch (COMState) {
    checkAbort();

    case (IDLE): //Includes polling
      idle();
      dataSend();

      if (SWITCH_ARMED.on()) {COMState=ARMED;}
      break;

    case (ARMED):
      dataSend();
      if (DAQState == ARMED) {digitalWrite(LED_ARMED, HIGH);}
      if (SWITCH_PRESS.on()) { COMState=PRESS; }
      if(!SWITCH_ARMED.on() && SWITCHES) { COMState=IDLE; }
      break;

    case (PRESS):
      dataSend();
      // if (DAQState == PRESS) { digitalWrite(LED_PRESS, HIGH); }
      // // if (incomingDAQReadings.ethComplete) { digitalWrite(LED_PRESSETH, HIGH); }
      // // if (incomingDAQReadings.oxComplete) { digitalWrite(LED_PRESSLOX, HIGH); }
      // // if (!incomingDAQReadings.ethComplete) { digitalWrite(LED_PRESSETH, LOW); }
      // // if (!incomingDAQReadings.oxComplete) { digitalWrite(LED_PRESSLOX, LOW); }
      // if (SWITCH_QD.on()) { COMState = QD; }
      // if(!SWITCH_PRESS.on() && !SWITCH_ARMED.on() && SWITCHES) { COMState = IDLE; }
      break;

    case (QD):
      if (DAQState == QD) {digitalWrite(LED_QD, HIGH);}
      if (SWITCH_IGNITION.on()) { COMState = IGNITION; }
      if(!SWITCH_QD.on() && !SWITCH_PRESS.on() && !SWITCH_ARMED.on() && SWITCHES) { COMState = IDLE; }
      dataSend();
      break;


    case (IGNITION):
      if (DAQState == IGNITION) {digitalWrite(LED_IGNITION, HIGH);}
      if (SWITCH_HOTFIRE.on()) { COMState = HOTFIRE; }
      if(!SWITCH_QD.on() && !SWITCH_PRESS.on() && !SWITCH_ARMED.on() && !SWITCH_IGNITION.on() && SWITCHES) { COMState = IDLE; }
      dataSend();
      break;

    case (HOTFIRE):
      if (DAQState == HOTFIRE) {digitalWrite(LED_HOTFIRE, HIGH);}
      if (!SWITCH_ARMED.on() && !SWITCH_HOTFIRE.on() && SWITCHES) { COMState = IDLE; }
      dataSend();
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
      COMState = ABORT;
      dataSend();
      if(!SWITCH_QD.on() && !SWITCH_PRESS.on() && !SWITCH_ARMED.on() && !SWITCH_IGNITION.on() && !SWITCH_HOTFIRE.on() && !SWITCH_ABORT.on() && SWITCHES) {COMState = IDLE;}
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
  turnoffLEDs();
}

void checkAbort() {
  if (SWITCH_ABORT.on()) {
    COMState = ABORT;
  }
}

void dataSend() {
  // Set values to send
  sendCommands.sender = COM_ID;
  sendCommands.COMState = COMState;
  if (DEBUG){
    Serial.print("COMSTATE: ");
    Serial.println(COMState);
  }

  // Send ABORT to flight
  if (COMState != FlightState) { 

    esp_err_t result = esp_now_send(FlightBroadcastAddress, (uint8_t *) &sendCommands, sizeof(sendCommands));
    if(result == ESP_OK) {
      Serial.println("Successful Send to FLIGHT!");
    } else {
      Serial.println("Failed Send to FLIGHT");
    }
  }

  // Don't send data if states are already synced
  if (COMState != DAQState) {
    
    esp_err_t result = esp_now_send(DAQBroadcastAddress, (uint8_t *) &sendCommands, sizeof(sendCommands));
    if (result == ESP_OK) {
    Serial.println("Sent with success to DAQ");
    }
    else {
      Serial.println("Error sending the data to DAQ");
    }

  }
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  struct_message incomingReadings;
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));

  if (incomingReadings.sender == DAQ_ID) {
    incomingDAQReadings = incomingReadings;
    DAQState = incomingReadings.DAQState;
    if (DEBUG){
      Serial.print("DAQSTATE: ");
      Serial.println(DAQState);
    }
    // receiveDataPrint(incomingDAQReadings);
  }
  else if (incomingReadings.sender == FLIGHT_ID) {
    incomingFlightReadings = incomingReadings;
    FlightState = incomingReadings.FlightState;
    receiveDataPrint(incomingFlightReadings);
    if (incomingReadings.AUTOABORT){
      COMState = ABORT;
    }
  }
}

void receiveDataPrint(struct_message &incomingReadings) {
  String serialMessage = " ";
  serialMessage.concat(millis());
  serialMessage.concat(" ");
  serialMessage.concat(incomingReadings.filteredReadings.PT_O1);
  serialMessage.concat(" ");
  serialMessage.concat(incomingReadings.filteredReadings.PT_O2);
  serialMessage.concat(" ");
  serialMessage.concat(incomingReadings.filteredReadings.PT_E1);
  serialMessage.concat(" ");
  serialMessage.concat(incomingReadings.filteredReadings.PT_E2);
  serialMessage.concat(" ");
  serialMessage.concat(incomingReadings.filteredReadings.PT_C1);
  serialMessage.concat(" ");
  serialMessage.concat(incomingReadings.filteredReadings.TC_1);
  serialMessage.concat(" ");
  serialMessage.concat(incomingReadings.filteredReadings.TC_2);
  serialMessage.concat(" ");
  serialMessage.concat(incomingReadings.filteredReadings.TC_3);
  serialMessage.concat(" ");
  serialMessage.concat(incomingReadings.filteredReadings.TC_4);
  serialMessage.concat("\nEth comp: ");
  serialMessage.concat(incomingDAQReadings.ethComplete ? "True" : "False");
  serialMessage.concat(" Ox comp: ");
  serialMessage.concat(incomingDAQReadings.oxComplete  ? "True" : "False");
  serialMessage.concat("\nEth vent: ");
  // OR the bits in case either FLIGHT or DAQ is in Abort mode
  serialMessage.concat(incomingDAQReadings.ethVentComplete | incomingFlightReadings.ethVentComplete ? "True" : "False");
  serialMessage.concat(" Ox vent: ");
  serialMessage.concat(incomingDAQReadings.oxVentComplete | incomingFlightReadings.oxVentComplete ? "True" : "False");

  serialMessage.concat("\n COM State: ");
  serialMessage.concat(stateNames[COMState]);
  serialMessage.concat("   DAQ State: ");
  serialMessage.concat(stateNames[DAQState]);
  serialMessage.concat("   Flight State: ");
  serialMessage.concat(stateNames[FlightState]);
  serialMessage.concat("\n Flight Q Length: ");
  serialMessage.concat(incomingReadings.FlightQueueLength);
  serialMessage.concat("\n SD Card Initialized");
  serialMessage.concat(incomingReadings.sdCardInitialized ? "True" : "False");
  Serial.println(serialMessage);
}