/*
This code runs on the COM ESP32 and has a couple of main tasks.
1. Receive sensor data from DAQ ESP32
2. Send servo commands to DAQ ESP32
*/

#include <esp_wifi.h>
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
bool DAQ_DEBUG = false;
bool DEBUG = false;
bool WIFIDEBUG = false;
bool SWITCHES = false; // If we are using switches
bool GUI_DEBUG = false;

int timerind = 0;
int daqtimerind = 0;
int sendperiod = 25;

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

// Number of PTs in system
#define NUM_PTS 6

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
uint8_t DAQBroadcastAddress[] = {0xC8, 0xF0, 0x9E, 0x50, 0x23, 0x34};

uint8_t FlightBroadcastAddress[] = {0x48, 0x27, 0xE2, 0x88, 0x39, 0x44}; //CORE A
// uint8_t FlightBroadcastAddress[] = {0xC0, 0x4E, 0x30, 0x1A, 0x00, 0x14}; //RANDOM ESP

//Structure example to send data
//Must match the receiver structure
struct struct_pt_offsets {
  bool PT_O1_set;
  bool PT_O2_set;
  bool PT_E1_set;
  bool PT_E2_set;
  bool PT_C1_set;
  bool PT_X_set;

  int PT_O1_offset;
  int PT_O2_offset;
  int PT_E1_offset;
  int PT_E2_offset;
  int PT_C1_offset;
  int PT_X_offset;
};

struct struct_readings {
  float PT_O1;
  float PT_O2;
  float PT_E1;
  float PT_E2;
  float PT_C1;
  float PT_X;
  // int TC_1;
  // int TC_2;
  // int TC_3;
  // int TC_4;
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
  struct_readings rawReadings;
  struct_pt_offsets pt_offsets;
};

// Create a struct_message called Readings to recieve sensor readings remotely
struct_message incomingDAQReadings;
struct_message incomingFlightReadings;
// Create a struct_message to send commands
struct_message sendCommands;

int COMState = IDLE;
int DAQState = IDLE;
int FlightState = IDLE;

// Turn this on to send PT offset updates to flight
bool updatePTOffsets = false;

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
  //LOOK AT THIS LOOK AT THIS LOOK AT THIS THIS IS THE TESTING FOR LONG RANGE AGAIN LOOK AT THIS
  // esp_wifi_set_protocol( WIFI_IF_STA , WIFI_PROTOCOL_LR);


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

  if (GUI_DEBUG) {
    incomingFlightReadings.AUTOABORT = false;
    incomingFlightReadings.ethVentComplete = true;
    incomingFlightReadings.oxVentComplete = true;

    incomingDAQReadings.oxComplete = true;
    receiveDataPrint(incomingFlightReadings);
  }

  dataSend(); //initiate send process
  // Get the state from Serial input
  if (Serial.available() > 0) {
    char header = Serial.read();
    if (header == 's') {
      COMState = Serial.read() - '0';
      Serial.println(COMState);
    } else if (header == 'o') {
      char ptNumber = Serial.read() - '0';
      int newOffset = Serial.readString().toInt();

      updateSendDataWithOffsets(ptNumber, newOffset);
    }
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
    Serial.print("Size of send commands: ");
    Serial.println(sizeof(sendCommands));
    // if(!SWITCH_PRESS.on() && !SWITCH_ARMED.on() && !SWITCH_ABORT.on() && !SWITCH_QD.on() && !SWITCH_IGNITION.on() && !SWITCH_HOTFIRE.on()) {
    //   Serial.println("GOING BACK TO IDLE");
    //    COMState = IDLE; }
  }

  switch (COMState) {

    checkAbort();
    case (IDLE): //Includes polling
      idle();
      if (SWITCH_ARMED.on()) {COMState=ARMED;}
      break;

    case (ARMED):
      if (DAQState == ARMED) {digitalWrite(LED_ARMED, HIGH);}
      if (SWITCH_PRESS.on()) { COMState=PRESS; }
      if(!SWITCH_ARMED.on() && SWITCHES) { COMState=IDLE; }
      break;

    case (PRESS):
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
      break;


    case (IGNITION):
      if (DAQState == IGNITION) {digitalWrite(LED_IGNITION, HIGH);}
      if (SWITCH_HOTFIRE.on()) { COMState = HOTFIRE; }
      if(!SWITCH_QD.on() && !SWITCH_PRESS.on() && !SWITCH_ARMED.on() && !SWITCH_IGNITION.on() && SWITCHES) { COMState = IDLE; }
      break;

    case (HOTFIRE):
      if (DAQState == HOTFIRE) {digitalWrite(LED_HOTFIRE, HIGH);}
      if (!SWITCH_ARMED.on() && !SWITCH_HOTFIRE.on() && SWITCHES) { COMState = IDLE; }
      break;

    case (ABORT):
      //break; // REDS ABORT SYSTEM
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
  // Serial.println("com: ");
  // Serial.println(COMState);

  // Send ABORT to flight
  if (COMState != FlightState || updatePTOffsets) {
    if (millis() - timerind > sendperiod) {
      esp_err_t result = esp_now_send(FlightBroadcastAddress, (uint8_t *) &sendCommands, sizeof(sendCommands));
      updatePTOffsets = false;
      timerind = millis();
      if (WIFIDEBUG) {
        if(result == ESP_OK) {
          Serial.println("Successful Send to FLIGHT!");
        } else {
          Serial.println("Failed Send to FLIGHT");
        }
      }
  }
  }

  // Don't send data if states are already synced
  if (COMState != DAQState) {
    if (millis() - daqtimerind > sendperiod) {
      esp_err_t result = esp_now_send(DAQBroadcastAddress, (uint8_t *) &sendCommands, sizeof(sendCommands));
      daqtimerind = millis();
      if (WIFIDEBUG) { //printouts to debug wifi/comms
        if (result == ESP_OK) {
        Serial.println("Sent with success to DAQ");
        }
        else {
          Serial.println("Error sending the data to DAQ");
        }
      }
    }
}
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  struct_message incomingReadings;
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));

  if (incomingReadings.sender == DAQ_ID) {
    incomingDAQReadings = incomingReadings;
    DAQState = incomingReadings.DAQState;
    Serial.print(COMState); Serial.print(" "); Serial.println(DAQState);
    if (DAQ_DEBUG){
      Serial.print("DAQSTATE: ");
      Serial.println(DAQState);
    }

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
  String serialMessage = "";
  // TIME
  serialMessage.concat(millis());
  // FILTERED READINGS
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
  serialMessage.concat(incomingReadings.filteredReadings.PT_X);

  // serialMessage.concat(" ");
  // serialMessage.concat(incomingReadings.filteredReadings.TC_1);
  // serialMessage.concat(" ");
  // serialMessage.concat(incomingReadings.filteredReadings.TC_2);
  // serialMessage.concat(" ");
  // serialMessage.concat(incomingReadings.filteredReadings.TC_3);
  // serialMessage.concat(" ");
  // serialMessage.concat(incomingReadings.filteredReadings.TC_4);
  // RAW READINGS
  serialMessage.concat(" ");
  serialMessage.concat(incomingReadings.rawReadings.PT_O1);
  serialMessage.concat(" ");
  serialMessage.concat(incomingReadings.rawReadings.PT_O2);
  serialMessage.concat(" ");
  serialMessage.concat(incomingReadings.rawReadings.PT_E1);
  serialMessage.concat(" ");
  serialMessage.concat(incomingReadings.rawReadings.PT_E2);
  serialMessage.concat(" ");
  serialMessage.concat(incomingReadings.rawReadings.PT_C1);
  serialMessage.concat(" ");
  serialMessage.concat(incomingReadings.rawReadings.PT_X);
  // serialMessage.concat(" ");
  // serialMessage.concat(incomingReadings.rawReadings.TC_1);
  // serialMessage.concat(" ");
  // serialMessage.concat(incomingReadings.rawReadings.TC_2);
  // serialMessage.concat(" ");
  // serialMessage.concat(incomingReadings.rawReadings.TC_3);
  // serialMessage.concat(" ");
  // serialMessage.concat(incomingReadings.rawReadings.TC_4);
  
  // Serial.print("SIZEOFFIRST"); Serial.print(sizeof(serialMessage));
  serialMessage.concat(" ");
  serialMessage.concat(COMState);
  serialMessage.concat(" ");
  serialMessage.concat(DAQState);
  serialMessage.concat(" ");
  serialMessage.concat(FlightState);
  // PRESS STATUS
  serialMessage.concat(" ");
  serialMessage.concat(incomingDAQReadings.ethComplete ? "True" : "False");
  serialMessage.concat(" ");
  serialMessage.concat(incomingDAQReadings.oxComplete  ? "True" : "False");
  // AUTO ABORT STATUS
  serialMessage.concat(" ");
  serialMessage.concat(incomingReadings.AUTOABORT ? "True" : "False");
  // // // OR the bits in case either FLIGHT or DAQ is in Abort mode
  serialMessage.concat(" ");
  serialMessage.concat(incomingDAQReadings.ethVentComplete | incomingFlightReadings.ethVentComplete ? "True" : "False");
  serialMessage.concat(" ");
  serialMessage.concat(incomingDAQReadings.oxVentComplete | incomingFlightReadings.oxVentComplete ? "True" : "False");
  // // // FLIGHT QUEUE LENGTH
  serialMessage.concat(" ");
  serialMessage.concat(incomingReadings.FlightQueueLength);
  // // SD CARD STATUS
  //serialMessage.concat(" ");
  //serialMessage.concat(incomingReadings.sdCardInitialized ? "True" : "False");
  // PT Offsets
  serialMessage.concat(" ");
  serialMessage.concat(incomingReadings.pt_offsets.PT_O1_offset);
  serialMessage.concat(" ");
  serialMessage.concat(incomingReadings.pt_offsets.PT_O2_offset);
  serialMessage.concat(" ");
  serialMessage.concat(incomingReadings.pt_offsets.PT_E1_offset);
  serialMessage.concat(" ");
  serialMessage.concat(incomingReadings.pt_offsets.PT_E2_offset);
  serialMessage.concat(" ");
  serialMessage.concat(incomingReadings.pt_offsets.PT_C1_offset);
  serialMessage.concat(" ");
  serialMessage.concat(incomingReadings.pt_offsets.PT_X_offset);

  Serial.println(serialMessage);
}

void updateSendDataWithOffsets(int ptNumber, int newOffset) {
    if (ptNumber < 0 || ptNumber >= NUM_PTS) {
      return;
    }
    updatePTOffsets = true;
    switch (ptNumber) {
        case 0:
            sendCommands.pt_offsets.PT_O1_set = true;
            sendCommands.pt_offsets.PT_O1_offset = newOffset;
            break;
        case 1:
            sendCommands.pt_offsets.PT_O2_set = true;
            sendCommands.pt_offsets.PT_O2_offset = newOffset;
            break;
        case 2:
            sendCommands.pt_offsets.PT_E1_set = true;
            sendCommands.pt_offsets.PT_E1_offset = newOffset;
            break;
        case 3:
            sendCommands.pt_offsets.PT_E2_set = true;
            sendCommands.pt_offsets.PT_E2_offset = newOffset;
            break;
        case 4:
            sendCommands.pt_offsets.PT_C1_set = true;
            sendCommands.pt_offsets.PT_C1_offset = newOffset;
            break;
        case 5:
            sendCommands.pt_offsets.PT_X_set = true;
            sendCommands.pt_offsets.PT_X_offset = newOffset;
            break;
    }
}
