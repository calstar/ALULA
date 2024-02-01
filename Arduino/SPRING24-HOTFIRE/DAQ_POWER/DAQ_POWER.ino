/*
11-7-2023


This code runs on the DAQ ESP32 and has a couple of main tasks.
1. Read sensor data
2. Send sensor data to COM ESP32
3. Actuate hotfire sequence
*/

//::::::Libraries::::::://
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <SPI.h>
#include "HX711.h"
#include "Adafruit_MAX31855.h"
#include <EasyPCF8575.h>
#include "RunningMedian.h"
#include "PCF8575.h"  // https://github.com/xreef/PCF8575_library

#define COM_ID 0
#define DAQ_SENSE_ID 1
#define DAQ_POWER_ID 2

// Set i2c address
PCF8575 pcf8575(0x20);

//::::::Global Variables::::::://

// DEBUG TRIGGER: SET TO 1 FOR DEBUG MODE.
// MOSFET must not trigger while in debug.
bool DEBUG = true;   // Simulate LOX and Eth fill.
bool WIFIDEBUG = true;  // Don't send/receive data.

// MODEL DEFINED PARAMETERS FOR TEST/HOTFIRE. Pressures in psi //
float pressureFuel = 390;   //405;  // Set pressure for fuel: 412
float pressureOx = 450;     //460;  // Set pressure for lox: 445
float threshold = 0.995;   // re-psressurrization threshold (/1x)
float ventTo = 5;          // c2se solenoids at this pressure to preserve lifetime.
#define abortPressure 525  // Cutoff pressure to automatically trigger abort
#define period 0.5         // Sets period for bang-bang control
// refer to https://docs.google.com/spreadsheets/d/17NrJWC0AR4Gjejme-EYuIJ5uvEJ98FuyQfYVWI3Qlio/edit#gid=1185803967 for all pinouts

// GPIO expander
#define I2C_SDA 21
#define I2C_SCL 22

////////////////////////////// MOSFETS ///////////////////////////////////////////////////////////////////
#define MOSFET_ETH_MAIN 7    //P07
#define MOSFET_ETH_PRESS 6   //P06
#define MOSFET_VENT_ETH 5    //P05
#define MOSFET_EXTRA 4       //CAN USE THIS PIN FOR ANYTHING JUST CHANGE ASSIGNMENT AND HARNESS
#define MOSFET_QD_LOX 3      //P03
#define MOSFET_IGNITER 8     //P10
#define MOSFET_LOX_MAIN 9    //P11
#define MOSFET_LOX_PRESS 10  //P12
#define MOSFET_VENT_LOX 11   //P13
#define MOSFET_QD_ETH 12     //P14


// Initialize mosfets' io expander.
//#define MOSFET_PCF_ADDR 0x20
//EasyPCF8575 mosfet_pcf;
bool mosfet_pcf_found;

//::::::STATE VARIABLES::::::://
enum STATES { IDLE, ARMED, PRESS, QD, IGNITION, HOTFIRE, ABORT };

String state_names[] = { "Idle", "Armed", "Press", "QD", "Ignition", "HOTFIRE", "Abort" };

int DAQPowerState = IDLE;
int DAQSenseState = IDLE;

bool ethComplete = false;
bool oxComplete = false;
bool oxVentComplete = false;
bool ethVentComplete = false;

int hotfireStart;

// Structure example to send data.
// Must match the receiver structure.
struct struct_message {
  int messageTime;
  int sender;
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
  int DAQSenseState;
  int DAQPowerState;
  short int COMQueueLength;
  short int DAQPowerQueueLength;
  bool ethComplete;
  bool oxComplete;
  bool oxVentComplete;
  bool ethVentComplete;
};

esp_now_peer_info_t peerInfo;

// Struct that holds data being sent out
struct_message dataPacket;

// Create a struct_message to hold incoming data
struct_message DAQSenseCommands;

uint8_t DAQSenseBroadcastAddress[] = {0xB0, 0xA7, 0x32, 0xDE, 0xC1, 0xFC};

// Callback when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&DAQSenseCommands, incomingData, sizeof(DAQSenseCommands));
  DAQSenseState = DAQSenseCommands.DAQSenseState;
}

// Initialize all sensors and parameters.
void setup() {
  Serial.begin(115200);

  while (!Serial) delay(1);  // wait for Serial on Leonardo/Zero, etc.

  // MOSFET.
  //  mosfet_pcf.startI2C(I2C_SDA, I2C_SCL, MOSFET_PCF_ADDR); // Only SEARCH, if using normal pins in Arduino
  mosfet_pcf_found = true;

  // Set pinMode to OUTPUT
  for (int i = 0; i < 16; i++) {
    pcf8575.pinMode(i, OUTPUT);
  }
  pcf8575.begin();
  mosfet_pcf_found = true;
  mosfetCloseAllValves();  // make sure everything is off by default (NMOS: Down = Off, Up = On)
  delay(500);              // startup time to make sure its good for personal testing

  // Broadcast setup.
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  // Print MAC Accress on startup for easier connections
  Serial.print("My MAC Address: ");
  Serial.println(WiFi.macAddress());

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register peer
  memcpy(peerInfo.peer_addr, DAQSenseBroadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

  DAQPowerState = IDLE;
}


//::::::STATE MACHINE::::::://

// Main Structure of State Machine.
void loop() {
  if (DEBUG || DAQSenseState == ABORT) {
    syncDAQState();
  }
  switch (DAQPowerState) {
    case (IDLE):
      if (DAQSenseState == ARMED) { syncDAQState(); }
      idle();
      break;

    case (ARMED):  // NEED TO ADD TO CASE OPTIONS //ALLOWS OTHER CASES TO TRIGGER //INITIATE TANK PRESS LIVE READINGS
      if (DAQSenseState == IDLE || DAQSenseState == PRESS) { syncDAQState(); }
      armed();
      break;

    case (PRESS):
      if (DAQSenseState == IDLE || (DAQSenseState== QD)) {
        syncDAQState();
        int QDStart = millis();
        mosfetCloseAllValves();
      }
      press();
      break;

    case (QD):
      if (DAQSenseState == IDLE || DAQSenseState == IGNITION) {
        syncDAQState();
        mosfetCloseAllValves();
      }
      quick_disconnect();
      break;

    case (IGNITION):
      if (DAQSenseState == IDLE || DAQSenseState == HOTFIRE) {
        syncDAQState();
        hotfireStart = millis();
      }
      ignition();
      break;

    case (HOTFIRE):
      hotfire();
      break;

    case (ABORT):
      abort_sequence();
      if (DAQSenseState == IDLE && oxVentComplete && ethVentComplete) { syncDAQState(); }
      break;
  }
  sendData(DAQSenseBroadcastAddress);
}

// State Functions.

// Everything should be off.
void reset() {
  oxComplete = false;
  ethComplete = false;
  oxVentComplete = false;
  ethVentComplete = false;
}

void idle() {
  mosfetCloseAllValves();
  reset();  // must set oxComplete and ethComplete to false!
}

// Oxygen and fuel should not flow yet.
// This function is the same as idle?
void armed() {
  // mosfetCloseValve(MOSFET_LOX_PRESS);
  // mosfetCloseValve(MOSFET_ETH_PRESS);
  mosfetCloseAllValves();
}

void press() {
  /*
  * TODO: HOW DO WE PREVENT FILL SEQUENCE FROM ENDING AFTER SPIKE IN PRESSURE VALUE??
  * TODO: HOW DO WE PREVENT FILL SEQUENCE FROM ENDING AFTER SPIKE IN PRESSURE VALUE??
  * TODO: HOW DO WE PREVENT FILL SEQUENCE FROM ENDING AFTER SPIKE IN PRESSURE VALUE??
  * TODO: HOW DO WE PREVENT FILL SEQUENCE FROM ENDING AFTER SPIKE IN PRESSURE VALUE??
  * TODO: HOW DO WE PREVENT FILL SEQUENCE FROM ENDING AFTER SPIKE IN PRESSURE VALUE??
  * TODO: HOW DO WE PREVENT FILL SEQUENCE FROM ENDING AFTER SPIKE IN PRESSURE VALUE??
  * TODO: HOW DO WE PREVENT FILL SEQUENCE FROM ENDING AFTER SPIKE IN PRESSURE VALUE??
  * TODO: HOW DO WE PREVENT FILL SEQUENCE FROM ENDING AFTER SPIKE IN PRESSURE VALUE??
  */
  if (!(oxComplete && ethComplete)) {
    if (DAQSenseCommands.PT_O1 < pressureOx * threshold) {
      mosfetOpenValve(MOSFET_LOX_PRESS);
    }
    else {
      mosfetCloseValve(MOSFET_LOX_PRESS);
      oxComplete = true;
    }
    if (DAQSenseCommands.PT_E1 < pressureFuel * threshold) {
      mosfetOpenValve(MOSFET_ETH_PRESS);
    }
    else {
      mosfetCloseValve(MOSFET_ETH_PRESS);
      ethComplete = true;
    }
  }
  CheckAbort();
}

// Disconnect harnessings and check state of rocket.
void quick_disconnect() {
  mosfetCloseValve(MOSFET_ETH_PRESS); //close press valves
  mosfetCloseValve(MOSFET_LOX_PRESS);

  // vent valves/vent the lines themselves
  // vent the pressure solenoid for 1 full second
  //if millis() >= (QDStart+1000){
  // then, disconnect the lines from the rocket itself
  // }
  CheckAbort();
}

void ignition() {
  mosfetOpenValve(MOSFET_IGNITER);
}

void hotfire() {
  mosfetCloseValve(MOSFET_IGNITER);
  mosfetOpenValve(MOSFET_ETH_MAIN);
  //
  //  if (millis() >= hotfireStart+3000) {
  //    mosfetCloseValve(MOSFET_LOX_MAIN);
  //  } else {

  if (millis() >= hotfireStart + 5) {
    mosfetOpenValve(MOSFET_LOX_MAIN);
    // Serial.print(hotfireStart);
  }
  //  }
}

void abort_sequence() {
  /*
  * TODO: DOES ABORT SEQUENCE END AFTER SPIKE IN PRESSURE VALUE??
  * TODO: DOES ABORT SEQUENCE END AFTER SPIKE IN PRESSURE VALUE??
  * TODO: DOES ABORT SEQUENCE END AFTER SPIKE IN PRESSURE VALUE??
  * TODO: DOES ABORT SEQUENCE END AFTER SPIKE IN PRESSURE VALUE??
  * TODO: DOES ABORT SEQUENCE END AFTER SPIKE IN PRESSURE VALUE??
  * TODO: DOES ABORT SEQUENCE END AFTER SPIKE IN PRESSURE VALUE??
  * TODO: DOES ABORT SEQUENCE END AFTER SPIKE IN PRESSURE VALUE??
  * TODO: DOES ABORT SEQUENCE END AFTER SPIKE IN PRESSURE VALUE??
  */
  if (DEBUG) {
    mosfetOpenValve(MOSFET_VENT_LOX);
    mosfetOpenValve(MOSFET_VENT_ETH);
//    delay(50);
  }
  // Waits for LOX pressure to decrease before venting Eth through pyro
  mosfetCloseValve(MOSFET_LOX_PRESS);
  mosfetCloseValve(MOSFET_ETH_PRESS);
  mosfetCloseValve(MOSFET_LOX_MAIN);
  mosfetCloseValve(MOSFET_ETH_MAIN);
  mosfetCloseValve(MOSFET_IGNITER);
  //
  int currtime = millis();
  if (DAQSenseCommands.PT_O1 > 1.3 * ventTo) {  // 1.3 is magic number.
    oxVentComplete = false;
  }
  if (DAQSenseCommands.PT_E1 > 1.3 * ventTo) {  // 1.3 is magic number.
    ethVentComplete = false;
  }

  if (!(oxVentComplete && ethVentComplete)) {
    if (DAQSenseCommands.PT_O1 > ventTo) {  // vent only lox down to vent to pressure
      mosfetOpenValve(MOSFET_VENT_LOX);
    }
    else {                              // lox vented to acceptable hold pressure
      mosfetCloseValve(MOSFET_VENT_LOX);  // close lox
      oxVentComplete = true;
    }
    if (DAQSenseCommands.PT_E1 > ventTo) {
      mosfetOpenValve(MOSFET_VENT_ETH);  // vent ethanol
    } 
    else {
      mosfetCloseValve(MOSFET_VENT_ETH);
      ethVentComplete = true;
    }
  }
}

// Sync state of DAQ board with COM board.
void syncDAQState() {
  DAQPowerState = DAQSenseState;
}

void CheckAbort() {
  if (DAQSenseState == ABORT || DAQSenseCommands.PT_O1 >= abortPressure || DAQSenseCommands.PT_E1 >= abortPressure) {
    mosfetCloseValve(MOSFET_ETH_PRESS);
    mosfetCloseValve(MOSFET_LOX_PRESS);
    DAQPowerState = ABORT;
  }
}

void mosfetCloseAllValves() {
  if (mosfet_pcf_found /*&& !DEBUG*/) {
    for (int i = 0; i < 16; i++) {
      pcf8575.digitalWrite(i, LOW);
    }
  }
}

void mosfetCloseValve(int num) {
  if (mosfet_pcf_found /* && !DEBUG*/) {
    pcf8575.digitalWrite(num, LOW);
  }
}

void mosfetOpenValve(int num) {
  if (mosfet_pcf_found) {
    pcf8575.digitalWrite(num, HIGH);
  }
}

void sendData(uint8_t broadcastAddress[]) {
  if (WIFIDEBUG) {
    return;
  }

  updateDataPacket();
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&dataPacket, sizeof(dataPacket));

  if (result != ESP_OK) {
    Serial.println("Error sending the data");
  }
}

void updateDataPacket() {
  dataPacket.messageTime = millis();
  dataPacket.sender = DAQ_POWER_ID;
  dataPacket.DAQPowerState = DAQPowerState;
  dataPacket.ethComplete = ethComplete;
  dataPacket.oxComplete = oxComplete;
  dataPacket.oxVentComplete = oxVentComplete;
  dataPacket.ethVentComplete = ethVentComplete;
}