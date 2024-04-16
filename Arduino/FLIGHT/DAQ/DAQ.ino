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
#define DAQ_ID 1
#define FLIGHT_ID 2

// Set i2c address
PCF8575 pcf8575(0x20);

//::::::Global Variables::::::://

// DEBUG TRIGGER: SET TO 1 FOR DEBUG MODE.
// MOSFET must not trigger while in debug.
bool DEBUG = false;       // Simulate LOX and Eth fill.
bool WIFIDEBUG = false;  // Don't send/receive data.

#define SIMULATION_DELAY 25

// MODEL DEFINED PARAMETERS FOR TEST/HOTFIRE. Pressures in psi //
float pressureFuel = 390;  //405;  // Set pressure for fuel: 412
float pressureOx = 450;    //460;  // Set pressure for lox: 445
float threshold = 0.995;   // re-psressurrization threshold (/1x)
float ventTo = 5;          // c2se solenoids at this pressure to preserve lifetime.
#define abortPressure 525  // Cutoff pressure to automatically trigger abort
// refer to https://docs.google.com/spreadsheets/d/17NrJWC0AR4Gjejme-EYuIJ5uvEJ98FuyQfYVWI3Qlio/edit#gid=1185803967 for all pinouts

#define ABORT_ACTIVATION_DELAY 500 // Number of milliseconds to wait at high pressure before activating abort

// GPIO expander
#define I2C_SDA 21
#define I2C_SCL 22

////////////////////////////// MOSFETS ///////////////////////////////////////////////////////////////////
#define MOSFET_ETH_MAIN 10    
#define MOSFET_ETH_PRESS 6   
#define MOSFET_VENT_ETH 12   
#define MOSFET_QD_MUSCLE 7      
#define MOSFET_IGNITER 8     
#define MOSFET_LOX_MAIN 9    
#define MOSFET_LOX_PRESS 4  
#define MOSFET_VENT_LOX 11   
#define MOSFET_ETH_LINE_VENT 5     
#define MOSFET_LOX_LINE_VENT 3     


// Initialize mosfets' io expander.
//#define MOSFET_PCF_ADDR 0x20
//EasyPCF8575 mosfet_pcf;
bool mosfet_pcf_found;

//::::::STATE VARIABLES::::::://
enum STATES { IDLE,
              ARMED,
              PRESS,
              QD,
              IGNITION,
              HOTFIRE,
              ABORT };

String state_names[] = { "Idle", "Armed", "Press", "QD", "Ignition", "HOTFIRE", "Abort" };

int DAQState = IDLE;
int COMState = IDLE;
int FlightState = IDLE;

bool ethComplete = false;
bool oxComplete = false;
bool oxVentComplete = false;
bool ethVentComplete = false;

int hotfireStart;

#define SEND_DELAY 20
float sendTime = 0;
bool flight_toggle = false;

// Structure example to send data.
// Must match the receiver structure.
struct struct_readings {
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
};

struct struct_message {
  int messageTime;
  int sender;
  struct_readings rawReadings;
  struct_readings filteredReadings;
  int COMState;
  int DAQState;
  int FlightState;
  short int FlightToDAQQueueLength;
  short int FlightToCOMQueueLength;
  bool ethComplete;
  bool oxComplete;
  bool oxVentComplete;
  bool ethVentComplete;
};

esp_now_peer_info_t peerInfo;

unsigned long QD_start_time;

// Struct that holds data being sent out
struct_message outgoingData;


// Create a struct_message to hold incoming data
struct_message incomingCOMReadings;
struct_message FLIGHT;

uint8_t COMBroadcastAddress[] = { 0xEC, 0x64, 0xC9, 0x86, 0x1E, 0x4C };
uint8_t FlightBroadcastAddress[] = { 0xE8, 0x6B, 0xEA, 0xD4, 0x10, 0x4C };

// Callback when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  struct_message Packet;
  memcpy(&Packet, incomingData, sizeof(Packet));

  if (Packet.sender == COM_ID) {
    incomingCOMReadings = Packet;
    COMState = Packet.COMState;
  } else if (Packet.sender == FLIGHT_ID) {
    FLIGHT = Packet;
    FlightState = FLIGHT.FlightState;
    Serial.println(FlightState);
    flight_toggle = true; //set flag up to send data to COM
    Serial.println("Data: ");
  }
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

  esp_now_register_recv_cb(OnDataRecv);

  // Register peer
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  memcpy(peerInfo.peer_addr, FlightBroadcastAddress, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  memcpy(peerInfo.peer_addr, COMBroadcastAddress, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received

  DAQState = IDLE;

  Serial.println("Finished setup");
}


int cumulativeAbortTime = 0; // How long we have been in high-pressure state
int lastAbortCheckTime = -1; // Last time we called checkAbort()
void checkAbort() {
  if (lastAbortCheckTime == -1) {
    lastAbortCheckTime = millis();
    return;
  }

  int deltaTime = millis() - lastAbortCheckTime;
  if (FLIGHT.filteredReadings.PT_O1 >= abortPressure || FLIGHT.filteredReadings.PT_E1 >= abortPressure) {
    cumulativeAbortTime += deltaTime;
  }
  else {
    cumulativeAbortTime = max(cumulativeAbortTime - deltaTime, 0);
  }
  lastAbortCheckTime = millis();

  if (COMState == ABORT || cumulativeAbortTime >= ABORT_ACTIVATION_DELAY) {
    abort_sequence();
    DAQState = ABORT;
  }

}


void loop() {
  
  if (DEBUG || COMState == ABORT) { //check abort
    syncDAQState();
  }

  // if flight data received, relay to COM
  // We also relay data if we are in Abort; this is in case of an emergency where
  // Flight breaks; we still want COM to receive updates from DAQ
  if (flight_toggle == true || DAQState == ABORT) {
    sendTime = millis();
    sendData(COMBroadcastAddress);
    flight_toggle = false; //reset toggle
  }

  if (DAQState != FLIGHT.DAQState) { //if state discrepancy, send commands to flight
    delay(5); //delay so voltage can recover; should change to something better
    sendData(FlightBroadcastAddress); 
    // Serial.print("sdhgklsdhfljksf");
  }

  ///////////// STATE MACHINE ///////////
  switch (DAQState) { //CHANGE STATES BASED ON DATA RECEIVED(ondatarecv) FROM COM
    case (IDLE):
      if (COMState == ARMED) { syncDAQState(); }
      idle();
      break;

    case (ARMED):  // NEED TO ADD TO CASE OPTIONS //ALLOWS OTHER CASES TO TRIGGER //INITIATE TANK PRESS LIVE READINGS
      if (COMState == IDLE || COMState == PRESS) { syncDAQState(); }
      armed();
      break;

    case (PRESS):
      if (COMState == IDLE || (COMState == QD)) {
        syncDAQState();
        QD_start_time = millis();
        mosfetCloseAllValves();
      }
      press();
      break;

    case (QD):
      if (COMState == IDLE || COMState == IGNITION) {
        syncDAQState();
        mosfetCloseAllValves();
      }
      quick_disconnect();
      break;

    case (IGNITION):
      if (COMState == IDLE || COMState == HOTFIRE) {
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
      if (COMState == IDLE && oxVentComplete && ethVentComplete) { syncDAQState(); }
      break;
  }
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
  mosfetCloseAllValves();
}

void press() {
  if (FLIGHT.filteredReadings.PT_O1 < pressureOx * threshold) {
    oxComplete = false;
    mosfetOpenValve(MOSFET_LOX_PRESS);
    if (DEBUG) {
      FLIGHT.filteredReadings.PT_O1 += (0.00075 * SIMULATION_DELAY);
    }
  } else if (FLIGHT.filteredReadings.PT_O1 >= pressureOx) {
    mosfetCloseValve(MOSFET_LOX_PRESS);
    oxComplete = true;
  }
  if (FLIGHT.filteredReadings.PT_E1 < pressureFuel * threshold) {
    ethComplete = false;
    mosfetOpenValve(MOSFET_ETH_PRESS);
    if (DEBUG) {
      FLIGHT.filteredReadings.PT_E1 += (0.001 * SIMULATION_DELAY);
    }
  } else if (FLIGHT.filteredReadings.PT_E1 >= pressureFuel) {
    mosfetCloseValve(MOSFET_ETH_PRESS);
    ethComplete = true;
  }
  checkAbort();
}

// Disconnect harnessings and check state of rocket.
void quick_disconnect() {
  if (millis() - QD_start_time <= 1000){
    mosfetCloseValve(MOSFET_ETH_PRESS);  //close press valves
    mosfetCloseValve(MOSFET_LOX_PRESS);
  }
  if (millis() - QD_start_time > 1000 && millis() - QD_start_time <= 2000){
    mosfetOpenValve(MOSFET_ETH_LINE_VENT);
    mosfetOpenValve(MOSFET_LOX_LINE_VENT);
  }
  if (millis() - QD_start_time > 2000){
    mosfetOpenValve(MOSFET_QD_MUSCLE);
  }
  checkAbort();
}

void ignition() {
  mosfetOpenValve(MOSFET_IGNITER);
}

void hotfire() {
  mosfetCloseValve(MOSFET_IGNITER);
  mosfetOpenValve(MOSFET_ETH_MAIN);
  if (millis() >= hotfireStart + 5) { //delay lox command by 5ms to bring propellant injection closer together. Still, lox leads into chamber
    mosfetOpenValve(MOSFET_LOX_MAIN);
    // Serial.print(hotfireStart);
  }
}

void abort_sequence() {
  mosfetOpenValve(MOSFET_VENT_LOX);
  mosfetOpenValve(MOSFET_VENT_ETH);
  // Waits for LOX pressure to decrease before venting Eth through pyro
  mosfetCloseValve(MOSFET_LOX_PRESS);
  mosfetCloseValve(MOSFET_ETH_PRESS);
  mosfetCloseValve(MOSFET_LOX_MAIN);
  mosfetCloseValve(MOSFET_ETH_MAIN);
  mosfetCloseValve(MOSFET_IGNITER);

  int currtime = millis();

  if (FLIGHT.filteredReadings.PT_O1 > ventTo) {  // vent only lox down to vent to pressure
    mosfetOpenValve(MOSFET_VENT_LOX);
    if (DEBUG) {
      FLIGHT.filteredReadings.PT_O1 = FLIGHT.filteredReadings.PT_O1 - (0.0005 * SIMULATION_DELAY);
    }
  } else {                              // lox vented to acceptable hold pressure
    mosfetCloseValve(MOSFET_VENT_LOX);  // close lox
    oxVentComplete = true;
  }
  if (FLIGHT.filteredReadings.PT_E1 > ventTo) {
    mosfetOpenValve(MOSFET_VENT_ETH);  // vent ethanol
    if (DEBUG) {
      FLIGHT.filteredReadings.PT_E1 = FLIGHT.filteredReadings.PT_E1 - (0.0005 * SIMULATION_DELAY);
    }
  } else {
    mosfetCloseValve(MOSFET_VENT_ETH);
    ethVentComplete = true;
  }
}

// Sync state of DAQ board with COM board.
void syncDAQState() {
  // Sync with COM only if DAQ doesn't detect an Abort, and COM doesn't want us to go back to Idle after Abort
  if (DAQState != ABORT || COMState == IDLE) {
    DAQState = COMState;
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


//////////////////// COMMUNICATION /////////////////////////////

void sendData(uint8_t broadcastAddress[]) {
  if (WIFIDEBUG) {
    return;
  }
  updateDataPacket();
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&outgoingData, sizeof(outgoingData));
  if (result != ESP_OK) {
    Serial.println("Error sending the data");
  }
}


void updateDataPacket() {
  outgoingData.messageTime = millis();
  outgoingData.sender = DAQ_ID;
  outgoingData.COMState = COMState;
  outgoingData.DAQState = DAQState;
  outgoingData.ethComplete = ethComplete;
  outgoingData.oxComplete = oxComplete;
  outgoingData.oxVentComplete = oxVentComplete;
  outgoingData.ethVentComplete = ethVentComplete;
}
