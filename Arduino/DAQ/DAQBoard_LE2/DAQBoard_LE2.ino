/*
This code runs on the DAQ ESP32 and has a couple of main tasks.
1. Read sensor data
2. Send sensor data to COM ESP32
3. Actuate hotfire sequence
*/

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Arduino.h>
#include "HX711.h"
#define IDLE_DELAY 250
#define GEN_DELAY 25

// USER DEFINED PARAMETERS FOR TEST/HOTFIRE //
#define BangBangPressFuel 425 //Indicates transition from normal fill to bang-bang
#define pressureFuel 450    //In units of psi. Defines set pressure for fuel
#define BangBangPressOx 425 //Indicates transition from normal fill to bang-bang
#define pressureOx 450    //In units of psi. Defines set pressure for ox
#define abortPressure 525 //Cutoff pressure to automatically trigger abort
#define period 0.5   //Sets period for bang-bang control
float sendDelay = 250; //Sets frequency of data collection. 1/(sendDelay*10^-3) is frequency in Hz
// END OF USER DEFINED PARAMETERS //
//refer to https://docs.google.com/spreadsheets/d/17NrJWC0AR4Gjejme-EYuIJ5uvEJ98FuyQfYVWI3Qlio/edit#gid=1185803967 for all pinouts
// Set sensor pinouts 
// USED FOR LOX TANK(gain 32) AND ETH TANK(gain 64)
#define PTOUT1 36
#define CLKPT1 27
float PT_Tanks_Offset_LOX = 10.663;
float PT_Tanks_Slope_LOX = 0.0001181;
float PT_Tanks_Offset_ETH = 10.663;
float PT_Tanks_Slope_ETH = 0.0001181;


// USED FOR THE DOWNSTREAM PTs ON LOX(gain 32) AND ETH(gain 64) sides
#define PTOUT2 39
#define CLKPT2 27
float PT_Down_Offset_LOX = 10.663;
float PT_Down_Slope_LOX = 0.0001181;
float PT_Down_Offset_ETH = 10.663;
float PT_Down_Slope_ETH = 0.0001181;

// USED FOR CHAMBER PRESSURE(gain 32) AND LOADCELL1(gain 64)
#define PTOUT3 34
#define CLKPT3 27
float PT_Chamber_Offset = 10.663;
float PT_Chamber_Slope = 0.0001181;
float LoadCell1_Offset = 10.663;
float LoadCell1_Slope = 0.0001181;


// USED FOR LOADCELL2(gain 32) and LOADCELL3(gain 64)
#define PTOUT4 35
#define CLKPT4 27
float LoadCell2_Offset = 10.663;
float LoadCell2_Slope = 0.0001181;
float LoadCell3_Offset = 10.663;
float LoadCell3_Slope = 0.0001181;

#define TC1 33
#define TC2 25
#define TCCLK 4

// #define CAPSENS1DATA 40
// #define CAPSENS1CLK 52
// #define CAPSENS2DATA 13
// #define CAPSENS2CLK 21
// End of sensor pinouts //

// Relays pinouts. Use for solenoidPinOx, oxSolVent, oxQD, and fuelQD // 
#define RELAYPIN_VENT 21
#define RELAYPIN_QD_ETH 23
#define RELAYPIN_QD_OX 22
#define RELAYPIN_PRESSLOX 19
#define RELAYPIN_PRESSETH 18
#define RELAYPIN_IGNITER 5
#define RELAYPIN_PYROLOX 17
#define RELAYPIN_PYROETH 16
// End of relay pinouts //

String serialMessage;

// Initialize the PT and LC sensor objects which use the HX711 breakout board
HX711 scale1;
HX711 scale2;
HX711 scale3;
HX711 scale4;
// End of HX711 initialization

//////////////
//IMPORTANT//
/////////////
// REPLACE WITH THE MAC Address of your receiver

//OLD COM BOARD {0xC4, 0xDD, 0x57, 0x9E, 0x91, 0x6C}
//COM BOARD {0x7C, 0x9E, 0xBD, 0xD7, 0x2B, 0xE8}
//HEADERLESS BOARD {0x7C, 0x87, 0xCE, 0xF0 0x69, 0xAC}
//NEWEST COM BOARD IN EVA {0x24, 0x62, 0xAB, 0xD2, 0x85, 0xDC}
// uint8_t broadcastAddress[] = {0x24, 0x62, 0xAB, 0xD2, 0x85, 0xDC};
uint8_t broadcastAddress[] ={0x7C, 0x9E, 0xBD, 0xD7, 0x2B, 0xE8};
// {0x7C, 0x87, 0xCE, 0xF0, 0x69, 0xAC};
//{0x3C, 0x61, 0x05, 0x4A, 0xD5, 0xE0};
// {0xC4, 0xDD, 0x57, 0x9E, 0x96, 0x34}

//STATEFLOW VARIABLES
enum STATES {IDLE, ARMED, PRESS, QD, IGNITION, HOTFIRE, ABORT, DEBUG=99};
int state;

short int queueLength=0;
int commandedState;
int currDAQState;
bool ethComplete = false;
bool oxComplete = false;
bool pressComplete = false ;

float sendTime;
bool gainbool = true; //gainbool == true => gain = 64; else gain = 32
// Define variables to store readings to be sent
int readingPT1=1;
int readingPT2=1;
int readingPT3=1;
int readingPT4=1;
int readingPT5=1;
int readingLC1=1;
int readingLC2=1;
int readingLC3=1;
int readingTC1=1;
int readingTC2=1;
float readingCap1=0;
float readingCap2=0;
short int queueSize=0;

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
    int commandedState;
    int DAQState;
    short int queueSize;
    bool pressComplete;
    bool ethComplete;
    bool oxComplete;
    // bool oxvent;
    // bool ethVent;
    // bool VentComplete;
} struct_message;

// Create a struct_message called Readings to hold sensor readings
struct_message Readings;
//create a queue for readings in case
struct_message ReadingsQueue[120];

// Create a struct_message to hold incoming commands
struct_message Commands;

esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  sendTime = millis();
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&Commands, incomingData, sizeof(Commands));
  commandedState = Commands.commandedState;
}

// Initialize all sensors and parameters
void setup() {
  // pinMode(ONBOARD_LED,OUTPUT);
  Serial.begin(115200);

  pinMode(RELAYPIN_VENT, OUTPUT);
  pinMode(RELAYPIN_QD_OX, OUTPUT);
  pinMode(RELAYPIN_QD_ETH, OUTPUT);
  pinMode(RELAYPIN_PRESSETH, OUTPUT);
  pinMode(RELAYPIN_PRESSLOX, OUTPUT);
  pinMode(RELAYPIN_IGNITER, OUTPUT);
  pinMode(RELAYPIN_PYROETH, OUTPUT);
  pinMode(RELAYPIN_PYROLOX, OUTPUT);

  //EVERYTHING SHOULD BE WRITTEN HIGH EXCEPT QDs, WHICH SHOULD BE LOW
  digitalWrite(RELAYPIN_VENT, HIGH);
  digitalWrite(RELAYPIN_QD_OX, LOW);
  digitalWrite(RELAYPIN_QD_ETH, LOW);
  digitalWrite(RELAYPIN_PRESSETH, HIGH);
  digitalWrite(RELAYPIN_PRESSLOX, HIGH);
  digitalWrite(RELAYPIN_IGNITER, HIGH);
  digitalWrite(RELAYPIN_PYROETH, HIGH);
  digitalWrite(RELAYPIN_PYROLOX, HIGH);

  //set gains for pt pins
  scale1.begin(PTOUT1, CLKPT1); scale1.set_gain(64);
  scale2.begin(PTOUT2, CLKPT2); scale2.set_gain(64);
  scale3.begin(PTOUT3, CLKPT3); scale3.set_gain(64);
  scale4.begin(PTOUT4, CLKPT4); scale4.set_gain(64);
  

  pinMode(TC1, INPUT);
  pinMode(TC2, INPUT);

  // pinMode(CAPSENS1DATA, INPUT);
  // pinMode(CAPSENS1CLK, OUTPUT);
  // pinMode(CAPSENS2DATA, INPUT);
  // pinMode(CAPSENS2CLK, OUTPUT);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  //Print MAC Accress on startup for easier connections
  Serial.println(WiFi.macAddress());

  // Init ESP-NOWf
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
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

  sendTime = millis();
  state = IDLE;
  currDAQState = IDLE;
}

// Implementation of State Machine
void loop() {

  switch (state) {

  case (IDLE):
    sendDelay = IDLE_DELAY;
    idle();
    if (commandedState==ABORT) {state=ABORT; currDAQState=ABORT;}
    if (commandedState==ARMED) {state=ARMED; currDAQState=ARMED;}
    
    break;

  case (ARMED): //NEED TO ADD TO CASE OPTIONS //ALLOWS OTHER CASES TO TRIGGER //INITIATE TANK PRESS LIVE READINGS
    sendDelay = GEN_DELAY;
    armed();
    // if (commandedState==IDLE) {state=IDLE; currDAQState=IDLE;}
    if (commandedState==ABORT) {state=ABORT; currDAQState=ABORT;}
    if (commandedState==PRESS) {state=PRESS; currDAQState=PRESS;}
    
    break;

  case (PRESS):
    sendDelay = GEN_DELAY;
    pressComplete = press();
    if (commandedState==ABORT) {state=ABORT; currDAQState=ABORT;}
    if (pressComplete && commandedState==QD) {state=QD; currDAQState=QD;}
    if (pressComplete && commandedState==IGNITION) {state=IGNITION; currDAQState=IGNITION;}
    
    break;

  case (QD):
    sendDelay = GEN_DELAY;
    quick_disconnect();
    if (commandedState==ABORT) {state=ABORT; currDAQState=ABORT;}
    if (commandedState==IGNITION) {state=IGNITION; currDAQState=IGNITION;}
    
    
    break;

  case (IGNITION): 
    sendDelay = GEN_DELAY;
    ignition();
    if (commandedState==ABORT) {state=ABORT; currDAQState=ABORT;}
    if (commandedState==HOTFIRE) {state=HOTFIRE; currDAQState=HOTFIRE;}
    
    
    break;

  case (HOTFIRE): 
    sendDelay = GEN_DELAY;
    hotfire();
    if (commandedState==ABORT) {state=ABORT; currDAQState=ABORT;}
    break;

  case (ABORT):
    sendDelay = GEN_DELAY;
    abort_sequence();
    break;

  case (DEBUG):
    sendDelay = GEN_DELAY;
    debug();
    if (commandedState==IDLE) {state=IDLE; currDAQState=IDLE;}
    break;
  }
}


/// STATE FUNCTION DEFINITIONS ///

void idle() {
  sendData();
}

void armed() {
  sendData();
}

bool press() {
  sendDelay = 25;
  oxComplete = (readingPT1 >= pressureOx);
  ethComplete = (readingPT2 >= pressureFuel);

  if (!sendData()) {
    getReadings();
  }
  while (readingPT1 < BangBangPressOx && readingPT2 < BangBangPressFuel) {
    openSolenoidOx();
    openSolenoidFuel();
    if (commandedState == ABORT) {
      closeSolenoidOx();
      closeSolenoidFuel();
      state = ABORT;
      return false;
    }
    if (!sendData()) {
      getReadings();
    }    
  }

  if (readingPT1 < BangBangPressOx && readingPT2 > BangBangPressFuel) {
    closeSolenoidFuel();
    while (readingPT1 < BangBangPressOx && readingPT2 < pressureFuel) {
      openSolenoidOx();
      if (commandedState == ABORT) {
        closeSolenoidOx();
        state = ABORT;
        return false;
      }
      if (!sendData()) {
        getReadings();
      }
    }
  }
  if (readingPT1 > BangBangPressOx && readingPT2 < BangBangPressFuel) {
    closeSolenoidOx();
    while (readingPT2 < BangBangPressFuel && readingPT1 < pressureOx) {
      openSolenoidFuel();
      if (commandedState == ABORT) {
        closeSolenoidFuel();
        state = ABORT;
        return false;
      }
      if (!sendData()) {
        getReadings();
      }
    }    
  }

  while (readingPT1 < pressureOx || readingPT2 < pressureFuel) {
    if (readingPT1 >= abortPressure || readingPT2 >= abortPressure) {
      closeSolenoidFuel();
      closeSolenoidOx();
      state = ABORT;
      return false;
    }
    if (readingPT1 >= pressureOx) {
      closeSolenoidOx();
      oxComplete = true;
    }
    if (readingPT2 >= pressureFuel) {
      closeSolenoidFuel();
      ethComplete = true;
    }
    if (commandedState == ABORT) {
      closeSolenoidOx();
      closeSolenoidFuel();
      state = ABORT;
      return false;      
    }
    if (!oxComplete) {
      openSolenoidOx();
    }
    if (!ethComplete) {
      openSolenoidFuel();
    }
    delay(period * 500);
    closeSolenoidOx();
    closeSolenoidFuel();
    delay((1-period) * 500);
    if (!sendData()) {
      getReadings();
    }
  }
  return true;
}

void quick_disconnect() {

  digitalWrite(RELAYPIN_QD_OX, HIGH);
  digitalWrite(RELAYPIN_QD_ETH, HIGH);

}

void ignition() {
  digitalWrite(RELAYPIN_IGNITER, LOW);
  sendData();  
}

void hotfire() {
  digitalWrite(RELAYPIN_PYROLOX, LOW);
  digitalWrite(RELAYPIN_PYROETH, LOW);
}

void abort_sequence() {
  getReadings();
  // Waits for LOX pressure to decrease before venting Eth through pyro
  while (readingPT1 > 50) {
    digitalWrite(RELAYPIN_VENT, LOW);
    getReadings();
  }
  digitalWrite(RELAYPIN_VENT, HIGH);
  digitalWrite(RELAYPIN_PYROETH, LOW);
  while (readingPT2 > 5){
    getReadings();
  }
  digitalWrite(RELAYPIN_PYROETH,HIGH);
  while (true) {
  digitalWrite(RELAYPIN_VENT, LOW);
  getReadings();
  }


}

void debug() {
  //FILL IN
}


/// END OF STATE FUNCTION DEFINITIONS ///



/// HELPER FUNCTIONS ///

void openSolenoidFuel() {
  digitalWrite(RELAYPIN_PRESSETH, LOW);
}

void closeSolenoidFuel() {
  digitalWrite(RELAYPIN_PRESSETH, HIGH);
}

void openSolenoidOx() {
  digitalWrite(RELAYPIN_PRESSLOX, LOW);
}

void closeSolenoidOx() {
  digitalWrite(RELAYPIN_PRESSLOX, HIGH);
}

void vent() {
  digitalWrite(RELAYPIN_VENT, LOW);
}

/// END OF HELPER FUNCTIONS ///


/// DATA LOGGING AND COMMUNICATION ///

bool sendData() {
  if ((millis()-sendTime)>sendDelay) { 
    addReadingsToQueue();
    sendQueue();
    return true;
  }
  return false;
}

void addReadingsToQueue() {
  getReadings();
  if (queueLength<40) {
    queueLength+=1;
    ReadingsQueue[queueLength].messageTime=millis();
    ReadingsQueue[queueLength].pt1=readingPT1;
    ReadingsQueue[queueLength].pt2=readingPT2;
    ReadingsQueue[queueLength].pt3=readingPT3;
    ReadingsQueue[queueLength].pt4=readingPT4;
    ReadingsQueue[queueLength].pt5=readingPT5;
    ReadingsQueue[queueLength].lc1=readingLC1;
    ReadingsQueue[queueLength].lc2=readingLC2;
    ReadingsQueue[queueLength].lc3=readingLC3;
    ReadingsQueue[queueLength].tc1=readingTC1;
    ReadingsQueue[queueLength].tc2=readingTC2;
    ReadingsQueue[queueLength].queueSize=queueLength;
    ReadingsQueue[queueLength].DAQState=currDAQState;
    ReadingsQueue[queueLength].pressComplete=pressComplete;
    ReadingsQueue[queueLength].oxComplete=oxComplete;
    ReadingsQueue[queueLength].ethComplete=ethComplete;
  }
}

void getReadings(){
  if(gainbool){
    scale1.set_gain(64);
    scale2.set_gain(64);
    scale3.set_gain(64);
    scale4.set_gain(64);
    readingPT1 = PT_Tanks_Offset_LOX + PT_Tanks_Slope_LOX * scale1.read(); 
    readingPT2 = ReadingsQueue[queueLength].pt2 ;
    readingPT3 = PT_Down_Offset_LOX + PT_Down_Slope_LOX * scale2.read(); 
    readingPT4 = ReadingsQueue[queueLength].pt4;
    readingPT5 = PT_Chamber_Offset + PT_Chamber_Slope * scale3.read();
    readingLC1 = ReadingsQueue[queueLength].lc1;
    readingLC2 = LoadCell2_Offset + LoadCell2_Offset*scale4.read();
    readingLC3 = ReadingsQueue[queueLength].lc3;
  } else {
    scale1.set_gain(32,false);
    scale2.set_gain(32,false);
    scale3.set_gain(32,false);
    scale4.set_gain(32,false);
    readingPT1 = ReadingsQueue[queueLength].pt1;
    readingPT2 = PT_Tanks_Offset_ETH + PT_Tanks_Slope_ETH * scale1.read(); 
    readingPT3 = ReadingsQueue[queueLength].pt3; 
    readingPT4 = PT_Down_Offset_ETH + PT_Down_Slope_ETH * scale2.read(); 
    readingPT5 = ReadingsQueue[queueLength].pt5;
    readingLC1 = LoadCell1_Offset + LoadCell2_Offset*scale3.read();
    readingLC2 = ReadingsQueue[queueLength].lc2;
    readingLC3 = LoadCell1_Offset + LoadCell2_Offset*scale4.read();
  }
  gainbool = !gainbool;
  
  readingTC1 = analogRead(TC1);
  readingTC2 = analogRead(TC2);
  // readingCap1 = analogRead(CAPSENS1DATA);
  // readingCap2 = analogRead(CAPSENS2DATA);

  printSensorReadings();
}

void printSensorReadings() {
 serialMessage = "";
 //
 serialMessage.concat(millis());
 serialMessage.concat(" ");
 serialMessage.concat(readingPT1);
 serialMessage.concat(" ");
 serialMessage.concat(readingPT2);
 serialMessage.concat(" ");
 serialMessage.concat(readingPT3);
 serialMessage.concat(" ");
 serialMessage.concat(readingPT4);
 serialMessage.concat(" ");
 serialMessage.concat(readingPT5);
 serialMessage.concat(" ");
 serialMessage.concat(readingLC1);
 serialMessage.concat(" ");
 serialMessage.concat(readingLC2);
 serialMessage.concat(" ");
 serialMessage.concat(readingLC3);
 serialMessage.concat(" ");
 serialMessage.concat(readingTC1);
 serialMessage.concat(" ");
 serialMessage.concat(readingTC2);
 serialMessage.concat(" ");
//  serialMessage.concat(readingCap1);
//  serialMessage.concat(" ");
//  serialMessage.concat(readingCap2);
 serialMessage.concat(" Queue Length: ");
 serialMessage.concat(queueLength);
 Serial.println(serialMessage);
}

void sendQueue() {
  if (queueLength>0){
    dataSend();
  }
}

void dataSend() {
   // Set values to send
  Readings.messageTime=ReadingsQueue[queueLength].messageTime;
  Readings.pt1 = ReadingsQueue[queueLength].pt1;
  Readings.pt2 = ReadingsQueue[queueLength].pt2;
  Readings.pt3 = ReadingsQueue[queueLength].pt3;
  Readings.pt4 = ReadingsQueue[queueLength].pt4;
  Readings.pt5 = ReadingsQueue[queueLength].pt5;
  Readings.lc1 = ReadingsQueue[queueLength].lc1;
  Readings.lc2 = ReadingsQueue[queueLength].lc2;
  Readings.lc3  = ReadingsQueue[queueLength].lc3;
  Readings.tc1 = ReadingsQueue[queueLength].tc1;
  Readings.tc2 = ReadingsQueue[queueLength].tc2;
  Readings.DAQState = ReadingsQueue[queueLength].DAQState;
  Readings.pressComplete = ReadingsQueue[queueLength].pressComplete;
  Readings.oxComplete = ReadingsQueue[queueLength].oxComplete;
  Readings.ethComplete = ReadingsQueue[queueLength].ethComplete;

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &Readings, sizeof(Readings));

  if (result == ESP_OK) {
     Serial.println("Sent with success Data Send");
   //  ReadingsQueue[queueLength].pt1val=0;
     queueLength-=1;
  }
  else {
     Serial.println("Error sending the data");
  }
}