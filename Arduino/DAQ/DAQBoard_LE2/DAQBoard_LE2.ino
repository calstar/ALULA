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
#include <SPI.h>
#include "Adafruit_MAX31855.h"

#include <EasyPCF8575.h>
EasyPCF8575 pcf;

#define IDLE_DELAY 250
#define GEN_DELAY 25


// MODEL DEFINED PARAMETERS FOR TEST/HOTFIRE //
#define BangBangPressFuel 425 //Indicates transition from normal fill to bang-bang
#define pressureFuel 450    //In units of psi. Defines set pressure for fuel
#define BangBangPressOx 425 //Indicates transition from normal fill to bang-bang
#define pressureOx 450    //In units of psi. Defines set pressure for ox
#define abortPressure 525 //Cutoff pressure to automatically trigger abort
#define period 0.5   //Sets period for bang-bang control
float sendDelay = 250; //Sets frequency of data collection. 1/(sendDelay*10^-3) is frequency in Hz
// END OF USER DEFINED PARAMETERS //
//refer to https://docs.google.com/spreadsheets/d/17NrJWC0AR4Gjejme-EYuIJ5uvEJ98FuyQfYVWI3Qlio/edit#gid=1185803967 for all pinouts

//::::::DEFINE INSTRUMENT PINOUTS::::::://

// DEFINE CLK PIN (SHARED ACROSS ALL HX INSTRUMENTS)
#define CLK = 27

//::SET SENSOR PINOUTS:://

// LOX System
#define PT_O1 39 //LOX Tank PT
#define PT_O2 36 //LOX Injector PT
float PT_O1_Offset = 10.663;
float PT_O1_Slope = 0.0001181;
float PT_O2_Offset = 10.663;
float PT_O2_Slope = 0.0001181;


// ETH System
#define PT_E1 34 //ETH tank PT
float PT_E1_Offset = 10.663;
float PT_E1_Slope = 0.0001181;

#define PT_E2 35 //ETH Injector PT
float PT_E2_Offset = 10.663;
float PT_E2_Slope = 0.0001181;


// Combustion Chamber
#define PT_C1 32
float PT_C1_Offset = 10.663;
float PT_C1_Slope = 0.0001181;


// LOADCELLS
#define LC1 33
float LC1_Offset = 10.663;
float LC1_Slope = 0.0001181;

#define LC2 25
float LC2_Offset = 10.663;
float LC2_Slope = 0.0001181;

#define LC_3 26
float LC3_Offset = 10.663;
float LC3_Slope = 0.0001181;


//DEFINE THERMOCOUPLE PINS
#define TCSDO 5
#define TCSDI 17
#define TCCLK 18
#define TC1_CS 16
#define TC2_CS 4
// #define CAPSENS1DATA 40
// #define CAPSENS1CLK 52
// #define CAPSENS2DATA 13
// #define CAPSENS2CLK 21
// End of sensor pinouts //

// GPI expander // 
#define I2C_SDA 21
#define I2C_SCL 22

#define MOSFET_IGNITER 0
#define MOSFET_ETH_MAIN 1
#define MOSFET_EXTRA 2
#define MOSFET_LOX_MAIN 3
#define MOSFET_ETH_PRESS 4
#define MOSFET_LOX_PRESS 5
#define MOSFET_ETH_VENT 6
#define MOSFET_LOX_VENT 7

// MOSFET pinouts //


String serialMessage;

// Initialize the PT and LC sensor objects which use the HX711 breakout board
HX711 scale_PT_O1;
HX711 scale_PT_O2;
HX711 scale_PT_E1;
HX711 scale_PT_E2;
HX711 scale_PT_C1;
HX711 scale_LC1;
HX711 scale_LC2;
HX711 scale_LC3;
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
uint8_t broadcastAddress[] ={0x08, 0x3A, 0xF2, 0xB7, 0x0E, 0x4A};
// {0x7C, 0x87, 0xCE, 0xF0, 0x69, 0xAC};
//{0x3C, 0x61, 0x05, 0x4A, 0xD5, 0xE0};
// {0xC4, 0xDD, 0x57, 0x9E, 0x96, 0x34}

//STATEFLOW VARIABLES
enum STATES {IDLE, ARMED, PRESS, QD, IGNITION, HOTFIRE, ABORT};
#define DEBUG 99
#define DEBUG_IDLE 90
#define DEBUG_ARMED 91
#define DEBUG_PRESS 92
#define DEBUG_QD 93
#define DEBUG_IGNITION 94
#define DEBUG_HOTFIRE 95
#define DEBUG_ABORT 96
int state;

short int queueLength=0;
int commandedState;
int currDAQState;
bool ethComplete = false;
bool oxComplete = false;
bool pressComplete = false ;



//::::DEFINE READOUT VARIABLES:::://

float sendTime;

// Define variables to store readings to be sent
int debug_state = 0;
int reading_PT_O1=1;
int reading_PT_O2=1;
int reading_PT_E1=1;
int reading_PT_E2=1;
int reading_PT_C1=1;
int reading_LC1=1;
int reading_LC2=1;
int reading_LC3=1;
int reading_TC1=1;
int reading_TC2=1;
float readingCap1=0;
float readingCap2=0;
short int queueSize=0;

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    int messageTime;
    int pt1;  //PTO1
    int pt2;  //PTO2
    int pt3;  //PTE1
    int pt4;  //PTE2
    int pt5;  //PTC1
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

  while (!Serial) delay(1); // wait for Serial on Leonardo/Zero, etc

  // Serial.println("MAX31855 test");
  // // wait for MAX chip to stabilize
  // delay(500);
  // Serial.print("Initializing sensor...");
  // if (!thermocouple.begin()) {
  //   Serial.println("ERROR.");
  //   while (1) delay(10);
  // }

  // OPTIONAL: Can configure fault checks as desired (default is ALL)
  // Multiple checks can be logically OR'd together.
  // thermocouple.setFaultChecks(MAX31855_FAULT_OPEN | MAX31855_FAULT_SHORT_VCC);  // short to GND fault is ignored

  Serial.println("DONE.");
 
  pcf.startI2C(21, 22, SEARCH); //Only SEARCH, if using normal pins in Arduino
  if (!pcf.check(SEARCH)) {
    Serial.println("Device not found. Try to specify the address");
    Serial.println(pcf.whichAddr());
    while (true);
  }
  pcf.setAllBitsUp(); // make sure everything is off by default (Up = Off, Down = On)
  delay(500); // startup time to make sure its good for personal testing


  //EVERYTHING SHOULD BE WRITTEN HIGH EXCEPT QDs, WHICH SHOULD BE LOW
  pcf.setAllBitsUp();

  //set gains for pt pins
  scale_PT_O1.begin(PT_O1, CLK); scale_PT_O1.set_gain(64);
  scale_PT_O2.begin(PT_O2, CLK); scale_PT_O2.set_gain(64);
  scale_PT_E1.begin(PT_E1, CLK); scale_PT_E1.set_gain(64);
  scale_PT_E2.begin(PT_E2, CLK); scale_PT_E2.set_gain(64);
  scale_PT_C1.begin(PT_C1, CLK); scale_PT_C1.set_gain(64);
  

  pinMode(TC1_CS, INPUT);
  pinMode(TC2_CS, INPUT);

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
  if(Serial.available() > 1){if(Serial.parseInt() == 99){state = DEBUG;}}

  switch (state) {
    

  case (IDLE):
    sendDelay = IDLE_DELAY;
    if (commandedState==IDLE) {state=IDLE; currDAQState=IDLE;}
    idle();
    if (commandedState==ABORT) {state=ABORT; currDAQState=ABORT;}
    if (commandedState==ARMED) {state=ARMED; currDAQState=ARMED;}
    
    break;

  case (ARMED): //NEED TO ADD TO CASE OPTIONS //ALLOWS OTHER CASES TO TRIGGER //INITIATE TANK PRESS LIVE READINGS
    sendDelay = GEN_DELAY;
    armed();
    if (commandedState==IDLE) {state=IDLE; currDAQState=IDLE;}
    if (commandedState==ABORT) {state=ABORT; currDAQState=ABORT;}
    if (commandedState==PRESS) {state=PRESS; currDAQState=PRESS;}
    
    break;

  case (PRESS):
    sendDelay = GEN_DELAY;
    pressComplete = press();
    if (commandedState==IDLE) {state=IDLE; currDAQState=IDLE;}
    if (commandedState==ABORT) {state=ABORT; currDAQState=ABORT;}
    if (pressComplete && commandedState==QD) {state=QD; currDAQState=QD;}
    if (pressComplete && commandedState==IGNITION) {state=IGNITION; currDAQState=IGNITION;}
    
    break;

  case (QD):
    sendDelay = GEN_DELAY;
    quick_disconnect();
    if (commandedState==IDLE) {state=IDLE; currDAQState=IDLE;}
    if (commandedState==ABORT) {state=ABORT; currDAQState=ABORT;}
    if (commandedState==IGNITION) {state=IGNITION; currDAQState=IGNITION;}
    
    
    break;

  case (IGNITION): 
    sendDelay = GEN_DELAY;
    ignition();
    if (commandedState==IDLE) {state=IDLE; currDAQState=IDLE;}
    if (commandedState==ABORT) {state=ABORT; currDAQState=ABORT;}
    if (commandedState==HOTFIRE) {state=HOTFIRE; currDAQState=HOTFIRE;}
    

    break;

  case (HOTFIRE): 
    sendDelay = GEN_DELAY;
    hotfire();
    if (commandedState==IDLE) {state=IDLE; currDAQState=IDLE;}
    if (commandedState==ABORT) {state=ABORT; currDAQState=ABORT;}
    break;

  case (ABORT):
    sendDelay = GEN_DELAY;
    abort_sequence();
    if (commandedState==IDLE) {state=IDLE; currDAQState=IDLE;}
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
  oxComplete = (reading_PT_O1 >= pressureOx);
  ethComplete = (reading_PT_E1 >= pressureFuel);

  if (!sendData()) {
    getReadings();
  }
  
  while (reading_PT_O1 < pressureOx && reading_PT_E1 < pressureFuel) {
    if (!sendData()) {
      getReadings();
    }
      
    if (reading_PT_O1 < BangBangPressOx) {
      openSolenoidOx();
    else
      closeSolenoidOx();
      oxComplete = true;
    }
    if (reading_PT_E1 < BangBangPressFuel) {
      openSolenoidFuel();
    else
      closeSolenoidFuel();
      ethComplete = true;
    }

//ABORT CASES
  if (reading_PT_O1 >= abortPressure || reading_PT_E1 >= abortPressure) {
      closeSolenoidFuel();
      closeSolenoidOx();
      state = ABORT;
      return false;
    }
    
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

//:::BANGBANG CODE::://
//  if (reading_PT_O1 < BangBangPressOx && reading_PT_E1 > BangBangPressFuel) {
//    closeSolenoidFuel();
//    while (reading_PT_O1 < BangBangPressOx && reading_PT_E1 < pressureFuel) {
//      openSolenoidOx();
//      if (commandedState == ABORT) {
//        closeSolenoidOx();
//        state = ABORT;
//        return false;
//      }
//      if (!sendData()) {
//        getReadings();
//      }
//    }
//  }
//  if (reading_PT_O1 > BangBangPressOx && reading_PT_E1 < BangBangPressFuel) {
//    closeSolenoidOx();
//    while (reading_PT_E1 < BangBangPressFuel && reading_PT_O1 < pressureOx) {
//      openSolenoidFuel();
//      if (commandedState == ABORT) {
//        closeSolenoidFuel();
//        state = ABORT;
//        return false;
//      }
//      if (!sendData()) {
//        getReadings();
//      }
//    }    
//  }
//
//  while (reading_PT_O1 < pressureOx || reading_PT_E1 < pressureFuel) {
//    if (reading_PT_O1 >= abortPressure || reading_PT_E1 >= abortPressure) {
//      closeSolenoidFuel();
//      closeSolenoidOx();
//      state = ABORT;
//      return false;
//    }
//    if (reading_PT_O1 >= pressureOx) {
//      closeSolenoidOx();
//      oxComplete = true;
//    }
//    if (reading_PT_E1 >= pressureFuel) {
//      closeSolenoidFuel();
//      ethComplete = true;
//    }
//    if (commandedState == ABORT) {
//      closeSolenoidOx();
//      closeSolenoidFuel();
//      state = ABORT;
//      return false;      
//    }
//    if (!oxComplete) {
//      openSolenoidOx();
//    }
//    if (!ethComplete) {
//      openSolenoidFuel();
//    }
//    
//    delay(period * 500);
//    closeSolenoidOx();
//    closeSolenoidFuel();
//    delay((1-period) * 500);
//    if (!sendData()) {
//      getReadings();
//    }
//  }
 
  return true;
}



void ignition() {
  pcf.setLeftBitUp(MOSFET_IGNITER);
  sendData();  
}

void hotfire() {
  pcf.setLeftBitDown(MOSFET_LOX_MAIN);
  pcf.setLeftBitDown(MOSFET_ETH_MAIN);
  sendData();
}

void quick_disconnect() {
    if (!sendData()) {
      getReadings();
    }
}


void abort_sequence() {
  getReadings();
  // Waits for LOX pressure to decrease before venting Eth through pyro
  pcf.setLeftBitDown(MOSFET_VENT_LOX);
  currtime = millis();
  while(millis() - currtime < 10000){
  getReadings();
  }
  pcf.setLeftBitDown(MOSFET_VENT_ETH);

  if (reading_PT_O1 > 5) {
    pcf.setLeftBitDown(MOSFET_VENT_LOX);
  else
    pcf.setLeftBitUp(MOSFET_VENT_LOX); 
  }
  if (reading_PT_E1 > 5) {
    pcf.setLeftBitDown(MOSFET_VENT_ETH);
  else
    pcf.setLeftBitUp(MOSFET_VENT_ETH); 
  }
}

void closeSolenoidOx(){
  pcf.setLeftBitUp(MOSFET_LOX_PRESS);
}
 void closeSolenoidFuel(){
  pcf.setLeftBitUp(MOSFET_ETH_PRESS);
}
void openSolenoidFuel(){
  pcf.setLeftBitDown(MOSFET_ETH_PRESS);
}
void openSolenoidOx(){
  pcf.setLeftBitDown(MOSFET_LOX_PRESS);
}

void debug() {
  //just a mini state machine]
  //debug = 99, debug_states = 90+state
  //to return to idle , input 0
  while(currDAQState == DEBUG){
    debug_state = Serial.parseInt();
  switch (debug_state) {
  case (DEBUG_IDLE):
    sendDelay = IDLE_DELAY;
    idle();
 

  case (DEBUG_ARMED): //NEED TO ADD TO CASE OPTIONS //ALLOWS OTHER CASES TO TRIGGER //INITIATE TANK PRESS LIVE READINGS
    sendDelay = GEN_DELAY;
    armed();
    


  case (DEBUG_PRESS):
    sendDelay = GEN_DELAY;
    pressComplete = press();
    


  case (DEBUG_QD):
    sendDelay = GEN_DELAY;
    getReadings();

  case (DEBUG_IGNITION): 
    sendDelay = GEN_DELAY;
    ignition();


  case (DEBUG_HOTFIRE): 
    sendDelay = GEN_DELAY;
    hotfire();


  case (DEBUG_ABORT):
    sendDelay = GEN_DELAY;
    abort_sequence();

  }
  default : 
  currDAQstate = IDLE;
    break;
  }

  
}


/// END OF STATE FUNCTION DEFINITIONS ///


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
    ReadingsQueue[queueLength].pt1=reading_PT_O1;
    ReadingsQueue[queueLength].pt2=reading_PT_O2;
    ReadingsQueue[queueLength].pt3=reading_PT_E1;
    ReadingsQueue[queueLength].pt4=reading_PT_E2;
    ReadingsQueue[queueLength].pt5=reading_PT_C1;
    ReadingsQueue[queueLength].lc1=reading_LC1;
    ReadingsQueue[queueLength].lc2=reading_LC2;
    ReadingsQueue[queueLength].lc3=reading_LC3;
    ReadingsQueue[queueLength].tc1=reading_TC1;
    ReadingsQueue[queueLength].tc2=reading_TC2;
    ReadingsQueue[queueLength].queueSize=queueLength;
    ReadingsQueue[queueLength].DAQState=currDAQState;
    ReadingsQueue[queueLength].pressComplete=pressComplete;
    ReadingsQueue[queueLength].oxComplete=oxComplete;
    ReadingsQueue[queueLength].ethComplete=ethComplete;
  }
}

void getReadings(){
    scale_PT_O1.set_gain(64);
    scale_PT_O2.set_gain(64);
    scale_PT_E1.set_gain(64);
    scale_PT_E2.set_gain(64);
    scale_PT_C1.set_gain(64);
    scale_LC1.set_gain(64);
    scale_LC2.set_gain(64);
    scale_LC3.set_gain(64);
    reading_PT_O1 = PT_Tanks_Offset_LOX + PT_Tanks_Slope_LOX * scale_PT_O1.read(); 
    reading_PT_O2 = PT_Down_Offset_LOX + PT_Down_Slope_LOX * scale_PT_O2.read(); 
    reading_PT_E1 = PT_Chamber_Offset + PT_Chamber_Slope * scale_PT_E1.read();
    reading_PT_E2 = LoadCell2_Offset + LoadCell2_Offset * scale_PT_E2.read();
    reading_PT_C1 = PT_Tanks_Offset_ETH + PT_Tanks_Slope_ETH * scale_PT_C1.read(); 
    reading_LC1 = PT_Down_Offset_ETH + PT_Down_Slope_ETH * scale_LC1.read(); 
    reading_LC2 = LoadCell1_Offset + LoadCell2_Offset*scale_LC2.read();
    reading_LC3 = LoadCell1_Offset + LoadCell2_Offset*scale_LC3.read();
    reading_TC1 = analogRead(TC1);
    reading_TC2 = analogRead(TC2);
    
  // readingCap1 = analogRead(CAPSENS1DATA);
  // readingCap2 = analogRead(CAPSENS2DATA);

  printSensorReadings();
}

void printSensorReadings() {
 serialMessage = "";
 //
 serialMessage.concat(millis());
 serialMessage.concat(" ");
 serialMessage.concat(reading_PT_O1);
 serialMessage.concat(" ");
 serialMessage.concat(reading_PT_O2);
 serialMessage.concat(" ");
 serialMessage.concat(reading_PT_E1);
 serialMessage.concat(" ");
 serialMessage.concat(reading_PT_E2);
 serialMessage.concat(" ");
 serialMessage.concat(reading_PT_C1);
 serialMessage.concat(" ");
 serialMessage.concat(reading_LC1);
 serialMessage.concat(" ");
 serialMessage.concat(reading_LC2);
 serialMessage.concat(" ");
 serialMessage.concat(reading_LC3);
 serialMessage.concat(" ");
 serialMessage.concat(reading_TC1);
 serialMessage.concat(" ");
 serialMessage.concat(reading_TC2);
 serialMessage.concat(" ");
 serialMessage.concat(debug_state);
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
