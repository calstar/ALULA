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

//DEBUG TRIGGER: SET TO 1 FOR DEBUG MODE
int DEBUG = 1;
int WIFIDEBUG = 1;

// MODEL DEFINED PARAMETERS FOR TEST/HOTFIRE. Pressures in psi //
float pressureFuel=400;    //Set pressure for fuel: 412
float pressureOx=100;    //Set pressure for lox: 445
float threshold = 0.925; //re-pressurrization threshold (/1x)
float ventTo = 10; //close solenoids at this pressure to preserve lifetime.
float LOXventing = 30; //pressure at which ethanol begins venting
#define abortPressure 525 //Cutoff pressure to automatically trigger abort
#define period 0.5   //Sets   period for bang-bang control
float sendDelay = 250; //Sets frequency of data collection. 1/(sendDelay*10^-3) is frequency in Hz
// END OF USER DEFINED PARAMETERS //
//refer to https://docs.google.com/spreadsheets/d/17NrJWC0AR4Gjejme-EYuIJ5uvEJ98FuyQfYVWI3Qlio/edit#gid=1185803967 for all pinouts

//::::::DEFINE INSTRUMENT PINOUTS::::::://

// DEFINE CLK PIN (SHARED ACROSS ALL HX INSTRUMENTS)
#define CLK 27

//::SET SENSOR PINOUTS:://

// LOX System
#define PT_O1 36 //LOX Tank PT
#define PT_O2 39 //LOX Injector PT
float PT_O1_Offset = 4.40;
float PT_O1_Slope = .0000404;
float PT_O2_Offset = 7.25;
float PT_O2_Slope = 0.000102;
// ETH Systemlope = 0.0001024;


#define PT_E1 34 //ETH tank PT SWAPPED FROM PINO2
float PT_E1_Offset = 5.522;
float PT_E1_Slope = 0.000103;

#define PT_E2 35 //ETH Injector PT
float PT_E2_Offset = 10.663;
float PT_E2_Slope = 0.0001013;


// Combustion Chamber should be 32, swapped atm
//#define PT_C1 32
#define PT_C1 32
float PT_C1_Offset = 2.97;
float PT_C1_Slope = 0.0001181;


// LOADCELLS
#define LC1 33
float LC1_Offset = 10.663;
float LC1_Slope = 0.0001181;

#define LC2 25
float LC2_Offset = 10.663;
float LC2_Slope = 0.0001181;

#define LC3 26
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
#define MOSFET_EXTRA 10
#define MOSFET_LOX_MAIN 3
#define MOSFET_ETH_PRESS 4
#define MOSFET_LOX_PRESS 5
#define MOSFET_VENT_ETH 6
#define MOSFET_VENT_LOX 2


bool sendData();
void getReadings();


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
#define MAXDO   5
#define MAXD1 17
#define MAXCS1   16
#define MAXCS2   4
#define MAXCLK  18
Adafruit_MAX31855 thermocouple1(MAXCLK, MAXCS1, MAXDO);
Adafruit_MAX31855 thermocouple2(MAXCLK, MAXCS2, MAXDO);

//Adafruit_MAX31855 thermocouple1(MAXCLK, MAXCS1, MAXDO);
//Adafruit_MAX31855 thermocouple2(MAXCLK, MAXCS2, MAXDO);


//////////////
//IMPORTANT//
/////////////
// REPLACE WITH THE MAC Address of your receiver

//OLD COM BOARD {0xC4, 0xDD, 0x57, 0x9E, 0x91, 0x6C}
//COM BOARD {0x7C, 0x9E, 0xBD, 0xD7, 0x2B, 0xE8}
//HEADERLESS BOARD {0x7C, 0x87, 0xCE, 0xF0 0x69, 0xAC}
//NEWEST COM BOARD IN EVA {0x24, 0x62, 0xAB, 0xD2, 0x85, 0xDC}
// uint8_t broadcastAddress[] = {0x24, 0x62, 0xAB, 0xD2, 0x85, 0xDC};
uint8_t broadcastAddress[] ={0xC8, 0xF0, 0x9E, 0x50, 0x23, 0x34};
// {0x7C, 0x87, 0xCE, 0xF0, 0x69, 0xAC};
//{0x3C, 0x61, 0x05, 0x4A, 0xD5, 0xE0};
// {0xC4, 0xDD, 0x57, 0x9E, 0x96, 0x34}



//STATEFLOW VARIABLES
enum STATES {IDLE, ARMED, PRESS, QD, IGNITION, HOTFIRE, ABORT};

int state;

short int queueLength=0;
int commandedState;
int currDAQState;
bool ethComplete = false;
bool oxComplete = false;
bool pressComplete = false ;
bool oxVentComplete = false;
bool ethVentComplete = false;



//::::DEFINE READOUT VARIABLES:::://

float sendTime;

// Define variables to store readings to be sent
int debug_state = 0;
float reading_PT_O1=1;
float reading_PT_O2=1;
float reading_PT_E1=1;
float reading_PT_E2=1;
float reading_PT_C1=1;
float reading_LC1=1;
float reading_LC2=1;
float reading_LC3=1;
float reading_TC1=1;
float reading_TC2=1;
float readingCap1=0;
float readingCap2=0;
short int queueSize=0;

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    int messageTime;
    float pt1;  //PTO1
    float pt2;  //PTO2
    float pt3;  //PTE1
    float pt4;  //PTE2
    float pt5;  //PTC1
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
//void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
//  sendTime = millis();
//}

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


  //EVERYTHING SHOULD BE WRITTEN HIGH 
  pcf.setAllBitsUp();

  //set gains for pt pins
   scale_PT_O1.begin(PT_O1, CLK); scale_PT_O1.set_gain(64);
   scale_PT_O2.begin(PT_O2, CLK); scale_PT_O2.set_gain(64);
   scale_PT_E1.begin(PT_E1, CLK); scale_PT_E1.set_gain(64);
   scale_PT_E2.begin(PT_E2, CLK); scale_PT_E2.set_gain(64);
   scale_PT_C1.begin(PT_C1, CLK); scale_PT_C1.set_gain(64);
   scale_LC1.begin(LC1, CLK); scale_LC1.set_gain(64);
   scale_LC2.begin(LC2, CLK); scale_LC2.set_gain(64);
   scale_LC3.begin(LC3, CLK); scale_LC3.set_gain(64);
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
//  esp_now_register_send_cb(OnDataSent);

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
 if (WIFIDEBUG !=1) {
  esp_now_register_recv_cb(OnDataRecv);}

  sendTime = millis();
  state = IDLE;
  currDAQState = IDLE;
}






// Implementation of State Machine
void loop() {
 
SerialRead();
 //Serial.println(state);
 if (DEBUG == 1) {
  Serial.println(state);
  //Serial.println(commandedState);
  CheckDebug();
 }

  switch (state) {
    

  case (IDLE):
    
    sendDelay = IDLE_DELAY;
    if (commandedState==IDLE) {state=IDLE; currDAQState=IDLE;}
    idle();
    if (DEBUG ==1) {
    Serial.print("idle");
    CheckDebug();
    }
    closeSolenoidOx();
    closeSolenoidFuel();
    if (commandedState==ABORT) {state=ABORT; currDAQState=ABORT;}
    if (commandedState==ARMED) {state=ARMED; currDAQState=ARMED;}
    break;

  case (ARMED): //NEED TO ADD TO CASE OPTIONS //ALLOWS OTHER CASES TO TRIGGER //INITIATE TANK PRESS LIVE READINGS
    sendDelay = GEN_DELAY;
    //debug stuff
    if (DEBUG ==1) {
    Serial.print("arm");
    CheckDebug();
    }
    armed();
    closeSolenoidOx();
    closeSolenoidFuel();
    if (commandedState==IDLE) {state=IDLE; currDAQState=IDLE;}
    if (commandedState==ABORT) {state=ABORT; currDAQState=ABORT;}
    if (commandedState==PRESS) {state=PRESS; currDAQState=PRESS;}
    
    break;

  case (PRESS):
    sendDelay = GEN_DELAY;
    press();
    if (DEBUG ==1) {
    Serial.print("press");
    CheckDebug();
    }    
    if (commandedState==IDLE) {state=IDLE; currDAQState=IDLE;}
    if (commandedState==ABORT) {state=ABORT; currDAQState=ABORT;}
    if (commandedState==QD) {state=QD; currDAQState=QD;}
    if (commandedState==IGNITION) {state=IGNITION; currDAQState=IGNITION;}
    break;

  case (QD):
    sendDelay = GEN_DELAY;
    quick_disconnect();
    if (DEBUG ==1){
      Serial.print("Stby");
      CheckDebug();
      }
    if (commandedState==IDLE) {state=IDLE; currDAQState=IDLE;}
    if (commandedState==ABORT) {state=ABORT; currDAQState=ABORT;}
    if (commandedState==IGNITION) {state=IGNITION; currDAQState=IGNITION;}
    break;

  case (IGNITION): 
    sendDelay = GEN_DELAY;
    ignition();
    if (DEBUG ==1){
    Serial.print("Ign");
    CheckDebug();
    }
    if (commandedState==IDLE) {state=IDLE; currDAQState=IDLE;}
    if (commandedState==ABORT) {state=ABORT; currDAQState=ABORT;}
    if (commandedState==HOTFIRE) {state=HOTFIRE; currDAQState=HOTFIRE;}
    
    break;

  case (HOTFIRE): 
    sendDelay = GEN_DELAY;
    if (DEBUG ==1) {
    Serial.print("HOTFIRE");
    CheckDebug();}
    hotfire();
    if (commandedState==IDLE) {state=IDLE; currDAQState=IDLE;}
    if (commandedState==ABORT) {state=ABORT; currDAQState=ABORT;}
    break;

  case (ABORT):
    sendDelay = GEN_DELAY;
    abort_sequence();
    if (commandedState==IDLE) {state=IDLE; currDAQState=IDLE;}
    break;
  }
}








/// STATE FUNCTION DEFINITIONS ///


void SerialRead() {
    if (Serial.available() > 0) {
 commandedState=Serial.read()-48;
 delay(50);
 Serial.println(commandedState);
  }
}


void CheckDebug() {
  if (commandedState==IDLE) {state=IDLE; currDAQState=IDLE;}
  if (commandedState==ARMED) {state=ARMED; currDAQState=ARMED;}
  if (commandedState==PRESS) {state=PRESS; currDAQState=PRESS;}
  if (commandedState==QD) {state=QD; currDAQState=QD;}
  if (commandedState==IGNITION) {state=IGNITION; currDAQState=IGNITION;}
  if (commandedState==ABORT) {state=ABORT; currDAQState=ABORT;}
  if (commandedState==HOTFIRE) {state=HOTFIRE; currDAQState=HOTFIRE;}
}



void idle() {
  //Serial.println(sendData());
  if (!sendData()) {
  getReadings();
}
  pcf.setLeftBitUp(MOSFET_LOX_MAIN);
  pcf.setLeftBitUp(MOSFET_ETH_MAIN);
  pcf.setLeftBitUp(MOSFET_IGNITER);
  closeSolenoidOx();
  closeSolenoidFuel();
  pcf.setLeftBitUp(MOSFET_VENT_LOX);
  pcf.setLeftBitUp(MOSFET_VENT_ETH);
  
}

void armed() {
  if (!sendData()) {
  getReadings();
}
}


void press() {
  sendDelay = GEN_DELAY;

  if (!sendData()) {
    getReadings();
  }
  
  if (reading_PT_O1 < pressureOx*threshold || reading_PT_E1 < pressureFuel*threshold) {
    oxComplete = false;
    ethComplete = false;
    
    while (!oxComplete || !ethComplete) {
      //Serial.print("IN press WHILE LOOP");
      if (!sendData()) {
        getReadings();
      }
        
      if (reading_PT_O1 < pressureOx) {
        openSolenoidOx();
        if (DEBUG == 1) {
        reading_PT_O1 = reading_PT_O1 + 0.1;
        }
      } 
      else {
        closeSolenoidOx();
        oxComplete = true;
      }
      if (reading_PT_E1 < pressureFuel) {
        openSolenoidFuel();
        if (DEBUG == 1) {
        reading_PT_E1 = reading_PT_E1 + 0.2;
        }
      } else {
        closeSolenoidFuel();
        ethComplete = true;
      }
    
    //ABORT CASES
    CheckAbort();
    if (commandedState==IDLE) {state=IDLE; currDAQState=IDLE; break;}
    if (commandedState==ABORT) {state=ABORT; currDAQState=ABORT; break;}
    if (commandedState==QD) {state=QD; currDAQState=QD; break;}
    }
  }
  CheckAbort();
  }
 //End of Void Press

void CheckAbort() {
    if (reading_PT_O1 >= abortPressure || reading_PT_E1 >= abortPressure) {
      closeSolenoidFuel();
      closeSolenoidOx();
      state = ABORT;
    }
    
    if (commandedState == ABORT) {
      closeSolenoidOx();
      closeSolenoidFuel();
      state = ABORT;
      currDAQState=ABORT;
    }
    if (!sendData()) {
      getReadings();
    } 
    if (commandedState==IDLE) {state=IDLE; currDAQState=IDLE;}
    if (commandedState==ABORT) {state=ABORT; currDAQState=ABORT;}
  }

void ignition() {
    if (!sendData()) {
  getReadings();
}
  pcf.setLeftBitDown(MOSFET_IGNITER);  
}

void hotfire() {
    if (!sendData()) {
  getReadings();
}
  pcf.setLeftBitDown(MOSFET_LOX_MAIN);
  pcf.setLeftBitDown(MOSFET_ETH_MAIN);
}

void quick_disconnect() {
    closeSolenoidOx();
    closeSolenoidFuel();
    if (!sendData()) {
      getReadings();
    }
    //QD code here
  CheckAbort();
}


void abort_sequence() {
 pcf.setLeftBitDown(MOSFET_VENT_LOX);
 pcf.setLeftBitDown(MOSFET_VENT_ETH);
if (!sendData()) {
      getReadings();
    }
  // Waits for LOX pressure to decrease before venting Eth through pyro
 closeSolenoidOx();
 closeSolenoidFuel();
 
 int currtime = millis();
 oxVentComplete = !(reading_PT_O1 > 1.3*ventTo);
 ethVentComplete = !(reading_PT_E1 > 1.3*ventTo);
  while(!oxVentComplete || !ethVentComplete){
    getReadings();
  if (reading_PT_O1 > LOXventing) { //vent only lox down to loxventing pressure
    pcf.setLeftBitDown(MOSFET_VENT_LOX);
    if (DEBUG == 1) {
        reading_PT_O1 = reading_PT_O1 - 0.25;
        }
  } 
  else {
      if (reading_PT_E1 > ventTo) {
      pcf.setLeftBitDown(MOSFET_VENT_ETH); //vent ethanol
      if (DEBUG == 1) {
          reading_PT_E1 = reading_PT_E1 - 0.2;
          }
    } else {
      pcf.setLeftBitUp(MOSFET_VENT_ETH); 
      ethVentComplete = true;
    }
    
    if (reading_PT_O1 > ventTo) {
    pcf.setLeftBitUp(MOSFET_VENT_LOX); //vent lox
        if (DEBUG == 1) {
          reading_PT_O1 = reading_PT_O1 - 0.1;
          }
    }
    else { //lox vented to acceptable hold pressure
     pcf.setLeftBitUp(MOSFET_VENT_LOX); // close lox
     oxVentComplete = true;
    }
  
 } 
 
if (!sendData()) {
      getReadings();
    }
if (commandedState==IDLE) {
  state=IDLE; currDAQState=IDLE; break;}
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



/// END OF STATE FUNCTION DEFINITIONS ///


/// DATA LOGGING AND COMMUNICATION ///

bool sendData() {
  if ((millis()-sendTime)>sendDelay) { 
    sendTime = millis();
    addReadingsToQueue();
    
    sendQueue();
    return true;
  }
  return false;
}

void addReadingsToQueue() {
  
  //getReadings();
  
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
    ReadingsQueue[queueLength].oxComplete=oxComplete;
    ReadingsQueue[queueLength].ethComplete=ethComplete;
  }
}

void getReadings(){
    if (DEBUG != 1) {
     reading_PT_O1 = PT_O1_Offset + PT_O1_Slope * scale_PT_O1.read(); 
     reading_PT_O2 = PT_O2_Offset + PT_O2_Slope * scale_PT_O2.read(); 
     reading_PT_E1 = PT_E1_Offset + PT_E1_Slope * scale_PT_E1.read();
     //Serial.println(reading_PT_E1);
     reading_PT_E2 = PT_E2_Offset + PT_E2_Slope * scale_PT_E2.read();
     reading_PT_C1 = PT_C1_Offset + PT_C1_Slope * scale_PT_C1.read(); 
     reading_LC1 = LC1_Offset + LC1_Slope * scale_LC1.read(); 
     reading_LC2 = LC2_Offset +LC2_Slope *scale_LC2.read();
     reading_LC3 = LC3_Offset + LC3_Slope *scale_LC3.read();
     //reading_TC1 = thermocouple1.readCelsius();
     //reading_TC2 = thermocouple2.readCelsius();
//     readingCap1 = analogRead(CAPSENS1DATA);
//     readingCap2 = analogRead(CAPSENS2DATA);
    }
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
 serialMessage.concat(state);
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

if (WIFIDEBUG != 1) {
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
}
