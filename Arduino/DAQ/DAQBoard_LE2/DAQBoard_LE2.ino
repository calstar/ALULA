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

// USER DEFINED PARAMETERS FOR TEST/HOTFIRE //
#define BangBangPressFuel 425
#define pressureFuel 450    //In units of psi. Defines set pressure for fuel
#define BangBangPressOx 425
#define pressureOx 450    //In units of psi. Defines set pressure for ox
#define tolerance 0.05   //Acceptable range within set pressure
#define CAPTHRESH1 1
#define CAPTHRESH2 1
float period = 0.5;
#define pressureDelay 0.5   //Sets percentage of time bang-bang is open
float sendDelay = 50; //Sets frequency of data collection. 1/(sendDelay*10^-3) is frequency in Hz
// END OF USER DEFINED PARAMETERS //

// Set sensor pinouts //
float reading = 0;
// USED FOR LOX TANK
#define PTOUT1 32
#define CLKPT1 5
float PTOffset1 = 10.663;
float PTSlope1 = 0.0001181;
// USED FOR FUEL TANK
#define PTOUT2 15
#define CLKPT2 2
float PTOffset2 = 10.663;
float PTSlope2 = 0.0001181;

#define PTOUT3 22
#define CLKPT3 23
float PTOffset3 = 10.663;
float PTSlope3 = 0.0001181;

#define PTOUT4 19
#define CLKPT4 21
float PTOffset4 = 10.663;
float PTSlope4 = 0.0001181;

#define PTOUT5 35
#define CLKPT5 25
float PTOffset5 = 10.663;
float PTSlope5 = 0.0001181;

#define LCOUT1 34
#define CLKLC1 26
float LCOffset1 = 10.663;
float LCSlope1 = 0.0001181;

#define LCOUT2 39
#define CLKLC2 33
float LCOffset2 = 10.663;
float LCSlope2 = 0.0001181;

#define LCOUT3 38
#define CLKLC3 41
float LCOffset3 = 10.663;
float LCSlope3 = 0.0001181;

#define TC1 10
#define TC2 15

#define CAPSENS1DATA 40
#define CAPSENS1CLK 52
#define CAPSENS2DATA 13
#define CAPSENS2CLK 21
// End of sensor pinouts //

// Relays pinouts. Use for solenoidPinOx, oxSolVent, oxQD, and fuelQD // 
#define RELAYPIN_LOXFILL 14
#define RELAYPIN_VENT 27
#define RELAYPIN_QD 28
#define RELAYPIN_PRESSLOX 35
#define RELAYPIN_PRESSETH 55
#define RELAYPIN_IGNITER 43
#define RELAYPIN_PYROLOX 95
#define RELAYPIN_PYROETH 24
// End of relay pinouts //

String serialMessage;

// Initialize the PT and LC sensor objects which use the HX711 breakout board
HX711 scale1;
HX711 scale2;
HX711 scale3;
HX711 scale4;
HX711 scale5;
HX711 scale6;
HX711 scale7;
HX711 scale8;
// End of HX711 initialization

///////////////
//IMPORTANT
//////////////
// REPLACE WITH THE MAC Address of your receiver

//OLD COM BOARD {0xC4, 0xDD, 0x57, 0x9E, 0x91, 0x6C}
// COM BOARD {0x7C, 0x9E, 0xBD, 0xD7, 0x2B, 0xE8}
//HEADERLESS BOARD {0x7C, 0x87, 0xCE, 0xF0 0x69, 0xAC}
//NEWEST COM BOARD IN EVA {0x24, 0x62, 0xAB, 0xD2, 0x85, 0xDC}
// uint8_t broadcastAddress[] = {0x24, 0x62, 0xAB, 0xD2, 0x85, 0xDC};
uint8_t broadcastAddress[] ={0x7C, 0x9E, 0xBD, 0xD7, 0x2B, 0xE8};
// {0x7C, 0x87, 0xCE, 0xF0, 0x69, 0xAC};
//{0x3C, 0x61, 0x05, 0x4A, 0xD5, 0xE0};
// {0xC4, 0xDD, 0x57, 0x9E, 0x96, 0x34}

//STATEFLOW VARIABLES
enum STATES {IDLE, ARMED, FILL, PRESS_ETH, PRESS_LOX, QD, IGNITION, HOTFIRE, ABORT, DEBUG=99};
int state;
int loopStartTime=0;
int lastPrintTime=0;

short int queueLength=0;
int commandedState;

int igniterTime=750;
int hotfireTimer=0;
int igniterTimer=0;

float startTime;
float endTime;
float timeDiff;
float sendTime;

// Variable to store if sending data was successful
String success;

// Define variables to store readings to be sent
int messageTime=10;
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
bool fillComplete=false;
bool pressEthComplete=false;
bool pressLOXComplete=false;

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

  pinMode(RELAYPIN_LOXFILL, OUTPUT);
  pinMode(RELAYPIN_VENT, OUTPUT);
  pinMode(RELAYPIN_QD, OUTPUT);
  pinMode(RELAYPIN_PRESSETH, OUTPUT);
  pinMode(RELAYPIN_PRESSLOX, OUTPUT);
  pinMode(RELAYPIN_IGNITER, OUTPUT);
  pinMode(RELAYPIN_PYROETH, OUTPUT);
  pinMode(RELAYPIN_PYROLOX, OUTPUT);

  //EVERYTHING SHOULD BE WRITTEN HIGH EXCEPT QDs, WHICH SHOULD BE LOW
  digitalWrite(RELAYPIN_LOXFILL, HIGH);
  digitalWrite(RELAYPIN_VENT, HIGH);
  digitalWrite(RELAYPIN_QD, LOW);
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
  scale5.begin(PTOUT5, CLKPT5); scale5.set_gain(64);
  scale6.begin(LCOUT1, CLKLC1); scale6.set_gain(64);
  scale7.begin(LCOUT2, CLKLC2); scale7.set_gain(64);
  scale8.begin(LCOUT3, CLKLC3); scale8.set_gain(64);

  pinMode(TC1, INPUT);
  pinMode(TC2, INPUT);

  pinMode(CAPSENS1DATA, INPUT);
  pinMode(CAPSENS1CLK, OUTPUT);
  pinMode(CAPSENS2DATA, INPUT);
  pinMOde(CAPSENS2CLK, OUTPUT);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  //Print MAC Accress on startup for easier connections
  Serial.println(WiFi.macAddress());

  // Init ESP-NOW
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
}

// Implementation of State Machine
void loop() {
  loopStartTime=millis();

  switch (state) {

  case (IDLE):
    idle();
    if (commandedState==ARMED) {state=ARMED;}
    if (commandedState==ABORT) {state=ABORT;}
    break;

  case (ARMED): //NEED TO ADD TO CASE OPTIONS //ALLOWS OTHER CASES TO TRIGGER //INITIATE TANK PRESS LIVE READINGS
    armed();
    if (commandedState==FILL) {state=FILL;}
    if (commandedState==ABORT) {state=ABORT;}
    break;

  case (FILL):
    fillComplete = fill();
    if (commandedState==PRESS_ETH && fillComplete) {state=PRESS_ETH;}
    if (commandedState==IDLE) {state = IDLE;}
    if (commandedState==ABORT) {state = ABORT;}
    break;

  case (PRESS_ETH):
    pressEthComplete = press_eth();
    if (pressEthComplete) {state=PRESS_LOX;}
    if (commandedState==IDLE) {state=IDLE;}
    if (commandedState==ABORT) {state=ABORT;}
    break;

  case (PRESS_LOX):
    pressLOXComplete = press_lox();
    if (commandedState==QD && pressLOXComplete) {state=QD;}
    if (commandedState==IDLE) {state=IDLE;}
    if (commandedState==ABORT) {state=ABORT;}
    break;

  case (QD):
    quick_disconnect();
    if (commandedState==IGNITION) {state=IGNITION;}
    if (commandedState==IDLE) {state=IDLE;}
    if (commandedState==ABORT) {state=ABORT;}
    break;

  case (IGNITION): 
    ignition();
    if (commandedState==HOTFIRE) {state=HOTFIRE;}
    if (commandedState==IDLE) {state=IDLE;}
    if (commandedState==ABORT) {state=ABORT;}
    break;

  case (HOTFIRE): 
    hotfire();
    if (commandedState==IDLE) {state=IDLE;}
    if (commandedState==ABORT) {state=ABORT;}
    break;

  case (ABORT):
    abort_sequence();
    if (commandedState==IDLE) {state=IDLE;}
    break;

  case (DEBUG):
    debug();
    if (commandedState==IDLE) {state=IDLE;}
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

bool fill() {
  while (!capSens()) {
    digitalWrite(RELAYPIN_LOXFILL, LOW);
    sendData();
  }
  digitalWrite(RELAYPIN_LOX_FULL, HIGH);
}

bool press_eth() {
  getReadings();
  while (readingPT2 < BangBangPressEth) {
    openSolenoidEth();
    if (commandedState == ABORT) {
      closeSolenoidEth();
      vent();
      state = ABORT;
      return false;
    }
    if (!sendData()) {
      getReadings();
    }
  }
  closeSolenoidEth();
  // Address potential overshoot & hold pressure (within tolerance)
  sleep(pressureDelay);
  while (readingPT2 < pressureEth && readingPT2 > BangBangPressEth) {
    openSolenoidEth();
    if (commandedState = ABORT) {
      closeSolenoidEth();
      vent();
      state = ABORT;
      return false;
    }
    delay(period*1000); //Delay in ms
    closeSolenoidEth();
    delay((1-period) * 1000);
    if (!sendData()) {
      getReadings();
    }
  }
  return true;
}

bool press_lox() {
  // Increase pressure
  getReadings();
  while (readingPT1 < BangBangPressOx) {
    openSolenoidOx();
    if (commandedState == ABORT) {
      closeSolenoidOx();
      vent();
      state = ABORT;
      return false;
    }
    if (!sendData()) {
      getReadings();
    }
  }
  closeSolenoidOx();
  // Address potential overshoot & hold pressure (within tolerance)
  sleep(pressureDelay);
  while (readingPT1 < pressureOx && readingPT1 > BangBangPressOx) {
    openSolenoidOx();
    if (commandedState = ABORT) {
      closeSolenoidOx();
      vent();
      state = ABORT;
      return false;
    }
    delay(period*1000); //Delay in ms
    closeSolenoidOx();
    delay((1-period) * 1000);
    if (!sendData()) {
      getReadings();
    }
  }
  return true;
}

void quick_disconnect() {
  digitalWrite(RELAYPIN_QD, HIGH);
}

void ignition() {
  if ((loopStartTime-igniterTimer) < igniterTime) { digitalWrite(RELAYPIN1, LOW); digitalWrite(RELAYPIN2, LOW); Serial.print("IGNITE"); }
  if ((loopStartTime-igniterTimer) > igniterTime) { digitalWrite(RELAYPIN1, HIGH); digitalWrite(RELAYPIN2, HIGH); Serial.print("NO"); }
  sendData();

  Serial.println(loopStartTime-igniterTimer);
  Serial.println("Igniter time");
  Serial.println(igniterTime);
  Serial.println(" ");
}

void hotfire() {
  //FILL IN
}

void abort_sequence() {
  getReadings();
  // Waits for LOX pressure to decrease before venting Eth through pyro
  while (readingPT1 > 50) {
    digitalWrite(RELAYPIN_VENT, LOW);
    getReadings();
  }
  digitalWrite(RELAYPIN_PYROETH, HIGH);

  
}

void debug() {
  //FILL IN
}


/// END OF STATE FUNCTION DEFINITIONS ///



/// HELPER FUNCTIONS ///

// Returns boolean for whether LOX tank is fill by reading cap sensor
bool capSens() {
  if (analogRead(CAPSENS1DATA) >= CAPTHRESH1 && analogRead(CAPSENS2DATA) >= CAPTHRESH2) {
    return true;
  }
  return false;
}

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

void disconnectQD() {
  digitalWrite(RELAYPIN_QD, HIGH);
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
    ReadingsQueue[queueLength].messageTime=loopStartTime;
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
    ReadingsQueue[queueLength].cap1=readingCap1;
    ReadingsQueue[queueLength].cap2=readingCap2;
    ReadingsQueue[queueLength].queueSize=queueLength;
  }
}

void getReadings(){

  readingPT1 = PTOffset1 + PTSlope1 * scale1.read(); 
  readingPT2 = PTOffset2 + PTSlope2 * scale2.read(); 
  readingPT3 = PTOffset3 + PTSlope3 * scale3.read(); 
  readingPT4 = PTOffset4 + PTSlope4 * scale4.read(); 
  readingPT5 = PTOffset5 + PTSlope5 * scale5.read(); 
  readingLC1 = LCOffset1 + LCSlope1 * scale6.read(); 
  readingLC2 = LCOffset2 + LCSlope2 * scale7.read();
  readingLC3 = LCOffset3 + LCSlope3 * scale8.read();
  readingTC1 = analogRead(TC1);
  readingTC2 = analogRead(TC2);
  readingCap1 = analogRead(capSens1);
  readingCap2 = analogRead(capSens2);

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
 serialMessage.concat(readingCap1);
 serialMessage.concat(" ");
 serialMessage.concat(readingCap2);
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
  Readings.cap1 = ReadingsQueue[queueLength].cap1;
  Readings.cap2 = ReadingsQueue[queueLength].cap2;

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