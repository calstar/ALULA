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


#include "PCF8575.h"  // https://github.com/xreef/PCF8575_library
 
// Set i2c address
PCF8575 pcf8575(0x20);

//::::::Global Variables::::::://


// DEBUG TRIGGER: SET TO 1 FOR DEBUG MODE.
// MOSFET must not trigger while in debug.
int DEBUG = 1;     // Simulate LOX and Eth fill.
int WIFIDEBUG = 0; // Don't send/receive data.

// MODEL DEFINED PARAMETERS FOR TEST/HOTFIRE. Pressures in psi //
float pressureFuel  = 70;//405;  // Set pressure for fuel: 412
float pressureOx    = 70;//460;  // Set pressure for lox: 445
float threshold     = 0.995; // re-psressurrization threshold (/1x)
float ventTo        = 5;   // c2se solenoids at this pressure to preserve lifetime.
#define abortPressure 525   // Cutoff pressure to automatically trigger abort
#define period        0.5   // Sets period for bang-bang control
float sendDelay     = 250;  // Sets frequency of data collection. 1/(sendDelay*10^-3) is frequency in Hz
// END OF USER DEFINED PARAMETERS //
// refer to https://docs.google.com/spreadsheets/d/17NrJWC0AR4Gjejme-EYuIJ5uvEJ98FuyQfYVWI3Qlio/edit#gid=1185803967 for all pinouts


//::::::DEFINE INSTRUMENT PINOUTS::::::://

typedef struct struct_hx711 {
  HX711 scale;
  float reading;
  int clk;
  int gpio;
  float offset;
  float slope;
} struct_hx711;

#define HX_CLK 27

// PRESSURE TRANSDUCERS
struct_hx711 PT_O1 {{}, -1, HX_CLK, 36, .offset=-36.5, .slope=0.01074};
struct_hx711 PT_O2 {{}, -1, HX_CLK, 39, .offset=-48.3, .slope=0.009309};
struct_hx711 PT_E1 {{}, -1, HX_CLK, 34, .offset=-70.8, .slope=0.009041};
struct_hx711 PT_E2 {{}, -1, HX_CLK, 35, .offset=-55.5, .slope=0.009588}; // Change GPIO PIN
struct_hx711 PT_C1 {{}, -1, HX_CLK, 32, .offset=-79.2, .slope=0.009753};

// LOADCELLS
struct_hx711 LC_1  {{}, -1, HX_CLK, 33, .offset=0, .slope=1};
struct_hx711 LC_2  {{}, -1, HX_CLK, 25, .offset=0, .slope=1};
struct_hx711 LC_3  {{}, -1, HX_CLK, 26, .offset=0, .slope=1};

// THERMOCOUPLES
typedef struct struct_max31855 {
  Adafruit_MAX31855 scale;
  float reading;
  float cs;
  float offset;
  float slope;
} struct_max31855;

#define TC_CLK 14
#define TC_DO  13

#define SD_CLK 18
#define SD_DO  23

struct_max31855 TC_1 {Adafruit_MAX31855(TC_CLK, 17, TC_DO), -1, 17, .offset=0, .slope=0};
struct_max31855 TC_2 {Adafruit_MAX31855(TC_CLK, 16, TC_DO), -1, 16, .offset=0, .slope=0};
struct_max31855 TC_3 {Adafruit_MAX31855(TC_CLK, 4, TC_DO), -1, 4, .offset=0, .slope=0};
struct_max31855 TC_4 {Adafruit_MAX31855(TC_CLK, 15, TC_DO), -1, 15, .offset=0, .slope=0};

// GPIO expander
#define I2C_SDA 211

#define I2C_SCL 22

////////////////////////////// MOSFETS ///////////////////////////////////////////////////////////////////
#define MOSFET_ETH_MAIN   7 //P07 
#define MOSFET_ETH_PRESS 6 //P06
#define MOSFET_VENT_ETH  5 //P05
#define MOSFET_EXTRA 4 //CAN USE THIS PIN FOR ANYTHING JUST CHANGE ASSIGNMENT AND HARNESS
#define MOSFET_QD_LOX     3 //P03
#define MOSFET_IGNITER    8 //P10
#define MOSFET_LOX_MAIN  9 //P11
#define MOSFET_LOX_PRESS 10 //P12
#define MOSFET_VENT_LOX  11 //P13
#define MOSFET_QD_ETH     12 //P14


// Initialize mosfets' io expander.
//#define MOSFET_PCF_ADDR 0x20
//EasyPCF8575 mosfet_pcf;
bool mosfet_pcf_found;

//::::::STATE VARIABLES::::::://
enum STATES {IDLE, ARMED, PRESS, QD, IGNITION, HOTFIRE, ABORT};
String state_names[] = {"Idle", "Armed", "Press", "QD", "Ignition", "HOTFIRE", "Abort"};
int COMState;
int DAQState;
bool ethComplete = false;
bool oxComplete = false;
bool oxVentComplete = false;
bool ethVentComplete = false;
int hotfireStart;

// Delay between loops.
#define IDLE_DELAY 250
#define GEN_DELAY 20


//::::DEFINE READOUT VARIABLES:::://
String serialMessage;
float sendTime;
short int queueLength = 0;

// Define variables to store readings to be sent

// Structure example to send data.
// Must match the receiver structure.
typedef struct struct_message {
  int messageTime;
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
  int COMState;
  int DAQState;
  short int queueLength;
  bool ethComplete;
  bool oxComplete;
  // bool oxvent;
  // bool ethVent;
  // bool VentComplete;
} struct_message;

// Create a struct_message called Packet to be sent.
struct_message Packet;
// Create a queue for Packet in case Packets are dropped.
struct_message PacketQueue[120];

// Create a struct_message to hold incoming commands
struct_message Commands;


//::::::Broadcast Variables::::::://
esp_now_peer_info_t peerInfo;
// REPLACE WITH THE MAC Address of your receiver

// OLD COM BOARD {0xC4, 0xDD, 0x57, 0x9E, 0x91, 0x6C}
// COM BOARD {0x7C, 0x9E, 0xBD, 0xD7, 0x2B, 0xE8}
// HEADERLESS BOARD {0x7C, 0x87, 0xCE, 0xF0 0x69, 0xAC}
// NEWEST COM BOARD IN EVA {0x24, 0x62, 0xAB, 0xD2, 0x85, 0xDC}
// uint8_t broadcastAddress[] = {0x24, 0x62, 0xAB, 0xD2, 0x85, 0xDC};
uint8_t broadcastAddress[] = {0xB0, 0xA7, 0x32, 0xDE, 0xC1, 0xFC};
// {0x7C, 0x87, 0xCE, 0xF0, 0x69, 0xAC};
// {0x3C, 0x61, 0x05, 0x4A, 0xD5, 0xE0};
// {0xC4, 0xDD, 0x57, 0x9E, 0x96, 0x34};
// Callback when data is sent
// void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
//  sendTime = millis();
// }

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&Commands, incomingData, sizeof(Commands));
  COMState = Commands.COMState;
}




// Initialize all sensors and parameters.
void setup() {
  // pinMode(ONBOARD_LED,OUTPUT);
  
  Serial.begin(115200);

  while (!Serial) delay(1); // wait for Serial on Leonardo/Zero, etc.

  // HX711.
  PT_O1.scale.begin(PT_O1.gpio, PT_O1.clk); PT_O1.scale.set_gain(64);
  PT_O2.scale.begin(PT_O2.gpio, PT_O2.clk); PT_O2.scale.set_gain(64);
  PT_E1.scale.begin(PT_E1.gpio, PT_E1.clk); PT_E1.scale.set_gain(64);
  PT_E2.scale.begin(PT_E2.gpio, PT_E2.clk); PT_E2.scale.set_gain(64);
  PT_C1.scale.begin(PT_C1.gpio, PT_C1.clk); PT_C1.scale.set_gain(64);
  LC_1.scale.begin(LC_1.gpio, LC_1.clk);    LC_1.scale.set_gain(64);
  LC_2.scale.begin(LC_2.gpio, LC_2.clk);    LC_2.scale.set_gain(64);
  LC_3.scale.begin(LC_3.gpio, LC_3.clk);    LC_3.scale.set_gain(64);

  // Thermocouple.
  pinMode(TC_1.cs, OUTPUT);
  pinMode(TC_2.cs, OUTPUT);
  pinMode(TC_3.cs, OUTPUT);
  pinMode(TC_4.cs, OUTPUT);

  // MOSFET.
//  mosfet_pcf.startI2C(I2C_SDA, I2C_SCL, MOSFET_PCF_ADDR); // Only SEARCH, if using normal pins in Arduino
  mosfet_pcf_found = true;
  
    // Set pinMode to OUTPUT
  for(int i=0;i<16;i++) {
    pcf8575.pinMode(i, OUTPUT);
  }
  pcf8575.begin();
  mosfet_pcf_found = true;
  mosfetCloseAllValves(); // make sure everything is off by default (NMOS: Down = Off, Up = On)
  delay(500); // startup time to make sure its good for personal testing

  // Broadcast setup.
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  // Print MAC Accress on startup for easier connections
  Serial.println(WiFi.macAddress());

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  // esp_now_register_send_cb(OnDataSent);

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
  if (!WIFIDEBUG) {
    esp_now_register_recv_cb(OnDataRecv);
  }

  sendTime = millis();
  DAQState = IDLE;
}


//::::::STATE MACHINE::::::://

// Main Structure of State Machine.
void loop() {
  fetchCOMState();
  if (DEBUG || COMState == ABORT) {
    syncDAQState();
  }
//  Serial.print("testing");
  logData();
//  Serial.print("made it");
  sendDelay = GEN_DELAY;
  switch (DAQState) {
    case (IDLE):
      sendDelay = IDLE_DELAY;
      if (COMState == ARMED) { syncDAQState(); }
      idle();
      break;

    case (ARMED): // NEED TO ADD TO CASE OPTIONS //ALLOWS OTHER CASES TO TRIGGER //INITIATE TANK PRESS LIVE READINGS
      if (COMState == IDLE || COMState == PRESS) { syncDAQState(); }
      armed();
      break;

    case (PRESS):
      if (COMState == IDLE || (COMState == QD && oxComplete && ethComplete)) {
        syncDAQState();
        int QDStart = millis();
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
  PT_O1.reading = -1;
  PT_O2.reading = -1;
  PT_E1.reading = -1;
  PT_E2.reading = -1;
  PT_C1.reading = -1;
  LC_1.reading = -1;
  LC_2.reading = -1;
  LC_3.reading = -1;
  // TC_1.reading = -1;
  // TC_2.reading = -1;
  // TC_2.reading = -1;
}

void idle() {

  mosfetCloseAllValves();
  reset(); // must set oxComplete and ethComplete to false!
}

// Oxygen and fuel should not flow yet.
// This function is the same as idle?
void armed() {
  // mosfetCloseValve(MOSFET_LOX_PRESS);
  // mosfetCloseValve(MOSFET_ETH_PRESS);
  mosfetCloseAllValves();
}

void press() {
  if (!(oxComplete && ethComplete)) {
    if (PT_O1.reading < pressureOx*threshold) {
      mosfetOpenValve(MOSFET_LOX_PRESS);
      if (DEBUG) {
        PT_O1.reading += (0.0005*GEN_DELAY);
      }
    } else {
      mosfetCloseValve(MOSFET_LOX_PRESS);
      oxComplete = true;
    }
    if (PT_E1.reading < pressureFuel*threshold) {
      mosfetOpenValve(MOSFET_ETH_PRESS);
      if (DEBUG) {
        PT_E1.reading += (0.0005*GEN_DELAY);
      }
    } else {
      mosfetCloseValve(MOSFET_ETH_PRESS);
      ethComplete = true;
    }
  }
  CheckAbort();
}

// Disconnect harnessings and check state of rocket.
void quick_disconnect() {
  mosfetOpenValve(MOSFET_QD_LOX);
  mosfetOpenValve(MOSFET_QD_ETH);
  // vent valves/vent the lines themselves
  // vent the pressure solenoid for 1 full second
  //if millis() >= (QDStart+1000){
  // then, disconnect the lines from the rocket itself
    //mosfetOpenValve(MOSFET_QD_LOX);
    //mosfetOpenValve(MOSFET_QD_ETH);
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
  if (DEBUG) {
      mosfetOpenValve(MOSFET_VENT_LOX);
      mosfetOpenValve(MOSFET_VENT_ETH); 
      delay(50);}
  // Waits for LOX pressure to decrease before venting Eth through pyro
  mosfetCloseValve(MOSFET_LOX_PRESS);
  mosfetCloseValve(MOSFET_ETH_PRESS);
  mosfetCloseValve(MOSFET_LOX_MAIN);  
  mosfetCloseValve(MOSFET_ETH_MAIN);  
  mosfetCloseValve(MOSFET_IGNITER);
//
  int currtime = millis();
  if (PT_O1.reading > 1.3*ventTo) { // 1.3 is magic number.
        oxVentComplete = false; }
  if (PT_E1.reading > 1.3*ventTo) { // 1.3 is magic number.
        ethVentComplete = false; }  
      
  if(!(oxVentComplete && ethVentComplete)){
    if (PT_O1.reading > ventTo) { // vent only lox down to vent to pressure
      mosfetOpenValve(MOSFET_VENT_LOX);
      if (DEBUG) {
        PT_O1.reading = PT_O1.reading - (0.0005*GEN_DELAY);
      }
    }
    else { // lox vented to acceptable hold pressure
        mosfetCloseValve(MOSFET_VENT_LOX); // close lox
        oxVentComplete = true;
      }
    if (PT_E1.reading > ventTo) {
      mosfetOpenValve(MOSFET_VENT_ETH); // vent ethanol
      if (DEBUG) {
        PT_E1.reading = PT_E1.reading - (0.0005*GEN_DELAY);
      }
    } else {
      mosfetCloseValve(MOSFET_VENT_ETH);
      ethVentComplete = true;
    }
    }
  if (DEBUG) {
    PT_O1.reading = PT_O1.reading + (0.00005*GEN_DELAY);
    PT_E1.reading = PT_E1.reading + (0.00005*GEN_DELAY);
  }
  }

//// Helper Functions
//
//// Get commanded state from COM board.
void fetchCOMState() {
  // Actually, COMState will be updated in the OnDataRec function.
  if (Serial.available() > 0) {
    // Serial.read reads a single character as ASCII. Number 1 is 49 in ASCII.
    // Serial sends character and new line character "\n", which is 10 in ASCII.
    int SERIALState = Serial.read() - 48;
    if (SERIALState >= 0 && SERIALState <= 9) {
      COMState = SERIALState;
    }
  }
}

// Sync state of DAQ board with COM board.
void syncDAQState() {
  DAQState = COMState;
}

void CheckAbort() {
  if (COMState == ABORT || PT_O1.reading >= abortPressure || PT_E1.reading >= abortPressure) {
    mosfetCloseValve(MOSFET_ETH_PRESS);
    mosfetCloseValve(MOSFET_LOX_PRESS);
    DAQState = ABORT;
  }
}

void mosfetCloseAllValves(){
  if (mosfet_pcf_found /*&& !DEBUG*/) {
      for(int i=0;i<16;i++) {
      pcf8575.digitalWrite(i, LOW);
  }
  }
}

void mosfetCloseValve(int num){
  if (mosfet_pcf_found/* && !DEBUG*/) {
      pcf8575.digitalWrite(num, LOW);
  }
}

void mosfetOpenValve(int num){
  if (mosfet_pcf_found) {
      pcf8575.digitalWrite(num, HIGH);
  }
}

//::::::DATA LOGGING AND COMMUNICATION::::::://
void logData() {
  getReadings();
  printSensorReadings();
  if (millis()-sendTime > sendDelay) {
    sendTime = millis();
    sendData();
    // saveData();
  }
}

void getReadings(){
   if (!DEBUG){
    PT_O1.reading = PT_O1.slope * PT_O1.scale.read() + PT_O1.offset;
    PT_O2.reading = PT_O2.slope * PT_O2.scale.read() + PT_O2.offset;
    PT_E1.reading = PT_E1.slope * PT_E1.scale.read() + PT_E1.offset;
    PT_E2.reading = PT_E2.slope * PT_E2.scale.read() + PT_E2.offset;
    PT_C1.reading = PT_C1.slope * PT_C1.scale.read() + PT_C1.offset;
    LC_1.reading  = LC_1.slope  * LC_1.scale.read()  + LC_1.offset;
    LC_2.reading  = LC_2.slope  * LC_2.scale.read()  + LC_2.offset;
    LC_3.reading  = LC_3.slope  * LC_3.scale.read()  + LC_3.offset;
    // TC_1.reading = TC_1.scale.readCelsius();
    // TC_2.reading = TC_2.scale.readCelsius();
    // TC_2.reading = TC_2.scale.readCelsius();
  }
}

void printSensorReadings() {
  serialMessage = " ";
  serialMessage.concat(millis());
  serialMessage.concat(" ");
  serialMessage.concat(PT_O1.reading);
  serialMessage.concat(" ");
  serialMessage.concat(PT_O2.reading);
  serialMessage.concat(" ");
  serialMessage.concat(PT_E1.reading);
  serialMessage.concat(" ");
  serialMessage.concat(PT_E2.reading);
  serialMessage.concat(" ");
  serialMessage.concat(PT_C1.reading);
  serialMessage.concat(" ");
  serialMessage.concat(LC_1.reading);
  serialMessage.concat(" ");
  serialMessage.concat(LC_2.reading);
  serialMessage.concat(" ");
  serialMessage.concat(LC_3.reading);
  serialMessage.concat(" ");
//  serialMessage.concat(TC_1.reading);
//  serialMessage.concat(" ");
//  serialMessage.concat(TC_2.reading);
//  serialMessage.concat(" ");
  serialMessage.concat(ethComplete);
  serialMessage.concat(" ");
  serialMessage.concat(oxComplete);
  serialMessage.concat(" ");
  serialMessage.concat(state_names[DAQState]);
  //  serialMessage.concat(readingCap1);
  //  serialMessage.concat(" ");
  //  serialMessage.concat(readingCap2);
  serialMessage.concat(" Queue Length: ");
  serialMessage.concat(queueLength);
  Serial.println(serialMessage);
}

// Send data to COM board.
void sendData() {
  addPacketToQueue();
  sendQueue();
}

void addPacketToQueue() {
  if (queueLength < 40) {
    queueLength += 1;
    PacketQueue[queueLength].messageTime = millis();
    PacketQueue[queueLength].PT_O1       = PT_O1.reading;
    PacketQueue[queueLength].PT_O2       = PT_O2.reading;
    PacketQueue[queueLength].PT_E1       = PT_E1.reading;
    // PacketQueue[queueLength].PT_E2       = PT_E2.reading;
    PacketQueue[queueLength].PT_C1       = PT_C1.reading;
    PacketQueue[queueLength].LC_1        = LC_1.reading;
    PacketQueue[queueLength].LC_2        = LC_2.reading;
    PacketQueue[queueLength].LC_3        = LC_3.reading;
    PacketQueue[queueLength].TC_1        = TC_1.reading;
    PacketQueue[queueLength].TC_2        = TC_2.reading;
    // PacketQueue[queueLength].TC_3        = TC_3.reading; // sinc daq and com when adding tcs
    PacketQueue[queueLength].queueLength = queueLength;
    PacketQueue[queueLength].DAQState    = DAQState;
    PacketQueue[queueLength].oxComplete  = oxComplete;
    PacketQueue[queueLength].ethComplete = ethComplete;
  }
}

void sendQueue() {
  if (queueLength < 0) {
    return;
  }
  // Set values to send
  Packet = PacketQueue[queueLength];

  if (!WIFIDEBUG) {
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &Packet, sizeof(Packet));

    if (result == ESP_OK) {
      // Serial.println("Sent with success Data Send");
      queueLength -= 1;
    } else {
      Serial.println("Error sending the data");
    }
  }
}
