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
// Set i2c address
PCF8575 pcf8575(0x20);

//::::::Global Variables::::::://

// DEBUG TRIGGER: SET TO 1 FOR DEBUG MODE.
// MOSFET must not trigger while in debug.
int DEBUG = 1;      // Simulate LOX and Eth fill.
int WIFIDEBUG = 1;  // Don't send/receive data.


float readDelay = 25;     // Frequency of data collection [ms]
float sendDelay = readDelay;
// END OF USER DEFINED PARAMETERS //
// refer to https://docs.google.com/spreadsheets/d/17NrJWC0AR4Gjejme-EYuIJ5uvEJ98FuyQfYVWI3Qlio/edit#gid=1185803967 for all pinouts


//::::::DEFINE INSTRUMENT PINOUTS::::::://
#define TimeOut 100

struct MovingMedianFilter {
private:
  const unsigned BUFFER_SIZE = 5;
  RunningMedian medianFilter;

public:
  MovingMedianFilter()
    : medianFilter(BUFFER_SIZE) {
  }

  void addReading(float newReading) {
    medianFilter.add(newReading);
  }

  float getReading() {
    return medianFilter.getMedian();
  }

  void resetReadings() {
    medianFilter.clear();
  }
};

template<class Board>
struct struct_data_board {
private:
  MovingMedianFilter filter = MovingMedianFilter();

public:
  Board scale;
  float offset;
  float slope;
  float filteredReading = -1;
  float rawReading = -1;

  struct_data_board(Board scale, float offset, float slope)
    : scale(scale) {
    this->scale = scale;
    this->offset = offset;
    this->slope = slope;
  }

  virtual float readRawFromBoard() {
    return analogRead(36); // This is arbitrary, function should ALWAYS be overriden in child class
  }

  void readDataFromBoard() {
    float newReading = readRawFromBoard();
    filter.addReading(newReading);

    rawReading = slope * newReading + offset;
    filteredReading = slope * filter.getReading() + offset;
  }

  void resetReading() {
    filter.resetReadings();
    filteredReading = -1;
    rawReading = -1;
  }
};
/*
struct struct_hx711 : struct_data_board<HX711> {
public:
  int clk;
  int gpio;

  struct_hx711(HX711 scale, int clk, int gpio, float offset, float slope)
    : struct_data_board(scale, offset, slope) {
    this->clk = clk;
    this->gpio = gpio;
  }

  float readRawFromBoard() override {
    if (scale.wait_ready_timeout(TimeOut)) {
      return scale.read();
    }
    return (rawReading - offset) / slope;
  }
};

struct struct_max31855 : struct_data_board<Adafruit_MAX31855> {
public:
  int cs;

  struct_max31855(Adafruit_MAX31855 scale, float cs, float offset, float slope)
    : struct_data_board(scale, offset, slope) {
    this->cs = cs;
  }

  float readRawFromBoard() override {
    return scale.readCelsius();
  }
};
*/


//sensor stuff

#define HX_CLK 27

struct_hx711 PT_O1{ {}, HX_CLK, 36, .offset = -115.9, .slope = 0.0110 }; //.offset = -71.93, .slope = 0.00822
struct_hx711 PT_O2{ {}, HX_CLK, 39, .offset = -81.62, .slope = 0.00710 };
struct_hx711 PT_E1{ {}, HX_CLK, 34, .offset = -130.26, .slope = 0.0108 };
struct_hx711 PT_E2{ {}, HX_CLK, 35, .offset = -62.90, .slope = 0.00763 };  // Change GPIO PIN
struct_hx711 PT_C1{ {}, HX_CLK, 32, .offset = -78.422, .slope = 0.00642};

// LOADCELLS
struct_hx711 LC_1{ {}, HX_CLK, 33, .offset = 0, .slope = 1 };
struct_hx711 LC_2{ {}, HX_CLK, 25, .offset = 0, .slope = 1 };
struct_hx711 LC_3{ {}, HX_CLK, 26, .offset = 0, .slope = 1 };

#define TC_CLK 14
#define TC4_CLK 18
#define TC_DO 13
#define TC4_DO 23


#define SD_CLK 18
#define SD_DO 23

struct_max31855 TC_1{ Adafruit_MAX31855(TC_CLK, 17, TC_DO), 17, .offset = 0, .slope = 1 };
struct_max31855 TC_2{ Adafruit_MAX31855(TC_CLK, 16, TC_DO), 16, .offset = 0, .slope = 1 };
struct_max31855 TC_3{ Adafruit_MAX31855(TC_CLK, 4, TC_DO), 4, .offset = 0, .slope = 1 };
struct_max31855 TC_4{ Adafruit_MAX31855(TC4_CLK, 15, TC4_DO), 15, .offset = 0, .slope = 1 };


// GPIO expander
#define I2C_SDA 21

#define I2C_SCL 22


// Initialize mosfets' io expander.
//#define MOSFET_PCF_ADDR 0x20
//EasyPCF8575 mosfet_pcf;
bool mosfet_pcf_found;
int hotfireStart;

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
  float TC_3;
  float TC_4;
  short int queueLength;
}struct_message;

// Create a struct_message called Packet to be sent to the DAQ Power.
struct_message Packet;
// Create a queue for Packet in case Packets are dropped.
struct_message PacketQueue[120];

//::::::Broadcast Variables::::::://
esp_now_peer_info_t peerInfo;
// REPLACE WITH THE MAC Address of your receiver

// OLD COM BOARD {0xC4, 0xDD, 0x57, 0x9E, 0x91, 0x6C}
// COM BOARD {0x7C, 0x9E, 0xBD, 0xD7, 0x2B, 0xE8}
// HEADERLESS BOARD {0x7C, 0x87, 0xCE, 0xF0 0x69, 0xAC}
// NEWEST COM BOARD IN EVA {0x24, 0x62, 0xAB, 0xD2, 0x85, 0xDC}
// uint8_t broadcastAddress[] = {0x24, 0x62, 0xAB, 0xD2, 0x85, 0xDC};
//uint8_t broadcastAddress[] = {0xC8, 0xF0, 0x9E, 0x4F, 0xAF, 0x40};
uint8_t broadcastAddress[] = {0xB0, 0xA7, 0x32, 0xDE, 0xC1, 0xFC};
// uint8_t broadcastAddress[] = {0x48, 0xE7, 0x29, 0xA3, 0x0D, 0xA8}; // TEST
// uint8_t broadcastAddress[] = { 0x48, 0xE7, 0x29, 0xA3, 0x0D, 0xA8 }; // TEST COM
// {0x7C, 0x87, 0xCE, 0xF0, 0x69, 0xAC};
// {0x3C, 0x61, 0x05, 0x4A, 0xD5, 0xE0};
// {0xC4, 0xDD, 0x57, 0x9E, 0x96, 0x34};
// Callback when data is sent
// void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
//  sendTime = millis();
// }


// Initialize all sensors and parameters. sense board fs
void setup() {
  // pinMode(ONBOARD_LED,OUTPUT);
  Serial.begin(115200);
  while (!Serial) delay(1);  // wait for Serial on Leonardo/Zero, etc.

  // HX711.
  PT_O1.scale.begin(PT_O1.gpio, PT_O1.clk);
  PT_O1.scale.set_gain(64);
  PT_O2.scale.begin(PT_O2.gpio, PT_O2.clk);
  PT_O2.scale.set_gain(64);
  PT_E1.scale.begin(PT_E1.gpio, PT_E1.clk);
  PT_E1.scale.set_gain(64);
  PT_E2.scale.begin(PT_E2.gpio, PT_E2.clk);
  PT_E2.scale.set_gain(64);
  PT_C1.scale.begin(PT_C1.gpio, PT_C1.clk);
  PT_C1.scale.set_gain(64);
  LC_1.scale.begin(LC_1.gpio, LC_1.clk);
  LC_1.scale.set_gain(64);
  LC_2.scale.begin(LC_2.gpio, LC_2.clk);
  LC_2.scale.set_gain(64);
  LC_3.scale.begin(LC_3.gpio, LC_3.clk);
  LC_3.scale.set_gain(64);

  // Thermocouple.
  pinMode(TC_1.cs, OUTPUT);
  pinMode(TC_2.cs, OUTPUT);
  pinMode(TC_3.cs, OUTPUT);
  pinMode(TC_4.cs, OUTPUT);

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
esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  if (!WIFIDEBUG) {
    esp_now_register_recv_cb(OnDataRecv);
  }

  sendTime = millis();
}


//::::::STATE MACHINE::::::://
/*
// Main Structure of State Machine.
void loop() {
  //  Serial.print("looping");
  logData();
  //  Serial.print("logged");
  switch (DAQState) {
    case (IDLE):
      sendDelay = IDLE_DELAY;
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
  PT_O1.resetReading();
  PT_O2.resetReading();
  PT_E1.resetReading();
  PT_E2.resetReading();
  PT_C1.resetReading();
  LC_1.resetReading();
  LC_2.resetReading();
  LC_3.resetReading();
  TC_1.resetReading();
  TC_2.resetReading();
  TC_3.resetReading();
  TC_4.resetReading();
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
  if (!(oxComplete && ethComplete)) {
    if (PT_O1.rawReading < pressureOx * threshold) {
      mosfetOpenValve(MOSFET_LOX_PRESS);
      if (DEBUG) {
        PT_O1.rawReading += (0.00075 * GEN_DELAY);
      }
    } else {
      mosfetCloseValve(MOSFET_LOX_PRESS);
      oxComplete = true;
    }
    if (PT_E1.rawReading < pressureFuel * threshold) {
      mosfetOpenValve(MOSFET_ETH_PRESS);
      if (DEBUG) {
        PT_E1.rawReading += (0.001 * GEN_DELAY);
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
  mosfetCloseValve(MOSFET_ETH_PRESS); //close press valves
  mosfetCloseValve(MOSFET_LOX_PRESS);
  mosfetOpenValve(MOSFET_QD_LOX);
  mosfetOpenValve(MOSFET_QD_ETH);
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
*/
/*
void abort_sequence() {
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
  if (PT_O1.filteredReading > 1.3 * ventTo) {  // 1.3 is magic number.
    oxVentComplete = false;
  }
  if (PT_E1.filteredReading > 1.3 * ventTo) {  // 1.3 is magic number.
    ethVentComplete = false;
  }

  if (!(oxVentComplete && ethVentComplete)) {
    if (PT_O1.filteredReading > ventTo) {  // vent only lox down to vent to pressure
      mosfetOpenValve(MOSFET_VENT_LOX);
      if (DEBUG) {
        PT_O1.rawReading = PT_O1.rawReading - (0.0005 * GEN_DELAY);
      }
    } else {                              // lox vented to acceptable hold pressure
      mosfetCloseValve(MOSFET_VENT_LOX);  // close lox
      oxVentComplete = true;
    }
    if (PT_E1.filteredReading > ventTo) {
      mosfetOpenValve(MOSFET_VENT_ETH);  // vent ethanol
      if (DEBUG) {
        PT_E1.rawReading = PT_E1.rawReading - (0.0005 * GEN_DELAY);
      }
    } else {
      mosfetCloseValve(MOSFET_VENT_ETH);
      ethVentComplete = true;
    }
  }
  if (DEBUG) {
    PT_O1.rawReading = PT_O1.rawReading + (0.00005 * GEN_DELAY);
    PT_E1.rawReading = PT_E1.rawReading + (0.00005 * GEN_DELAY);
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
  if (COMState == ABORT || PT_O1.filteredReading >= abortPressure || PT_E1.filteredReading >= abortPressure) {
    mosfetCloseValve(MOSFET_ETH_PRESS);
    mosfetCloseValve(MOSFET_LOX_PRESS);
    DAQState = ABORT;
  }
}
*/

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



//::::::DATA LOGGING AND COMMUNICATION::::::://
void logData() {
  getReadings();
  printSensorReadings();
  if (millis() - sendTime > sendDelay) {
    sendTime = millis();
    sendData();
    // saveData();
  }
}

void getReadings() {
  if (!DEBUG) {
    PT_O1.readDataFromBoard();
    PT_O2.readDataFromBoard();
    PT_E1.readDataFromBoard();
    PT_E2.readDataFromBoard();
    PT_C1.readDataFromBoard();
    LC_1.readDataFromBoard();
    LC_2.readDataFromBoard();
    LC_3.readDataFromBoard();
    TC_1.readDataFromBoard();
    TC_2.readDataFromBoard();
    TC_3.readDataFromBoard();
    TC_4.readDataFromBoard();
  }
}

void printSensorReadings() {
  serialMessage = " ";
  serialMessage.concat(millis());
  serialMessage.concat(" ");
  serialMessage.concat(PT_O1.filteredReading);
  serialMessage.concat(" ");
  serialMessage.concat(PT_O2.filteredReading);
  serialMessage.concat(" ");
  serialMessage.concat(PT_E1.filteredReading);
  serialMessage.concat(" ");
  serialMessage.concat(PT_E2.filteredReading);
  serialMessage.concat(" ");
  serialMessage.concat(PT_C1.filteredReading);
  serialMessage.concat(" ");
  serialMessage.concat(LC_1.filteredReading);
  serialMessage.concat(" ");
  serialMessage.concat(LC_2.filteredReading);
  serialMessage.concat(" ");
  serialMessage.concat(LC_3.filteredReading);
  serialMessage.concat(" ");
  serialMessage.concat(TC_1.filteredReading);
  serialMessage.concat(" ");
  serialMessage.concat(TC_2.filteredReading);
  serialMessage.concat(" ");
  serialMessage.concat(TC_3.filteredReading);
  serialMessage.concat(" ");
  serialMessage.concat(TC_4.filteredReading);
  serialMessage.concat(" ");
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
    PacketQueue[queueLength].PT_O1 = PT_O1.rawReading;
    PacketQueue[queueLength].PT_O2 = PT_O2.rawReading;
    PacketQueue[queueLength].PT_E1 = PT_E1.rawReading;
    PacketQueue[queueLength].PT_E2 = PT_E2.rawReading;
    PacketQueue[queueLength].PT_C1 = PT_C1.rawReading;
    PacketQueue[queueLength].LC_1 = LC_1.rawReading;
    PacketQueue[queueLength].LC_2 = LC_2.rawReading;
    PacketQueue[queueLength].LC_3 = LC_3.rawReading;
    PacketQueue[queueLength].TC_1 = TC_1.rawReading;
    PacketQueue[queueLength].TC_2 = TC_2.rawReading;
    PacketQueue[queueLength].TC_3 = TC_3.rawReading;
    PacketQueue[queueLength].TC_4 = TC_4.rawReading;

    // PacketQueue[queueLength].TC_3 = TC_3.rawReading; // sinc daq and com when adding tcs
    PacketQueue[queueLength].queueLength = queueLength;
    PacketQueue[queueLength].DAQState = DAQState;
    PacketQueue[queueLength].oxComplete = oxComplete;
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
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&Packet, sizeof(Packet));

    if (result == ESP_OK) {
      // Serial.println("Sent with success Data Send");
      queueLength -= 1;
    } else {
      Serial.println("Error sending the data");
    }
  }
}
