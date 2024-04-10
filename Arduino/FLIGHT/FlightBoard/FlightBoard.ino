/*
11-7-2023


This code runs on the DAQ ESP32 and has a couple of main tasks.
1. Read sensor data
2. Send sensor data to DAQ ESP32
3. Follow launch sequence actuation procedures

TO RUN:
1. Set Board to ESP32S3 Dev Module
2. Set USB-CDC ON, Flash Size 4MB
2. hold down BOOT while uploading, til done.
3. check pinouts 
4. if nothing else works, plug and unplug (LITERALLY) 
5. check if the USB is working

FOR DEBUGGING:
1. Set boolean DEBUG and/or WIFIDEBUG to true
2. Set your serial input from "New Line" to "No Line Ending"
2.5. Set Baud Rate to 115200
3. Run through states using serial inputs 1-6 (1 idle, ..., 6 abort)
*/

//::::::Libraries::::::://
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <SPI.h>
#include "HX711.h"
#include "Adafruit_MAX31855.h"
#include "RunningMedian.h"
// #include <EasyPCF8575.h>
// #include "PCF8575.h"  //use this one. Add zip from https://github.com/xreef/PCF8575_library

// These are sender ids, this is just a convention, should be same across all scripts
#define COM_ID 0
#define DAQ_ID 1

#define FLIGHT_ID 2

// DEBUG TRIGGER: SET TO 1 FOR DEBUG MODE.
// MOSFET must not trigger while in debug.
bool DEBUG = true;   // Simulate LOX and Eth fill.
bool WIFIDEBUG = false; // Don't send/receive data.
// refer to https://docs.google.com/spreadsheets/d/17NrJWC0AR4Gjejme-EYuIJ5uvEJ98FuyQfYVWI3Qlio/edit#gid=1185803967 for all pinouts

// ABORT VARIABLES //
#define abortPressure 625  // Cutoff pressure to automatically trigger abort
#define ventTo -50   
bool oxVentComplete = false;
bool ethVentComplete = false;   
#define MOSFET_VENT_LOX 48
#define MOSFET_VENT_ETH 47

#define DATA_TIMEOUT 100
#define IDLE_DELAY 250
#define GEN_DELAY 20

float readDelay = 150;     // Frequency of data collection [ms]
float sendDelay = IDLE_DELAY; // Frequency of sending data [ms]

enum STATES { IDLE, ARMED, PRESS, QD, IGNITION, LAUNCH, ABORT };
String stateNames[] = { "Idle", "Armed", "Press", "QD", "Ignition", "LAUNCH", "Abort" };

#define MAX_QUEUE_LENGTH 40

template <class T>
struct Queue {
private:
  int queueLength = 0;
  T queue[MAX_QUEUE_LENGTH];

public:
  Queue() {
    queueLength = 0;
  }

  void addPacket(T packet) {
    if (queueLength < MAX_QUEUE_LENGTH) {
      queue[queueLength] = packet;
      queueLength++;
    }
    else {
      queue[queueLength - 1] = packet;
    }
  }

  T peekPacket() {
    return queue[queueLength - 1];
  }

  T popPacket() {
    return queue[--queueLength];
  }

  int size() {
    return queueLength;
  }
};

struct MovingMedianFilter {
private:
  const unsigned BUFFER_SIZE = 25;
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
  float unshiftedRawReading = -1; // Raw reading before slope offset transformation is applied to it

  struct_data_board(Board scale, float offset, float slope)
    : scale(scale) {
    this->scale = scale;
    this->offset = offset;
    this->slope = slope;
  }

  virtual float readRawFromBoard() {
    return analogRead(36); // This is arbitrary, function should ALWAYS be overriden in child class (could make this class abstract but thats annoying)
  }

  void readDataFromBoard() {
    float newReading = readRawFromBoard();
    filter.addReading(newReading);

    unshiftedRawReading = newReading;
    rawReading = slope * newReading + offset;
    filteredReading = slope * filter.getReading() + offset;
  }

  void resetReading() {
    filter.resetReadings();
    filteredReading = -1;
    rawReading = -1;
    unshiftedRawReading = -1;
  }
};

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
    if (scale.wait_ready_timeout(DATA_TIMEOUT)) {
      return scale.read();
    }
    // Returns the last raw reading upon timeout
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

#define HX_CLK 17
// EXTRA PIN THAT CAN BE USED: 16 (PT6)
struct_hx711 PT_O1{ {}, HX_CLK, 4, .offset = 0, .slope = 0.1 }; 
struct_hx711 PT_O2{ {}, HX_CLK, 5, .offset = 0, .slope = 0.1 };
struct_hx711 PT_E1{ {}, HX_CLK, 6, .offset = 0, .slope = 1 };
struct_hx711 PT_E2{ {}, HX_CLK, 7, .offset = 0, .slope = 1 }; 
struct_hx711 PT_C1{ {}, HX_CLK, 15, .offset = 0, .slope = 1 }; 

// LOADCELLS UNUSED IN FLIGHT

//THERMOCOUPLE DEFINITIONS//
#define TC_CLK 12//NEEDS CHECK
// #define TC4_CLK 18
#define TC_DO 13
// #define TC4_DO 23

#define SD_CLK 12
#define SD_DO 13

struct_max31855 TC_1{ Adafruit_MAX31855(TC_CLK, 39, TC_DO), 39, .offset = 0, .slope = 1 };
struct_max31855 TC_2{ Adafruit_MAX31855(TC_CLK, 38, TC_DO), 38, .offset = 0, .slope = 1 };
struct_max31855 TC_3{ Adafruit_MAX31855(TC_CLK, 35, TC_DO), 35, .offset = 0, .slope = 1 };
struct_max31855 TC_4{ Adafruit_MAX31855(TC_CLK, 34, TC_DO), 34, .offset = 0, .slope = 1 };

//::::DEFINE READOUT VARIABLES:::://
float sendTime;
float readTime;

int COMState = IDLE;
int DAQState = IDLE;
int FlightState = IDLE;

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

struct_message dataPacket;

// Received Commands from COM
struct_message incomingDAQData;

// Create a queue for Packet in case Packets are dropped.
Queue<struct_message> COMQueue = Queue<struct_message>();
Queue<struct_message> DAQQueue = Queue<struct_message>();


//::::::Broadcast Variables::::::://
esp_now_peer_info_t peerInfo;

uint8_t COMBroadcastAddress[] = {0xEC, 0x64, 0xC9, 0x86, 0x1E, 0x4C};
uint8_t DAQBroadcastAddress[] = {0x48, 0xE7, 0x29, 0xA3, 0x0D, 0xA8};

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  struct_message Packet;
  memcpy(&Packet, incomingData, sizeof(Packet));
    
  incomingDAQData = Packet;
  // Update states
  COMState = incomingDAQData.COMState;
  DAQState = incomingDAQData.DAQState;
}

// Initialize all sensors and parameters.
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(1);  // wait for Serial on Leonardo/Zero, etc.
  Serial.println("Finished Serial Setup");

  // MOSFET PIN SETUP
  // pinMode(MOSFET_VENT_LOX, OUTPUT);
  // pinMode(MOSFET_VENT_ETH, OUTPUT);
  Serial.println("Finished MOSFET Setup");

  // HX711 Pressure Transducer Setup
  int gain = 128;
  PT_O1.scale.begin(PT_O1.gpio, PT_O1.clk);
  PT_O1.scale.set_gain(gain);
  PT_O2.scale.begin(PT_O2.gpio, PT_O2.clk);
  PT_O2.scale.set_gain(gain);
  PT_E1.scale.begin(PT_E1.gpio, PT_E1.clk);
  PT_E1.scale.set_gain(gain);
  PT_E2.scale.begin(PT_E2.gpio, PT_E2.clk);
  PT_E2.scale.set_gain(gain);
  PT_C1.scale.begin(PT_C1.gpio, PT_C1.clk);
  PT_C1.scale.set_gain(gain);
  Serial.println("PT finished");
  // LOAD CELLS UNUSED IN FLIGHT

  // Thermocouple.
  pinMode(TC_1.cs, OUTPUT);
  pinMode(TC_2.cs, OUTPUT);
  pinMode(TC_3.cs, OUTPUT);
  pinMode(TC_4.cs, OUTPUT);
  Serial.println("Finised TC");

  // Broadcast setup.
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  // Print MAC Accress on startup for easier connections
  Serial.print("My MAC Address :");
  Serial.println(WiFi.macAddress());
  Serial.println("Finished broadcast wifi");

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
    Serial.println("working on ESP");
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("registering esp");

  // Register peer
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  memcpy(peerInfo.peer_addr, COMBroadcastAddress, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
    
  }

  memcpy(peerInfo.peer_addr, DAQBroadcastAddress, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  sendTime = millis();
  readTime = millis();
}

//::::::STATE MACHINE::::::://

// Main Structure of State Machine.
void loop() {
  logData();
  fetchDAQState();
  if (DEBUG || DAQState == ABORT) {
    syncFlightState();
  }
  switch (FlightState) {
    case (IDLE):
      if (DAQState == ARMED) { syncFlightState(); }
      idle();
      break;

    case (ARMED):
      if (DAQState == IDLE || DAQState == PRESS) { syncFlightState(); }
      armed();
      break;

    case (PRESS):
      if (DAQState == IDLE || (DAQState == QD)) { syncFlightState(); }
      press();
      break;

    case (QD):
      if (DAQState == IDLE || DAQState == IGNITION) { syncFlightState(); }
      quick_disconnect();
      break;

    case (IGNITION):
      if (DAQState == IDLE || DAQState == LAUNCH) { syncFlightState(); }
      ignition();
      break;

    case (LAUNCH):
      launch();
      break;

    case (ABORT):
      abort_sequence();
      if (DAQState == IDLE) { syncFlightState(); }
      break;
  }
}

// State Functions.

// Everything should be off.
void reset() {
  PT_O1.resetReading();
  PT_O2.resetReading();
  PT_E1.resetReading();
  PT_E2.resetReading();
  PT_C1.resetReading();
  TC_1.resetReading();
  TC_2.resetReading();
  TC_3.resetReading();
  TC_4.resetReading();
}

void fetchDAQState() {
    if (Serial.available() > 0) {
    // Serial.read reads a single character as ASCII. Number 1 is 49 in ASCII.
    // Serial sends character and new line character "\n", which is 10 in ASCII.
    int SERIALState = Serial.read() - 48;
    if (SERIALState >= 0 && SERIALState <= 9) {
      DAQState = SERIALState;
    }
  }
}

// Sync state of Flight board with DAQ board
void syncFlightState() {
  FlightState = DAQState;
}

void idle() {
  sendDelay = IDLE_DELAY;
  mosfetCloseAllValves();
  reset();
}

void armed() {
  sendDelay = GEN_DELAY;
  mosfetCloseAllValves();
}

void press() {
  sendDelay = GEN_DELAY;
  mosfetCloseAllValves(); //might need changes for le3
}

void quick_disconnect() {
  sendDelay = GEN_DELAY;
  mosfetCloseAllValves(); //might need changes for le3
}

void ignition() {
  sendDelay = GEN_DELAY;
  mosfetCloseAllValves(); //might need changes for le3
}

void launch() {
  sendDelay = GEN_DELAY;  
  mosfetCloseAllValves(); //might need changes for le3
}

void abort_sequence() {
  oxVentComplete = false;
  ethVentComplete = false;
  if (DEBUG) {
    mosfetOpenValve(MOSFET_VENT_LOX);
    mosfetOpenValve(MOSFET_VENT_ETH);
  }
  // mosfetCloseValve(MOSFET_LOX_MAIN);
  // mosfetCloseValve(MOSFET_ETH_MAIN);
  // mosfetCloseValve(MOSFET_IGNITER);
  //
  int currtime = millis();

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
    PT_E1.rawReading = PT_E1.rawReading + (0.00005 * GEN_DELAY);
    PT_O1.rawReading = PT_O1.rawReading + (0.00005 * GEN_DELAY);
  }
}

void mosfetCloseAllValves() {
digitalWrite(MOSFET_VENT_LOX, LOW);
digitalWrite(MOSFET_VENT_ETH, LOW);
}

void mosfetCloseValve(int num) {
digitalWrite(num, LOW);
}

void mosfetOpenValve(int num) {
digitalWrite(num, HIGH);
}

void CheckAbort() {
  if (COMState == ABORT || PT_O1.filteredReading >= abortPressure || PT_E1.filteredReading >= abortPressure) {
    DAQState = ABORT;
  }
}

//::::::DATA LOGGING AND COMMUNICATION::::::://
void logData() {
  getReadings();
  if (millis() - sendTime > sendDelay) {
    sendTime = millis();
    sendData();
  }
}

void getReadings() {
  if (DEBUG) {
    return;
  }
  if (millis() - readTime > readDelay) {
    readTime = millis();
    PT_O1.readDataFromBoard();
    PT_O2.readDataFromBoard();
    PT_E1.readDataFromBoard();
    PT_E2.readDataFromBoard();
    PT_C1.readDataFromBoard();
    TC_1.readDataFromBoard();
    TC_2.readDataFromBoard();
    TC_3.readDataFromBoard();
    TC_4.readDataFromBoard();

    printSensorReadings();
  }
}

// Send data to COM board.
void sendData() {
  updateDataPacket();
  COMQueue.addPacket(dataPacket);
  DAQQueue.addPacket(dataPacket);
  sendQueue(DAQQueue, DAQBroadcastAddress);
  // delay(10);
  sendQueue(COMQueue, COMBroadcastAddress);
}

void updateDataPacket() {
  dataPacket.messageTime = millis();
  dataPacket.sender = FLIGHT_ID;
  
  dataPacket.rawReadings.PT_O1 = PT_O1.rawReading;
  dataPacket.rawReadings.PT_O2 = PT_O2.rawReading;
  dataPacket.rawReadings.PT_E1 = PT_E1.rawReading;
  dataPacket.rawReadings.PT_E2 = PT_E2.rawReading;
  dataPacket.rawReadings.PT_C1 = PT_C1.rawReading;
  dataPacket.rawReadings.TC_1 = TC_1.rawReading;
  dataPacket.rawReadings.TC_2 = TC_2.rawReading;
  dataPacket.rawReadings.TC_3 = TC_3.rawReading;
  dataPacket.rawReadings.TC_4 = TC_4.rawReading;

  dataPacket.filteredReadings.PT_O1 = PT_O1.filteredReading;
  dataPacket.filteredReadings.PT_O2 = PT_O2.filteredReading;
  dataPacket.filteredReadings.PT_E1 = PT_E1.filteredReading;
  dataPacket.filteredReadings.PT_E2 = PT_E2.filteredReading;
  dataPacket.filteredReadings.PT_C1 = PT_C1.filteredReading;
  dataPacket.filteredReadings.TC_1 = TC_1.filteredReading;
  dataPacket.filteredReadings.TC_2 = TC_2.filteredReading;
  dataPacket.filteredReadings.TC_3 = TC_3.filteredReading;
  dataPacket.filteredReadings.TC_4 = TC_4.filteredReading;

  dataPacket.COMState = COMState;
  dataPacket.DAQState = DAQState;
  dataPacket.FlightState = FlightState;
  dataPacket.FlightToCOMQueueLength = COMQueue.size();
  dataPacket.FlightToDAQQueueLength = DAQQueue.size();
}

void sendQueue(Queue<struct_message> queue, uint8_t broadcastAddress[]) {
  if (WIFIDEBUG) {
    return;
  }

  if (queue.size() == 0) {
    return;
  }
  // Set values to send
  struct_message Packet = queue.peekPacket();


  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&Packet, sizeof(Packet));

  if (result == ESP_OK) {
    queue.popPacket();
  } else {
    Serial.println("Error sending the data");
  }
}

void printSensorReadings() {
  String serialMessage = " ";
  serialMessage.concat(millis());
  serialMessage.concat(" ");
  serialMessage.concat(PT_O1.rawReading);
  serialMessage.concat(" ");
  serialMessage.concat(PT_O2.rawReading);
  serialMessage.concat(" ");
  serialMessage.concat(PT_E1.filteredReading);
  serialMessage.concat(" ");
  serialMessage.concat(PT_E2.filteredReading);
  serialMessage.concat(" ");
  serialMessage.concat(PT_C1.filteredReading);
  serialMessage.concat(" ");
  serialMessage.concat(TC_1.filteredReading);
  serialMessage.concat(" ");
  serialMessage.concat(TC_2.filteredReading);
  serialMessage.concat(" ");
  serialMessage.concat(TC_3.filteredReading);
  serialMessage.concat(" ");
  serialMessage.concat(TC_4.filteredReading);
  // serialMessage.concat("\nEth comp: ");
  // serialMessage.concat(incomingDAQData.ethComplete ? "True" : "False");
  // serialMessage.concat(" Ox comp: ");
  // serialMessage.concat(incomingDAQData.oxComplete  ? "True" : "False");
  serialMessage.concat("\n COM State: ");
  serialMessage.concat(stateNames[COMState]);
  serialMessage.concat("   Flight State: ");
  serialMessage.concat(stateNames[FlightState]);
  serialMessage.concat("   DAQ State: ");
  serialMessage.concat(stateNames[DAQState]);
  //  serialMessage.concat(readingCap1);
  //  serialMessage.concat(" ");
  //  serialMessage.concat(readingCap2);
  serialMessage.concat("\nCOM Q Length: ");
  serialMessage.concat(COMQueue.size());
  serialMessage.concat("  DAQ Q Length: ");
  serialMessage.concat(DAQQueue.size());
  Serial.println(serialMessage);
}

