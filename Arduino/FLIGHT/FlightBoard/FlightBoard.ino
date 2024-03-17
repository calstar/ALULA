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

// These are sender ids, this is just a convention, should be same across all scripts
#define COM_ID 0
#define DAQ_ID 1
#define FLIGHT_ID 2

// DEBUG TRIGGER: SET TO 1 FOR DEBUG MODE.
// MOSFET must not trigger while in debug.
bool DEBUG = false;   // Simulate LOX and Eth fill.
bool WIFIDEBUG = false; // Don't send/receive data.
// refer to https://docs.google.com/spreadsheets/d/17NrJWC0AR4Gjejme-EYuIJ5uvEJ98FuyQfYVWI3Qlio/edit#gid=1185803967 for all pinouts

#define DATA_TIMEOUT 100

#define IDLE_DELAY 250
#define GEN_DELAY 20

float readDelay = 20;     // Frequency of data collection [ms]
float sendDelay = IDLE_DELAY; // Frequency of sending data [ms]

enum STATES { IDLE, ARMED, PRESS, QD, IGNITION, HOTFIRE, ABORT };
String stateNames[] = { "Idle", "Armed", "Press", "QD", "Ignition", "HOTFIRE", "Abort" };

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

//::::DEFINE READOUT VARIABLES:::://
float sendTime;

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
uint8_t DAQBroadcastAddress[] = {0xE8, 0x6B, 0xEA, 0xD4, 0x10, 0x4C};

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  struct_message Packet;
  memcpy(&Packet, incomingData, sizeof(Packet));
    
  incomingDAQData = Packet;
  // Update states
  COMState = incomingDAQData.COMState;
  DAQState = incomingDAQData.DAQState;
  FlightState = incomingDAQData.FlightState;

  Serial.println(DAQState);
}

// Initialize all sensors and parameters.
void setup() {
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
  Serial.print("My MAC Address :");
  Serial.println(WiFi.macAddress());

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

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
}

//::::::STATE MACHINE::::::://

// Main Structure of State Machine.
void loop() {
  if (DEBUG || DAQState == ABORT) {
    syncDAQState();
  }
  switch (FlightState) {
    case (IDLE):
      if (DAQState == ARMED) { syncDAQState(); }
      idle();
      break;

    case (ARMED):
      if (DAQState == IDLE || COMState == PRESS) { syncDAQState(); }
      armed();
      break;

    case (PRESS):
      if (DAQState == IDLE || (COMState == QD)) { syncDAQState(); }
      press();
      break;

    case (QD):
      if (DAQState == IDLE || COMState == IGNITION) { syncDAQState(); }
      quick_disconnect();
      break;

    case (IGNITION):
      if (DAQState == IDLE || COMState == HOTFIRE) { syncDAQState(); }
      ignition();
      break;

    case (HOTFIRE):
      hotfire();
      break;

    case (ABORT):
      abort_sequence();
      if (COMState == IDLE) { syncDAQState(); }
      break;
  }

  logData();
}

// State Functions.

// Everything should be off.
void reset() {
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
  sendDelay = IDLE_DELAY;
  reset();
}

void armed() {
  sendDelay = GEN_DELAY;
}

void press() {
  sendDelay = GEN_DELAY;
}

void quick_disconnect() {
  sendDelay = GEN_DELAY;
}

void ignition() {
  sendDelay = GEN_DELAY;
}

void hotfire() {
  sendDelay = GEN_DELAY;
}

void abort_sequence() {
  sendDelay = GEN_DELAY;
  // TODO: Add abort sequence valve actuation
}

// Sync state of Flight board with DAQ board
void syncDAQState() {
  FlightState = DAQState;
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
    simulateReadings();
    return;
  }

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

// Send data to COM board.
void sendData() {
  updateDataPacket();
  COMQueue.addPacket(dataPacket);
  DAQQueue.addPacket(dataPacket);
  sendQueue(DAQQueue, DAQBroadcastAddress);
  delay(10);
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
  dataPacket.rawReadings.LC_1 = LC_1.rawReading;
  dataPacket.rawReadings.LC_2 = LC_2.rawReading;
  dataPacket.rawReadings.LC_3 = LC_3.rawReading;
  dataPacket.rawReadings.TC_1 = TC_1.rawReading;
  dataPacket.rawReadings.TC_2 = TC_2.rawReading;
  dataPacket.rawReadings.TC_3 = TC_3.rawReading;
  dataPacket.rawReadings.TC_4 = TC_4.rawReading;

  dataPacket.filteredReadings.PT_O1 = PT_O1.filteredReading;
  dataPacket.filteredReadings.PT_O2 = PT_O2.filteredReading;
  dataPacket.filteredReadings.PT_E1 = PT_E1.filteredReading;
  dataPacket.filteredReadings.PT_E2 = PT_E2.filteredReading;
  dataPacket.filteredReadings.PT_C1 = PT_C1.filteredReading;
  dataPacket.filteredReadings.LC_1 = LC_1.filteredReading;
  dataPacket.filteredReadings.LC_2 = LC_2.filteredReading;
  dataPacket.filteredReadings.LC_3 = LC_3.filteredReading;
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
  serialMessage.concat("\nEth comp: ");
  serialMessage.concat(incomingDAQData.ethComplete ? "True" : "False");
  serialMessage.concat(" Ox comp: ");
  serialMessage.concat(incomingDAQData.oxComplete  ? "True" : "False");
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

