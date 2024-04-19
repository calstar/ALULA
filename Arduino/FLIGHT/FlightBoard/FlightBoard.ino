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
#include <SD.h>
// #include <EasyPCF8575.h>
// #include "PCF8575.h"  //use this one. Add zip from https://github.com/xreef/PCF8575_library

// These are sender ids, this is just a convention, should be same across all scripts
#define COM_ID 0
#define DAQ_ID 1
#define FLIGHT_ID 2

#define SIMULATION_DELAY 25

// DEBUG TRIGGER: SET TO 1 FOR DEBUG MODE.
// MOSFET must not trigger while in debug.
bool DEBUG = false;   // Simulate LOX and Eth fill.
bool WIFIDEBUG = false; // Don't send/receive data.
// refer to https://docs.google.com/spreadsheets/d/17NrJWC0AR4Gjejme-EYuIJ5uvEJ98FuyQfYVWI3Qlio/edit#gid=1185803967 for all pinouts

// ABORT VARIABLES //
#define abortPressure 625  // Cutoff pressure to automatically trigger abort
#define ventTo -50   
bool oxVentComplete = false;
bool ethVentComplete = false;   
#define MOSFET_VENT_LOX 48
#define MOSFET_VENT_ETH 47
#define MOSFET_LOX_MAIN 9    
#define MOSFET_LOX_PRESS 4
#define MOSFET_ETH_MAIN 10    
#define MOSFET_ETH_PRESS 6   
#define MOSFET_IGNITER 8

#define DATA_TIMEOUT 100
#define IDLE_DELAY 250
#define GEN_DELAY 20

#define ABORT_ACTIVATION_DELAY 500 // Number of milliseconds to wait at high pressure before activating abort
#define SD_CARD_FLUSH_PERIOD 15000 // Interval duration for periodically flushing SD card

float readDelay = 150;     // Frequency of data collection [ms]
float sendDelay = IDLE_DELAY; // Frequency of sending data [ms]

enum STATES { IDLE, ARMED, PRESS, QD, IGNITION, LAUNCH, ABORT };
String stateNames[] = { "Idle", "Armed", "Press", "QD", "Ignition", "LAUNCH", "Abort" };

// SD Card Parameters
#define SD_CARD_CS 5 // Chip select pin
const char* sdCardFilename = "./data.txt";
File sdCardFile;

// Last SD card write time
int lastSDCardWriteTime = 0;


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
  short int FlightQueueLength;
  bool ethComplete;
  bool oxComplete;
  bool oxVentComplete;
  bool ethVentComplete;
  bool sdCardInitialized;
};

struct_message dataPacket;

// Received State from DAQ, abort from COM
struct_message incomingDAQData;
struct_message incomingCOMData;


// Create a queue for Packet in case Packets are dropped.
Queue<struct_message> dataQueue = Queue<struct_message>();

//::::::Broadcast Variables::::::://
esp_now_peer_info_t peerInfo;

uint8_t COMBroadcastAddress[] = {0xC8, 0xF0, 0x9E, 0x50, 0x23, 0x34};
uint8_t DAQBroadcastAddress[] = {0xEC, 0x64, 0xC9, 0x86, 0x1E, 0x4C};

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  struct_message Packet;
  memcpy(&Packet, incomingData, sizeof(Packet));

  if (Packet.sender == COM_ID) {
    incomingCOMData = Packet;
    COMState = Packet.COMState;
  } else if (Packet.sender == DAQ_ID) {
    incomingDAQData = Packet;
    DAQState = Packet.DAQState;
  }
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

  setupSDCard();

  sendTime = millis();
  readTime = millis();
}

//::::::STATE MACHINE::::::://

// Main Structure of State Machine.
void loop() {
  logData();
  serialReadFlightState();
  if (DEBUG || COMState == ABORT || DAQState == ABORT) {
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
      if (DAQState == IDLE || DAQState == QD) { syncFlightState(); }
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

void serialReadFlightState() {
    if (Serial.available() > 0) {
    // Serial.read reads a single character as ASCII. Number 1 is 49 in ASCII.
    // Serial sends character and new line character "\n", which is 10 in ASCII.
    int SERIALState = Serial.read() - 48;
    if (SERIALState >= 0 && SERIALState <= 9) {
      FlightState = SERIALState;
    }
  }
}

// Sync state of Flight board with DAQ board
void syncFlightState() {
  // Sync with DAQ only if Flight doesn't detect an Abort, and DAQ doesn't want us to go back to Idle after Abort
  if (FlightState != ABORT || DAQState == IDLE) {
    FlightState = DAQState;
  }
  if (COMState == ABORT) {
    FlightState = ABORT;
  }
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
  checkAbort();
}

void quick_disconnect() {
  sendDelay = GEN_DELAY;
  mosfetCloseAllValves(); //might need changes for le3
  checkAbort();
}

void ignition() {
  sendDelay = GEN_DELAY;
  mosfetCloseAllValves(); //might need changes for le3
}

void launch() {
  sendDelay = GEN_DELAY;
  lastSDCardWriteTime = millis();  // Ensures SD card will not flush during burn
  mosfetCloseAllValves(); //might need changes for le3
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

  if (dataPacket.filteredReadings.PT_O1 > ventTo) {  // vent only lox down to vent to pressure
    mosfetOpenValve(MOSFET_VENT_LOX);
    if (DEBUG) {
      dataPacket.filteredReadings.PT_O1 = dataPacket.filteredReadings.PT_O1 - (0.0005 * SIMULATION_DELAY);
    }
  } else {                              // lox vented to acceptable hold pressure
    mosfetCloseValve(MOSFET_VENT_LOX);  // close lox
    oxVentComplete = true;
  }
  if (dataPacket.filteredReadings.PT_E1 > ventTo) {
    mosfetOpenValve(MOSFET_VENT_ETH);  // vent ethanol
    if (DEBUG) {
      dataPacket.filteredReadings.PT_E1 = dataPacket.filteredReadings.PT_E1 - (0.0005 * SIMULATION_DELAY);
    }
  } else {
    mosfetCloseValve(MOSFET_VENT_ETH);
    ethVentComplete = true;
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

int cumulativeAbortTime = 0; // How long we have been in high-pressure state
int lastAbortCheckTime = -1; // Last time we called checkAbort()
void checkAbort() {
  if (lastAbortCheckTime == -1) {
    lastAbortCheckTime = millis();
    return;
  }

  int deltaTime = millis() - lastAbortCheckTime;
  if (dataPacket.filteredReadings.PT_O1 >= abortPressure || dataPacket.filteredReadings.PT_E1 >= abortPressure) {
    cumulativeAbortTime += deltaTime;
  }
  else {
    cumulativeAbortTime = max(cumulativeAbortTime - deltaTime, 0);
  }
  lastAbortCheckTime = millis();

  if (COMState == ABORT || cumulativeAbortTime >= ABORT_ACTIVATION_DELAY) {
    abort_sequence();
    FlightState = ABORT;
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
    // Make Flight use DAQ readings; this is purely for simulating fill sequence
    // since DAQ has simulation logic
    dataPacket.filteredReadings = incomingDAQData.filteredReadings;
    dataPacket.rawReadings = incomingDAQData.rawReadings;
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

    updateDataPacket();
    if (FlightState != IDLE) {
      writeSDCard(packetToString(&dataPacket));
    }
    printSensorReadings();
  }
}

// Send data to COM and DAQ board.
void sendData() {
  if (DEBUG) {
    printSensorReadings();
  }
  dataQueue.addPacket(dataPacket);
  sendQueue(dataQueue, 0);
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
  dataPacket.FlightQueueLength = dataQueue.size();

  dataPacket.ethVentComplete = ethVentComplete;
  dataPacket.oxVentComplete = oxVentComplete;
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

void setupSDCard() {
  Serial.print("Initializing SD card...");
  if (!(dataPacket.sdCardInitialized = SD.begin(SD_CARD_CS))) {
    Serial.println("initialization failed!");
    return;
  }

  Serial.println("SD card initialization done.");
  dataPacket.sdCardInitialized = SD.open(sdCardFilename, FILE_WRITE);

  if (!dataPacket.sdCardInitialized) {
    Serial.println("Error opening SD card file"); // Error handling if file opening fails
  }
}

void writeSDCard(String data) { 
  if (!sdCardFile) {
    if (DEBUG) { Serial.println("Error writing to sd card"); }
    return;
  }
    
  int currTime = millis();
  sdCardFile.println(data);
  if (currTime - lastSDCardWriteTime >= SD_CARD_FLUSH_PERIOD) {
    sdCardFile.flush();
    lastSDCardWriteTime = currTime;
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
  serialMessage.concat("\Flight Q Length: ");
  serialMessage.concat(dataQueue.size());
  Serial.println(serialMessage);
}

String readingsToString(const struct_readings *packet) {
  String data = "";
  data = data + packet->PT_O1 + " ";
  data = data + packet->PT_O2 + " ";
  data = data + packet->PT_E1 + " ";
  data = data + packet->PT_E2 + " ";
  data = data + packet->PT_C1 + " ";

  data = data + packet->LC_1 + " ";
  data = data + packet->LC_2 + " ";
  data = data + packet->LC_3 + " ";

  data = data + packet->TC_1 + " ";
  data = data + packet->TC_2 + " ";
  data = data + packet->TC_3 + " ";
  data = data + packet->TC_4;

  return data;
}

String packetToString(const struct_message *packet) {
  String data = "START\n";
  data = data + millis() + "\n";
  data += readingsToString(&(packet->rawReadings)) + "\n";
  data += readingsToString(&(packet->filteredReadings)) + "\n";
  data = data + packet->COMState + " " + packet->DAQState + " " + packet->FlightState + "\n";
  data = data + packet->FlightQueueLength + "\n";
  data = data + packet->oxComplete + " " + packet->ethComplete + " " + packet->oxVentComplete + " " + packet->ethVentComplete + "\n";

  return data;
}

