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
#include "RunningMedian.h"
#include "PCF8575.h"  // https://github.com/xreef/PCF8575_library
// Set i2c address
PCF8575 pcf8575(0x20);

#define COM_ID 1
#define DAQ_POWER_ID 2
#define DAQ_SENSE_ID 3

//::::::Global Variables::::::://

// DEBUG TRIGGER: SET TO 1 FOR DEBUG MODE.
// MOSFET must not trigger while in debug.
int DEBUG = 0;      // Simulate LOX and Eth fill.
int WIFIDEBUG = 0;  // Don't send/receive data.

//vars for unifying structure between P/S
int COMState = 0;
int DAQState = 0;
bool ethComplete = false;
bool oxComplete = false;

float readDelay = 25;     // Frequency of data collection [ms]
float sendDelay = readDelay;
// END OF USER DEFINED PARAMETERS //
// refer to https://docs.google.com/spreadsheets/d/17NrJWC0AR4Gjejme-EYuIJ5uvEJ98FuyQfYVWI3Qlio/edit#gid=1185803967 for all pinouts


//::::::DEFINE INSTRUMENT PINOUTS::::::://


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



//sensor stuff

#define HX_CLK 27

struct_hx711 PT_O1{ {}, HX_CLK, 36, .offset = -85.5051, .slope = 0.004964 }; //.offset = -71.93, .slope = 0.00822
struct_hx711 PT_O2{ {}, HX_CLK, 39, .offset = -79.7068, .slope = 0.004247 };
struct_hx711 PT_E1{ {}, HX_CLK, 34, .offset = -75.1059, .slope = 0.004700 };
struct_hx711 PT_E2{ {}, HX_CLK, 35, .offset = -66.2774, .slope = 0.004553 };  // Change GPIO PIN
struct_hx711 PT_C1{ {}, HX_CLK, 32, .offset = -99.6688, .slope = 0.003776 };

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



//::::DEFINE READOUT VARIABLES:::://
String serialMessage;
float sendTime;
short int queueLength = 0;

// Define variables to store readings to be sent

// Structure example to send data.
// Must match the receiver structure.
typedef struct struct_message {
  int id;
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
  int COMState;
  int DAQState; 
  short int queueLength;
  bool ethComplete;
  bool oxComplete;
  // bool oxvent;
  // bool ethVent;
  // bool VentComplete;
} struct_message;

// Create a struct_message called Packet to be sent to the DAQ Power.
struct_message dataPacket;

//::::::Broadcast Variables::::::://
esp_now_peer_info_t peerInfo;
uint8_t COMBroadcastAddress[] = {0xC8, 0xF0, 0x9E, 0x4F, 0x3C, 0xA4}; //Core board 1 //COM CIRCUIT
// uint8_t COMBroadcastAddress[] = {0xC8, 0xF0, 0x9E, 0x51, 0xEC, 0x94}; //TEST ESP
// uint8_t COMBroadcastAddress[] = {0x08, 0x3A, 0xF2, 0xB7, 0x29, 0xBC}; //Test ESP 2/10/24

// uint8_t DAQPowerBroadcastAddress[] = {0xC8, 0xF0, 0x9E, 0x4F, 0x3C, 0xA4}; //CORE1
uint8_t DAQPowerBroadcastAddress[] = {0xB0, 0xA7, 0x32, 0xDE, 0xD3, 0x1C}; //CORE2


void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
 sendTime = millis();
}

Queue<struct_message> COMQueue = Queue<struct_message>();
Queue<struct_message> DAQPowerQueue = Queue<struct_message>();


// Initialize all sensors and parameters. sense board fs
void setup() {
  // pinMode(ONBOARD_LED,OUTPUT);
  Serial.begin(115200);
  while (!Serial) delay(1);  // wait for Serial on Leonardo/Zero, etc.

  // HX711.
  PT_O1.scale.begin(PT_O1.gpio, PT_O1.clk);
  PT_O1.scale.set_gain(128);
  PT_O2.scale.begin(PT_O2.gpio, PT_O2.clk);
  PT_O2.scale.set_gain(128);
  PT_E1.scale.begin(PT_E1.gpio, PT_E1.clk);
  PT_E1.scale.set_gain(128);
  PT_E2.scale.begin(PT_E2.gpio, PT_E2.clk);
  PT_E2.scale.set_gain(128);
  PT_C1.scale.begin(PT_C1.gpio, PT_C1.clk);
  PT_C1.scale.set_gain(128);
  LC_1.scale.begin(LC_1.gpio, LC_1.clk);
  LC_1.scale.set_gain(128);
  LC_2.scale.begin(LC_2.gpio, LC_2.clk);
  LC_2.scale.set_gain(128);
  LC_3.scale.begin(LC_3.gpio, LC_3.clk);
  LC_3.scale.set_gain(128);

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

    // Register peer
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, DAQPowerBroadcastAddress, 6);

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer DAQ POWER");
    return;
  }

  memcpy(peerInfo.peer_addr, COMBroadcastAddress, 6);

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer COM");
    return;
  }


  sendTime = millis();
}


//::::::STATE MACHINE:::::::///
// Main Structure of State Machine
void loop() {
  //  Serial.print("looping");
  logData();
  //  Serial.print("logged");
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

//::::::DATA LOGGING AND COMMUNICATION::::::://
void logData() {
  getReadings();
  printSensorReadings();
  if (millis() - sendTime > sendDelay) {
    sendTime = millis();
    sendData();
  }
}

void getReadings() {
  if (DEBUG) { return; }

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
  DAQPowerQueue.addPacket(dataPacket);
  sendQueue(DAQPowerQueue, DAQPowerBroadcastAddress);
  sendQueue(COMQueue, COMBroadcastAddress);
}

void updateDataPacket() {
  dataPacket.messageTime = millis();
  dataPacket.id = DAQ_SENSE_ID;
  dataPacket.PT_O1 = PT_O1.filteredReading;
  dataPacket.PT_O2 = PT_O2.filteredReading;
  dataPacket.PT_E1 = PT_E1.filteredReading;
  dataPacket.PT_E2 = PT_E2.filteredReading;
  dataPacket.PT_C1 = PT_C1.filteredReading;
  dataPacket.LC_1 = LC_1.filteredReading;
  dataPacket.LC_2 = LC_2.filteredReading;
  dataPacket.LC_3 = LC_3.filteredReading;
  dataPacket.TC_1 = TC_1.filteredReading;
  dataPacket.TC_2 = TC_2.filteredReading;
  dataPacket.TC_3 = TC_3.filteredReading;
  dataPacket.TC_4 = TC_4.filteredReading;
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
  // serialMessage.concat(DAQPowerCommands.ethComplete ? "True" : "False");
  serialMessage.concat(" Ox comp: ");
  // serialMessage.concat(DAQPowerCommands.oxComplete  ? "True" : "False");
  serialMessage.concat("COM State: ");
  // serialMessage.concat(stateNames[COMState]);
  serialMessage.concat("   Sense State: ");
  // serialMessage.concat(stateNames[DAQSenseState]);
  serialMessage.concat("   Power State: ");
  // serialMessage.concat(stateNames[DAQPowerState]);
  //  serialMessage.concat(readingCap1);
  //  serialMessage.concat(" ");
  //  serialMessage.concat(readingCap2);
  serialMessage.concat(" ");
  serialMessage.concat(COMQueue.size());
  serialMessage.concat("  ");
  serialMessage.concat(DAQPowerQueue.size());
  Serial.println(serialMessage);
}