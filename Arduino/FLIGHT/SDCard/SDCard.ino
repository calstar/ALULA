/*
This code runs on the COM ESP32 and has a couple of main tasks.
1. Receive sensor data from DAQ ESP32
2. Send servo commands to DAQ ESP32
*/

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Arduino.h>
#include "HX711.h"
//#include <ezButton.h>
#include "freertos/FreeRTOS.h"
#include "freertos/Task.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"

enum STATES { IDLE, ARMED, PRESS, QD, IGNITION, LAUNCH, ABORT };

#define SD_CARD_CS 5

const char* sdCardFilename = "/data.txt";
File sdCardFile;

bool WIFIDEBUG = false;

esp_now_peer_info_t peerInfo;

// uint8_t FlightBroadcastAddress[] = {0x48, 0x27, 0xE2, 0x2C, 0x80, 0xD8}; //CORE 1 V2
uint8_t FlightBroadcastAddress[] = {0x34, 0x85, 0x18, 0x71, 0x06, 0x60}; // CORE 2 V2
// uint8_t FlightBroadcastAddress[] = {0x48, 0x27, 0xE2, 0x2F, 0x22, 0x08}; //CORE 3 V2

//Structure example to send data
//Must match the receiver structure
struct struct_pt_offsets {
  bool PT_O1_set;
  bool PT_O2_set;
  bool PT_E1_set;
  bool PT_E2_set;
  bool PT_C1_set;
  bool PT_X_set;

  float PT_O1_offset;
  float PT_O2_offset;
  float PT_E1_offset;
  float PT_E2_offset;
  float PT_C1_offset;
  float PT_X_offset;
};

struct struct_readings {
  float PT_O1;
  float PT_O2;
  float PT_E1;
  float PT_E2;
  float PT_C1;
  float PT_X;
  float TC_1;
  float TC_2;
  float TC_3;
  float TC_4;
};

struct struct_message {
  int messageTime;
  int sender;
  int COMState;
  int DAQState;
  int FlightState;
  bool AUTOABORT;

  short int FlightQueueLength;
  bool ethComplete;
  bool oxComplete;
  bool oxVentComplete;
  bool ethVentComplete;
  bool sdCardInitialized;

  struct_readings filteredReadings;
  struct_readings rawReadings;
  struct_pt_offsets pt_offsets;
};

struct_message incomingFlightReadings;

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingFlightReadings, incomingData, sizeof(incomingFlightReadings));
  writeSDCard(packetToString(incomingFlightReadings));
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  Serial.println(WiFi.macAddress());
  //set device as WiFi station
  WiFi.mode(WIFI_STA);

  //initialize ESP32
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register peer
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  memcpy(peerInfo.peer_addr, FlightBroadcastAddress, 6);

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
  }

  Serial.print("My MAC Address ");
  Serial.println(WiFi.macAddress());

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

  setupSDCard();
}

void loop() {

}

void setupSDCard() {
  Serial.print("Initializing SD card...");
  if (!SD.begin(SD_CARD_CS)) {
    Serial.println("initialization failed!");
    return;
  }

  Serial.println("SD card initialization done.");

  if (!SD.open(sdCardFilename, FILE_WRITE) && WIFIDEBUG) {
    Serial.println("Error opening SD card file"); // Error handling if file opening fails
  }
}
void writeSDCard(String data) {
  if (!sdCardFile) {
    if (WIFIDEBUG) { Serial.println("Error writing to sd card"); }
    return;
  }

  sdCardFile.println(data);
  sdCardFile.flush();
}

String readingsToString(const struct_readings &packet) {
  String data = "";
  data = data + packet.PT_O1 + " ";
  data = data + packet.PT_O2 + " ";
  data = data + packet.PT_E1 + " ";
  data = data + packet.PT_E2 + " ";
  data = data + packet.PT_C1 + " ";
  data = data + packet.PT_X + " ";

  data = data + packet.TC_1 + " ";
  data = data + packet.TC_2 + " ";
  data = data + packet.TC_3 + " ";
  data = data + packet.TC_4;

  return data;
}

String offsetsToString(struct_pt_offsets offsets) {
  String data = "";
  data = data + offsets.PT_O1_offset + " ";
  data = data + offsets.PT_O2_offset + " ";
  data = data + offsets.PT_E1_offset + " ";
  data = data + offsets.PT_E2_offset + " ";
  data = data + offsets.PT_C1_offset + " ";
  data = data + offsets.PT_X_offset + " ";

  return data;
}

String packetToString(const struct_message &packet) {
  String data = "START\n";
  data = data + millis() + "\n";
  data += readingsToString(packet.filteredReadings) + "\n";
  data += readingsToString(packet.rawReadings) + "\n";
  data = data + packet.COMState + " " + packet.DAQState + " " + packet.FlightState + "\n";
  data = data + packet.FlightQueueLength + "\n";
  data = data + packet.oxComplete + " " + packet.ethComplete + " " + packet.oxVentComplete + " " + packet.ethVentComplete + "\n";
  data = data + offsetsToString(packet.pt_offsets) + "\n";

  return data;
}

