/*
This code runs on the COM ESP32 and has a couple of main tasks.
1. Receive sensor data from DAQ ESP32
2. Send servo commands to DAQ ESP32
*/

#include <Ra01S.h>
#include <map>
#include <list>
#include <vector>
#include <SD.h>
#include <esp_now.h>
#include <WiFi.h>

#define RF_FREQUENCY                                915000000 // Hz  center frequency
#define TX_OUTPUT_POWER                             22        // dBm tx output power
#define LORA_SPREADING_FACTOR                       6         // spreading factor [SF5..SF12]
#define LORA_BANDWIDTH                              5         // bandwidth
                                                              // 2: 31.25Khz
                                                              // 3: 62.5Khz
                                                              // 4: 125Khz
                                                              // 5: 250KHZ
                                                              // 6: 500Khz 
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]

#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_PAYLOAD_LENGTH                          0         // 0: Variable length packet (explicit header)
                                                              // 1..255  Fixed length packet (implicit header)

enum STATES { IDLE, ARMED, PRESS, QD, IGNITION, LAUNCH, ABORT };

#define SD_CARD_CS 10

const char* sdCardFilename = "/alula.txt";
File sdCardFile;

bool WIFIDEBUG = true;

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

SX126x  lora(15,               //Port-Pin Output: SPI select
             21,               //Port-Pin Output: Reset 
             39               //Port-Pin Input:  Busy
             );

void setup() {
  delay(1000);

  Serial.print("Setting up...");
  // put your setup code here, to run once:
  Serial.begin(115200);
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

  int16_t ret = lora.begin(RF_FREQUENCY,              //frequency in Hz
                           TX_OUTPUT_POWER);          //tx power in dBm

  lora.LoRaConfig(LORA_SPREADING_FACTOR, 
                  LORA_BANDWIDTH, 
                  LORA_CODINGRATE, 
                  LORA_PREAMBLE_LENGTH, 
                  LORA_PAYLOAD_LENGTH, 
                  true,               //crcOn  
                  false);             //invertIrq

  setupSDCard();

  Serial.println("DONE WRITINg");
}

int i = 0;
void loop() {
  // writeSDCard("alualau1");
  // writeSDCard(String(i++));
}

void setupSDCard() {
  if (!SD.begin(SD_CARD_CS)) {
    Serial.println("SD card mount failed!");
    delay(100);
  }
  sdCardFile = SD.open(sdCardFilename, FILE_WRITE);
  if (SD.exists(sdCardFilename)) {
    Serial.println("File created successfully!");
  } 
  else {
    Serial.println("File not created :^(");
  }

  if (sdCardFile) {
    sdCardFile.print("Writing to file...");   
    sdCardFile.println("Space Technologies and Rocketry");
    sdCardFile.println("Computerized Utility Receiver");
    sdCardFile.println("Version 47 Prototype 2 Code");
    sdCardFile.println("Designed, Assembled, and Coded by Conor Van Bibber");
    sdCardFile.println("Launch Number: 01");
    sdCardFile.close();
    
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening file");
  }  
}

void writeSDCard(String data) {
  // Serial.println(data);
  digitalWrite(15, HIGH);
  digitalWrite(SD_CARD_CS, LOW);
  sdCardFile = SD.open(sdCardFilename, FILE_APPEND);
  if (!sdCardFile) {
    if (WIFIDEBUG) { Serial.println("Error writing to sd card"); }
    return;
  }

  for (int i = 0; i < data.length(); i++) {
    sdCardFile.print((char) data[i]);
  }
  sdCardFile.println();
  
  sdCardFile.close();
  digitalWrite(SD_CARD_CS, HIGH);
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

