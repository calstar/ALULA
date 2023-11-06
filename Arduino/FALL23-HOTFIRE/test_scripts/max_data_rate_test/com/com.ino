#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Arduino.h>
#include "HX711.h"

uint8_t broadcastAddress[] = { 0xC8, 0xF0, 0x9E, 0x50, 0x23, 0x34 };  //Core board 1
esp_now_peer_info_t peerInfo;

struct message {
  int time;
  int a1, a2, a3, a4, a5, a6, a7, a8;
  int a9, a10, a11, a12, a13, a14, a15, a16;

  message(int time) {
    this->time = time;
    a1 = a2 = a3 = a4 = a5 = a6 = a7 = a8 = 0;
    a9 = a10 = a11 = a12 = a13 = a14 = a15 = a16 = 0;
  }
};


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);

  Serial.println(WiFi.macAddress());

  //initialize ESP32
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
}

void loop() {
}

message m(0);

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&m, incomingData, sizeof(m));
  Serial.println(m.time);
}
