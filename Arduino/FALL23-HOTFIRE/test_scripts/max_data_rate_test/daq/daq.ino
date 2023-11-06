/*
This code runs on the DAQ ESP32 and has a couple of main tasks.
1. Read sensor data
2. Send sensor data to COM ESP32
3. Actuate hotfire sequence
*/

// Max pressure : 800 psi
// 

//::::::Libraries::::::://
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <SPI.h>
#include "HX711.h"
#include "Adafruit_MAX31855.h"
#include <EasyPCF8575.h>
#include "RunningMedian.h" // Just search for RunningMedian libarary to install

uint8_t broadcastAddress[] = {0x08, 0x3A, 0xF2, 0xB7, 0xEE, 0x00};
esp_now_peer_info_t peerInfo;

const int SEND_DELAY = 1;

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

int sendTime = 0;
message m(0);

//::::::Global Variables::::::://
void setup() {
  // pinMode(ONBOARD_LED,OUTPUT);
  Serial.begin(115200);
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
}

//::::::STATE MACHINE::::::://

// Main Structure of State Machine.
void loop() {
  if (millis() - sendTime > SEND_DELAY) {
    // Set values to send
    sendTime = millis();
    m = message(millis());

    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &m, sizeof(m));

    if (result != ESP_OK) {
      Serial.println("Error sending the data");
    }
  }
}
