/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-two-way-communication-esp32/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.



  CODE FOR COMMUNICATION TESTING 

  THIS TEST IS FOR THE DAQ BOARD (RECEIVER)
*/

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <String.h>

#define GET_COM 33

String success;
String serialMessage;

bool is_perceive;

float pt1;

// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress[] = {0x08, 0x3A, 0xF2, 0xB7, 0x1C, 0XDC}; //current address is for: ESP 1


//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    int command;
    float pt_value;
} struct_message;


// Create a struct_message called BME280Readings to hold sensor readings
struct_message incomingReadings;

// Create a struct_message to hold incoming sensor readings
struct_message outgoingMessage;

esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  Serial.print("Bytes received: ");
  Serial.println(len);
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  pinMode(GET_COM, OUTPUT);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

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
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  
}
 
void loop() {
  is_perceive = digitalRead(GET_COM);

  if (is_perceive){

    // Register for a callback function that will be called when data is received
    esp_now_register_recv_cb(OnDataRecv);
 
    serialMessage = "";
    serialMessage.concat(pt1);
    serialMessage.concat(" ");
    Serial.println(serialMessage);

    delay(5)
;  
    // Set values to send
    

    

    // Display Readings in Serial Monitor
    Serial.println("INCOMING READINGS");
    Serial.print("Command: ");
    Serial.println(incomingReadings.command);
    Serial.print("PT Value: ");
    Serial.println(incomingReadings.pt_value);
    Serial.println(" %");
    Serial.println();
  } else {
    Serial.println("I do not perceive.... no data received.");
  }

  outgoingMessage.command = is_perceive;
  outgoingMessage.pt_value = 0;

  delay(2000);

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoingMessage, sizeof(outgoingMessage));
  if (result == ESP_OK) {
      Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }
}



