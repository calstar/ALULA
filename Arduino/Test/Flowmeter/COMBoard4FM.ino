#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Arduino.h>
#include "HX711.h"
// import random
int angle = 90;
float pressTime = 0;
const int buttonpin1 = 15;
const int LEDpin = 27;
String success;
int servo1_curr = 90;
int servo2_curr = 0;
int incomingS1 = 0;
float incomingPT1 = 0;
float incomingPT2 = 0;
float incomingPT3 = 0;
float incomingPT4 = 0;
float incomingPT5 = 0;
float incomingFM = 0;
float incomingLC1 = 0;
float incomingLC2 = 0;
float incomingLC3 = 0;
esp_now_peer_info_t peerInfo;
bool pressed = false;
bool prevPressed = false;
bool valveOpened = false;

// delete these test variables
int testNum = 1;
int testFMOutputInterval = 200;
int testTime = millis();


uint8_t broadcastAddress[] = {0x24, 0x62, 0xAB, 0xD2, 0x85, 0xDC}; //change to new Mac Address

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    float pt1;
    float pt2;
    float pt3;
    float pt4;
    float pt5;
    float lc1;
    float lc2;
    float lc3;
    float fm;
    int S1;
    int S2;
    int S1S2;
    boolean I;
} struct_message;

// Create a struct_message called Readings to recieve sensor readings remotely
struct_message incomingReadings;

// Create a struct_message to send commands
struct_message Commands;

//
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
 // Serial.print("Bytes received: ");
  //Serial.println(len);
  incomingPT1 = incomingReadings.pt1;
  incomingPT2 = incomingReadings.pt2;
  incomingPT3 = incomingReadings.pt3;
  incomingPT4 = incomingReadings.pt4;
  incomingPT5 = incomingReadings.pt5;
  incomingFM = incomingReadings.fm;
  incomingLC1 = incomingReadings.lc1;
  incomingLC2 = incomingReadings.lc2;
  incomingLC3 = incomingReadings.lc3;
  incomingS1 = incomingReadings.S1;

}
void setup() {
  Commands.S1 = 90;
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(buttonpin1,INPUT);
  pinMode(LEDpin,OUTPUT);

  //set device as WiFi station
  WiFi.mode(WIFI_STA);

  //initialize ESP32
   if (esp_now_init() != ESP_OK) {
    //Serial.println("Error initializing ESP-NOW");
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
    //Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}


void loop() {
  // put your main code here, to run repeatedly:
  pressed = digitalRead(buttonpin1); //push button to send servo signals

  // Serial.println("incomingS1, "+String(incomingS1)+", servo1_curr, "+String(servo1_curr));
  if (incomingS1 == 90){
  valveOpened = true;
} else {
  valveOpened = false;
}
if (valveOpened) {
  digitalWrite(LEDpin,HIGH);
} else {
  digitalWrite(LEDpin,LOW);
}

if (prevPressed && (millis() - pressTime > 5000)) {
  prevPressed = false;
  Commands.S1 = 90 - servo1_curr;
  servo1_curr = 90 - servo1_curr;
}

  if (pressed && !prevPressed) {
    Commands.S1 = 90 - servo1_curr;
    servo1_curr = 90 - servo1_curr;
    pressTime = millis();
    //remove the following line with code that detects the status of the valve
  //  valveOpened = !valveOpened;
  // ADDED
  prevPressed = pressed;
  }

  if (millis() - testTime >= testFMOutputInterval) {
    testTime = millis();
    incomingFM = random(2,10000);
  } else {
    incomingFM = 1;
  }
  //prevPressed = pressed;

  //servo2control
//  currval2 = digitalRead(buttonpin2);
//  if(prevval2==0 && currval2==1){
//  Commands.S2 = 90 - servo2_curr;
//  servo2_curr = 90 - servo2_curr;
//  }
//
//  //servo1andservo2control
//  currval3 = digitalRead(buttonpin3);
//  if(prevval3==0 && currval3==1){
//  Commands.S2 = 90 - servo2_curr;
//  servo2_curr = 90 - servo2_curr;
//  Commands.S1 = 90 - servo1_curr;
//  servo1_curr = 90 - servo1_curr;
//  }
  //start igniter, leave on for a second
//  Commands.I =  (digitalRead(buttonpin4)==1);
//  if(Commands.I){
//  delay(1000);}
//  Commands.I = false;
//  }
//
  //sends inputs for servos based on buttons
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &Commands, sizeof(Commands));
  if (result == ESP_OK) {
   // Serial.println("Sent with success");
  }
  else {
    //Serial.println("Error sending the data");
  }
//   prevval2 = currval2;
//   prevval3 = currval3;


///part to deal with MATLAB interfacing


 Serial.print(millis());
   Serial.print(" ");
   // Print the flow rate for this second in litres / minute
   // Serial.print("Flow rate: ");
   //Serial.print(incomingFM);  // Print the integer part of the variable
   //Serial.print("L/min");
   //Serial.print(" ");       // Print tab space




  //PT TEST


   //Serial.print("Output Pressures: ");
   Serial.print(incomingPT1);
   Serial.print(" ");
   Serial.print(incomingPT2);
   Serial.print(" ");
   //Serial.println("psi / ");

 //  Serial.print(" ");       // Print tab space

 // FM Test

    Serial.println(incomingFM);

    // Print the cumulative total of litres flowed since starting
   //Serial.print("Output Liquid Quantity: ");
    //Serial.print(totalMilliLitres);
    //Serial.print("mL / ");
    //Serial.print(totalMilliLitres / 1000);
    //Serial.print("L");
    //Serial.print("\t");       // Print tab space

  //LC TEST

  //Serial.print("Load Cell Stuff:");
  //Serial.print(incomingLC1);
  //Serial.print(incomingLC2);
  //Serial.print(incomingLC3);

  delay(50); //delay of 50 optimal for recieving and transmitting

}
