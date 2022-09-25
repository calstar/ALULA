/*
This code runs on the DAQ ESP32 and has a couple of main functions.
1. Read sensor data
2. Send sensor data to COM ESP32
3. Recieve commands from COM ESP32
4. Send PWM signals to servos (if in use)
*/

#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Arduino.h>
#include "HX711.h"

/* TEST SPECIFIC PARAMETERS:
numPTs - Number of pressure transducers connected
numLCs - Number of load cells connected
numRTDs - Number of RTDs connected
flowInc - Boolean determining whether a flowmeter is connected
test - Name of the testcase being used

Pinouts for different sensors. Assign sensors in ASCENDING NUMERICAL ORDER
(i.e. If there are three PTs, assign PT1, PT2, and PT3). Note that these can 
change depending on the board being used. Note that each sensor requires a
different clock pin.
If a sensor is not used, the numerical pin assignment is arbitrary. It is 
recommended to assign a negative value to ensure that they are not accidentally
used and to improve readability for which sensors are in use.
*/
#define numPTs 0
#define numLCs 0
#define numRTDs 0
bool flowInc = false;
String test = "LE2"

#define FMPIN 4 //Flowmeter pin
#define PTDOUT1 32
#define CLKPT1 5
#define PTDOUT2 15
#define CLKPT2 2
#define PTDOUT3 22
#define CLKPT3 23
#define PTDOUT4 19
#define CLKPT4 21
#define PTDOUT5 35
#define CLKPT5 25
#define PTDOUT6 34
#define CLKPT6 26
#define PTDOUT7 39
#define CLKPT7 33

#define SERVOPIN1 13
#define SERVOPIN2 12
#define RELAYPIN1 14
#define RELAYPIN2 27

/* Manually set hardware specific parameters. These parameters should not
change between testcases but may change if the hardware is altered. If the
system isn't exhibiting expected behavior, check these parameters.
*/

// Define the closed and open angles for servos
#define servo1ClosedPosition 100
#define servo1OpenPosition 10
#define servo2ClosedPosition 80
#define servo2OpenPosition 160

// Define servo min and max values
#define SERVO_MIN_USEC (800)
#define SERVO_MAX_USEC (2100)

float currentPosition1 = float('inf');
float currentPosition2 = float('inf');

//define servo necessary values
#define ADC_Max 4096;

//Initialize flow meter variables for how it computes the flow amount
short currentMillis = 0;
short goalTime = 50;
short currReading1;
short currReading2;
short loopTime=10;

unsigned long igniteTimeControl = 0;
unsigned long igniteTime =  250;

float servo1curr =0;
float servo2curr=0;

//FM counter
float fmcount;
float flowRate;
boolean currentState;
boolean lastState = false;

// Serial Message setup
String serialMessage = "";

//Measuring output from voltage divider
int readVoltage;
float convertedVoltage;

//Initialize the PT and LC sensor objects which use the HX711 breakout board
HX711 scale1;
HX711 scale2;
HX711 scale3;
HX711 scale4;
HX711 scale5;
HX711 scale6;
HX711 scale7;
HX711 scale8;
HX711 scale9;
HX711 scale10;
HX711 scale11;

//Initialize the servo objects
Servo servo1;
Servo servo2;

///////////////
//IMPORTANT
//////////////
// REPLACE WITH THE MAC Address of your receiver

//OLD COM BOARD {0xC4, 0xDD, 0x57, 0x9E, 0x91, 0x6C}
// COM BOARD {0x7C, 0x9E, 0xBD, 0xD7, 0x2B, 0xE8}
//HEADERLESS BOARD {0x7C, 0x87, 0xCE, 0xF0 0x69, 0xAC}
//NEWEST COM BOARD IN EVA {0x24, 0x62, 0xAB, 0xD2, 0x85, 0xDC}
// uint8_t broadcastAddress[] = {0x24, 0x62, 0xAB, 0xD2, 0x85, 0xDC};
uint8_t broadcastAddress[] ={0x7C, 0x9E, 0xBD, 0xD7, 0x2B, 0xE8};
// {0x7C, 0x87, 0xCE, 0xF0, 0x69, 0xAC};
//{0x3C, 0x61, 0x05, 0x4A, 0xD5, 0xE0};
// {0xC4, 0xDD, 0x57, 0x9E, 0x96, 0x34}

int count=3;

//STATEFLOW VARIABLES
int state=-1;
unsigned int dataArraySize =0;
int loopStartTime=0;
int MeasurementDelay=1000; //Delay between data measurment periods in the idle loop state in ms
int idleMeasurementDelay=1000;
int pollingMeasurementDelay=200;
int hotfireMeasurementDelay=20;

int lastPrintTime=0;
int PrintDelay =1000;

int lastMeasurementTime=-1;
short int queueLength=0;
int commandedState;

int hotfireStage1Time=750;
int hotfireStage2Time=8500;
int hotfireStage3Time=9300;
int hotfireStage4Time=10800;
int igniterTime=750;

int hotfireTimer=0;
int igniterTimer=0;



//the following are only used in the oposite direction, they are included because it may be necessary for the structure to be the same in both directions
int S1;
int S2;
// int S1S2;
int I;

// int commandedState;
// int prev_S1S2 = 0;
bool ignite = 1;

// Define variables to store incoming commands, servos and igniter
int incomingS1;
int incomingS2;
// int incomingS1S2;
bool incomingI;

int DAQstate = 0;

// int I = 0;

float startTime;
float endTime;
float timeDiff;

// Variable to store if sending data was successful
String success;

// Define variables to store readings to be sent
int messageTime=10;
int pt1val=1;
int pt2val=1;
int pt3val=1;
int pt4val=1;
int pt5val=1;
int pt6val=1;
int pt7val=1;
int fmval=2;
float rtd1val=1;
float rtd2val=1;
float rtd3val=1;
float rtd4val=1;


//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    int messageTime;
    int pt1;  
    int pt2;  
    int pt3;  
    int pt4;  
    int pt5;  
    int pt6; 
    int pt7;
    int pt8;
    int lc1;
    int lc2;
    int lc3;
    float rtd1;
    float rtd2;
    float rtd3;
    float rtd4;
    int flow;

    unsigned char S1; unsigned char S2; int commandedState=1; 
    int DAQstate=0;unsigned char I; short int queueSize;
    int Debug;
} struct_message;

// Create a struct_message called Readings to hold sensor readings
struct_message Readings;
//create a queue for readings in case
struct_message ReadingsQueue[120];

// Create a struct_message to hold incoming commands
struct_message Commands;

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
  memcpy(&Commands, incomingData, sizeof(Commands));
 //  Serial.print("Bytes received: ");
 //  Serial.print(len);
      // digitalWrite(ONBOARD_LED,HIGH);

  S1 =Commands.S1;
  S2 = Commands.S2;
  // Serial.print(Commands.S1);
  // Serial.print(" ");
  // Serial.println(Commands.S2);

 // UNCOMMENT THIS LATER!!!!!!!!!!!!!!!!
 commandedState = Commands.commandedState;
 // Serial.println(commandedState);

}


void SerialRead() {
    if (Serial.available() > 0) {
 commandedState=Serial.read()-48;
 Serial.print("AVAILABLE--------------------");
    Serial.println(commandedState);
    Serial.println(" ");
  }
     //   Serial.println(commandedState);
    //    Serial.println(" ");
}




void setup() {
  //attach servo pins
  servo1.attach(SERVOPIN1,SERVO_MIN_USEC,SERVO_MAX_USEC);
  servo2.attach(SERVOPIN2,SERVO_MIN_USEC,SERVO_MAX_USEC);

  // attach onboard LED
  // pinMode(ONBOARD_LED,OUTPUT);
  pinMode(RELAYPIN1, OUTPUT);
  pinMode(RELAYPIN2, OUTPUT);

  digitalWrite(RELAYPIN1, HIGH);
  digitalWrite(RELAYPIN2, HIGH);

  //set gains for pt pins
  scale1.begin(PTDOUT1, CLKPT1); scale1.set_gain(64);
  scale2.begin(PTDOUT2, CLKPT2); scale2.set_gain(64);
  scale3.begin(PTDOUT3, CLKPT3); scale3.set_gain(64);
  scale4.begin(PTDOUT4, CLKPT4); scale4.set_gain(64);
  scale5.begin(PTDOUT5, CLKPT5); scale5.set_gain(64);
  scale6.begin(PTDOUT6, CLKPT6); scale6.set_gain(64);
  scale7.begin(PTDOUT7, CLKPT7); scale7.set_gain(64);

  //Flowmeter untreupt
  pinMode(FMPIN, INPUT);           //Sets the pin as an input

  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  //Print MAC Address on startup for easier connections
  Serial.println(WiFi.macAddress());

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Transmitted packet
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
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}

void servoWrite() {
    servo1.write(servo1curr);
    servo2.write(servo2curr);
    S1=servo1curr;
    S2=servo2curr;
}

void loop() {
loopStartTime=millis();
SerialRead();
// State selector

statePrint();


switch (state) {

  case (17): //BASIC WIFI TEST DEBUG STATE B
   wifiDebug();
   
    if (commandedState==1) {state=1;} 
  break;

  case (-1): //start single loop

  servo1curr=servo1ClosedPosition;
  servo2curr=servo2ClosedPosition;
  servoWrite();
  
  state=0;
  break;

  case (0): //Default/idle
      idle();

      if (commandedState==1) { state=1; MeasurementDelay=pollingMeasurementDelay; }
      if (commandedState==2) { state=2; MeasurementDelay=pollingMeasurementDelay; }
      if (commandedState==3) { state=3; MeasurementDelay=pollingMeasurementDelay; }
    break;

  case (1): //Polling
      polling();



      if (commandedState==0) { state=0; MeasurementDelay=idleMeasurementDelay; }
      if (commandedState==2){  state=2; MeasurementDelay=pollingMeasurementDelay; }
      if (commandedState==3) { state=3; MeasurementDelay=pollingMeasurementDelay; }
    if (commandedState==17) {state=17;} 

    break;

  case (2): //Manual Servo Control


 manualControl();
      if (commandedState==0) { state=0; MeasurementDelay=idleMeasurementDelay; }
      if (commandedState==1) { state=1; MeasurementDelay=pollingMeasurementDelay; }
      if (commandedState==3) { state=3; MeasurementDelay=pollingMeasurementDelay; }
    break;

  case (3): //Armed

armed();

      if (commandedState==0) { state=0; MeasurementDelay=idleMeasurementDelay; }
      if (commandedState==1) { state=1; MeasurementDelay=idleMeasurementDelay; }
      if (commandedState==4) { state=4; igniterTimer=loopStartTime; }
    break;


  case (4): //Ignition

    ignition();
    if (commandedState==0) { state=0; MeasurementDelay=idleMeasurementDelay; }
    if (commandedState==1) { state=1; MeasurementDelay=idleMeasurementDelay; }

    if (commandedState==5) { state=5; hotfireTimer=loopStartTime; MeasurementDelay=hotfireMeasurementDelay; }


    break;
  case (5): //Hotfire stage 1

    hotfire1();

    if ((loopStartTime-hotfireTimer) > hotfireStage1Time) state=6;


    break;


  case (6): //Hotfire stage 2

    hotfire2();
  if ((loopStartTime-hotfireTimer) > hotfireStage2Time) state=7;

    break;


  case (7): //Hotfire stage 3

    hotfire3();
  if ((loopStartTime-hotfireTimer) > hotfireStage3Time) state=8;

    break;

  case (8): //Hotfire stage 4

    hotfire4();
  if ((loopStartTime-hotfireTimer) > hotfireStage4Time){  state=0; MeasurementDelay=idleMeasurementDelay; }

    break;



}

}

void statePrint() {
  
  if ((loopStartTime-lastPrintTime) > PrintDelay) { Serial.println(state); lastPrintTime=loopStartTime; }

}


void idle() {
    DAQstate = state;

dataCheck();

}

void dataCheck() {
  if ((loopStartTime-lastMeasurementTime) > MeasurementDelay) addReadingsToQueue();
  checkQueue();
}

void polling() {
    DAQstate = state;
  dataCheck();
}
void manualControl() {
  DAQstate = state;
  servo1curr=S1;
  servo2curr=S2;
  servoWrite();
  dataCheck();
}
void armed() {
  DAQstate = state;
  dataCheck();
}

void ignition() {
DAQstate = state;

    if ((loopStartTime-igniterTimer) < igniterTime) { digitalWrite(RELAYPIN1, LOW); digitalWrite(RELAYPIN2, LOW); Serial.print("IGNITE"); }
    if ((loopStartTime-igniterTimer) > igniterTime) {  digitalWrite(RELAYPIN1, HIGH); digitalWrite(RELAYPIN2, HIGH); Serial.print("NO"); }
        dataCheck();

  Serial.println(loopStartTime-igniterTimer);
  Serial.println("Igniter time");
  Serial.println(igniterTime);
  Serial.println(" ");
}



void hotfire1() {
  DAQstate = 5;
  dataCheck();

  servo1curr=servo1ClosedPosition;
  servo2curr=servo2OpenPosition;
  servoWrite();

}
void hotfire2() {
  dataCheck();

  servo1curr=servo1OpenPosition;
  servo2curr=servo2OpenPosition;
  servoWrite();
}
void hotfire3() {
  dataCheck();

  servo1curr=servo1OpenPosition;
  servo2curr=servo2ClosedPosition;
  servoWrite();
}
void hotfire4() {
  dataCheck();

  servo1curr=servo1ClosedPosition;
  servo2curr=servo2ClosedPosition;
  servoWrite();
}





void addReadingsToQueue() {
  getReadings();
  if (queueLength<40) queueLength+=1;
  ReadingsQueue[queueLength].messageTime=loopStartTime;
  ReadingsQueue[queueLength].pt1=pt1val;
  ReadingsQueue[queueLength].pt2=pt2val;
  ReadingsQueue[queueLength].pt3=pt3val;
  ReadingsQueue[queueLength].pt4=pt4val;
  ReadingsQueue[queueLength].pt5=pt5val;
  ReadingsQueue[queueLength].pt6=pt6val;
  ReadingsQueue[queueLength].pt7=pt7val;
  ReadingsQueue[queueLength].pt8=pt8val;
  ReadingsQueue[queueLength].lc1=lc1val;
  ReadingsQueue[queueLength].lc2=lc2val;
  ReadingsQueue[queueLength].lc3=lc3val;
  ReadingsQueue[queueLength].flow=fmval;
  ReadingsQueue[queueLength].queueSize=queueLength;
  ReadingsQueue[queueLength].I = I;
  ReadingsQueue[queueLength].DAQstate = DAQstate;
  ReadingsQueue[queueLength].S1 = servo1curr;
  ReadingsQueue[queueLength].S2 = servo2curr;

  
  
}



void getReadings(){

 pt1val = scale1.read(); 
 pt2val = scale2.read() ; 
 pt3val = scale3.read(); 
 pt4val = scale4.read(); 
 pt5val = scale5.read(); 
 pt6val = scale6.read(); 
 pt7val = scale7.read();
 pt8val = scale8.read();
 lc1val = scale9.read();
 lc2val = scale10.read();
 lc3val = scale11.read();

   // flowMeterReadings();
    printSensorReadings();
    lastMeasurementTime=loopStartTime;
   // Serial.print("Queue Length :");
  //  Serial.println(queueLength);

   // Serial.print("Current State: ");
    //Serial.println(state);
}



void flowMeterReadings() {
  currentMillis = millis();
  fmcount = 0;

   while (millis() - currentMillis < goalTime) {
    servo1.write(servo1curr);
    servo2.write(servo2curr);


    currentState = digitalRead(FMPIN);
    if (!(currentState == lastState)) {

     lastState = currentState;
     fmcount += 1;
   }
 }
  flowRate = fmcount;
  fmval =int(flowRate+1);  // Print the integer part of the variable
}



void printSensorReadings() {
   serialMessage = "";
 //
 serialMessage.concat(millis());
 serialMessage.concat(" ");
 serialMessage.concat(pt1val);
 serialMessage.concat(" ");
 serialMessage.concat(pt2val);
 serialMessage.concat(" ");
 serialMessage.concat(pt3val);
 serialMessage.concat(" ");
 serialMessage.concat(pt4val);
 serialMessage.concat(" ");
 serialMessage.concat(pt5val);
 serialMessage.concat(" ");
 serialMessage.concat(pt6val);
 serialMessage.concat(" ");
 serialMessage.concat(pt7val);
 serialMessage.concat(" ");
 serialMessage.concat(pt8val);
 serialMessage.concat(" ");
 serialMessage.concat(lc1val);
 serialMessage.concat(" ");
 serialMessage.concat(lc2val);
 serialMessage.concat(" ");
 serialMessage.concat(lc3val);
 serialMessage.concat(" Queue Length: ");
 serialMessage.concat(queueLength);
 serialMessage.concat(" Current State: ");
 serialMessage.concat(state);
 Serial.println(serialMessage);

}


void checkQueue() {
  if (queueLength>0){
    dataSend();
  }
}

void dataSend() {
   // Set values to send
  Readings.messageTime=ReadingsQueue[queueLength].messageTime;
  Readings.pt1 = ReadingsQueue[queueLength].pt1;
  Readings.pt2 = ReadingsQueue[queueLength].pt2;
  Readings.pt3 = ReadingsQueue[queueLength].pt3;
  Readings.pt4 = ReadingsQueue[queueLength].pt4;
  Readings.pt5 = ReadingsQueue[queueLength].pt5;
  Readings.pt6 = ReadingsQueue[queueLength].pt6;
  Readings.pt7 = ReadingsQueue[queueLength].pt7;
  Readings.pt8 = ReadingsQueue[queueLength].pt8;
  Readings.lc1 = ReadingsQueue[queueLength].lc1;
  Readings.lc2 = ReadingsQueue[queueLength].lc2;
  Readings.lc3 = ReadingsQueue[queueLength].lc3;
  Readings.flow  = ReadingsQueue[queueLength].flow;
  Readings.I = ReadingsQueue[queueLength].I;
  Readings.DAQstate = ReadingsQueue[queueLength].DAQstate;
  Readings.S1 = ReadingsQueue[queueLength].S1;
  Readings.S2 = ReadingsQueue[queueLength].S2;

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &Readings, sizeof(Readings));

  if (result == ESP_OK) {
     Serial.println("Sent with success Data Send");
   //  ReadingsQueue[queueLength].pt1val=0;
     queueLength-=1;
  }
  else {
     Serial.println("Error sending the data");
  }
}

void wifiDebug() {
  Readings.Debug=17;
  dataSend();
  Serial.println(Commands.Debug);  
}
