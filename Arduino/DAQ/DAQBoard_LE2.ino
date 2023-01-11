/*
This code runs on the DAQ ESP32 and has a couple of main functions.
1. Read sensor data
2. Send sensor data to COM ESP32
3. Recieve servo commands from COM ESP32
4. Send PWM signals to servos
*/

#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Arduino.h>
#include "HX711.h"


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

//SET PINOUTS
#define solenoidPinFuel
#define solenoidPinOx
#define fuelSolVent
#define oxSolVent
#define oxQD
#define fuelQD

#define servo1ClosedPosition 100
#define servo1OpenPosition 10
#define servo2ClosedPosition 80
#define servo2OpenPosition 160

#define pressureFuel 450    //In units of psi. May need to convert to different val in terms of sensor units
#define pressureOx 450    //In units of psi. May need to convert to different val in terms of sensor units
#define tolerance 0.10   //Acceptable range within set pressure
#define pressureDelay 0.5

float currentPosition1 = float('inf');
float currentPosition2 = float('inf');

//define servo min and max values
#define SERVO_MIN_USEC (800)
#define SERVO_MAX_USEC (2100)
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
int MeasurementDelay=1000; //Delay between data measurment periods in the idle loop state in m
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


//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    int messageTime;
     int pt1val;  int pt2val;  int pt3val;  int pt4val;  int pt5val;  int pt6val; int pt7val;
     int fmval;

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

  pinMode(solenoidPinFuel, OUTPUT);
  pinMode(solenoidPinOx, OUTPUT);
  pinMode(fuelSolVent, OUTPUT);
  pinMode(oxSolVent, OUTPUT);
  pinMode(oxQD, OUTPUT);
  pinMode(fuelQD, OUTPUT);

  digitalWrite(RELAYPIN1, HIGH);
  digitalWrite(RELAYPIN2, HIGH);
  digitalWrite(solenoidPinFuel, HIGH);
  digitalWrite(solenoidPinOx, HIGH);
  digitalWrite(fuelSolVent, HIGH);
  digitalWrite(oxSolVent, HIGH);
  digitalWrite(oxQD, LOW);
  digitalWrite(fuelQD, LOW);

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
  //Print MAC Accress on startup for easier connections
  Serial.println(WiFi.macAddress());

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
      if (commandedState==30) {state=30;}

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

  case (30): //Pressurization
    bool fuelStatus = pressurizeFuel();
    bool OxStatus = pressurizeOx();
    if (commandedState == 1) {state = 1; closeSolenoidOx(); closeSolenoidFuel(); vent();}
    if ((fuelStatus && OxStatus) && commandedState = 31) {
      state = 31;
    }

  case (31): //QDs
    disconnectOx();
    disconnectFuel();
    if (commandedState == 1) {state=1; reconnect();}
    if (commandedState == 3) {state=3; MeasurementDelay=pollingMeasurementDelay;}

}

}

void statePrint() {
  
  if ((loopStartTime-lastPrintTime) > PrintDelay) { Serial.println(state); lastPrintTime=loopStartTime; }

}

void reconnect() {
  digitalWrite(oxQD, LOW);
  digitalWrite(fuelQD, LOW);
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

bool pressurizeFuel() {
  // Increase pressure
  while (Readings.pt1val < pressureFuel) {
    openSolenoidFuel();
    if (commandedState = 1) {
      closeSolenoidFuel();
      ventFuel();
      state = 1;
      return false;
    }
  }
  closeSolenoidFuel();
  // Address potential overshoot & hold pressure (within tolerance)
  sleep(pressureDelay);
  if (Readings.pt1val > (1+tolerance)*pressureFuel) {
    while (Readings.pt1val > pressure) {
      openSolenoidFuel();
      if (commandedState = 1) {
        closeSolenoidFuel();
        ventFuel();
        state = 1;
        return false;
      }
    }
    closeSolenoidFuel();
    sleep(pressureDelay);
    if (Readings.pt1val < (1-tolerance)*pressureFuel) {
      pressurizeFuel();
    }
  }
  return true;
}

bool pressurizeOx() {
  // Increase pressure
  while (Readings.pt2val < pressureOx) {
    openSolenoidOx();
    if (commandedState = 1) {
      closeSolenoidOx();
      ventOx();
      state = 1;
      return false;
    }
  }
  closeSolenoidOx();
  // Address potential overshoot & hold pressure (within tolerance)
  sleep(pressureDelay);
  if (Readings.pt1val > (1+tolerance)*pressureOx) {
    while (Readings.pt1val > pressure) {
      openSolenoidOx();
      if (commandedState = 1) {
        closeSolenoidOx();
        ventOx();
        state = 1;
        return false;
      }
    }
    closeSolenoidOx();
    sleep(pressureDelay);
    if (Readings.pt1val < (1-tolerance)*pressureOx) {
      pressurizeOx();
    }
  }
  return true;
}

void openSolenoidFuel() {
  digitalWrite(solenoidPinFuel, LOW);
}

void closeSolenoidFuel() {
  digitalWrite(solenoidPinOx, HIGH);
}

void openSolenoidOx() {
  digitalWrite(solenoidPinOx, LOW);
}

void closeSolenoidOx() {
  digitalWrite(solenoidPinOx, HIGH);
}

void vent() {
  ventOx();
  ventFuel();
}

void ventFuel() {
  digitalWrite(fuelSolVent, LOW);
}

void ventOx() {
  digitalWrite(oxSolVent, LOW);
}

void disconnectOx() {
  digitalWrite(oxQD, HIGH);
}

void disconnectFuel() {
  digitalWrite(fuelQD, HIGH);
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
  ReadingsQueue[queueLength].pt1val=pt1val;
  ReadingsQueue[queueLength].pt2val=pt2val;
  ReadingsQueue[queueLength].pt3val=pt3val;
  ReadingsQueue[queueLength].pt4val=pt4val;
  ReadingsQueue[queueLength].pt5val=pt5val;
  ReadingsQueue[queueLength].pt6val=pt6val;
  ReadingsQueue[queueLength].pt7val=pt7val;
  ReadingsQueue[queueLength].fmval=fmval;
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
  Readings.pt1val = ReadingsQueue[queueLength].pt1val;
  Readings.pt2val = ReadingsQueue[queueLength].pt2val;
  Readings.pt3val = ReadingsQueue[queueLength].pt3val;
  Readings.pt4val = ReadingsQueue[queueLength].pt4val;
  Readings.pt5val = ReadingsQueue[queueLength].pt5val;
  Readings.pt6val = ReadingsQueue[queueLength].pt6val;
  Readings.pt7val = ReadingsQueue[queueLength].pt7val;
  Readings.fmval  = ReadingsQueue[queueLength].fmval;
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
