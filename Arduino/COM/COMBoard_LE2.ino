#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Arduino.h>
#include "HX711.h"

#define BUTTON1 19
#define BUTTON2 17
#define BUTTON3 16
#define BUTTON4 21 //SET PIN NUMBER BUTTON
#define PRESS_BUTTON
#define QD_BUTTON

#define INDICATOR1  4 // state 2 light
#define INDICATOR2 23 // armed indicator
#define INDICATOR3 22//Servo1 indicator
#define INDICATOR4 14//seevo 2 
#define INDICATOR5 25//daq indicaor
#define INDICATOR6 5//com indicator



// Comes from manual calibration. Adjust if servos no longer
// in correct closed or open position
#define servo1ClosedPosition 100
#define servo1OpenPosition 10
#define servo2ClosedPosition 130
#define servo2OpenPosition 0

float pressTime = 0;




String success;
String message;
int incomingMessageTime;
int servo1_curr = servo1ClosedPosition;
int servo2_curr = servo2ClosedPosition;
float incomingS1 = 0;
float incomingS2 = 0;
 int incomingPT1 = 4;
 int incomingPT2 = 4;
 int incomingPT3 = 4;
 int incomingPT4 = 4;
 int incomingFM = 0;
 int incomingPT5 = 4;
 int incomingPT6 = 4;
 int incomingPT7 = 4;
short int incomingI = 0;
int incomingDebug=0;
int actualState = -5;
short int queueSize=0;
esp_now_peer_info_t peerInfo;
bool pressed1 = false;
bool pressed2 = false;
bool pressed3 = false;
int commandstate = 0;


//TIMING VARIABLES
int state=-1;
int loopStartTime=0;
int ignitionSendDelay=50;
int pollingSendDelay=100;
int dataCollectionDelay=10;
int SendDelay=pollingSendDelay;

int commandedState;
int serialState;

int S1=servo1ClosedPosition; 
int S2=servo2ClosedPosition;
int lastSendTime=0;

int lastPrintTime=0;
int PrintDelay =1000;



// SET FOR ACTIVE MODE //
bool hotfireMode = true;
//SET IF PLOTTING WITH MATLAB OR NOT. SERVO MANUAL CONTROL AND
//MATLAB PLOTTING ARE NOT COMPATIBLE DUE TO USING THE SAME SERIAL
//INPUT. IF TRUE, PLOTTING ENABLED. IF FALSE, MANUAL CONTROL ENABLED
bool MatlabPlot = true;


float button1Time = 0;
float currTime = 0;
float loopTime = 0;

float receiveTimeDAQ = 0;
float receiveTimeCOM = 0;

//for blinking LED during Data Collection
int x = 1;
unsigned long t1;
unsigned long t2;

//

//ENSURE IP ADDRESS IS CORRECT FOR DEVICE IN USE!!!
//DAQ Breadboard {0x24, 0x62, 0xAB, 0xD2, 0x85, 0xDC}
//DAQ Protoboard {0x0C, 0xDC, 0x7E, 0xCB, 0x05, 0xC4}
//NON BUSTED DAQ {0x7C, 0x9E, 0xBD, 0xD8, 0xFC, 0x14}
// uint8_t broadcastAddress[] = {0x7C, 0x9E, 0xBD, 0xD8, 0xFC, 0x14}; //change to new Mac Address
// {0xC4, 0xDD, 0x57, 0x9E, 0x96, 0x34};

uint8_t broadcastAddress[] = {0x30, 0xC6, 0xF7, 0x2A, 0x28, 0x04};
//{0x30, 0xC6, 0xF7, 0x2A, 0x28, 0x04}

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    int messageTime;
     int pt1val;
     int pt2val;
     int pt3val;
     int pt4val;
     int pt5val;
     int pt6val;
     int pt7val;
     int fmval;
    unsigned char S1;
    unsigned char S2;
        int commandedState = 0;
        int DAQstate=0;

    unsigned char I;
    short int queueSize;
    int Debug;
} struct_message;

// Create a struct_message called Readings to recieve sensor readings remotely
struct_message incomingReadings;

// Create a struct_message to send commands
struct_message Commands;

//


void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
 // Serial.print("\r\nLast Packet Send Status:\t");
 // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");

  if (status == 0){
    success = "Delivery Success :)";
    digitalWrite(INDICATOR2, HIGH);
    receiveTimeDAQ = millis();
  }
  else{
    success = "Delivery Fail :(";
    digitalWrite(INDICATOR2, LOW);
  }
  // Serial.print(Commands.S1);
  // Serial.print(" ");
  // Serial.println(Commands.S2);

}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
 // Serial.print("Bytes received: ");
   // Serial.println(len);
  incomingPT1 = incomingReadings.pt1;
     // Serial.print(incomingPT1);

  digitalWrite(INDICATOR1,HIGH);

  incomingMessageTime= incomingReadings.messageTime;
  incomingPT2 = incomingReadings.pt2;
  incomingPT3 = incomingReadings.pt3;
  incomingPT4 = incomingReadings.pt4;
  incomingFM = incomingReadings.fmval;
  incomingPT5 = incomingReadings.pt5;
  incomingPT6 = incomingReadings.pt6;
  incomingPT7 = incomingReadings.pt7;
  incomingS1 = incomingReadings.S1;
  incomingS2 = incomingReadings.S2;
  queueSize= incomingReadings.queueSize;
  incomingI = incomingReadings.I;
  actualState = incomingReadings.DAQstate;
  incomingDebug= incomingReadings.Debug;

  
  receiveTimeCOM = millis();
  // Serial.println("Data received");
  RecieveDataPrint();

}

void SerialRead() {
    if (Serial.available() > 0) {
 serialState=Serial.read()-48;
// Serial.print("AVAILABLE--------------------");
   // Serial.println(serialState);
   // Serial.println(" ");

  }
     //   Serial.println(commandedState); 
     //    Serial.println(" ");

}

void setup() {
  Commands.S1 = servo1ClosedPosition;
  Commands.S2 = servo2ClosedPosition;
  // put your setup code here, to run once:
  Serial.begin(115200);
   // Serial.println("Start of Setup");

  // Use buttons for moving between states
  pinMode(BUTTON1,INPUT);
  pinMode(BUTTON2, INPUT);
  pinMode(BUTTON3,INPUT);
  pinMode(BUTTON4, INPUT);

  pinMode(INDICATOR2, OUTPUT);
  pinMode(INDICATOR3, OUTPUT);
  pinMode(INDICATOR4, OUTPUT);
  pinMode(INDICATOR5, OUTPUT);
  pinMode(INDICATOR6, OUTPUT);
  pinMode(INDICATOR1, OUTPUT);

  pinMode(PRESS_BUTTON, OUTPUT);
  pinMode(QD_BUTTON, OUTPUT);

  digitalWrite(INDICATOR2,LOW);
  digitalWrite(INDICATOR3, LOW);
  digitalWrite(INDICATOR4, LOW);
  digitalWrite(INDICATOR5, LOW);
  digitalWrite(INDICATOR6, LOW);
  digitalWrite(INDICATOR1, LOW);

  digitalWrite(PRESS_BUTTON, LOW);
  digitalWrite(QD_BUTTON, LOW);

  //set device as WiFi station
  WiFi.mode(WIFI_STA);

  //initialize ESP32
   if (esp_now_init() != ESP_OK) {
  //  Serial.println("Error initializing ESP-NOW");
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
 //   Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

  Serial.println(WiFi.macAddress());


}



void printLine(String string) {
  Serial.println(string);
}


void LEDUpdate() {

  if (incomingS1 == servo1OpenPosition) digitalWrite(INDICATOR3,HIGH); else digitalWrite(INDICATOR3,LOW);
  if (incomingS2 == servo2OpenPosition) digitalWrite(INDICATOR4,HIGH); else digitalWrite(INDICATOR4,LOW);
  digitalWrite(INDICATOR1,LOW);
}



void loop() {
  loopStartTime=millis();
  SerialRead();
  // State selector
  //Serial.println(actualState);

  LEDUpdate();


switch (state) {

  case (17): //BASIC WIFI TEST DEBUG STATE A STATE
   wifiDebug();
   
    if (serialState==1) {state=1;} 
  break;
  

  case (18): //COM INTERFACE BUTTON DEBUG STATE  B STATE
  comDebug();

    if (serialState==1) {state=1;} 
  break;

  case (-1): //start single loop

    state=0; 
  break;


  case (0): //Default/idle
      idle();
      state=1;  
    break;

  case (1): //Polling
    polling();

    if ((digitalRead(BUTTON2)==1)||(serialState==2)) { state=3; S1=servo1ClosedPosition; S2=servo2ClosedPosition; SendDelay=pollingSendDelay; }
    if (serialState==18) {state=18;} 
    if (serialState==17) {state=17;} 
    if (digitalRead(PRESS_BUTTON)==30) {state=30;}

    break;

  case (2): //Manual Servo Control

 manualControl();
  if ((serialState==40)) { state=0; SendDelay=pollingSendDelay; }

  break;

  case (3): //Armed
    armed();

    //button to ignition 
    if ((digitalRead(BUTTON3)==1)||(serialState==3)) { state=4; S1=servo1ClosedPosition; S2=servo2ClosedPosition; SendDelay=ignitionSendDelay; }
    //RETURN BUTTON
    if ((digitalRead(BUTTON1)==1)||(serialState==1)) { state=1; S1=servo1ClosedPosition; S2=servo2ClosedPosition; SendDelay=pollingSendDelay; }
      
    break;


  case (4): //Ignition

    ignition();
    //HOTFIRE BUTTON
      if ((digitalRead(BUTTON4)==1)||(serialState==4)) state=5; 
      //RETURN BUTTON
      if ((digitalRead(BUTTON1)==1)||(serialState==1)) { state=1; S1=servo1ClosedPosition; S2=servo2ClosedPosition; SendDelay=pollingSendDelay; }
      


    break;
  case (5): //Hotfire stage 1

    hotfire();

    if (actualState==0) state=0;


    break;

  case (30): //Pressurization
    pressurize();
    if ((digitalRead(QD_BUTTON) == 31)) {
      state = 31;
    }
    if (digitalRead(BUTTON1)==1) {
      state = 1;
    }
  
  case (31): //QDs
    disconnect();
    if (digitalRead(BUTTON1) == 1) {
      state==1;
    }
    if (digitalRead(BUTTON2 == 1)) {
      state = 3;
    }

}

}
void statePrint() {
  

}



void pressurize() {
  commandedState=30;
  dataSendCheck();
}

void disconnect() {
  commandedState=31;
  dataSendCheck();
}

void idle() {
  dataSendCheck();
  digitalWrite(INDICATOR5,LOW);

}

void polling() {
  commandedState=1;
  dataSendCheck();
  digitalWrite(INDICATOR5,HIGH);

}

void manualControl() {
  commandedState=2;
  dataSendCheck();

    digitalWrite(INDICATOR5,LOW);


}

void armed() {
  commandedState=3;
  dataSendCheck();
  digitalWrite(INDICATOR5,LOW);

}

void ignition() {
  commandedState=4;
  dataSendCheck();

}

void hotfire() {
  commandedState=5;
  dataSendCheck();

}

void dataSendCheck() {
  if ((loopStartTime-lastSendTime) > SendDelay) dataSend(); 
}


void dataSend() {
   // Set values to send
  Commands.S1= S1;
  Commands.S2= S2;
  Commands.commandedState = commandedState;

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &Commands, sizeof(Commands));

  if (result == ESP_OK) {
   //  Serial.println("Sent with success Data Send");
  }
  else {
  //   Serial.println("Error sending the data");
  }

  lastSendTime=loopStartTime;
}







void oldcode() {
  message = "";





  switch (state) {
    case 0:


      digitalWrite(INDICATOR2, LOW);
      digitalWrite(INDICATOR1, LOW);

      currTime = millis();
      if (pressed1 && ((currTime - button1Time) > 1000)) {
        state = 1;
        button1Time = currTime;
        // Serial.println("State 1");
        // Serial.println("BUTTON 1 GOOD");
      }
  
      break;

    case 1:

 
      pressed2 = digitalRead(BUTTON2);
      pressed3 = digitalRead(BUTTON1);
      currTime = millis();
      if (pressed2) {

        if (hotfire) state = 4; else state = 2;

        if (hotfire) {
          state = 4;
        } else {
          state = 2;
      }

        // Serial.println("State 2");
      }
      if (pressed3 && ((currTime - button1Time) > 1000)) {
        button1Time = currTime;
        state = 0;
        // Serial.println("State 0");
      }
      if ((millis() - receiveTimeDAQ) > 50) {
        digitalWrite(INDICATOR5, LOW);
      }
      if ((millis() - receiveTimeCOM) > 50) {
        digitalWrite(INDICATOR6, LOW);
      }
      break;
    case 2:
    //  Serial.println("State 2");
      //Serial.println("IN CASE 2");
      if (MatlabPlot) {
        Commands.S1 = 90 - servo1_curr;
        Commands.S2 = 90 - servo2_curr;
        servo1_curr = 90 - servo1_curr;
        servo2_curr = 90 - servo2_curr;
        pressTime = millis();
        digitalWrite(INDICATOR1, HIGH);

        while (millis() - pressTime <= 5000) {

          t2=millis();
          if (t2-t1 >=100){
              x=1-x;
              t1=millis();
              digitalWrite(INDICATOR2,x);
              // Serial.print(t1);
            }

          pressed3 = digitalRead(BUTTON1);
          if (pressed3) {
            state = 0;
            break;
          }
          if (state == 3) {
            break;
          }
          if ((millis() - receiveTimeDAQ) > 50) {
            digitalWrite(INDICATOR5, LOW);
          }
          if ((millis() - receiveTimeCOM) > 50) {
            digitalWrite(INDICATOR6, LOW);
          }

          if ((millis() - loopTime) >= 50) {
            esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &Commands, sizeof(Commands));
            if (result != ESP_OK) {
              break;
            // Serial.println("Sent with success");
            }
            //else {
              //Serial.println("Error sending the data");

   

         }
        }

      } else {
        while (!Serial.available()) {
          pressed3 = digitalRead(BUTTON1);
          // Serial.print(pressed3);
          delay(5);
          if ((millis() - receiveTimeDAQ) > 50) {
            digitalWrite(INDICATOR5, LOW);
          }
          if ((millis() - receiveTimeCOM) > 50) {
            digitalWrite(INDICATOR6, LOW);
          }
          esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &Commands, sizeof(Commands));
            if (result != ESP_OK) {
              break;
            // Serial.println("Sent with success");
            }
            //else {
              //Serial.println("Error sending the data");
            //}
          if (pressed3) {
            state = 0;
            // Serial.println("State 0");
            break;}  //waiting for inputs
          }
        String angles = Serial.readString();
        int index = angles.indexOf(",");
        String readAngle1 = angles.substring(0, index);
        String readAngle2 = angles.substring(index+1, angles.length());
        if (angles.length() >= 3) {
          Serial.println(readAngle1);
          Serial.println(readAngle2);
          Commands.S1 = readAngle1.toInt();
          Commands.S2 = readAngle2.toInt();
          if (Commands.S1 == servo1ClosedPosition) {
            digitalWrite(INDICATOR3, LOW);
          } else {
            digitalWrite(INDICATOR3, HIGH);
          }

          if (Commands.S2 == servo2ClosedPosition) {
            digitalWrite(INDICATOR4, LOW);
          } else {
            digitalWrite(INDICATOR4, HIGH);
          }
          esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &Commands, sizeof(Commands));
          if (result != ESP_OK) {
              break;
            // Serial.println("Sent with success");
            }
            //else {
              //Serial.println("Error sending the data");
            //}

        }

      }
      break;
     case 3:
    //  Serial.println("State 3");
      Commands.S1 = servo1ClosedPosition;
      Commands.S2 = servo2ClosedPosition;
      digitalWrite(INDICATOR5, LOW);
      digitalWrite(INDICATOR6, LOW);
      digitalWrite(INDICATOR3, LOW);
      digitalWrite(INDICATOR4, LOW);
      if ((millis() - receiveTimeDAQ) > 50) {
        digitalWrite(INDICATOR5, LOW);
      }
      if ((millis() - receiveTimeCOM) > 50) {
        digitalWrite(INDICATOR6, LOW);
      }

      break;
    case 4:


      // Serial.println("State 4");
      if (incomingReadings.DAQstate == 4 || commandstate == 4){
        Commands.commandedState = 4;
        Serial.println("case 3 command state 3");


      } else {
          Commands.commandedState = 3;
          Serial.println("case 4 command state 3");
      }

     
      digitalWrite(INDICATOR1, HIGH);
      if (digitalRead(BUTTON1)) {
        state = 0;
        }
        if ((millis() - receiveTimeDAQ) > 100) {
          digitalWrite(INDICATOR5, LOW);
        }
      if ((millis() - receiveTimeCOM) > 100) {
        digitalWrite(INDICATOR6, LOW);
        }


 
      if (digitalRead(BUTTON1)) {
        state = 0;
      }
      if ((millis() - receiveTimeDAQ) > 100) {
        digitalWrite(INDICATOR5, LOW);
      }
      if ((millis() - receiveTimeCOM) > 100) {
        digitalWrite(INDICATOR6, LOW);
      }
      break;
    case 5:
    Serial.println(commandstate);
    if (incomingReadings.DAQstate == 5 || commandstate == 5) {
        Commands.commandedState = 5;

    } else  {
        Commands.commandedState = 4;
      }



      if (digitalRead(BUTTON4)) {
        Commands.commandedState = 5;
        commandstate = 5;
        Serial.print("yyyyyyy");

      if (digitalRead(BUTTON4)) {
        // Commands.I = true;
        Commands.S1 = servo1OpenPosition;
        Commands.S2 = servo2OpenPosition;
        // Serial.println(Commands.I);
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &Commands, sizeof(Commands));

        if (result != ESP_OK) {

          break;
          }


        state = 0;
      
        float now = millis();
        float runningTime = millis();
        digitalWrite(INDICATOR3, HIGH);
        digitalWrite(INDICATOR4, HIGH);
        while ((now - runningTime) <= 3000) {
          now = millis();

        }
        digitalWrite(INDICATOR3, LOW);
        Commands.S1 = servo1ClosedPosition;

        now = millis();
        runningTime = millis();
        while ((now- runningTime) <= 500) {
          now = millis();
          

         // printLine(message);
          now = millis();
        }
        Commands.S2 = servo2ClosedPosition;
       
        digitalWrite(INDICATOR4, LOW);
      }
      if (digitalRead(BUTTON1)) {
        state = 0;
      }

    delay(30);
  }




  RecieveDataPrint();


}
}

void RecieveDataPrint() {

  message = "";
  message.concat(millis());
  message.concat(" ");
  message.concat(incomingPT1);
  message.concat(" ");
  message.concat(incomingPT2);
  message.concat(" ");
  message.concat(incomingPT3);
  message.concat(" ");
  message.concat(incomingPT4);
  message.concat(" ");
  message.concat(incomingPT5);
  message.concat(" ");
  message.concat(incomingPT6);
  message.concat(" ");
  message.concat(incomingPT7);
  message.concat(" ");
  message.concat(incomingFM);
  message.concat(" ");
  message.concat(incomingS1);
  message.concat(" ");
  message.concat(incomingS2);
  message.concat(" ");
  message.concat(Commands.commandedState);
  message.concat(" ");
  message.concat(Commands.DAQstate);
  message.concat(" ");
  message.concat(queueSize);

  printLine(message);
}


void wifiDebug() {
  Commands.Debug=-64;
  dataSend();
  commandedState=17;

  Serial.println(incomingReadings.Debug);
  
}



void comDebug() {
Serial.print("COM Interface Debug Mode");
int Push_button_state = digitalRead(BUTTON1);
int Push_button_state2 = digitalRead(BUTTON2);
int Push_button_state3 = digitalRead(BUTTON3);
int Push_button_state4 = digitalRead(BUTTON4);

// if condition checks if push button is pressed
// if pressed LED will turn on otherwise remain off 

if ( Push_button_state == HIGH )
{ 
  Serial.println("button1");
}

if ( Push_button_state2 == HIGH )
{ 
  Serial.println("button2");
}

if ( Push_button_state3 == HIGH )
{ 
  Serial.println("button3");
}

if ( Push_button_state4 == HIGH )
{ 
  Serial.println("button4");
}


digitalWrite(INDICATOR1, HIGH); 
digitalWrite(INDICATOR2, HIGH); 
delay(200);

digitalWrite(INDICATOR1, LOW); 
digitalWrite(INDICATOR2, LOW); 


digitalWrite(INDICATOR3, HIGH); 
digitalWrite(INDICATOR4, HIGH); 
delay(200);

digitalWrite(INDICATOR3, LOW); 
digitalWrite(INDICATOR4, LOW); 


digitalWrite(INDICATOR5, HIGH); 
digitalWrite(INDICATOR6, HIGH); 
delay(200);

digitalWrite(INDICATOR5, LOW); 
digitalWrite(INDICATOR6, LOW); 
 




  
}
