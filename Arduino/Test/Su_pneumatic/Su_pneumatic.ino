#define SOLENOID 27
<<<<<<< HEAD
#define PTPIN 15
=======
#define PTPIN 32
>>>>>>> 1020b562c84ff3de4bbe5fba2d8baf22c8e4d01d
#define CLK 2
#include "HX711.h"
//int counter;
HX711 scale1;
//int counter2;
<<<<<<< HEAD
float threshold1 = 120;
float threshold2 = 150;
float period = 0.5;
float CalOffset = -1469.42;
float CalSlope = 0.9927;
float reading;
bool filled;
bool printed;
=======
float threshold1 = 100;
float threshold2 = 125;
float period = 0.5;
float CalOffset = 8.115;
float CalSlope = 0.0001389;
float reading;
bool filled;
bool printed1;
bool printed2;
>>>>>>> 1020b562c84ff3de4bbe5fba2d8baf22c8e4d01d
//float logperiod;
//float prevtime;
//float controlconstant = 0.3;
//float ptrdns[2];
//int pointer;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  digitalWrite(SOLENOID,HIGH);
  pinMode(SOLENOID, OUTPUT);
  scale1.begin(PTPIN, CLK);
  scale1.set_gain(64);
  reading = 0;
<<<<<<< HEAD
  Serial.print("Start fill");
  filled = false;
  printed = false;
=======
  Serial.println("Start fill");
  filled = false;
  printed1 = false;
  printed2 = false;
>>>>>>> 1020b562c84ff3de4bbe5fba2d8baf22c8e4d01d
  

}
//code for constant bang-bang
void loop() {
   digitalWrite(SOLENOID, HIGH);
   reading = CalOffset + CalSlope*scale1.read();
<<<<<<< HEAD
   delay(1000);
=======
   Serial.println(reading);
   delay(period*500);
>>>>>>> 1020b562c84ff3de4bbe5fba2d8baf22c8e4d01d
  while(reading <= threshold1 && !filled){

  // if(millis() - prevtime > 1000){
  //   prevtime = millis();
  //   counter = counter + 1;
  //   Serial.println(counter);
  // }
  
  digitalWrite(SOLENOID, LOW);
<<<<<<< HEAD
  filled = false;
  Serial.println(reading);
  reading = CalOffset + CalSlope*scale1.read();
  delay(1000);

}  
  Serial.println("exited first phase");
  delay(1000);
  reading = CalOffset + CalSlope*scale1.read();
=======
  reading = CalOffset + CalSlope*scale1.read();
  Serial.println(reading);
  delay(period*500);

}  
  if (!printed1) {
    Serial.println("exited first phase");
    printed1 = true;
  }
  delay(period*500);
  reading = CalOffset + CalSlope*scale1.read();
  Serial.println(reading);
>>>>>>> 1020b562c84ff3de4bbe5fba2d8baf22c8e4d01d

  //prevtime = millis();
  
while(reading < threshold2 && threshold1 < reading && !filled){
<<<<<<< HEAD
  if (!printed) {
    Serial.println("entering bang-bang");
    printed = true;
  }
  digitalWrite(SOLENOID, HIGH);
  reading = CalOffset + CalSlope*scale1.read();
  delay(period*500);
  digitalWrite(SOLENOID, LOW);
  reading = CalOffset + CalSlope*scale1.read();
=======
  if (!printed2) {
    Serial.println("entering bang-bang");
    printed2 = true;
  }
  digitalWrite(SOLENOID, HIGH);
  reading = CalOffset + CalSlope*scale1.read();
  Serial.println(reading);
  delay(period*500);
  digitalWrite(SOLENOID, LOW);
>>>>>>> 1020b562c84ff3de4bbe5fba2d8baf22c8e4d01d
  delay((1-period)*500);
  reading = CalOffset + CalSlope*scale1.read();
  Serial.println(reading);
  }
  
<<<<<<< HEAD
  if(reading >= threshold2){
=======
  if(reading >= threshold2 && !filled){
>>>>>>> 1020b562c84ff3de4bbe5fba2d8baf22c8e4d01d
    Serial.println("pressurization completed");
    filled = true;
    digitalWrite(SOLENOID, HIGH);  //double check during hotfire
  }
  }
