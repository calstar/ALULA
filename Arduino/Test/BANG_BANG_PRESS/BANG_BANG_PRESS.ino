#define SOLENOID 27
#define PTPIN 22
#define CLK 23
#include "HX711.h"
//int counter;
HX711 scale1;
//int counter2;
float threshold1 = 350;
float threshold2 = 500;
float period = 0.5;
float CalOffset = 10.663;
float CalSlope = 0.0001181;
float reading;
bool filled;
bool printed1;
bool printed2;
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
  
  Serial.println("Start fill");
  filled = false;
  printed1 = false;
  printed2 = false;


}
//code for constant bang-bang
void loop() {
   digitalWrite(SOLENOID, HIGH);
   reading = CalOffset + CalSlope*scale1.read();

   Serial.println(reading);
   delay(period*500);

  while(reading <= threshold1 && !filled){

  // if(millis() - prevtime > 1000){
  //   prevtime = millis();
  //   counter = counter + 1;
  //   Serial.println(counter);
  // }
  
  digitalWrite(SOLENOID, LOW);
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
  //prevtime = millis();
  
while(reading < threshold2 && threshold1 < reading && !filled){
  if (!printed2) {
    Serial.println("entering bang-bang");
    printed2 = true;
  }
  digitalWrite(SOLENOID, HIGH);
  reading = CalOffset + CalSlope*scale1.read();
  Serial.println(reading);
  delay(period*500);
  digitalWrite(SOLENOID, LOW);
  delay((1-period)*500);
  reading = CalOffset + CalSlope*scale1.read();
  Serial.println(reading);
  }
  

  if(reading >= threshold2 && !filled){
    Serial.println("pressurization completed");
    filled = true;
    digitalWrite(SOLENOID, HIGH);  //double check during hotfire
  }
  }
