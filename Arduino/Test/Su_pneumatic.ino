#define SOLENOID 27
#define PTPIN 15
#define CLK 2
#include "HX711.h"
//int counter;
HX711 scale1;
//int counter2;
float threshold1 = 170;
float threshold2 = 200;
float period = 0.5;
float CalOffset = 5.2025;
float CalSlope = 0.0001104;
float reading;
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
  

}
//code for constant bang-bang
void loop() {
   digitalWrite(SOLENOID, HIGH);
   reading = CalOffset + CalSlope*scale1.read();
   delay(1000);
while(reading <= threshold1){

  // if(millis() - prevtime > 1000){
  //   prevtime = millis();
  //   counter = counter + 1;
  //   Serial.println(counter);
  // }
  
  digitalWrite(SOLENOID, LOW);
reading = CalOffset + CalSlope*scale1.read();  
  delay(2000);
  reading = CalOffset + CalSlope*scale1.read();
  Serial.println(reading);
}  
  Serial.println("exited first phase");
  delay(2000);
  reading = CalOffset + CalSlope*scale1.read();

  //prevtime = millis();
  
while(reading < threshold2 && threshold1 < reading){
  Serial.println("entering bang-bang");
  digitalWrite(SOLENOID, HIGH);
    reading = CalOffset + CalSlope*scale1.read();
    delay(period*500);
  digitalWrite(SOLENOID, LOW);
  reading = CalOffset + CalSlope*scale1.read();
  delay((1-period)*500);
  reading = CalOffset + CalSlope*scale1.read();
  Serial.println(reading);
  }
  if(reading > threshold2){
    Serial.println("pressurization completed");
    digitalWrite(SOLENOID, HIGH);  //double check during hotfire
  }
  }
