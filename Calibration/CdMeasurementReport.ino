#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Arduino.h>
#include "HX711.h"
#define FMPIN 18


// define PT pins (3)
#define PTDOUT1 35
#define CLKPT1 25
#define PTDOUT2 19
#define CLKPT2 2
#define PTDOUT3 36
#define CLKPT3 5
//#define PTDOUT1 21
//#define CLKPT1 17
//#define PTDOUT2 15
//#define CLKPT2 4
//#define PTDOUT3 27
//#define CLKPT3 5
#define psi2Pa 6894.75729

// how mamy pulses per gallon
// plastic sensor
#define PULSE_RATE 1694.9

double flowFreq;
volatile float count;
bool pulse = 0;


double pulseStart, pulseEnd, period;
int delayTimeMs = 50;
float frequency;


double totalVL = 0;
double totalVGal = 0;
double q = 0;

unsigned long timeDelta = 0;
unsigned long firstTime;
unsigned long secondTime;
unsigned long duration;


// Set up PTs
//Initialize the PT and LC sensor objects which use the HX711 breakout board
HX711 scale1;
HX711 scale2;
HX711 scale3;
int pt1val = 1;
int pt2val = 1;
int pt3val = 1;
int OperateTime = 0;

void setup() {
  pinMode(FMPIN, INPUT);
  Serial.begin(115200);
  //set gains for pt pins
  scale1.begin(PTDOUT1, CLKPT1); scale1.set_gain(64);
  scale2.begin(PTDOUT2, CLKPT2); scale2.set_gain(64);
  scale3.begin(PTDOUT3, CLKPT3); scale3.set_gain(64);
}

void loop() {
  OperateTime = millis();
  scaleReading();
  // 1 pulse ~= 2.25 mL
  pulseStart = pulseIn(FMPIN, HIGH);
  pulseEnd = pulseIn(FMPIN, LOW);
  flowFreq = FlowRateCalc(pulseStart,pulseEnd);
//
  Serial.print(OperateTime);
  Serial.print(" ");
  Serial.print(pt1val*psi2Pa);
  Serial.print(" ");
  Serial.print(14.7*psi2Pa);
  Serial.print(" ");
  Serial.print(pt3val);
  Serial.print(" ");
  Serial.print(flowFreq);
  Serial.println(" ");


  delay(delayTimeMs);
}

double FlowRateCalc(double pulseStart, double pulseEnd)
{
  period = ((pulseStart+pulseEnd)/1000000) ;
  if (period <= 0) {
     OperateTime = millis();
//    Serial.print("period 0, time is  ");
//    Serial.println(OperateTime);
    return 0;
  } else {
    OperateTime = millis();
//    Serial.print("period Not 0, time is  ");
//    Serial.println(OperateTime);
    // frequency in 1/seconds
    frequency = 1.00 / period;
    // convert frequency to 1/minute, 1/s * s / m
//    frequency = frequency * 60 / 1;
    // flowrate (Vol/Time) = frequency (1/Time) / Pulserate (1/Vol)
    return frequency;
  }
}

void scaleReading() {
  pt1val = scale1.read()*2.9256e-05+20.5070;
  pt2val = scale2.read() ;
  pt3val = scale3.read();
}


 //  secondTime = millis();
 //  // how many counts per second
 // timeDelta = secondTime - firstTime;
 // count = 1000 / timeDelta;
 // firstTime = secondTime;
 // q = 2 + exp(count-1847);
 // // gal
 // totalVGal = totalVGal + q*timeDelta/1000;
 // totalVL =  totalVGal*3.78541;
 // Serial.println(count, 4);
