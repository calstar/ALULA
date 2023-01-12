
#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Arduino.h>
#define FMPIN 19

// how mamy pulses per gallon
// plastic sensor
#define PULSE_RATE 1694.9

double flowRate;
volatile float count;
unsigned long openTimeControl = 0;
unsigned long openTime = 3000;
bool pulse = 0;

double pulseStart, pulseEnd, period, timeDiff;
float frequency;


double totalVL = 0;
double totalVGal = 0;
double q = 0;

unsigned long timeDelta = 0;
unsigned long firstTime;
unsigned long secondTime;
unsigned long duration;


void setup() {
  pinMode(FMPIN, INPUT);
  Serial.begin(115200);
}

void loop() {
  // 1 pulse ~= 2.25 mL
//  firstTime = millis();
  pulseStart = pulseIn(FMPIN, HIGH);
  pulseEnd = pulseIn(FMPIN, LOW);
//  secondTime = millis();
//  timeDiff = secondTime-firstTime;
  
  period = ((pulseStart+pulseEnd)/1000) ;

  if (period <= 0) {
    Serial.println("Period 0");
    flowRate = 0;
  } else {
    Serial.print("period Not 0, equals to ");
    Serial.println(period);
    // frequency in 1/seconds
    frequency = 1.00 / period;
    // convert frequency to 1/minute, 1/s * s / m
    frequency = frequency * 60 / 1;
    // flowrate (Vol/Time) = frequency (1/Time) / Pulserate (1/Vol)
    flowRate = frequency / PULSE_RATE ;
      // Serial.println(flowRate);
  }

//  Serial.println(flowRate);
Serial.println(period);
//Serial.print("System Time");
//Serial.println(timeDiff);

  delay(20);
}

void Flow()
{
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

}
