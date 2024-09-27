/*
  Application:
  - Interface water flow sensor with ESP32 board.
  
  Board:
  - ESP32 Dev Module
    https://my.cytron.io/p-node32-lite-wifi-and-bluetooth-development-kit

  Sensor:
  - G 1/2 Water Flow Sensor
    https://my.cytron.io/p-g-1-2-water-flow-sensor
 */

#define LED_BUILTIN 2
#define SENSOR  27

#define ONBOARD_LED  2
#define PTDOUT1 26 // PT1
#define PTDOUT2 25   // PT2

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Arduino.h>

float PT2_Slope = 0.181300812;
float PT2_Offset = -87.45003982;
float PT1_Slope = 0.091161617;
float PT1_Offset = -48.50108657;

float pt1 = -1;
float pt2 = -1;

long currentMillis = 0;
long previousMillis = 0;
int interval = 250;
boolean ledState = LOW;
float calibrationFactor = 0.04984;
volatile byte pulseCount;
byte pulse1Sec = 0;
float flowRate;
unsigned int flowMilliLitres;
unsigned long totalMilliLitres;

void IRAM_ATTR pulseCounter()
{
  pulseCount++;
}

void setup()
{
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SENSOR, INPUT_PULLUP);

  pulseCount = 0;
  flowRate = 0.0;
  flowMilliLitres = 0;
  totalMilliLitres = 0;
  previousMillis = 0;

  attachInterrupt(digitalPinToInterrupt(SENSOR), pulseCounter, FALLING);
  pinMode(PTDOUT1, INPUT);
  pinMode(PTDOUT2, INPUT);

}

void loop()
{
  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    
    pulse1Sec = pulseCount;
    pulseCount = 0;

    pt1 = analogRead(PTDOUT1)*PT1_Slope + PT1_Offset;
    pt2 = analogRead(PTDOUT2)*PT2_Slope + PT2_Offset;


    // Because this loop may not complete in exactly 1 second intervals we calculate
    // the number of milliseconds that have passed since the last execution and use
    // that to scale the output. We also apply the calibrationFactor to scale the output
    // based on the number of pulses per second per units of measure (litres/minute in
    // this case) coming from the sensor.
    flowRate = ((1000.0 / (millis() - previousMillis)) * pulse1Sec) * calibrationFactor;
    previousMillis = millis();

    // Divide the flow rate in litres/minute by 60 to determine how many litres have
    // passed through the sensor in this 1 second interval, then multiply by 1000 to
    // convert to millilitres.
    flowMilliLitres = (flowRate / (60*1000/interval)) * 1000;

    // Add the millilitres passed in this second to the cumulative total
    totalMilliLitres += flowMilliLitres;
    
    // Print time in milliseconds
    Serial.print(millis());
    Serial.print("\t");

    // Print the flow rate for this second in litres / minute
    //Serial.print("Flow rate:");
    Serial.print(int(flowRate));  // Print the integer part of the variable
    //Serial.print("L/min");
    Serial.print("\t");       // Print tab space

    // Print the cumulative total of litres flowed since starting
    //Serial.print("Output Liquid Quantity:");
    Serial.print(totalMilliLitres);
    //Serial.print("mL/");
    //Serial.print(totalMilliLitres/1000);
    //Serial.print("L");
    Serial.print("\t");

    // Print PT readings
    //Serial.print("PT1:");
    Serial.print(pt1);
    Serial.print("\t");

    //Serial.print("PT2:");
    Serial.println(pt2);
  }
}