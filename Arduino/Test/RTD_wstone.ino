#include <Arduino.h>
#define V_READ_PIN 36
#define R_MIN 18 //resistance (Ohms) where bridge voltage = 0
#define R_REF 2200 //resistance of other two resistors in bridge
#define V_IN 3.3 //input voltage in mV
#define ALPHA 0.00385
#define R_0 100


float wstone_factor; //made this a thing to make calculations easier
float v_out; //in mV
float bridge_ref; //the voltage of the other side of the bridge (v_out must be this val to balance the bridge)
float bridge_voltage; //voltage diff across both sides of bridge
float read_value; //output of analog read
float r_rtd;
float temp;

void setup() {
  Serial.begin(115200);
  bridge_ref = (R_MIN / (R_MIN + R_REF)) * V_IN; 
}

void loop() {
  read_value = analogRead(V_READ_PIN);
  v_out = 3.3 - ((read_value * 3.3) / 5100); //values for adafruit feather esp32

  bridge_voltage = v_out - bridge_ref; 

  r_rtd = ( (R_REF*(R_MIN + R_REF)) / (R_REF - bridge_voltage*(R_MIN + R_REF)) ) - R_REF;

  
  Serial.print("analog read: ");
  Serial.print(read_value);
  Serial.print("  voltage: ");
  Serial.print(v_out);
  Serial.print("  bridge voltage: ");
  Serial.print(bridge_voltage);
  Serial.print("  resistance: ");
  Serial.print(r_rtd);
  Serial.print("  temp: ");
  Serial.println(temp);
}
