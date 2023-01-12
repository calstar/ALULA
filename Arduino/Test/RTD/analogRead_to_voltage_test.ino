// Just some code to check if analogRead works aaaaaa cries

#include <Arduino.h>
#define V_READ_PIN A0
#define R_MIN 18 //resistance (Ohms) where bridge voltage = 0

float anlg_read;
float v_read; 


void setup() {
  Serial.begin(115200);
}

void loop() {
  anlg_read = analogRead(V_READ_PIN);
  v_read = (anlg_read * 3.3) / 5100; //adafruit esp32 feather CHANGE DEPENDING ON BOARD USED
  Serial.print(anlg_read);
  Serial.print("     ");
  Serial.println(v_read);

}
