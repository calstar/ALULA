#include <Adafruit_I2CDevice.h>

int addr = 0x00;
Adafruit_I2CDevice i2c_dev = Adafruit_I2CDevice(addr);

void setup() {
  Serial.begin(115200);
  Serial.println("I2C address detection test");

  for(addr; addr < 0xFF; addr = addr + 0x01) {
    i2c_dev = Adafruit_I2CDevice(addr);
    if (!i2c_dev.begin()) {
    Serial.print("Did not find device at 0x");
    Serial.println(i2c_dev.address(), HEX);
    } else {
      Serial.print("Device found on address 0x");
      Serial.println(i2c_dev.address(), HEX); 
      while(1) {}; 
    } 
  } 
}

void loop() {
  
}
