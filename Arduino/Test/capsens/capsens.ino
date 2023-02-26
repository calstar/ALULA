
#include <Wire.h>
#define SLAVE 0x48 // adress of device during write cycle
int buffer[] = {0,0,0};
long value = 0;
float capacitance = 0;



void setup()
{
 
  
  Serial.begin(115200);
  Serial.println("bruh");
  Wire.begin();
  Wire.beginTransmission(SLAVE);
  Wire.write(0x07);  //cap-setup register
  Wire.write(0xA0);
  Wire.endTransmission();
  Wire.end();
  
  delay(4);
  Wire.begin();
  Wire.beginTransmission(SLAVE);
  Wire.write(0x0A);  //configuration register
  Wire.write(0x01);

  Wire.endTransmission();
  Wire.end();
  Wire.begin();
  Wire.beginTransmission(SLAVE);
  Wire.write(0x09);  //excitation setup register 
  Wire.write(0x06);
  Wire.endTransmission();
  Wire.end();
  Wire.begin();
  Wire.beginTransmission(SLAVE);
  Wire.write(0x0B);     //CAPDAC A
  Wire.write(0x80);
  Wire.endTransmission();
  Wire.end();
  Wire.begin();
  Wire.beginTransmission(SLAVE);
  Wire.write(0x0C);   //CAPDAC B
  Wire.write(0x80);
  Wire.endTransmission();
  Wire.end();
  
  

}
void loop(){



      Wire.begin();
  
      Wire.beginTransmission(SLAVE);
      Wire.write(0x00);
      Wire.endTransmission();
      Wire.requestFrom(SLAVE,4);
      int count = 0;
      while(Wire.available())
     {
      int c = Wire.read();
      //Serial.print(c, HEX);
      //Serial.print(",");
      
      if(count > 0){
      buffer[count-1] = c;
      if(count == 3){
        count = 0;
        value = long((long)buffer[0] << 16) + ((long)buffer[1] << 8) + (long)buffer[2] - 0x800000;
        capacitance = (float)value/ 1024000.0;
        Serial.println(capacitance);
      } 
      }
      count++;
     }
      Serial.println();
      delay(500);
      Wire.end();
}

