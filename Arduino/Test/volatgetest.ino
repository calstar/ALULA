//#define VOLTAGEIN 39
//
//float value;
//float val;
//
//void setup() {
//  Serial.begin(9600);
//
//}
//
//void loop() {
//  value = analogRead(VOLTAGEIN);
//  val = value * (3.3/4095);
//  Serial.println(val);
//  Serial.println("#######");
//  Serial.println(val * (37/15));
//  delay(500);
//
//}

const int Analog_channel_pin= 39;
float ADC_VALUE = 0;
float voltage_value = 0; 
float original_factor = 37.00/15.00;
float bottom_resistor = 15000;
void setup() 
{
Serial.begin(115200);
}
void loop() 
{
ADC_VALUE = analogRead(Analog_channel_pin);
Serial.print("ADC VALUE = ");
Serial.println(ADC_VALUE);
delay(1000);
voltage_value = (ADC_VALUE * 3.3 ) / (4095);
Serial.print("Raw Voltage:");
Serial.println(voltage_value);
Serial.print("Input Voltage = ");
Serial.println(voltage_value * original_factor);
Serial.print("original factor");
Serial.println(original_factor);
Serial.print("volts");
Serial.print("Current:");
Serial.println((volatage_value * original_factor) / bottom_resistor);
delay(1000);
}
