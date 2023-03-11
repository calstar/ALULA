#define PIN1 22
int count;
float times;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(PIN1,OUTPUT);
  count = 0;
  times = millis();
}

void loop() {
  if(millis() - times > 1000){
    Serial.println(count);
    times = millis();
    count = count + 1;}
    if(count % 2 == 0){
      digitalWrite(PIN1,HIGH);
    } else { digitalWrite(PIN1,LOW);}
   
  

}
