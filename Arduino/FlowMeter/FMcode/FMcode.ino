void setup() {
  // put your setup code here, to run once:
  
//Initialize flow meter variables for how it computes the flow amount
short currentMillis = 0;
short goalTime = 50;
short currReading1;
short currReading2;
short loopTime=10;


#define FMPIN 4 //Flowmeter pin



//Flowmeter untreupt
  pinMode(FMPIN, INPUT);           //Sets the pin as an input


}

void loop() {
  // put your main code here, to run repeatedly:

}


//NOTES FROM OTHER CODE SCRAPS

   flowMeterReadings();



void flowMeterReadings() {
 if (digitalRead(FMPIN)){
   counter = counter + 1;
   while (digitalRead(FMPIN)){
   }
 } 
 frequency = counter*2;
 if(millis() - CurrentMillis > 20){
   counter = 0;
   currentMilllis = millis();
   fmval = (frequency*CalFac);
 }
}

Readings.fmval  = ReadingsQueue[queueLength].fmval;
