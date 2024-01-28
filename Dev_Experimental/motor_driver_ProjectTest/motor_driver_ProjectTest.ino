#include <ESP32Encoder.h>
#define BIN_1 25
#define BIN_2 26
#define PWM 39
#define LED_PIN 13

ESP32Encoder encoder;

//Setup interrupt variables ----------------------------
volatile float count = 0; // encoder count
volatile bool interruptCounter = false;    // check timer interrupt 1
volatile bool deltaT = false;     // check timer interrupt 2
int totalInterrupts = 0;   // counts the number of triggering of the alarm
hw_timer_t * timer0 = NULL;
hw_timer_t * timer1 = NULL;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux1 = portMUX_INITIALIZER_UNLOCKED;

// setting PWM properties ----------------------------
const int freq = 5000;
const int ledChannel_1 = 1;
const int ledChannel_2 = 2;
const int ledChannel_PWM = 3;
const int resolution = 8;
int MAX_PWM_VOLTAGE = 255;
int motor_PWM;
int i = 0;
int pwm_sign;
float timed = 0;
float rate = 500; //ADJUST RATE VALUE LOWER FOR HIGHER 
float cpr = 211.2; //encoder multiplied by gear ratio
int memerr = 0;

// encoder properties ------------------------------
float v = 0;
float theta = 0;
float Kp = 900;
float command = 0;
float error = 0;
float Ki = 4;
float pwm = 0;
float interr = 0;

//Initialization ------------------------------------
void IRAM_ATTR onTime0() {
  portENTER_CRITICAL_ISR(&timerMux0);
  interruptCounter = true; // the function to be called when timer interrupt is triggered
  portEXIT_CRITICAL_ISR(&timerMux0);
}

void IRAM_ATTR onTime1() {
  portENTER_CRITICAL_ISR(&timerMux1);
  count = encoder.getCount( );
  encoder.clearCount ( );
  deltaT = true; // the function to be called when timer interrupt is triggered
  portEXIT_CRITICAL_ISR(&timerMux1);
}


void setup() {
  pinMode(LED_PIN, OUTPUT); // configures the specified pin to behave either as an input or an output
  digitalWrite(LED_PIN, LOW); // sets the initial state of LED as turned-off
  
  Serial.begin(115200);
  ESP32Encoder::useInternalWeakPullResistors = UP; // Enable the weak pull up resistors
  encoder.attachFullQuad(33, 27); // Attach pins for use as encoder pins
  encoder.setCount(0);  // set starting count value after attaching

  // configure LED PWM functionalitites
  ledcSetup(ledChannel_1, freq, resolution);
  ledcSetup(ledChannel_2, freq, resolution);
  ledcSetup(ledChannel_PWM, freq, resolution);

  // attach the channel to the GPIO to be controlled 
  ledcAttachPin(BIN_1, ledChannel_1);
  ledcAttachPin(BIN_2, ledChannel_2);
  ledcAttachPin(PWM, ledChannel_PWM);

  // initialize timer
  timer0 = timerBegin(0, 80, true);  // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer0, &onTime0, true); // edge (not level) triggered 
  timerAlarmWrite(timer0, 2000000, true); // 2000000 * 1 us = 2 s, autoreload true

  timer1 = timerBegin(1, 80, true);  // timer 1, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer1, &onTime1, true); // edge (not level) triggered 
  timerAlarmWrite(timer1, 10000, true); // 10000 * 1 us = 10 ms, deltaT period

  // at least enable the timer alarms
  timerAlarmEnable(timer0); // enable
  timerAlarmEnable(timer1); // enable
  ledcWrite(ledChannel_PWM, LOW);
  pwm_sign = MAX_PWM_VOLTAGE;
}

void loop() {
  getcommand();
  error = command - theta;
  if (error*memerr < 0) {interr = 0;}
  memerr = error;
  pwm = error*Kp + interr*Ki;
  if (pwm > 0) {
  ledcWrite(ledChannel_2, LOW); // clockwise
  ledcWrite(ledChannel_1, pwm);}
  else {pwm = -pwm; ledcWrite(ledChannel_2, pwm); // clockwise
  ledcWrite(ledChannel_1, LOW);}

//     float sineValue = sin(millis()/rate);
//    pwm_sign = round(MAX_PWM_VOLTAGE * sineValue);
//     // pwm_sign = 200;
//     if (i == 0) { 
//         digitalWrite(LED_PIN, HIGH);
//         ledcWrite(ledChannel_2, LOW); // clockwise
//         motor_PWM = pwm_sign;
//         ledcWrite(ledChannel_1, motor_PWM); // power control while cw
//         if (pwm_sign < 0) {
//             i = 1;
// //            timed = millis()/rate;
//         }
//     } else { 
//         digitalWrite(LED_PIN, LOW);
//         ledcWrite(ledChannel_1, LOW); // ccw
//         motor_PWM = -pwm_sign;
//         ledcWrite(ledChannel_2, motor_PWM); // power control while ccw
//         if (pwm_sign > 0) {
//             i = 0;
//             timed = millis() / rate;
//         }

  if (deltaT) {
    deltaT = false;// RESET deltaT FLAG HERE (hint: see the 3 lines after if(interruptCounter))
    theta = count/(cpr) + theta; ///
    interr = interr + error;
    Serial.print("PWM, command, pos, err ");

//    Serial.print(sineValue);
//    Serial.print(" ");
    Serial.print(pwm);
    Serial.print(" ");
    Serial.print(command);
    Serial.print(" ");
    Serial.println(theta);
    Serial.print(" ");
    Serial.println(error);
  }
}

void getcommand() {
  // Actually, COMState will be updated in the OnDataRec function.
  command = 3;
  // if (Serial.available() > 0) {
  //   // Serial.read reads a single character as ASCII. Number 1 is 49 in ASCII.
  //   // Serial sends character and new line character "\n", which is 10 in ASCII.
  //   int SERIALState = Serial.read() - 49;
  //   command = SERIALState;
  // }
}
