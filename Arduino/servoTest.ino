#include <ESP32Servo.h>

#define servoPin 13

#define SERVO_MIN_USEC (800)
#define SERVO_MAX_USEC (2100)
#define servoClosedPosition 100
#define servoOpenPosition 10


Servo servo;

void setup() {
    Serial.begin(115200);

    servo.attach(servoPin,SERVO_MIN_USEC,SERVO_MAX_USEC);
}

void loop() {
    servo.write(servoClosedPosition);
    sleep(1000);
    servo.write(servoOpenPosition);
    sleep(1000);
}

