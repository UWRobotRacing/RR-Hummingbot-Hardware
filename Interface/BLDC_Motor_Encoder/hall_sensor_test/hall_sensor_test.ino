#include <Servo.h>
int tick =0;
Servo motor;
void setup() {
  // begin the serial monitor @ 9600 baud
  Serial.begin(9600);
  motor.attach(9);
}
 
void loop() {
  // read the value from the sensor:
  uint8_t sensorValue = analogRead(A0);
  uint8_t sensorValue2 = analogRead(A1);
  uint8_t sensorValue3 = analogRead(A2);
  Serial.print(sensorValue);
  Serial.print(" ");
  Serial.print(sensorValue2);
  Serial.print(" ");
  Serial.print(sensorValue3);
  Serial.println(" ");
//  if(tick > 1000)
//    motor.writeMicroseconds(1560);
//  else
//    motor.writeMicroseconds(0);
//  if(tick > 2000)
//    tick = 0;
  tick++;
}
