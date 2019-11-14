#include <Servo.h>

int value = 0; // set values you need to zero

Servo motorESC; //Create as much as Servoobject you want. You can controll 2 or more Servos at the same time

void setup() {

  motorESC.attach(9);    // attached to pin 9 I just do this with 1 Servo
  Serial.begin(9600);    // start serial at 9600 baud

}

void loop() {

//First connect your ESC WITHOUT Arming. Then Open Serial and follo Instructions
 
  motorESC.writeMicroseconds(value);
 
  if(Serial.available())
  {
    value = Serial.parseInt();    // Parse an Integer from Serial
    Serial.println(value);
    if(value == 1540){
      Serial.println("Reverse Begin");
      motorESC.writeMicroseconds(1580); // neutral
      delay(1000);
      motorESC.writeMicroseconds(700); // reverse
      delay(100);
      motorESC.writeMicroseconds(1558); // neutral
      delay(100);
      motorESC.writeMicroseconds(700); // reverse
      delay(5000);
      Serial.println("Reset");
      value = 1550;
    }
  }
  
}
