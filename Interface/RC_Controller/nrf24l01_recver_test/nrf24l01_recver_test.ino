#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

Servo myservo; 
Servo firstESC;

RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";
uint16_t buf[2]= {0};

#define SPECIAL_PATTERN_MASK  0xFAE0 //lower 2 bit has to be 00,   

void setup() {
  Serial.begin(9600);
  radio.begin();
//  radio.setDataRate( RF24_250KBPS );//low data rate => longer range and reliable
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();
//  myservo.attach(9);
//  firstESC.attach(11);
}

void loop() {
  if (radio.available()) {
    radio.read(&buf, sizeof(buf));
    uint8_t temp1 = ((buf[1]) & 0xFF);
    uint8_t temp2 = buf[1]>>8;
    uint16_t temp3 = (buf[0]);
    if(temp3&SPECIAL_PATTERN_MASK){
      Serial.print(temp1,DEC);  
      Serial.print(",");  
      Serial.print(temp2,DEC);
      Serial.print(",");   
      Serial.print(temp3,BIN);
      Serial.println("");
    }else{
      Serial.print(temp3,BIN);
      Serial.println(" Invalid msg");
    }
  }
//  uint8_t steer = (buf>>2)&0xFF;
//  uint8_t power = buf&1;
//
//  uint8_t steer_angle = uint16_t(steer - 256/2)*60/256+85;
//  myservo.write(steer_angle);
//  if(power)
//  {
//    firstESC.writeMicroseconds(1580);
//  }else
//  {
//    firstESC.writeMicroseconds(0);
//  }
////  Serial.println(steer);
////  Serial.println(power);
}
