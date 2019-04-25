#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

Servo myservo; 
Servo firstESC;

RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00101";

// 32 bit Frame Structure
uint32_t buf = 0;//[MSB] 12 bit (steer) | 12 bit (spd) | 8 bit (6 bit pattern + 2 bit modes)
#define MASK_UNIQUE_PATTERN   0xA4 //8 bit for flags (6 pattern + 2 bit switch state)  101001XX  
#define MASK_BUFFER_FLAG      0x000000FF  
#define MASK_BUFFER_SPD       0x000FFF00
#define MASK_BUFFER_STEER     0xFFF00000   
//macro access funcs
#define GET_SPD(x)    (((x)&MASK_BUFFER_SPD)>>8)
#define GET_STEER(x)  (((x)&MASK_BUFFER_STEER)>>20)
#define GET_FLAG(x)   ((x)&MASK_BUFFER_FLAG)

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.setDataRate( RF24_250KBPS );//low data rate => longer range and reliable
  radio.enableAckPayload();
  radio.setRetries(3,2);
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_HIGH);
  radio.startListening();
//  myservo.attach(9);
//  firstESC.attach(11);
}

void loop() {
  if (radio.available()) {
    radio.read(&buf, sizeof(buf));
    uint16_t temp1 = GET_SPD(buf);//spd
    uint16_t temp2 = GET_STEER(buf);//steer
    uint8_t temp3 =  GET_FLAG(buf);
    if(temp3&MASK_UNIQUE_PATTERN){
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
