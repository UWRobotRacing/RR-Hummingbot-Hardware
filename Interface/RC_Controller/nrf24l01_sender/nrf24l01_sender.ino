#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";

#define DEBUG_MODE            1

// GPIO PIN Defines
#define PIN_LED_ESTOP         4
#define PIN_LED_AUTO          6
#define PIN_LED_TRANSMITTED   5
#define PIN_ESTOP             2
#define PIN_AUTO              3
#define SPECIAL_PATTERN_MASK  0xFAE0 //lower 2 bit has to be 00,          
void setup() {
  //- radio configs
  radio.begin();
//  radio.enableAckPayload();
//  radio.setDataRate( RF24_250KBPS );//low data rate => longer range and reliable
//  radio.setRetries(3,2);
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);//(MIN<LOW<HIGH<MAX)
  radio.stopListening();

  //- pin configs
  pinMode(PIN_ESTOP, INPUT);
  pinMode(PIN_AUTO, INPUT);
  pinMode(PIN_LED_AUTO, OUTPUT);
  pinMode(PIN_LED_TRANSMITTED, OUTPUT);
  pinMode(PIN_LED_ESTOP, OUTPUT);

  digitalWrite(PIN_LED_TRANSMITTED, LOW);
  digitalWrite(PIN_LED_AUTO, LOW);
  digitalWrite(PIN_LED_ESTOP,LOW);

  //boot sequence
  for(int i =0; i<20; i++){
    digitalWrite(PIN_LED_AUTO, ((i%3))?HIGH:LOW);
    digitalWrite(PIN_LED_ESTOP, ((i%3)-1)?HIGH:LOW);
    digitalWrite(PIN_LED_TRANSMITTED, ((i%3)-2)?HIGH:LOW);
    delay(200);
  }
  #if DEBUG_MODE
    Serial.begin(9600);
  #endif
}

unsigned long currentMillis;
unsigned long prevMillis;
#define TRANSMIT_INTERVAL         50 // ms
#define DEBOUNCING_SETTLE_TIMES   10 // times
uint8_t debouncing_cnter = 0;

uint16_t adc_values[2] = {0};// spd, steering
uint16_t buf[2] = {SPECIAL_PATTERN_MASK, 0}; // switch flags, analog values
 
void loop() {
  currentMillis = millis();
  //every 2ms
  if ((currentMillis - prevMillis)%2) {
      // ------ switch check -----
      bool btn_estop = digitalRead(PIN_ESTOP);
      bool btn_auto = digitalRead(PIN_AUTO);
      uint16_t flag = SPECIAL_PATTERN_MASK | btn_estop | btn_auto<<1;
      if(debouncing_cnter<DEBOUNCING_SETTLE_TIMES)
      {
        //reset cnter if flag changes before stablized
        if(buf[0] != flag){
          debouncing_cnter = 0;
        }
        debouncing_cnter++;
      }else if(debouncing_cnter==DEBOUNCING_SETTLE_TIMES){
        buf[0] = flag;
      }else{
        if(buf[0] != flag){
          debouncing_cnter = 0;
        }
      }
    #if DEBUG_MODE
//      Serial.println(buf[0]);
    #endif
  }

  // every 4ms
  if ((currentMillis - prevMillis)%4) {
      // ------ ADC Sampling & low pass-----
       adc_values[0] = adc_values[0]/10 + analogRead(A0)*9/10;
       adc_values[1] = adc_values[1]/10 + analogRead(A1)*9/10;
  }

  //every transmit interval
  if (currentMillis - prevMillis >= TRANSMIT_INTERVAL) {
      prevMillis = millis();
      //- Prepare data >> lower resolution to 8 bit & pad into 16 bit packet
      buf[1] = (map(adc_values[0],0,1024,0,255)) | (map(adc_values[1],0,1024,0,255)) <<8; 
      //- Send
      if(send_packet())
      {
        // ---- [Sent Successfully] -----
        //- Trigger led >> User Feedback
        digitalWrite(PIN_LED_TRANSMITTED, HIGH);
        digitalWrite(PIN_LED_AUTO, (buf[0]&0x2)?HIGH:LOW);
        digitalWrite(PIN_LED_ESTOP, (buf[0]&0x1)?HIGH:LOW);
        #if DEBUG_MODE
          Serial.print(" ACK ");
        #endif
      }else{
        // ---- [Sent Failed] -----
        digitalWrite(PIN_LED_TRANSMITTED, LOW);
        digitalWrite(PIN_LED_AUTO, (buf[0]&0x2)?HIGH:LOW);
        digitalWrite(PIN_LED_ESTOP, (buf[0]&0x1)?HIGH:LOW);
        #if DEBUG_MODE
          Serial.print(" NCK ");
        #endif
      }
     #if DEBUG_MODE
      Serial.print(buf[0]);
      Serial.print("  ");
      Serial.print(buf[1]&0xFF);
      Serial.print("  ");
      Serial.println(buf[1]>>8);
    #endif
  }
}

bool send_packet(){
  return (radio.write(&buf, sizeof(buf)));
  {
    /* TO Habdle ack replied packets if needed
    if ( radio.isAckPayloadAvailable() ) {
        radio.read(&ackData, sizeof(ackData));
        newData = true;
    }
    else {
        Serial.println("  Acknowledge but no data ");
    }
    */
  }
}

