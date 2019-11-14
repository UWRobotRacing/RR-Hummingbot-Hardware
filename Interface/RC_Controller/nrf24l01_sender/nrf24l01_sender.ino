#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00101";

#define DEBUG_MODE                  1
#define DEBUG_PRINT_MSG             1
#define DEBUG_PLOT_CONTRL_RESPONSE  0
// only one debug mode can be selected (DEBUG_PRINT_MSG higher priority)
#if (DEBUG_PLOT_CONTRL_RESPONSE && DEBUG_PRINT_MSG)
  #undef DEBUG_PLOT_CONTRL_RESPONSE
  #define DEBUG_PLOT_CONTRL_RESPONSE 0 
#endif
// GPIO PIN Defines
#define PIN_LED_ESTOP         4
#define PIN_LED_AUTO          6
#define PIN_LED_TRANSMITTED   5
#define PIN_ESTOP             2
#define PIN_AUTO              3
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

//---- LOW PASS FILTER FOR POTENTIOMETERS (JOYSTICKS) ----//
#define WINDOW_SIZE_BASE_2  4
#define MAX_SPD_VAL         250
uint16_t mCtrl_values[2] = {0};// spd, steering

//---- Control values ------//
unsigned long currentMillis;
unsigned long prevMillis;
#define TRANSMIT_INTERVAL         50 // ms
#define DEBOUNCING_SETTLE_TIMES   10 // times
uint8_t debouncing_cnter = 0;

//---- SETUP ------//
void setup() {
  //- radio configs
  radio.begin();
  radio.enableAckPayload();
  radio.setDataRate( RF24_250KBPS );//low data rate => longer range and reliable
  radio.setRetries(3,2);
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_HIGH);//(MIN<LOW<HIGH<MAX)
  radio.stopListening();

  //- pin configs
  pinMode(PIN_ESTOP, INPUT);
  pinMode(PIN_AUTO, INPUT);
  pinMode(PIN_LED_AUTO, OUTPUT);
  pinMode(PIN_LED_TRANSMITTED, OUTPUT);
  pinMode(PIN_LED_ESTOP, OUTPUT);

  //boot sequence
  for(int i =0; i<10; i++){
    digitalWrite(PIN_LED_AUTO, ((i%3))?HIGH:LOW);
    digitalWrite(PIN_LED_ESTOP, ((i%3)-1)?HIGH:LOW);
    digitalWrite(PIN_LED_TRANSMITTED, ((i%3)-2)?HIGH:LOW);
    delay(100);
  }

  //turn off leds
  digitalWrite(PIN_LED_TRANSMITTED, LOW);
  digitalWrite(PIN_LED_AUTO, LOW);
  digitalWrite(PIN_LED_ESTOP,LOW);

  #if DEBUG_MODE
    Serial.begin(9600);
  #endif
}

void loop() {
  currentMillis = millis();
  //every 2ms  - Switch Input 
  if ((currentMillis - prevMillis)%2) {
      // ------ switch check -----
      bool btn_estop = digitalRead(PIN_ESTOP);
      bool btn_auto = digitalRead(PIN_AUTO);
      uint8_t flag = MASK_UNIQUE_PATTERN | btn_estop | btn_auto<<1;
      // - if the buffer has an err. pattern, reset it
      if(!(buf & MASK_UNIQUE_PATTERN))
      {
        buf = flag;
      }
      if(debouncing_cnter<DEBOUNCING_SETTLE_TIMES)
      {
        //reset cnter if flag changes before stablized
        Serial.println(flag);
        if((buf&MASK_BUFFER_FLAG) != flag){
          debouncing_cnter = 0;
        }
        debouncing_cnter++;
      }else if(debouncing_cnter==DEBOUNCING_SETTLE_TIMES){
        buf &= (~MASK_BUFFER_FLAG); //clear flag
        buf |= flag; //apply new flag
      }
  }

  // every 4ms  - JOYSTICK Input 
  if ((currentMillis - prevMillis)%4) {
      // ------ ADC Sampling & low pass-----
      uint16_t steer_raw_input = analogRead(A3)+200;
      uint16_t spd_raw_input = analogRead(A6)+200;
      // Low pass filter
      // CONTROLLER - STEER
      steer_raw_input &= 0xFFF;
      mCtrl_values[1] *= 9;
      mCtrl_values[1] += steer_raw_input;
      mCtrl_values[1] /= 10;
      // CONTROLLER - SPEED 
      spd_raw_input &= 0xFFF;
      mCtrl_values[0] *= 19;
      mCtrl_values[0] += spd_raw_input;
      mCtrl_values[0] /= 20;

#if DEBUG_PLOT_CONTRL_RESPONSE 
      Serial.print(steer_raw_input);
      Serial.print(",");
      Serial.print(spd_raw_input);
      Serial.print(",");
      Serial.print(mCtrl_values[1]);
      Serial.print(",");
      Serial.println(mCtrl_values[0]);
#endif
     
      //- Prepare data >> lower resolution to 8 bit & pad into 16 bit packet
      buf &= MASK_BUFFER_FLAG; //clear steer 
      uint32_t temp = 0;
      temp = mCtrl_values[0];
      buf |= (temp<<8);                  //set speed
      temp = mCtrl_values[1];
      buf |= (temp<<20);                 //set steering
  }

  //every transmit interval
  if (currentMillis - prevMillis >= TRANSMIT_INTERVAL) {
      prevMillis = millis();
      //- Send
      if(send_packet())
      {
        // ---- [Sent Successfully] -----
        //- Trigger led >> User Feedback
        digitalWrite(PIN_LED_TRANSMITTED, HIGH);
        digitalWrite(PIN_LED_AUTO, (buf&0x2)?HIGH:LOW);
        digitalWrite(PIN_LED_ESTOP, (buf&0x1)?HIGH:LOW);
        #if DEBUG_PRINT_MSG
          Serial.print(" ACK ");
        #endif
      }else{
        // ---- [Sent Failed] -----
        digitalWrite(PIN_LED_TRANSMITTED, LOW);
        digitalWrite(PIN_LED_AUTO, (buf&0x2)?HIGH:LOW);
        digitalWrite(PIN_LED_ESTOP, (buf&0x1)?HIGH:LOW);
        #if DEBUG_PRINT_MSG
          Serial.print(" NCK ");
        #endif
      }
     #if DEBUG_PRINT_MSG
      Serial.print(GET_FLAG(buf));
      Serial.print("  ");
      Serial.print(GET_STEER(buf));
      Serial.print("  ");
      Serial.println(GET_SPD(buf));
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

