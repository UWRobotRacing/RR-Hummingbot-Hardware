String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

#define COMMON_M4_JETSON_SYNC_START_BYTE  ('a')
#define COMMON_M4_JETSON_SYNC_END_BYTE    ('n')
#define COMMON_M4_JETSON_FRAME_SIZE       (sizeof(jetson_union_t))
#define TRANSMIT_S1_RAW           (1)
/* uart communication commons */
typedef struct
{
   int16_t jetson_ang;
   int16_t  jetson_spd;
   uint16_t jetson_flag;
}jetson_data_t;

typedef struct
{
   uint8_t        startByte;
   jetson_data_t  data;
   uint8_t        endByte;
}jetson_packet_t;

typedef union
{
  jetson_packet_t myFrame;
  uint8_t         serializedArray[8];
}jetson_union_t;

typedef struct{
  uint32_t  rf24_buf; //[MSB] 12 bit (steer) | 12 bit (spd) | 8 bit (6 bit pattern + 2 bit modes)

  // data extract from the radio
  uint16_t  raw_rf24_speed;
  uint16_t  raw_rf24_steer;
  uint8_t   raw_encoded_flags;
  uint16_t  rf24_error_count;
  uint16_t  rf24_timeout_count_tick;
  uint16_t  rf24_newData_available; //like a non-blocking semaphore

  bool      autoMode;
  bool      remoteESTOP;
  // uart comm.
  jetson_union_t rxPayload[2];
  jetson_union_t* readPtr_rxPayload;
  jetson_union_t* bufPtr_rxPayload;
  bool            newPayloadAvail;
  uint8_t serial_readByte_index;

  bool            toggleBit;

  // main status

}Hummingbot_firmware_FreeRTOS_2_data_S;
Hummingbot_firmware_FreeRTOS_2_data_S m_bot;

void setup() {
  // initialize serial:
  Serial.begin(9600);
  Serial1.begin(9600);
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);
  memset(&m_bot, 0, sizeof(m_bot));
  // prepare empty double buffer
  m_bot.readPtr_rxPayload = &m_bot.rxPayload[0];
  m_bot.bufPtr_rxPayload  = &m_bot.rxPayload[1];
  m_bot.newPayloadAvail = false;
  pinMode(40, OUTPUT);
}

void loop() {

  if (m_bot.newPayloadAvail) 
  {
   #if (!TRANSMIT_S1_RAW)
     for ( int i =0; i<8;i++)
       Serial1.print(char(m_bot.readPtr_rxPayload->serializedArray[i]));
   #endif    
     Serial.println("------------");
     for ( int i =0; i<8;i++)
       Serial.println(char(m_bot.readPtr_rxPayload->serializedArray[i]));
     Serial.println("------------");
     Serial.println(m_bot.readPtr_rxPayload->myFrame.startByte);
     Serial.println(m_bot.readPtr_rxPayload->myFrame.data.jetson_ang);
     Serial.println(m_bot.readPtr_rxPayload->myFrame.data.jetson_spd);
     Serial.println(m_bot.readPtr_rxPayload->myFrame.data.jetson_flag);
     Serial.println(m_bot.readPtr_rxPayload->myFrame.endByte);
     Serial.println("");
     m_bot.newPayloadAvail = false;
  }

  m_bot.toggleBit = !m_bot.toggleBit;
  digitalWrite(40, m_bot.toggleBit);
  delay(100);
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial.available()) {
         jetson_union_t* tempPtr;
      char incomingByte;
 #if (TRANSMIT_S1_RAW)
      Serial1.print((char)incomingByte);
 #endif
      if (Serial.available() > 0) 
      {
        incomingByte = (char)Serial.read();
        if (m_bot.serial_readByte_index == 0)
        {
          if (COMMON_M4_JETSON_SYNC_START_BYTE == incomingByte)
          {
            m_bot.bufPtr_rxPayload->serializedArray[m_bot.serial_readByte_index++] = incomingByte;
          }
        }
        else if (m_bot.serial_readByte_index < COMMON_M4_JETSON_FRAME_SIZE)
        {
          m_bot.bufPtr_rxPayload->serializedArray[m_bot.serial_readByte_index++] = incomingByte;
        }
        else if (m_bot.serial_readByte_index == COMMON_M4_JETSON_FRAME_SIZE)
        {
          // check if last byte matches, if so, store, and update
          if (m_bot.bufPtr_rxPayload->serializedArray[COMMON_M4_JETSON_FRAME_SIZE - 1] == COMMON_M4_JETSON_SYNC_END_BYTE)
          {
            tempPtr = m_bot.bufPtr_rxPayload;
            m_bot.bufPtr_rxPayload = m_bot.readPtr_rxPayload;
            m_bot.readPtr_rxPayload = tempPtr;
            m_bot.newPayloadAvail = true;
            m_bot.serial_readByte_index = 0;
          }
          else// else reset index, corrupted data confirmed
          {
            m_bot.serial_readByte_index = 0;
    #if (ENABLE_UART_SERIAL_ECHO)
            Serial.println("------ INVALID DATA, Recycle -----");
    #endif //(ENABLE_UART_SERIAL_ECHO)
          }
        }
      }
  }
}
