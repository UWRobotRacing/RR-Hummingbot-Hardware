#ifndef HUMMINGBOT_COMMON_H
#define HUMMINGBOT_COMMON_H

/*************************************  
 ********* Macro Helpers ********** 
 *************************************/
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

#define COMMON_M4_JETSON_SYNC_START_BYTE  ('a')
#define COMMON_M4_JETSON_SYNC_END_BYTE    ('n')
#define COMMON_M4_JETSON_FRAME_SIZE       (sizeof(jetson_union_t))

/* set status bit */
#define SET_STATUS_BIT(flag)         (m_bot.status |=(1U<<(flag)))
#define CLEAR_STATUS_BIT(flag)       (m_bot.status &=~(1U<<(flag)))
#define CHECK_STATUS_BIT(flag)       ((bool)(m_bot.status & (1U<<(flag))))

/* Humming bot status bit enum */
typedef enum {
    HUMMING_STATUS_BIT_RF24_ALIVE        =0U,
    HUMMING_STATUS_BIT_RF24_COMM_STABLE  =1U,
    HUMMING_STATUS_BIT_RF24_ONLINE       =2U,
    HUMMING_STATUS_BIT_VC_ALIVE          =3U,
    HUMMING_STATUS_BIT_AUTO_MODE         =4U,
    HUMMING_STATUS_BIT_REMOTE_ESTOP      =5U,
    //--------UNUSED--------//
    HUMMING_STATUS_BIT_UNUSED6                =6U,
    HUMMING_STATUS_BIT_UNUSED7                =7U
} HUMMING_STATUS_BIT_E; //8 bit

            
/* Debug PRINTF helper functions */
#define DEBUG_PRINTLN(...)
#define DEBUG_PRINT_ERR(...)
#define DEBUG_PRINT_WRN(...) 
#define DEBUG_PRINT_INFO(...)  

#define DEBUG_PRINTLN(x) \
            do { if (ENABLE_FEATURE_DEBUG_PRINT) Serial.println(x); } while (0)
#define DEBUG_PRINT_ERR(x) \
            do { if (ENABLE_FEATURE_DEBUG_PRINT) Serial.println(x); } while (0)
#define DEBUG_PRINT_WRN(x) \
            do { if (ENABLE_FEATURE_DEBUG_PRINT) Serial.println(x); } while (0)
#define DEBUG_PRINT_INFO(x) \
            do { if (ENABLE_FEATURE_DEBUG_PRINT) Serial.println(x); } while (0)

#endif //HUMMINGBOT_COMMON_H
