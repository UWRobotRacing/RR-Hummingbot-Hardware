/* Arduino Core Lib */
#include <SPI.h>
/* Arduino Sub Module Lib */
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
/* Custom Lib */
#include "RF24_common.h"
#include "Hummingbot_common.h"
#include "Hummingconfig.h"
#include "vehicleController.h"
/*************************************  
 ********* Macro Preference ********** 
 *************************************/
#define ENABLE_FEATURE_DEBUG_PRINT            0 //This will enable uart debug print out

#define ENABLE_TASK_RF24				        1
#define ENABLE_TASK_VEHICLE_CONTROL    	1
#define ENABLE_UART_SERIAL_COMM         1

#define ENTER_CALIB_MODE                0
#define ENABLE_UART_SERIAL_ECHO         0
// serial comm will override debug print
#if ((ENABLE_UART_SERIAL_COMM) && (ENABLE_FEATURE_DEBUG_PRINT))
#ifdef ENABLE_FEATURE_DEBUG_PRINT
#undef ENABLE_FEATURE_DEBUG_PRINT
#endif
#define ENABLE_FEATURE_DEBUG_PRINT    0
#endif //((ENABLE_UART_SERIAL_COMM) && (ENABLE_FEATURE_DEBUG_PRINT))

#define TASK_RF24_LOST_CONTROLLER_TICK_MAX        (2000U) 
#define CYCLE_DELAY                               (50U)//ms
/***************************************  
 *********  Struct Define ********** 
 ***************************************/
#if (ENABLE_UART_SERIAL_COMM)
void uart_run(void);
#endif //(ENABLE_UART_SERIAL_COMM)

typedef struct{
	uint32_t 	rf24_buf; //[MSB] 12 bit (steer) | 12 bit (spd) | 8 bit (6 bit pattern + 2 bit modes)
  uint8_t 	rf24_address[RF24_COMMON_ADDRESS_SIZE];

  // data extract from the radio
  uint16_t  raw_rf24_speed;
  uint16_t  raw_rf24_steer;
  uint8_t   raw_encoded_flags;
  uint16_t  rf24_error_count;
  uint16_t  rf24_timeout_count_tick;
  uint16_t  rf24_newData_available; //like a non-blocking semaphore

  // uart comm.
  jetson_union_t rxPayload[2];
  jetson_union_t* readPtr_rxPayload;
  jetson_union_t* bufPtr_rxPayload;
  bool            newPayloadAvail;
  uint8_t serial_readByte_index;

  bool            ifReversing;

  // main status
  HUMMING_STATUS_BIT_E status;
}Hummingbot_firmware_FreeRTOS_2_data_S;

/***************************************  
 *********  Private Variable ********** 
 ***************************************/
Hummingbot_firmware_FreeRTOS_2_data_S m_bot;

/***************************************  
 *********  Prototype Functions ********** 
 ***************************************/
#if (ENABLE_TASK_RF24)
RF24 rf24_Radio(7, 8); // CE, CSN
void rf24_init(void);
void rf24_run(void);
#endif //(ENABLE_TASK_RF24)

#if (ENABLE_TASK_VEHICLE_CONTROL)
Servo vc_ESC;
Servo vc_SERVO;
void vc_run(void);
#endif //(ENABLE_TASK_VEHICLE_CONTROL)

#if (ENABLE_UART_SERIAL_COMM)
void uart_run(void);
#endif //(ENABLE_UART_SERIAL_COMM)
/***************************************  
 *********  MAIN Functions ********** 
 ***************************************/
void setup() {
  /* Init Serial */
  Serial.begin(115200);
  /* Init private data */
  memset(&m_bot, 0, sizeof(m_bot));
  // prepare empty double buffer
  m_bot.readPtr_rxPayload = &m_bot.rxPayload[0];
  m_bot.bufPtr_rxPayload  = &m_bot.rxPayload[1];
  m_bot.newPayloadAvail = false;
  /* Init RF24 */
#if (ENABLE_TASK_RF24)
  memcpy(m_bot.rf24_address, RF24_COMMON_ADDRESS, sizeof(char)*RF24_COMMON_ADDRESS_SIZE);
  rf24_init();
#endif //(ENABLE_TASK_RF24)
  /* Init VC*/
#if (ENABLE_TASK_VEHICLE_CONTROL)
  vc_SERVO.attach(HUMMING_CONFIG_STEERING_SERVO_GPIO_PIN);
  vc_ESC.attach(HUMMING_CONFIG_THROTTLE_ESC_GPIO_PIN);
  VC_Config();
#endif //(ENABLE_TASK_VEHICLE_CONTROL)

}

#if (ENTER_CALIB_MODE)
uint16_t input = 0;
#endif //(ENTER_CALIB_MODE)
void loop() {
  #if (ENTER_CALIB_MODE)
  
    if(Serial.available())
    {
      input = Serial.parseInt();
      Serial.println(input);
    }
//    vc_SERVO.writeMicroseconds(input);
    vc_ESC.writeMicroseconds(input);
  #else
    #if (ENABLE_TASK_RF24)
      rf24_run();
    #endif //(ENABLE_TASK_RF24)
    #if (ENABLE_TASK_VEHICLE_CONTROL)
      vc_run();
    #endif //(ENABLE_TASK_VEHICLE_CONTROL)
    #if (ENABLE_UART_SERIAL_COMM)
      uart_run();
    #endif //(ENABLE_UART_SERIAL_COMM)
  #endif //(ENTER_CALIB_MODE)
 delay(CYCLE_DELAY);
}

/***************************************  
 *********  Private Functions ********** 
 ***************************************/
#if (ENABLE_TASK_RF24)
void rf24_init(void)
{
  bool status = rf24_Radio.begin();
  if(status)
  {
    rf24_Radio.setDataRate( RF24_250KBPS );//low data rate => longer range and reliable
    rf24_Radio.enableAckPayload();
    rf24_Radio.setRetries(3,2);
    rf24_Radio.openReadingPipe(0, m_bot.rf24_address);
    rf24_Radio.setPALevel(RF24_PA_HIGH);
    rf24_Radio.startListening();
    SET_STATUS_BIT(HUMMING_STATUS_BIT_RF24_ALIVE);
  }
  else
  {
    DEBUG_PRINT_ERR(" Failed to configure RF24 Module!");
    CLEAR_STATUS_BIT(HUMMING_STATUS_BIT_RF24_ALIVE);
  }
}

void rf24_run(void)
{
  uint8_t  newFlags;
//  DEBUG_PRINT_INFO("Scanning");
  newFlags = 0;
  if(!CHECK_STATUS_BIT(HUMMING_STATUS_BIT_RF24_ALIVE))
  {
    DEBUG_PRINT_ERR("RF24 Was Not Initialized Successfully");
  }
  else if (rf24_Radio.available())
  {
    // fetch data
    rf24_Radio.read(&m_bot.rf24_buf, sizeof(m_bot.rf24_buf));
    // parse data
    newFlags = RF24_COMMON_GET_FLAG(m_bot.rf24_buf);
    if(newFlags & RF24_COMMON_MASK_UNIQUE_PATTERN)
    {
      //xSemaphoreTake(m_bot.rf24_data_lock, HUMMING_CONFIG_BOT_RF24_SEMAPHORE_LOCK_MAX_TICK);
      m_bot.raw_rf24_speed    = RF24_COMMON_GET_SPD(m_bot.rf24_buf);
      m_bot.raw_rf24_steer    = RF24_COMMON_GET_STEER(m_bot.rf24_buf);
      m_bot.raw_encoded_flags = newFlags;
      // reset error & timeout counts
      m_bot.rf24_error_count = 0;
      m_bot.rf24_timeout_count_tick = 0;
      m_bot.rf24_newData_available ++;
      //xSemaphoreGive(m_bot.rf24_data_lock);
      SET_STATUS_BIT(HUMMING_STATUS_BIT_RF24_ONLINE);
#if (ENABLE_FEATURE_DEBUG_PRINT)
      Serial.print(m_bot.raw_rf24_speed,DEC);  
      Serial.print(" tik, ");  
      Serial.print(m_bot.raw_rf24_steer,DEC);
      Serial.print(" tik, ");   
      Serial.print( m_bot.raw_encoded_flags,BIN);
      Serial.println(" FLAG");
//      delay(100);
#endif //(ENABLE_FEATURE_DEBUG_PRINT)
      // DEBUG_PRINT_INFO("RCV: [SPD|STR|FLAG] [ %d | %d | %d ]", m_bot.raw_rf24_speed, m_bot.raw_rf24_steer, m_bot.raw_encoded_flags);
    }
      else
      {
        m_bot.rf24_error_count++;
				// DEBUG_PRINT_ERR("Invalid Message: %d", temp);
			}
         
      if(m_bot.rf24_error_count < HUMMING_CONFIG_BOT_UNSTABLE_RF_COMM_MIN_CNTS)
      {
        SET_STATUS_BIT(HUMMING_STATUS_BIT_RF24_COMM_STABLE);
      }
      else
      {
        CLEAR_STATUS_BIT(HUMMING_STATUS_BIT_RF24_COMM_STABLE);
        DEBUG_PRINT_ERR("RF24 Experiencing UNSTABLE connections!");        
      }
		}
    // if still within timeout period
    else if( CHECK_STATUS_BIT(HUMMING_STATUS_BIT_RF24_ONLINE) && 
             (m_bot.rf24_timeout_count_tick < (TASK_RF24_LOST_CONTROLLER_TICK_MAX)))
    {
      m_bot.rf24_timeout_count_tick ++;
    }
    // completely timeout, => not comm.
    else if(m_bot.rf24_timeout_count_tick == (TASK_RF24_LOST_CONTROLLER_TICK_MAX))
    {
      CLEAR_STATUS_BIT(HUMMING_STATUS_BIT_RF24_ONLINE);
      DEBUG_PRINT_ERR("RF24 Lost Controller");
    }
		// vTaskDelayUntil(&xLastWakeTime, TASK_RF24_RUNNING_PERIOD);  
}
#endif //(ENABLE_TASK_RF24)

#if (ENABLE_TASK_VEHICLE_CONTROL)
void vc_run(void)
{
  bool remoteESTOP = false;
  bool autoMode    = false;
  uint16_t  rf24_newdataAvailable = 0;
  uint16_t  rf24_speed = 0;
  uint16_t  rf24_steer = 0;
  uint8_t   rf24_flag  = 0;
  angle_deg_t       reqAng = 0;
  speed_cm_per_s_t  reqSpd = 0;
  pulse_us_t        outAngPW = 0;
  pulse_us_t        outSpdPW = 0;
  /* main code */
  /// 1. healthy state       
  if( CHECK_STATUS_BIT(HUMMING_STATUS_BIT_RF24_ONLINE) &&
      CHECK_STATUS_BIT(HUMMING_STATUS_BIT_RF24_ALIVE) )
  {
    // quick data copy
    // xSemaphoreTake(m_bot.rf24_data_lock, HUMMING_CONFIG_BOT_RF24_SEMAPHORE_LOCK_MAX_TICK);
    rf24_newdataAvailable = m_bot.rf24_newData_available;
    rf24_speed = m_bot.raw_rf24_speed;
    rf24_steer = m_bot.raw_rf24_steer;
    rf24_flag = m_bot.raw_encoded_flags;
    m_bot.rf24_newData_available = 0;
    // xSemaphoreGive(m_bot.rf24_data_lock);

    // if it is a new message
    if(rf24_newdataAvailable)
    {
      // ------- PARSING ------- //
      // determine flags & update status flag
      remoteESTOP = RF24_COMMON_CHECK_ESTOP_FLAG(rf24_flag);
      autoMode = RF24_COMMON_CHECK_AUTO_FLAG (rf24_flag);
      if(remoteESTOP)
      {
        SET_STATUS_BIT(HUMMING_STATUS_BIT_REMOTE_ESTOP);
      }
      else
      {
        CLEAR_STATUS_BIT(HUMMING_STATUS_BIT_REMOTE_ESTOP);
      }
      
      if(autoMode)
      {
        SET_STATUS_BIT(HUMMING_STATUS_BIT_AUTO_MODE);
      }
      else
      {
        CLEAR_STATUS_BIT(HUMMING_STATUS_BIT_AUTO_MODE);
      }
#if (ENABLE_FEATURE_DEBUG_PRINT)
      Serial.print("VC: [ ESTOP: ");  
      Serial.print(CHECK_STATUS_BIT(HUMMING_STATUS_BIT_REMOTE_ESTOP),DEC);  
      Serial.print(" | AUTO: ");  
      Serial.print(CHECK_STATUS_BIT(HUMMING_STATUS_BIT_AUTO_MODE),DEC);
      Serial.println(" ]");   
#endif //(ENABLE_FEATURE_DEBUG_PRINT)
      // DEBUG_PRINT_INFO("VC: [ ESTOP: %b | AUTO: %b ]", CHECK_STATUS_BIT(HUMMING_STATUS_BIT_REMOTE_ESTOP), CHECK_STATUS_BIT(HUMMING_STATUS_BIT_AUTO_MODE));
      // state machine
      if(remoteESTOP)
      {
        //keep neutral
        if(VC_STATE_NEUTRAL == VC_getVehicleControllerState())
        {
          vc_ESC.writeMicroseconds(onyx_bldc_esc_calib.neutral.pw_us);
        }
        else
        {
          vc_ESC.writeMicroseconds(VC_doBraking());
        }
      }
      else
      {
        if(autoMode)
        {
          //TODO: to be implemented, requires a coordination here!!! [TBI]
          if(m_bot.newPayloadAvail)
          {
            outAngPW = VC_requestSteering(m_bot.readPtr_rxPayload->myFrame.data.jetson_ang);
            outSpdPW = VC_requestThrottle(m_bot.readPtr_rxPayload->myFrame.data.jetson_spd);
            m_bot.newPayloadAvail = false;
            vc_ESC.writeMicroseconds(outSpdPW);
            vc_SERVO.writeMicroseconds(outAngPW);
          }
          
          if(reqAng>=0)
          {
            // DEBUG_PRINT_INFO("VC: [SPD|STR] [ %d cm/s| %d deg]", reqSpd, reqAng);

          }
          else
          {
            // DEBUG_PRINT_INFO("VC: [SPD|STR] [ %d cm/s| -%d deg]", reqSpd, reqAng);
          }
        }
        else
        {
          VC_joystick_control(rf24_steer, rf24_speed, &reqAng, &reqSpd, &outAngPW, &outSpdPW);
          vc_SERVO.writeMicroseconds(outAngPW);
          vc_ESC.writeMicroseconds(outSpdPW);
          if(reqAng>=0)
          {
            // DEBUG_PRINT_INFO("VC: [SPD|STR] [ %d cm/s| %d deg]", reqSpd, reqAng);
          }
          else
          {
            // DEBUG_PRINT_INFO("VC: [SPD|STR] [ %d cm/s| -%d deg]", reqSpd, reqAng);

          }
#if (ENABLE_FEATURE_DEBUG_PRINT)
      Serial.print("VC: [SPD|STR] [ ");  
      Serial.print(reqSpd,DEC);
      Serial.print(" cm/s| ");   
      Serial.print(reqAng,DEC);
      Serial.println(" deg]");
#endif //(ENABLE_FEATURE_DEBUG_PRINT)
        } 
      }
    }

  }
  /// 2. unhealthy state   
  else if( CHECK_STATUS_BIT(HUMMING_STATUS_BIT_RF24_COMM_STABLE) )
  {
    // let it roll a bit, in case the connection come back within 100ms
  }
  /// 3. lost remote controller state | WIRELESS ESTOP will not work
  else
  {
    // just braking
    //keep neutral
    if(VC_STATE_NEUTRAL == VC_getVehicleControllerState())
    {
      vc_ESC.writeMicroseconds(onyx_bldc_esc_calib.neutral.pw_us);
    }
    else
    {
      vc_ESC.writeMicroseconds(VC_doBraking());
    }
  }
}
#endif //(ENABLE_TASK_VEHICLE_CONTROL)

#if (ENABLE_UART_SERIAL_COMM)
void uart_run(void)
{
  jetson_union_t* tempPtr;
  String incomingFrame;
  if (Serial.available() > 0) 
  {
    incomingFrame = Serial.readString();
    if (incomingFrame.length() == COMMON_M4_JETSON_FRAME_SIZE)
    {
      if ((COMMON_M4_JETSON_SYNC_START_BYTE == incomingFrame[0]) &&
          (COMMON_M4_JETSON_SYNC_END_BYTE == incomingFrame[COMMON_M4_JETSON_FRAME_SIZE - 1]))
      {
        for(int i =0; i<COMMON_M4_JETSON_FRAME_SIZE;i++)
          m_bot.bufPtr_rxPayload->serializedArray[i] = incomingFrame[i];
        tempPtr = m_bot.bufPtr_rxPayload;
        m_bot.bufPtr_rxPayload = m_bot.readPtr_rxPayload;
        m_bot.readPtr_rxPayload = tempPtr;
        m_bot.newPayloadAvail = true;
        m_bot.serial_readByte_index = 0;
      }
    }
  } 

#if (ENABLE_UART_SERIAL_ECHO)
 //for testing
 if (m_bot.newPayloadAvail) 
 {
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
#endif //(ENABLE_UART_SERIAL_ECHO)

}
#endif //(ENABLE_UART_SERIAL_COMM)
