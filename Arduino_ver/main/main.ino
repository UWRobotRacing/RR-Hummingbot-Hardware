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
#define ENABLE_TASK_RF24				        1
#define ENABLE_TASK_VEHICLE_CONTROL    	1

#define TASK_RF24_LOST_CONTROLLER_TICK_MAX        (2000U) 
/***************************************  
 *********  Struct Define ********** 
 ***************************************/
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

/***************************************  
 *********  MAIN Functions ********** 
 ***************************************/
void setup() {
  /* Init Serial */
  Serial.begin(9600);
  /* Init private data */
  memset(&m_bot, 0, sizeof(m_bot));
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

void loop() {
#if (ENABLE_TASK_RF24)
  rf24_run();
#endif //(ENABLE_TASK_RF24)
#if (ENABLE_TASK_VEHICLE_CONTROL)
  vc_run();
#endif //(ENABLE_TASK_VEHICLE_CONTROL)
 delay(500);
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
      delay(100);
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
        vc_ESC.writeMicroseconds(VC_doBraking());
      }
      else
      {
        if(autoMode)
        {
          VC_doBraking();
          //TODO: to be implemented, requires a coordination here!!! [TBI]
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
          vc_ESC.writeMicroseconds(outSpdPW);
          vc_SERVO.writeMicroseconds(outAngPW);
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
    vc_ESC.writeMicroseconds(VC_doBraking());
  }
}
#endif //(ENABLE_TASK_VEHICLE_CONTROL)


