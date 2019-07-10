
/*
 * Copyright 2016-2018 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    Hummingbot_firmware_FreeRTOS.c
 * @brief   Application entry point.
 */


#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKE14F16.h"
#include "common.h"
#include "nrf24l01/RF24_common.h"
#include "nrf24l01/RF24.h"
#include "vehicleController/vehicleController.h"
#include "Hummingconfig.h"

#include "fsl_debug_console.h"
#include "fsl_device_registers.h"
#include "fsl_gpio.h"
#include "fsl_lpuart.h"
#include "fsl_lpspi.h"
#include "fsl_lpspi_freertos.h"
#include "fsl_lpi2c.h"
#include "fsl_lpi2c_freertos.h"

//libraries for freeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOSConfig.h"

/*************************************  
 ********* Macro Preference ********** 
 *************************************/
#define ENABLE_FEATURE_DEBUG_PRINT            1 //This will enable uart debug print out
#define DISABLE_FEATURE_DEBUG_PRINT_INFO      0
#define DISABLE_FEATURE_DEBUG_PRINT_ERR       0
#define DISABLE_FEATURE_DEBUG_PRINT_WRN       0

#define ENABLE_TASK_RF24				        1
#define ENABLE_TASK_VEHICLE_CONTROL    	1

/*************************************  
 ********* Calibbration Helper ********** 
 *************************************/
/* Helpful Macros for Calibration */
#define CALIB_PRINT_REMOTE              0
#define CALIB_PRINT_VC_SERVO            0
#define CALIB_PRINT_VC_SERVO_WITH_RF    0

#if (CALIB_PRINT_REMOTE)
//undefine
#ifdef ENABLE_TASK_VEHICLE_CONTROL
#undef ENABLE_TASK_VEHICLE_CONTROL
#endif
#ifdef ENABLE_TASK_RF24
#undef ENABLE_TASK_RF24
#endif
#ifdef ENABLE_FEATURE_DEBUG_PRINT
#undef ENABLE_FEATURE_DEBUG_PRINT
#endif
//redefine
#define ENABLE_FEATURE_DEBUG_PRINT      1 
#define ENABLE_TASK_RF24				        1
#define ENABLE_TASK_VEHICLE_CONTROL    	0
#endif

#if (CALIB_PRINT_VC_SERVO)
//undefine
#ifdef ENABLE_TASK_VEHICLE_CONTROL
#undef ENABLE_TASK_VEHICLE_CONTROL
#endif
#ifdef ENABLE_TASK_RF24
#undef ENABLE_TASK_RF24
#endif
#ifdef ENABLE_FEATURE_DEBUG_PRINT
#undef ENABLE_FEATURE_DEBUG_PRINT
#endif
//redefine
#define ENABLE_FEATURE_DEBUG_PRINT      0
#define ENABLE_TASK_RF24				        0
#define ENABLE_TASK_VEHICLE_CONTROL    	1
#endif

#if (CALIB_PRINT_VC_SERVO_WITH_RF)
//undefine
#ifdef ENABLE_TASK_VEHICLE_CONTROL
#undef ENABLE_TASK_VEHICLE_CONTROL
#endif
#ifdef ENABLE_TASK_RF24
#undef ENABLE_TASK_RF24
#endif
#ifdef ENABLE_FEATURE_DEBUG_PRINT
#undef ENABLE_FEATURE_DEBUG_PRINT
#endif
//redefine
#define ENABLE_FEATURE_DEBUG_PRINT      1
#define ENABLE_TASK_RF24                1
#define ENABLE_TASK_VEHICLE_CONTROL     1
#endif
/*************************************  
 ********* Macro Definitions ********** 
 *************************************/
/* Task Tick calculation */
#define HELPER_TASK_FREQUENCY_HZ(x)     (configTICK_RATE_HZ/(x))

/* set status bit */
#define SET_STATUS_BIT(flag)         (m_bot.status |=(1U<<(flag)))
#define CLEAR_STATUS_BIT(flag)       (m_bot.status &=~(1U<<(flag)))
#define CHECK_STATUS_BIT(flag)       ((bool)(m_bot.status & (1U<<(flag))))

/* Debug PRINTF helper functions */
#define DEBUG_PRINTLN(fmt, ...) \
            do { if (ENABLE_FEATURE_DEBUG_PRINT) PRINTF(fmt "\r\n", ##__VA_ARGS__); } while (0)
#define DEBUG_PRINT_ERR(fmt, ...) \
            do { if (ENABLE_FEATURE_DEBUG_PRINT && !DISABLE_FEATURE_DEBUG_PRINT_ERR) PRINTF("[ERR.]" fmt "\r\n", ##__VA_ARGS__); } while (0)
#define DEBUG_PRINT_WRN(fmt, ...) \
            do { if (ENABLE_FEATURE_DEBUG_PRINT && !DISABLE_FEATURE_DEBUG_PRINT_WRN) PRINTF("[WARN]" fmt "\r\n", ##__VA_ARGS__); } while (0)
#define DEBUG_PRINT_INFO(fmt, ...) \
            do { if (ENABLE_FEATURE_DEBUG_PRINT && !DISABLE_FEATURE_DEBUG_PRINT_INFO) PRINTF("[INFO]" fmt "\r\n", ##__VA_ARGS__); } while (0)

/***********************************
 ********* Macro Settings **********
 ***********************************/
/* Task Priority */
#define TASK_RF24_PRIORITY 							 			(configMAX_PRIORITIES - 1)
#define ENABLE_TASK_VEHICLE_CONTROL_PRIORITY 	(configMAX_PRIORITIES - 2)
/* Task Frequency */ //TODO: TBD
#define TASK_RF24_RUNNING_PERIOD                  (HELPER_TASK_FREQUENCY_HZ(10)) //Hz
#define TASK_VEHICLE_CONTROL_RUNNING_PERIOD       (HELPER_TASK_FREQUENCY_HZ(10)) //Hz

/***********************************
 ********* Macro Helpers **********
 ***********************************/

/***************************************  
 *********  Struct/Enums Defs ********** 
 ***************************************/
typedef struct{
	uint16_t 	rf24_buf[2]; //[MSB] 12 bit (steer) | 12 bit (spd) | 8 bit (6 bit pattern + 2 bit modes)
  pin_t 		rf24_ce;
  uint8_t 	rf24_address[RF24_COMMON_ADDRESS_SIZE];
	lpspi_t   spi;
	uint8_t   buf_rx[9];
	uint8_t   buf_tx[9];

  // data extract from the radio
  uint16_t  raw_rf24_speed;
  uint16_t  raw_rf24_steer;
  uint8_t   raw_encoded_flags;
  uint16_t  rf24_error_count;
  uint16_t  rf24_timeout_count_tick;
  uint16_t  rf24_newData_available; //like a non-blocking semaphore
  SemaphoreHandle_t rf24_data_lock;

  // data parsed & sent to the vehicle controller
  angle_deg_t       vc_steeringAngle;
  speed_cm_per_s_t  vc_throttleSpeed;
  uint16_t          vc_newData_available; //like a non-blocking semaphore
  SemaphoreHandle_t vc_data_lock;

  // main status
  HUMMING_STATUS_BIT_E status;
}Hummingbot_firmware_FreeRTOS_2_data_S;

typedef struct
{
	uint16_t jetson_ang;
	int16_t	jetson_spd;
	uint16_t jetson_flag;
	uint16_t jetson_pad;
}jetson_data;

/*************************************
 ********* Inline Definitions **********
 *************************************/
static inline void printHummingBoardLogo(void)
{
  // will not print, if the (ENABLE_FEATURE_DEBUG_PRINT) is disabled, [implicit relationship]
  DEBUG_PRINTLN("#############################################################################################");
  DEBUG_PRINTLN("##     ## ##     ## ##     ## ##     ## #### ##    ##  ######   ########   #######  ######## ");
  DEBUG_PRINTLN("##     ## ##     ## ###   ### ###   ###  ##  ###   ## ##    ##  ##     ## ##     ##    ##    ");
  DEBUG_PRINTLN("##     ## ##     ## #### #### #### ####  ##  ####  ## ##        ##     ## ##     ##    ##    ");
  DEBUG_PRINTLN("######### ##     ## ## ### ## ## ### ##  ##  ## ## ## ##   #### ########  ##     ##    ##    ");
  DEBUG_PRINTLN("##     ## ##     ## ##     ## ##     ##  ##  ##  #### ##    ##  ##     ## ##     ##    ##    ");
  DEBUG_PRINTLN("##     ## ##     ## ##     ## ##     ##  ##  ##   ### ##    ##  ##     ## ##     ##    ##    ");
  DEBUG_PRINTLN("##     ##  #######  ##     ## ##     ## #### ##    ##  ######   ########   #######     ##    ");
  DEBUG_PRINTLN("#############################################################################################");
}

/***************************************  
 *********  Private Variable ********** 
 ***************************************/
Hummingbot_firmware_FreeRTOS_2_data_S m_bot;

/************************************************  
 ********* Private Function Prototypes ********** 
 ***********************************************/
#if (ENABLE_TASK_RF24)
  static void task_rf24(void *pvParameters);
#endif // (ENABLE_TASK_RF24
#if (ENABLE_TASK_VEHICLE_CONTROL)
  static void task_vehicleControl(void *pvParameters);
#endif // (ENABLE_TASK_VEHICLE_CONTROL)

/**************************************  
 ********* Private Functions ********** 
 *************************************/
#if (ENABLE_TASK_RF24)
static void task_rf24(void *pvParameters)
{
  uint32_t temp;
  uint8_t  newFlags;
  TickType_t xLastWakeTime;
  DEBUG_PRINT_INFO(" [TASK] RF24 Remote Controller Begin ...");	
  // Initialize the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();
	while(1) 
	{
    /*
    raw_rf24_speed
    raw_rf24_steer
    raw_encoded_flags
    rf24_error_count
    rf24_timeout_count_tick
    */
	  DEBUG_PRINT_INFO("Scanning");
    temp = 0;
    newFlags = 0;
    if(!CHECK_STATUS_BIT(HUMMING_STATUS_BIT_RF24_ALIVE))
    {
      DEBUG_PRINT_ERR("RF24 Was Not Initialized Successfully");
    }
    else if (RF24_available())
		{
      // fetch data
			RF24_read(&m_bot.rf24_buf, sizeof(m_bot.rf24_buf));
      // parse data
			temp = (m_bot.rf24_buf[1]<<16) + m_bot.rf24_buf[0];
      newFlags = RF24_COMMON_GET_FLAG(temp);
			if(newFlags & RF24_COMMON_MASK_UNIQUE_PATTERN)
			{
        xSemaphoreTake(m_bot.rf24_data_lock, HUMMING_CONFIG_BOT_RF24_SEMAPHORE_LOCK_MAX_TICK);
        m_bot.raw_rf24_speed    = RF24_COMMON_GET_SPD(temp);
        m_bot.raw_rf24_steer    = RF24_COMMON_GET_STEER(temp);
        m_bot.raw_encoded_flags = newFlags;
        // reset error & timeout counts
        m_bot.rf24_error_count = 0;
        m_bot.rf24_timeout_count_tick = 0;
        m_bot.rf24_newData_available ++;
        xSemaphoreGive(m_bot.rf24_data_lock);

        SET_STATUS_BIT(HUMMING_STATUS_BIT_RF24_ONLINE);
				DEBUG_PRINT_INFO("RCV: [SPD|STR|FLAG] [ %d | %d | %d ]", m_bot.raw_rf24_speed, m_bot.raw_rf24_steer, m_bot.raw_encoded_flags);
			}
      else
      {
        m_bot.rf24_error_count++;
				DEBUG_PRINT_ERR("Invalid Message: %d", temp);
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
             (m_bot.rf24_timeout_count_tick < (HUMMING_CONFIG_BOT_LOST_CONTROLLER_TIMEOUT_MS/TASK_RF24_RUNNING_PERIOD)))
    {
      m_bot.rf24_timeout_count_tick ++;
    }
    // completely timeout, => not comm.
    else if(m_bot.rf24_timeout_count_tick == (HUMMING_CONFIG_BOT_LOST_CONTROLLER_TIMEOUT_MS/TASK_RF24_RUNNING_PERIOD))
    {
      CLEAR_STATUS_BIT(HUMMING_STATUS_BIT_RF24_ONLINE);
      DEBUG_PRINT_ERR("RF24 Lost Controller");
    }
    
		vTaskDelayUntil(&xLastWakeTime, TASK_RF24_RUNNING_PERIOD);
	}
}
#endif

#if (ENABLE_TASK_VEHICLE_CONTROL)
static void task_vehicleControl(void *pvParameters)
{
	TickType_t xLastWakeTime;
#if (CALIB_PRINT_VC_SERVO)
  uint16_t task_vc_tick = 0;
  uint16_t num = 1600, dir=50;
  uint8_t tick = 0;
#else
  bool remoteESTOP = false;
  bool autoMode    = false;
  uint16_t  rf24_newdataAvailable = 0;
  uint16_t  rf24_speed = 0;
  uint16_t  rf24_steer = 0;
  uint8_t   rf24_flag  = 0;
  angle_deg_t       reqAng = 0;
  speed_cm_per_s_t  reqSpd = 0;
  pulse_us_t        ang_pw_us = 0;
  pulse_us_t        spd_pw_us = 0;
#endif // (CALIB_PRINT_VC_SERVO)
	DEBUG_PRINT_INFO(" [TASK] Vehicle Control Begin ...");
	// Initialize the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();
	while(1) {
  #if (CALIB_PRINT_VC_SERVO)
    /* calibration code here */
    task_vc_tick ++;
    uint8_t step = 2;
//    int max = 1550, min= 1550;
    switch(step)
    {
      case 1:
        // STEP 1 - find 0 degree servo pw_us by writing raw, and record
    	  VC_requestPWM_force_raw(VC_CHANNEL_NAME_THROTTLE, 540);
    	  vTaskDelay(HELPER_TASK_FREQUENCY_HZ(10)*10);
    	  while(1) {

    		  VC_requestPWM_force_raw(VC_CHANNEL_NAME_THROTTLE, num);
//    		  VC_requestPWM_force_raw(VC_CHANNEL_NAME_STEERING, num);
    		  DEBUG_PRINT_INFO("%d\r\n", num);
    		  //if(num==max) num=min;//dir = -50;
    		  //else if(num==min) num=max;//dir = 50;
//    		  num += dir;
    		  vTaskDelay(HELPER_TASK_FREQUENCY_HZ(10));

    	  }
//    	  VC_requestPWM_force_raw(VC_CHANNEL_NAME_STEERING, 1650);
//        VC_requestPWM_force_raw(VC_CHANNEL_NAME_THROTTLE, 200);
        break;
      case 2:
        // STEP 2 - find + 30 degree of the wheel by gradually increasing pw_us,
        //          find servo pw_us by writing raw, and record
        VC_requestSteering(20);//PLEASE ENTER
        VC_requestThrottle(100);//100cm/s
        tick++;
        if(tick>20){//ms
          tick = 31;
          num = 540;
          //VC_requestThrottle(0);//0cm/s equivalent to doBraking
          VC_doBraking(0);
        }

        break;
      case 3:
        // STEP 3 - do calculation, and validate -30 degree
        VC_requestSteering_raw(1000);//PLEASE ENTER
        break;
      default:
        break;
    }
    DEBUG_PRINT_INFO("...");

  #else 
    /* main code */
    /// 1. healthy state       
    if( CHECK_STATUS_BIT(HUMMING_STATUS_BIT_RF24_ONLINE) &&
        CHECK_STATUS_BIT(HUMMING_STATUS_BIT_RF24_ALIVE) )
    {
      // quick data copy
      xSemaphoreTake(m_bot.rf24_data_lock, HUMMING_CONFIG_BOT_RF24_SEMAPHORE_LOCK_MAX_TICK);
      rf24_newdataAvailable = m_bot.rf24_newData_available;
      rf24_speed = m_bot.raw_rf24_speed;
      rf24_steer = m_bot.raw_rf24_steer;
      rf24_flag = m_bot.raw_encoded_flags;
      m_bot.rf24_newData_available = 0;
      xSemaphoreGive(m_bot.rf24_data_lock);

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
        DEBUG_PRINT_INFO("VC: [ ESTOP: %b | AUTO: %b ]", CHECK_STATUS_BIT(HUMMING_STATUS_BIT_REMOTE_ESTOP), CHECK_STATUS_BIT(HUMMING_STATUS_BIT_AUTO_MODE));
        // state machine
        if(remoteESTOP)
        {
          VC_doBraking(0);
        }
        else
        {
          if(autoMode)
          {
            VC_doBraking(0);
            //TODO: to be implemented, requires a coordination here!!! [TBI]
            if(reqAng>=0)
            {
              DEBUG_PRINT_INFO("VC: [SPD|STR] [ %d cm/s| %d deg]", reqSpd, reqAng);

            }
            else
            {
              DEBUG_PRINT_INFO("VC: [SPD|STR] [ %d cm/s| -%d deg]", reqSpd, reqAng);
            }
          }
          else
          {
            /// - remote controller mode !!! TODO: coordination TBI

            VC_joystick_control(rf24_steer, rf24_speed, &reqAng, &reqSpd);
            ang_pw_us = VC_getCurrentPulseWidth(VC_CHANNEL_NAME_STEERING);
            spd_pw_us = VC_getCurrentPulseWidth(VC_CHANNEL_NAME_THROTTLE);
            if(reqAng>=0)
            {
            	DEBUG_PRINT_INFO("VC: [SPD|STR] [ %d cm/s| %d deg] [ %d us| %d us]", reqSpd, reqAng, spd_pw_us, ang_pw_us);
            }
            else
            {
            	DEBUG_PRINT_INFO("VC: [SPD|STR] [ %d cm/s| -%d deg] [ %d us| %d us]", reqSpd, reqAng, spd_pw_us, ang_pw_us);

            }

            // store these values, NOTE: might be useful for later: closed feedback control loop, jetson, so on
            //xSemaphoreTake(m_bot.vc_data_lock, HUMMING_CONFIG_BOT_RF24_SEMAPHORE_LOCK_MAX_TICK);
            m_bot.vc_steeringAngle = reqAng;
            m_bot.vc_throttleSpeed = reqSpd;
            m_bot.vc_newData_available ++; 
            if (m_bot.vc_newData_available >= 65535)// 2^16 overflow
            {
              m_bot.vc_newData_available = 0; // reset
            }
            //xSemaphoreGive(m_bot.vc_data_lock);
          } 
        }
      }

    }
    /// 2. unhealthy state   
    else if( CHECK_STATUS_BIT(HUMMING_STATUS_BIT_RF24_COMM_STABLE) )
    {
      // let it roll a bit, in case the connection come back within 100ms
      VC_do_FreeWheeling(VC_CHANNEL_NAME_THROTTLE);
    }
    /// 3. lost remote controller state | WIRELESS ESTOP will not work
    else
    {
      // just braking
       VC_doBraking(0);
    }
    #endif //(CALIB_PRINT_VC_SERVO)

    DEBUG_PRINT_INFO("Vehicle Control Running ...");
		vTaskDelayUntil(&xLastWakeTime, TASK_RF24_RUNNING_PERIOD);
	}
}
#endif //(ENABLE_TASK_VEHICLE_CONTROL)

/*********************** 
 ********* APP ********* 
 **********************/
/*
 * @brief   Application entry point.
 */
int main(void) {
	/*---- INIT --------------------------------------------------------*/
	/* Init board hardware. */
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitBootPeripherals();
	BOARD_BootClockRUN();
	/* Init FSL debug console. */
	BOARD_InitDebugConsole();
	printHummingBoardLogo();
	DEBUG_PRINT_INFO(" ****** ******************* ******");
	DEBUG_PRINT_INFO(" ****** Hummingboard begin ******");


  /*---- Custom INIT --------------------------------------------------*/
	DEBUG_PRINT_INFO(" ****** Hummingboard Init ... ******");
	/* Init private data */
	memset(&m_bot, 0, sizeof(m_bot));
	/* Init RTOS related*/
  m_bot.rf24_data_lock = xSemaphoreCreateMutex();
  m_bot.vc_data_lock   = xSemaphoreCreateMutex();
	/* Init rf24 */
#if (ENABLE_TASK_RF24)
	m_bot.rf24_ce.port = RF24_COMMON_DEFAULT_CE_PORT;
	m_bot.rf24_ce.pin = RF24_COMMON_DEFAULT_CE_PIN;
	memcpy(m_bot.rf24_address, RF24_COMMON_ADDRESS, sizeof(char)*RF24_COMMON_ADDRESS_SIZE);
#endif // (ENABLE_TASK_RF24)
#if (ENABLE_TASK_VEHICLE_CONTROL)
	VC_Config(); // NOTE: please config directly within the vehicle control
#endif //(ENABLE_TASK_VEHICLE_CONTROL)


	/*---- CONFIG --------------------------------------------------------*/
	 DEBUG_PRINT_INFO(" ****** Hummingboard Config ... ******");
	/* Config rf24 */
#if (ENABLE_TASK_RF24)
   RF24_config(&m_bot.rf24_ce);
   RF24_INIT_STATUS_E status = RF24_init();
   if(status == RF24_INIT_STATUS_SUCCESS)
   {
     RF24_setDataRate( RF24_250KBPS );//low data rate => longer range and reliable
     RF24_enableAckPayload();
     RF24_setRetries(3,2);
     RF24_openReadingPipe(0, m_bot.rf24_address);
     RF24_setPALevel(RF24_PA_LOW);
     RF24_startListening();
     SET_STATUS_BIT(HUMMING_STATUS_BIT_RF24_ALIVE);
   }
	 else
	 {
			DEBUG_PRINT_ERR(" Failed to configure RF24 Module!");
      CLEAR_STATUS_BIT(HUMMING_STATUS_BIT_RF24_ALIVE);
	 }
#endif
		/* Config vehicle control steering & motoring */
#if (ENABLE_TASK_VEHICLE_CONTROL)
	if(VC_Init())
	{
	  // start vc
    if(!VC_Begin())
    {
      DEBUG_PRINT_INFO(" Failed to start devices for Vehicle Controller!");
    }
	}
	else
	{
	  DEBUG_PRINT_INFO(" Failed to initialize Vehicle Controller!");
	}
#endif //(ENABLE_TASK_VEHICLE_CONTROL)


	/*---- TASK CONFIGS --------------------------------------------------------*/
  DEBUG_PRINT_INFO(" ****** Hummingboard Config Tasks ... ******");
#if (ENABLE_TASK_RF24)
	if (xTaskCreate(task_rf24, "task_rf24", configMINIMAL_STACK_SIZE + 10, NULL, TASK_RF24_PRIORITY, NULL) != pdPASS)
	{
	  DEBUG_PRINT_ERR("Task creation failed!.");
		while (1);
	}
#endif
#if (ENABLE_TASK_VEHICLE_CONTROL)
	if (xTaskCreate(task_vehicleControl, "task_vehicleControl", configMINIMAL_STACK_SIZE + 10, NULL, ENABLE_TASK_VEHICLE_CONTROL_PRIORITY, NULL) != pdPASS)
	  {
	    DEBUG_PRINT_ERR("Task creation failed!.\r\n");
	    while (1);
	  }
#endif //(ENABLE_TASK_VEHICLE_CONTROL)


	/*---- TASK SCHEDULAR START --------------------------------------------------------*/
  DEBUG_PRINT_INFO(" ****** Hummingboard Running ******");
  DEBUG_PRINT_INFO(" ****** ******************* ******");
	vTaskStartScheduler();
	return 0;
}

