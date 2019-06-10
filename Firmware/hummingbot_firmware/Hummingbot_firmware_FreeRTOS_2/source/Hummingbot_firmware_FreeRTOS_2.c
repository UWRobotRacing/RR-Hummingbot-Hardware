
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

#include "fsl_debug_console.h"
#include "fsl_device_registers.h"
#include "fsl_gpio.h"
#include "fsl_lpuart.h"
#include "fsl_lpspi.h"
#include "fsl_lpspi_freertos.h"
#include "fsl_lpi2c.h"
#include "fsl_lpi2c_freertos.h"
#include "fsl_ftm.h"

//libraries for freeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOSConfig.h"

/*************************************  
 ********* Macro Preference ********** 
 *************************************/
#define ENABLE_FEATURE_DEBUG_PRINT      1 //This will enable uart debug print out
#define ENABLE_TASK_RF24								0
#define ENABLE_TASK_VEHICLE_CONTROL    	0
#define TEST_FTM_PWM										1
/*************************************  
 ********* Macro Definitions ********** 
 *************************************/
/* Task Tick calculation */
#define HELPER_TASK_FREQUENCY_HZ(x)     (configTICK_RATE_HZ/(x))

/* Debug PRINTF helper functions */
#define DEBUG_PRINTLN(fmt, ...) \
            do { if (ENABLE_FEATURE_DEBUG_PRINT) PRINTF(fmt "\r\n", ##__VA_ARGS__); } while (0)
#define DEBUG_PRINT_ERR(fmt, ...) \
            do { if (ENABLE_FEATURE_DEBUG_PRINT) PRINTF("[ERR.]" fmt "\r\n", ##__VA_ARGS__); } while (0)
#define DEBUG_PRINT_WRN(fmt, ...) \
            do { if (ENABLE_FEATURE_DEBUG_PRINT) PRINTF("[WARN]" fmt "\r\n", ##__VA_ARGS__); } while (0)
#define DEBUG_PRINT_INFO(fmt, ...) \
            do { if (ENABLE_FEATURE_DEBUG_PRINT) PRINTF("[INFO]" fmt "\r\n", ##__VA_ARGS__); } while (0)

/***********************************
 ********* Macro Settings **********
 ***********************************/
/* Task Priority */
#define TASK_RF24_PRIORITY 							 			(configMAX_PRIORITIES - 1)
#define ENABLE_TASK_VEHICLE_CONTROL_PRIORITY 	(configMAX_PRIORITIES - 2)
/* Task Frequency */
#define TASK_RF24_FREQ                   			(HELPER_TASK_FREQUENCY_HZ(10)) //Hz
#define TENABLE_TASK_VEHICLE_CONTROL_FREQ     (HELPER_TASK_FREQUENCY_HZ(10)) //Hz //TODO: TBI, to be implemented
/* Servo Macros */
#if (ENABLE_TASK_VEHICLE_CONTROL || TEST_FTM_PWM)
#define SERVO_PWM_PERIOD_MS       (uint8_t)20
#define SERVO_MIN_ANGLE         	60
#define SERVO_MAX_ANGLE         	110
#define SERVO_GPIO_PORT         	GPIOE
#define SERVO_GPIO_PIN          	13U // Servo connected to SERVO LEFT connector on hummingboard.
#endif //(ENABLE_TASK_VEHICLE_CONTROL)

/***********************************
 ********* Macro Helpers **********
 ***********************************/
#if (ENABLE_TASK_VEHICLE_CONTROL)
#define SERVO_PWM_PERIOD_TICKS      		(int) (SERVO_PWM_PERIOD_MS * ((float)configTICK_RATE_HZ / 1000))
#define SERVO_CONVERT_CYCLE_2_TICKS(c)  (int)(SERVO_PWM_PERIOD_MS * (c / (float) 100) * (configTICK_RATE_HZ / 1000))
#define SERVO_CONVERT_ANGLE_2_TICKS(a)  SERVO_CONVERT_CYCLE_2_TICKS((float)(2.6986 + (7.346 - 2.6986) / 90.0 * a))
#define SERVO_MIN_TICKS         				SERVO_CONVERT_ANGLE_2_TICKS(SERVO_MIN_ANGLE)
#define SERVO_MAX_TICKS         				SERVO_CONVERT_ANGLE_2_TICKS(SERVO_MAX_ANGLE)
#endif //(ENABLE_TASK_VEHICLE_CONTROL)

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
}Hummingbot_firmware_FreeRTOS_2_S;
//TODO: change name of struct??


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
Hummingbot_firmware_FreeRTOS_2_S m_data;

/************************************************  
 ********* Private Function Prototypes ********** 
 ***********************************************/
#if (ENABLE_TASK_RF24)
  static void task_rf24(void *pvParameters);
#endif // (ENABLE_TASK_RF24)
#if (ENABLE_TASK_VEHICLE_CONTROL)
  static void task_steeringControl(void *pvParameters);
#endif // (ENABLE_TASK_VEHICLE_CONTROL)
/**************************************  
 ********* Private Functions ********** 
 *************************************/
#if (ENABLE_TASK_RF24)
static void task_rf24(void *pvParameters)
{
  TickType_t xLastWakeTime;
  // Initialize the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();
	while(1) 
	{
	  DEBUG_PRINT_INFO("Scanning");
		if (RF24_available())
		{
//		    char text[32] = "";
//		    RF24_read(&text, sizeof(text));
//		    DEBUG_PRINT_INFO("RCV: %s",text);
			RF24_read(&m_data.rf24_buf, sizeof(m_data.rf24_buf));
			uint32_t temp = (m_data.rf24_buf[1]<<16) + m_data.rf24_buf[0];
			uint16_t temp1 = temp >>20;
			uint16_t temp2 = (temp>>8)&(0xFFF);
			uint16_t temp3 = temp&(0xFF);
			if(temp3!=0) //TODO: filter out with pattern
			{
				DEBUG_PRINT_INFO("RCV: %d | %d | %d", temp1, temp2, temp3);
			}else{
				DEBUG_PRINT_ERR("Invalid Message %d", temp3);
			}
		}
		vTaskDelayUntil(&xLastWakeTime, TASK_RF24_FREQ);
	}
}
#endif

#if (ENABLE_TASK_VEHICLE_CONTROL)
static void task_steeringControl(void *pvParameters)
{
	int angle = 85;

	while(1) {
	  //toggle High
	  GPIO_PortSet(SERVO_GPIO_PORT, 1u << SERVO_GPIO_PIN);

	  //delay ticks
	  vTaskDelay(SERVO_CONVERT_ANGLE_2_TICKS(angle));

	  // Toggle Low
	  GPIO_PortClear(SERVO_GPIO_PORT, 1u << SERVO_GPIO_PIN);

	   //delay 20ms - ticks
	   vTaskDelay(SERVO_PWM_PERIOD_TICKS - SERVO_CONVERT_ANGLE_2_TICKS(angle));
	}
}
#endif //(ENABLE_TASK_VEHICLE_CONTROL)

#if (TEST_FTM_PWM)
/* The Flextimer instance/channel used for board */
#define BOARD_FTM_BASEADDR FTM0
/* Interrupt number and interrupt handler for the FTM instance used */
#define BOARD_FTM_IRQ_NUM FTM0_IRQn
#define BOARD_FTM_HANDLER FTM0_IRQHandler
/* Get source clock for FTM driver */
#define FTM_SOURCE_CLOCK (CLOCK_GetFreq(kCLOCK_CoreSysClk)/4)
#define US_PER_TICK (10U)
#define GO_TO_NEXT_STATE(newState) (PWM_Status = (newState))
#define PWM_SERVO_STEERING_REFRESHING_PERIOD (3003) //us //333Hz
#define REFRESHING_PERIOD (PWM_SERVO_STEERING_REFRESHING_PERIOD)
volatile bool ftmIsrFlag = false;
volatile uint32_t milisecondCounts = 0U;
volatile uint16_t pulseWidth_us = 1500U;
volatile uint16_t requestedPulseWidth_us = 1500U; //800us ~ 2200us
volatile uint32_t PWM_microsecondCounts = 0U;
typedef enum 
{
	PWM_STATUS_UNKNOWN,
	PWM_STATUS_REINITED,
	PWM_STATUS_UPDATED,
	PWM_STATUS_ENABLED,
	PWM_STATUS_DISABLED,
	PWM_STATUS_FAULT
}PWM_STATUS_E; 
volatile PWM_STATUS_E PWM_Status = PWM_STATUS_REINITED;
void SERVO_writeMicroseconds(uint16_t newPulseWidth_us)
{
	requestedPulseWidth_us = newPulseWidth_us;
}
void BOARD_FTM_HANDLER(void)
{
    /* Clear interrupt flag.*/
    FTM_ClearStatusFlags(BOARD_FTM_BASEADDR, kFTM_TimeOverflowFlag);
    ftmIsrFlag = true;
		PWM_microsecondCounts ++;
		//state machine
		switch(PWM_Status)
		{
			case (PWM_STATUS_UNKNOWN):
				//DO NOTHING
				break;

			case (PWM_STATUS_REINITED):
				//toggle High
				pulseWidth_us = requestedPulseWidth_us;
				GO_TO_NEXT_STATE(PWM_STATUS_UPDATED);
				break;

			case (PWM_STATUS_UPDATED):
				//toggle High
        GPIO_PinWrite(SERVO_GPIO_PORT, SERVO_GPIO_PIN, 1);
				PWM_microsecondCounts = 0;
				PWM_Status = (PWM_STATUS_ENABLED);
				break;

			case (PWM_STATUS_ENABLED):
				if(PWM_microsecondCounts*US_PER_TICK >= (pulseWidth_us))
				{
				  GPIO_PinWrite(SERVO_GPIO_PORT, SERVO_GPIO_PIN, 0);
				  PWM_Status = (PWM_STATUS_DISABLED);
				}
				break;

			case (PWM_STATUS_DISABLED):
				if(PWM_microsecondCounts*US_PER_TICK >= (REFRESHING_PERIOD))
				{
					if(pulseWidth_us != requestedPulseWidth_us)
					{
					  PWM_Status = (PWM_STATUS_REINITED);
					}
					else
					{
					  PWM_Status = (PWM_STATUS_UPDATED);
					}
				}
				break;

			case (PWM_STATUS_FAULT):
			default:
				// Toggle Low
			  GPIO_PinWrite(SERVO_GPIO_PORT, SERVO_GPIO_PIN, 0);
				break;
		}

    __DSB();
}
#endif
/*********************** 
 ********* APP ********* 
 **********************/
/*
 * @brief   Application entry point.
 */
int main(void) {
#ifdef TEMPORARY_TEST
      BOARD_InitPins();
      BOARD_BootClockRUN();
      BOARD_InitDebugConsole();

#else

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
	memset(&m_data, 0, sizeof(m_data));
	/* Init rf24 */
#if (ENABLE_TASK_RF24)
	m_data.rf24_ce.port = RF24_COMMON_DEFAULT_CE_PORT;
	m_data.rf24_ce.pin = RF24_COMMON_DEFAULT_CE_PIN;
	memcpy(m_data.rf24_address, RF24_COMMON_ADDRESS, sizeof(char)*RF24_COMMON_ADDRESS_SIZE);
#endif // (ENABLE_TASK_RF24)

#if(TEST_FTM_PWM)
	// INIT =========
	/* Define the init structure for the output LED pin*/
		gpio_pin_config_t servo_motor_gpio_config = {
			kGPIO_DigitalOutput, 0,
		};

	// INIT Servo PWM GPIO Pin
	GPIO_PinInit(SERVO_GPIO_PORT, SERVO_GPIO_PIN, &servo_motor_gpio_config );

	// CODE ============
	uint32_t cnt;
	uint32_t loop = 4U;
	uint32_t secondLoop = 10000U;//10us
 	const char *signals = "-|";
	ftm_config_t ftmInfo;
	FTM_GetDefaultConfig(&ftmInfo);
	/* Divide FTM clock by 4 */
	ftmInfo.prescale = kFTM_Prescale_Divide_4;
	/* Initialize FTM module */
	FTM_Init(BOARD_FTM_BASEADDR, &ftmInfo);
	/*
	* Set timer period.
	*/
	FTM_SetTimerPeriod(BOARD_FTM_BASEADDR, USEC_TO_COUNT(US_PER_TICK, FTM_SOURCE_CLOCK)); // 10 usec

	FTM_EnableInterrupts(BOARD_FTM_BASEADDR, kFTM_TimeOverflowInterruptEnable);

	EnableIRQ(BOARD_FTM_IRQ_NUM);

	FTM_StartTimer(BOARD_FTM_BASEADDR, kFTM_SystemClock);
	DEBUG_PRINT_INFO(" ****** Counter begin ******");
	cnt = 0;
	volatile uint16_t temp = 600U;
	bool increment = true;
    while (true)
    {
        if (ftmIsrFlag)
        {
            milisecondCounts++;
            ftmIsrFlag = false;
            if (milisecondCounts >= secondLoop)//100ms
            {
//                DEBUG_PRINT_INFO(" %c", signals[cnt & 1]);
                cnt++;
                if (cnt >= loop)
                {
                    cnt = 0;
                }
                milisecondCounts = 0U;
//                switch(cnt)
//                {
//                  case 0:
//                    SERVO_writeMicroseconds(1500U);//1250
//                    break;
//
//                  case 1:
//                    SERVO_writeMicroseconds(2400U);
//                    break;
//
//                  case 2:
//                    SERVO_writeMicroseconds(1500U);
//                    break;
//
//                  default:
//                    SERVO_writeMicroseconds(600U);
//                    break;
//                }
                if(temp>=2400)
                  increment = false;
                else if (temp<=600)
                  increment = true;
                temp += increment?(100):(-100);
                SERVO_writeMicroseconds(temp);
            }
        }
        __WFI();
    }
#endif
	/*---- CONFIG --------------------------------------------------------*/
	 DEBUG_PRINT_INFO(" ****** Hummingboard Config ... ******");
	/* Config rf24 */
#if (ENABLE_TASK_RF24)
   RF24_config(&m_data.rf24_ce);
   RF24_INIT_STATUS_E status = RF24_init();
   if(status == RF24_INIT_STATUS_SUCCESS)
   {
     RF24_setDataRate( RF24_250KBPS );//low data rate => longer range and reliable
     RF24_enableAckPayload();
     RF24_setRetries(3,2);
     RF24_openReadingPipe(0, m_data.rf24_address);
     RF24_setPALevel(RF24_PA_LOW);
     RF24_startListening();
//     RF24_openReadingPipe(0, m_data.rf24_address);
//     RF24_setPALevel(RF24_PA_MIN);
//     RF24_startListening();
   }
	 else
	 {
			DEBUG_PRINT_ERR(" Failed to configure RF24 Module!");
	 }

 // TODO: remove these testing code
//  uint8_t temp = 10;
//  RF24_DEBUG_spiTestingCode();
//   while(1){
//     DEBUG_PRINT_INFO(" Ticking ...");
//     write_register_buf(1, &temp, 1);
//   }
#endif
		/* Config vehicle control steering & motoring */
#if (ENABLE_TASK_VEHICLE_CONTROL)
	 /* Define the init structure for the output LED pin*/
	   gpio_pin_config_t servo_motor_gpio_config = {
	     kGPIO_DigitalOutput, 0,
	   };

	 // INIT Servo PWM GPIO Pin
	 GPIO_PinInit(SERVO_GPIO_PORT, SERVO_GPIO_PIN, &servo_motor_gpio_config );
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
	if (xTaskCreate(task_steeringControl, "task_steeringControl", configMINIMAL_STACK_SIZE + 10, NULL, ENABLE_TASK_VEHICLE_CONTROL_PRIORITY, NULL) != pdPASS)
	  {
	    DEBUG_PRINT_ERR("Task creation failed!.\r\n");
	    while (1);
	  }
#endif //(ENABLE_TASK_VEHICLE_CONTROL)


	/*---- TASK SCHEDULAR START --------------------------------------------------------*/
  DEBUG_PRINT_INFO(" ****** Hummingboard Running ******");
  DEBUG_PRINT_INFO(" ****** ******************* ******");
	vTaskStartScheduler();
#endif
	return 0;
}

