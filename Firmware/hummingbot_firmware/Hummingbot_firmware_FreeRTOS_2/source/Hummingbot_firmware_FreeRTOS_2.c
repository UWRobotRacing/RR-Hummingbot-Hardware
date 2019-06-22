
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
#define ENABLE_TASK_RF24				0
#define ENABLE_PWM_STEERING_SERVO		0
#define ENABLE_FEATURE_DEBUG_PRINT      1
#define ENABLE_UART_TEST				1
#define ENABLE_LED						1

// Servo Macros
#define SERVO_PWM_PERIOD_MS       (uint8_t)20
#define SERVO_MIN_ANGLE         60
#define SERVO_MAX_ANGLE         110
#define SERVO_GPIO_PORT         GPIOB
#define SERVO_GPIO_PIN          12U // Servo connected to SERVO LEFT connector on hummingboard.

// LED Macro
#define LED_GPIO_PORT			GPIOC
#define LED_GPIO_PIN0			12U
#define LED_GPIO_PIN1			13U
/*************************************  
 ********* Macro Definitions ********** 
 *************************************/
#define TASK_RF24_PRIORITY 							(configMAX_PRIORITIES - 1)

//Servo Definitions
#define SERVO_PWM_PERIOD_TICKS      (int) (SERVO_PWM_PERIOD_MS * ((float)configTICK_RATE_HZ / 1000))
#define SERVO_CONVERT_CYCLE_2_TICKS(c)  (int)(SERVO_PWM_PERIOD_MS * (c / (float) 100) * (configTICK_RATE_HZ / 1000))
#define SERVO_CONVERT_ANGLE_2_TICKS(a)  SERVO_CONVERT_CYCLE_2_TICKS((float)(2.6986 + (7.346 - 2.6986) / 90.0 * a))
#define SERVO_MIN_TICKS         SERVO_CONVERT_ANGLE_2_TICKS(SERVO_MIN_ANGLE)
#define SERVO_MAX_TICKS         SERVO_CONVERT_ANGLE_2_TICKS(SERVO_MAX_ANGLE)

#define TASK_PWM_STEERING_SERVO_PRIORITY 							(configMAX_PRIORITIES - 1)
#define TASK_UART_TEST 							(configMAX_PRIORITIES - 1)

/* Debug PRINTF helper functions */
#define DEBUG_PRINT_ERR(fmt, ...) \
            do { if (ENABLE_FEATURE_DEBUG_PRINT) PRINTF("[ERR.] " fmt "\r\n", ##__VA_ARGS__); } while (0)
#define DEBUG_PRINT_WRN(fmt, ...) \
            do { if (ENABLE_FEATURE_DEBUG_PRINT) PRINTF("[WARN] " fmt "\r\n", fmt, ##__VA_ARGS__); } while (0)
#define DEBUG_PRINT_INFO(fmt, ...) \
            do { if (ENABLE_FEATURE_DEBUG_PRINT) PRINTF("[INFO] " fmt "\r\n", fmt, ##__VA_ARGS__); } while (0)

/***************************************  
 *********  Struct/Enums Defs ********** 
 ***************************************/
typedef struct{
	uint16_t 	rf24_buf[2]; //[MSB] 12 bit (steer) | 12 bit (spd) | 8 bit (6 bit pattern + 2 bit modes)
  pin_t 		rf24_ce;
  char 			rf24_address[RF24_COMMON_ADDRESS_SIZE];
}Hummingbot_firmware_FreeRTOS_2_S;
//TODO: change name of struct??

typedef struct{
	int8_t steering_angle;
	uint16_t ESC_speed;
	uint8_t flags;
}hummingbot_uart_handle_t;

/***************************************  
 *********  Private Variable ********** 
 ***************************************/
Hummingbot_firmware_FreeRTOS_2_S m_data;
lpuart_handle_t lpuart1_handle, lpuart0_handle;
volatile char ready_for_next_transmit;
/************************************************  
 ********* Private Function Prototypes ********** 
 ***********************************************/
static void task_test_lpuart_asyncrhonous_echo(void *pvParameters);
void lpuart1_callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData);
/**************************************  
 ********* Private Functions ********** 
 *************************************/
static void task_test_lpuart_asyncrhonous_echo(void *pvParameters)
{
#if ENABLE_UART_TEST
//	char arr[10] = "hello\r\n";
	hummingbot_uart_handle_t humuart;
	//humuart.int1 = 0.0;
	//humuart.int2 = 0.0;
	humuart.steering_angle = 1;
	humuart.ESC_speed = 2;
	humuart.flags = 3;
	lpuart_transfer_t lpuart1_transfer;
	lpuart1_transfer.data = (uint8_t*) &humuart;
	lpuart1_transfer.dataSize = sizeof(humuart);
//	lpuart1_transfer.data = (uint8_t*) arr;
//	lpuart1_transfer.dataSize = 1;
	size_t bytesReceived = 0;
	uint8_t next_receive = 1;



	while(1) {
//		LPUART_TransferSendNonBlocking(LPUART1, &lpuart1_handle, &lpuart1_transfer);
		if(next_receive) {
			LPUART_TransferReceiveNonBlocking(LPUART1, &lpuart1_handle, &lpuart1_transfer, &bytesReceived);
			next_receive = 0;
		}
		if(ready_for_next_transmit) {
			/*
			if(humuart.int1 == 1.1) {
#if	ENABLE_LED
				GPIO_PortSet(LED_GPIO_PORT, 1U << LED_GPIO_PIN0);
#endif
			} else {
#if	ENABLE_LED
				GPIO_PortClear(LED_GPIO_PORT, 1U << LED_GPIO_PIN0);
#endif
			}
			if(humuart.int2 == 2.2) {
#if	ENABLE_LED
				GPIO_PortSet(LED_GPIO_PORT, 1U << LED_GPIO_PIN1);
#endif
			} else {
#if	ENABLE_LED
				GPIO_PortClear(LED_GPIO_PORT, 1U << LED_GPIO_PIN1);
#endif
			}
			*/
			LPUART_TransferSendNonBlocking(LPUART0, &lpuart0_handle, &lpuart1_transfer);

//			PRINTF("%d\n",humuart.steering_angle);
//			PRINTF("%d\n",humuart.ESC_speed);
//			PRINTF("%d\n",humuart.flags);


			next_receive = 1;
			ready_for_next_transmit = 0;
		}
		vTaskDelay(configTICK_RATE_HZ);
//		vTaskDelay(configTICK_RATE_HZ*2);
	}
#endif
}

void lpuart1_callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData)
{
#if ENABLE_LED
	GPIO_PortToggle(LED_GPIO_PORT, 1U << LED_GPIO_PIN0);
#endif
	if(status == kStatus_LPUART_RxIdle) {
		ready_for_next_transmit = 1;
	}
}

/*********************** 
 ********* APP ********* 
 **********************/
/*
 * @brief   Application entry point.
 */
int main(void) {
//  DEBUG_PRINT_INFO(" ****** Hummingboard begin ******"); //doing this breaks the code
	/*---- INIT --------------------------------------------------------*/
//  DEBUG_PRINT_INFO(" ****** Hummingboard Init ... ******"); //doing this breaks the code
	/* Init board hardware. */
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitBootPeripherals();
	BOARD_BootClockRUN();
	/* Init FSL debug console. */
	BOARD_InitDebugConsole();

#if ENABLE_TASK_RF24
	/* Init private data */
	memset(&m_data, 0, sizeof(m_data));
	/* Init rf24 */

	m_data.rf24_ce.port = RF24_COMMON_DEFAULT_CE_PORT;
	m_data.rf24_ce.pin = RF24_COMMON_DEFAULT_CE_PIN;
	memcpy(m_data.rf24_address, RF24_COMMON_ADDRESS, sizeof(char)*RF24_COMMON_ADDRESS_SIZE);
#endif 

	/*---- CONFIG --------------------------------------------------------*/
	 DEBUG_PRINT_INFO(" ****** Hummingboard Config ... ******");
	/* Config rf24 */
#if ENABLE_TASK_RF24
//    RF24_config(&nrf24_ce);
//    RF24_INIT_STATUS_E status = RF24_init();
//    if(status == RF24_INIT_STATUS_SUCCESS)
//    {
//      RF24_setDataRate( RF24_250KBPS );//low data rate => longer range and reliable
//      RF24_enableAckPayload();
//      RF24_setRetries(3,2);
//      RF24_openReadingPipe(0, address);
//      RF24_setPALevel(RF24_PA_HIGH);
//      RF24_startListening();
//    }
#endif

#if ENABLE_PWM_STEERING_SERVO
	 /* Define the init structure for the output LED pin*/
	   gpio_pin_config_t servo_motor_gpio_config = {
	     kGPIO_DigitalOutput, 0,
	   };

	 // INIT Servo PWM GPIO Pin
	 GPIO_PinInit(SERVO_GPIO_PORT, SERVO_GPIO_PIN, &servo_motor_gpio_config );
#endif

#if ENABLE_LED
	/* Define the init structure for the output LED pin*/
	gpio_pin_config_t led_config = {
		kGPIO_DigitalOutput, 0,
	};
	GPIO_PinInit(LED_GPIO_PORT, LED_GPIO_PIN0, &led_config);
	GPIO_PinInit(LED_GPIO_PORT, LED_GPIO_PIN1, &led_config);
#endif

#if ENABLE_UART_TEST
	ready_for_next_transmit = 0x00;

	lpuart_config_t lpuartConfig;
	lpuartConfig.baudRate_Bps = 115200U;
	lpuartConfig.parityMode = kLPUART_ParityDisabled;
	lpuartConfig.stopBitCount = kLPUART_OneStopBit;
	lpuartConfig.txFifoWatermark = 0;
	lpuartConfig.rxFifoWatermark = 0;
	lpuartConfig.enableRx = true;
	lpuartConfig.enableTx = true;

	// TODO: optimize clock frequency
	//NOTE: clock frequency needs to match the clock register
	LPUART_Init(LPUART1, &lpuartConfig, 16000000U);
	LPUART_TransferCreateHandle(LPUART1, &lpuart1_handle, lpuart1_callback, NULL);
	LPUART_Init(LPUART0, &lpuartConfig, 16000000U);
	LPUART_TransferCreateHandle(LPUART0, &lpuart0_handle, NULL, NULL);
#endif
	/*---- TASK CONFIGS --------------------------------------------------------*/
  DEBUG_PRINT_INFO(" ****** Hummingboard Config Tasks ... ******");
#if ENABLE_TASK_RF24
	if (xTaskCreate(task_rf24, "task_rf24", configMINIMAL_STACK_SIZE + 10, NULL, TASK_RF24_PRIORITY, NULL) != pdPASS)
	{
		DEBUG_PRINT_ERR("Task creation failed!.\r\n");
		while (1);
	}
#endif
#if ENABLE_PWM_STEERING_SERVO
	if (xTaskCreate(task_steering_control, "task_steering_control", configMINIMAL_STACK_SIZE + 10, NULL, TASK_PWM_STEERING_SERVO_PRIORITY, NULL) != pdPASS)
	  {
		DEBUG_PRINT_ERR("Task creation failed!.\r\n");
	    while (1);
	  }
#endif
#if ENABLE_UART_TEST
	if (xTaskCreate(task_test_lpuart_asyncrhonous_echo, "task_test_lpuart_asyncrhonous_echo", configMINIMAL_STACK_SIZE + 10, NULL, TASK_UART_TEST, NULL) != pdPASS)
	{
		DEBUG_PRINT_ERR("Task creation failed!.\r\n");
		while (1);
	}
#endif
	/*---- TASK SCHEDULAR START --------------------------------------------------------*/
	 DEBUG_PRINT_INFO(" ****** Hummingboard Running ******");
	vTaskStartScheduler(); // timer interrupts are enabled here (https://www.freertos.org/FreeRTOS_Support_Forum_Archive/May_2008/freertos_vTaskDelay_stuck_2052592.html)

	return 0;
}

