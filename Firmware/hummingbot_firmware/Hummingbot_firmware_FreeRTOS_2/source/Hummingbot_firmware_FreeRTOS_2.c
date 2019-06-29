
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
#define ENABLE_LED						0

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
	int16_t fuck;
	uint16_t me;
	uint8_t this;
	uint16_t rip;
}hummingbot_uart_handle_t;

/***************************************  
 *********  Private Variable ********** 
 ***************************************/
Hummingbot_firmware_FreeRTOS_2_S m_data;
lpuart_handle_t lpuart1_handle, lpuart0_handle;

uint8_t g_rxRingBuffer[20U] = {0}; /* RX ring buffer. */

char g_txBuffer[sizeof(hummingbot_uart_handle_t)] = {0};
char g_rxBuffer[sizeof(hummingbot_uart_handle_t)] = {0};

volatile bool rxBufferEmpty = true;
volatile bool txBufferFull = false;
volatile bool txOnGoing = false;
volatile bool rxOnGoing = false;



/************************************************  
 ********* Private Function Prototypes ********** 
 ***********************************************/
static void task_test_lpuart_asyncrhonous_echo(void *pvParameters);
void lpuart1_callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData);
void lpuart0_callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData);
void block_sync_uart();
/**************************************  
 ********* Private Functions ********** 
 *************************************/

void lpuart1_callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData)
{
	if(status == kStatus_LPUART_RxIdle) {
		rxBufferEmpty = false;
		rxOnGoing = false;
	}
}

void lpuart0_callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData)
{
	if(status == kStatus_LPUART_TxIdle){
		txBufferFull = false;
		txOnGoing = false;

	}
}

//void block_sync_uart()
//{
//	uint8_t not_synced = 1;
//	while(not_synced)
//	{
//
//	}
//}



static void task_test_lpuart_asyncrhonous_echo(void *pvParameters)
{
#if ENABLE_UART_TEST
	lpuart_transfer_t sendXfer;
	lpuart_transfer_t receiveXfer;
	size_t receivedBytes = 0U;
	uint8_t synced = 0U;
	uint8_t sync_bytes[1] = {0};

	LPUART_TransferStartRingBuffer(LPUART1, &lpuart1_handle, g_rxRingBuffer, 20U);

	receiveXfer.data = (uint8_t*) sync_bytes;
	receiveXfer.dataSize = sizeof(sync_bytes);


	while (!synced){
		if(!rxOnGoing){
			rxOnGoing = true;
			LPUART_TransferReceiveNonBlocking(LPUART1, &lpuart1_handle, &receiveXfer, &receivedBytes);
			if (sync_bytes[0] == 255) synced = 1;
		}
		vTaskDelay(configTICK_RATE_HZ/160);
	}

	sendXfer.data = (uint8_t*) g_txBuffer;
	sendXfer.dataSize = sizeof(hummingbot_uart_handle_t);
	receiveXfer.data = (uint8_t*) g_rxBuffer;
	receiveXfer.dataSize = sizeof(hummingbot_uart_handle_t);

	while(1) {
		/* If RX is idle and g_rxBuffer is empty, start to read data to g_rxBuffer. */
        if ((!rxOnGoing) && rxBufferEmpty)
        {
            rxOnGoing = true;
            LPUART_TransferReceiveNonBlocking(LPUART1, &lpuart1_handle, &receiveXfer, &receivedBytes);
            if (sizeof(hummingbot_uart_handle_t) == receivedBytes)
			{
				rxBufferEmpty = false;
				rxOnGoing = false;
			}
        }

        /* If TX is idle and g_txBuffer is full, start to send data. */
        if ((!txOnGoing) && txBufferFull)
        {
            txOnGoing = true;
            LPUART_TransferSendNonBlocking(LPUART0, &lpuart0_handle, &sendXfer);
        }

        /* If g_txBuffer is empty and g_rxBuffer is full, copy g_rxBuffer to g_txBuffer. */
        if ((!rxBufferEmpty) && (!txBufferFull))
        {
            memcpy(&g_txBuffer, &g_rxBuffer, sizeof(g_txBuffer));
//        	g_txBuffer.fuck = g_rxBuffer.fuck;
//        	g_txBuffer.me = g_rxBuffer.me;
//        	g_txBuffer.this = g_rxBuffer.this;

            rxBufferEmpty = true;
            txBufferFull = true;
        }
		vTaskDelay(configTICK_RATE_HZ/160);
//		vTaskDelay(configTICK_RATE_HZ*2);
	}
#endif
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

	lpuart_config_t config;

	LPUART_GetDefaultConfig(&config);
	config.baudRate_Bps = 115200U;
	config.enableTx = true;
	config.enableRx = true;

	// TODO: optimize clock frequency
	//NOTE: clock frequency needs to match the clock register
	LPUART_Init(LPUART1, &config, 16000000U);
	LPUART_TransferCreateHandle(LPUART1, &lpuart1_handle, lpuart1_callback, NULL);
	LPUART_Init(LPUART0, &config, 16000000U);
	LPUART_TransferCreateHandle(LPUART0, &lpuart0_handle, lpuart0_callback, NULL);

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

