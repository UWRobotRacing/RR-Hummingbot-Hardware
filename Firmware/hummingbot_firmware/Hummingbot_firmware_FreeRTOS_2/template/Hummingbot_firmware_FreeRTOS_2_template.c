/*
 * Hummingbot_firmware_FreeRTOS_2_template.c
 *
 *  Created on: May 17, 2019
 *      Author: jackxu
 */



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

/* TODO: insert other definitions and declarations here. */
/************* Macro Preference ***************/
#define USE_NRF24L01          1
/************* Definitions ***************/
//#define xDelay (TickType_t)(1000 / portTICK_PERIOD_MS)
#define my_task_PRIORITY (configMAX_PRIORITIES - 1)


/**
 *              SERVO_DEFINITIONS
 */
//#define SERVO_POWER_7V

#define SERVO_PWM_PERIOD_MS       (uint8_t)20
#define SERVO_PWM_PERIOD_TICKS      (int) (SERVO_PWM_PERIOD_MS * ((float)configTICK_RATE_HZ / 1000))
//#define SERVO_MIN_DUTY_CYCLE      (float)5.54 //55 degrees
//#define SERVO_MAX_DUTY_CYCLE      (float)8.64 //115 degrees
#define SERVO_MIN_ANGLE         60
#define SERVO_MAX_ANGLE         110
#define SERVO_CONVERT_CYCLE_2_TICKS(c)  (int)(SERVO_PWM_PERIOD_MS * (c / (float) 100) * (configTICK_RATE_HZ / 1000))
#define SERVO_CONVERT_ANGLE_2_TICKS(a)  SERVO_CONVERT_CYCLE_2_TICKS((float)(2.6986 + (7.346 - 2.6986) / 90.0 * a))
#define SERVO_MIN_TICKS         SERVO_CONVERT_ANGLE_2_TICKS(SERVO_MIN_ANGLE)
#define SERVO_MAX_TICKS         SERVO_CONVERT_ANGLE_2_TICKS(SERVO_MAX_ANGLE)
// Servo connected to SERVO LEFT connector on hummingboard.
#define SERVO_GPIO_PORT         GPIOB
#define SERVO_GPIO_PIN          12U

/**
 *              ESC_DEFINITIONS
 */
#define ESC_PWM_PERIOD_MS         (uint8_t) 21
#define ESC_PWM_PERIOD_TICKS        (int) (ESC_PWM_PERIOD_MS * ((float)configTICK_RATE_HZ / 1000))
#define ESC_CONVERT_CYCLE_2_TICKS(c)    (int) (ESC_PWM_PERIOD_MS * (c / (float) 100) * (configTICK_RATE_HZ / 1000))
#define ESC_GPIO_PORT           GPIOA
#define ESC_GPIO_PIN            17U
// TODO: FIX THIS FUNCTION
#define ESC_CONVERT_MS_HIGH_2_TICKS(ms)   ESC_CONVERT_CYCLE_2_TICKS(ms/(float)ESC_PWM_PERIOD_MS)

static void task_steering_control(void *pvParameters);
#if 0
  static void task_motor_control(void *pvParameters);
  static void task_uart_receive(void *pvParameters);
#endif
volatile char inputCh = ' ';
volatile int inputChChanged = 1;

uint16_t m_buf[2]= {0};

void lpuart1_callback (LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData) {
  inputChChanged = 1;
  GPIO_PortToggle(GPIOC, 1u << 12U);
}

//void LPUART1_RX_IRQHandler(void) {
////  inputCh = LPUART_ReadByte(LPUART1);
////  PRINTF("%c", inputCh);
////  inputChChanged = 1;
//  GPIO_PortToggle(GPIOC, 1u << 12U);
//}

//static void task_uart_receive(void *pvParameters) {
//
//}
#if 1
static void task_steering_control(void *pvParameters)
{
  while(1) {
#if DEMO_PWM
    //toggle High
    GPIO_PortSet(ESC_GPIO_PORT, 1u << ESC_GPIO_PIN);

    //delay ticks
//    vTaskDelay(ESC_CONVERT_MS_HIGH_2_TICKS(1.560));
//    vTaskDelay(ESC_CONVERT_CYCLE_2_TICKS(7.428));
    vTaskDelay(ESC_CONVERT_CYCLE_2_TICKS(9));

    // Toggle Low
    GPIO_PortClear(ESC_GPIO_PORT, 1u << ESC_GPIO_PIN);

     //delay 20ms - ticks
     vTaskDelay(ESC_PWM_PERIOD_TICKS - ESC_CONVERT_CYCLE_2_TICKS(9));
#endif


    vTaskDelay(20);
  }
}
#endif

#if 0
/*!
 * @brief Task responsible for outputting PWM signal to steering servo.
 */
static void task_steering_control(void *pvParameters)
{
//    for (;;)
//    {
//        PRINTF("Hello World! In new task!\r\n");
////        vTaskSuspend(NULL);
//
//   }

  // WARNING: DO NOT ADD ANY vTaskDelay() OTHER THAN FOR
  // PWM PURPOSES (PULL HIGH AND LOW) AS THAT WILL MESS UP THE PWM CYCLE.

  int angle = 85;
  lpuart_handle_t lpuart1_handle;
  lpuart1_handle.callback = lpuart1_callback;
  lpuart_transfer_t lpuart1_transfer;
  lpuart1_transfer.data = &inputCh;
  lpuart1_transfer.dataSize = 1;
  size_t lpuart1_rec_bytes = 0;

  while(1) {
//    if(inputChChanged) {
//      if(inputCh == 0x61 && angle > SERVO_MIN_ANGLE) {//a
//        angle = angle - 5 >= SERVO_MIN_ANGLE ? angle - 5 : angle;
//      } else if(inputCh == 0x64 && angle < SERVO_MAX_ANGLE) {//d
//        angle = angle + 5 <= SERVO_MAX_ANGLE ? angle + 5 : angle;
//      }
//      inputChChanged = 0;
//      LPUART_TransferReceiveNonBlocking(LPUART1, &lpuart1_handle, &lpuart1_transfer, &lpuart1_rec_bytes);
//    }

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
#endif

/***
 *
 *  CONFIGURATION nrf24l01
 *
 ***/
#if USE_NRF24L01
  pin_t nrf24_ce = {GPIOA, 0};
  const char address[6] = "00101";

  /****** Macros ****/
  // TODO: These should be under rf24_common.h shared btwn arduino&m4
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
#endif


/*
 * @brief   Application entry point.
 */
int notmain(void) {
    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
    BOARD_BootClockRUN();
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
//
//  /* Set systick reload value to generate 1ms interrupt */
//  if(SysTick_Config(SystemCoreClock / 1000U))
//  {
//    while(1)
//    {
//    }
//  }


#if USE_NRF24L01
    RF24_config(&nrf24_ce);
    RF24_INIT_STATUS_E status = RF24_init();
    if(status == RF24_INIT_STATUS_SUCCESS)
    {
      RF24_setDataRate( RF24_250KBPS );//low data rate => longer range and reliable
      RF24_enableAckPayload();
      RF24_setRetries(3,2);
      RF24_openReadingPipe(0, address);
      RF24_setPALevel(RF24_PA_HIGH);
      RF24_startListening();
    }
#endif

  /* Define the init structure for the output LED pin*/
  gpio_pin_config_t servo_motor_gpio_config = {
    kGPIO_DigitalOutput, 0,
  };
  /* Init output LED GPIO. */
   GPIO_PinInit(GPIOC, 12U, &servo_motor_gpio_config);
//   GPIO_PinInit(GPIOC, 13U, &led_config);

  // INIT Servo PWM GPIO Pin
//   GPIO_PinInit(GPIOC, 7U, &led_config);

  GPIO_PinInit(SERVO_GPIO_PORT, SERVO_GPIO_PIN, &servo_motor_gpio_config );
  GPIO_PinInit(ESC_GPIO_PORT, ESC_GPIO_PIN, &servo_motor_gpio_config );


   lpuart_config_t lpuartConfig;

   lpuartConfig.baudRate_Bps = 115200U;
   lpuartConfig.parityMode = kLPUART_ParityDisabled;
   lpuartConfig.stopBitCount = kLPUART_OneStopBit;
   lpuartConfig.txFifoWatermark = 0;
   lpuartConfig.rxFifoWatermark = 1;
   lpuartConfig.enableRx = true;
   lpuartConfig.enableTx = true;
   // TODO: optimize clock frequency
   //NOTE: clock frequency needs to match the clock register
   LPUART_Init(LPUART1, &lpuartConfig, 16000000U);
//   LPUART_EnableInterrupts(LPUART1,kLPUART_RxActiveEdgeInterruptEnable);

#if USE_NRF24L01
   switch(status)
   {
     case RF24_INIT_STATUS_SUCCESS:
       PRINTF(" + Successfully loaded RF24 module \r\n");
       break;
     case RF24_INIT_STATUS_SETUP_ERR:
       PRINTF("[E] Error loading RF24 module \r\n");
       break;
     case RF24_INIT_STATUS_CONFIG_ERR:
       PRINTF("[E] No Config. found for RF24 module \r\n");
       break;
     default:
       PRINTF("[E] UNKNOWN ERR \r\n");
       break;
   }
   // Testing case
   while(1)
   {
     if (RF24_available) {
         RF24_read(&m_buf, sizeof(m_buf));
         uint8_t temp1 = ((m_buf[1]) & 0xFF);
         uint8_t temp2 = m_buf[1]>>8;
         uint16_t temp3 = (m_buf[0]);
         if(temp3!=0)
         {
           PRINTF(temp1);
           PRINTF(",");
           PRINTF(temp2);
           PRINTF(",");
           PRINTF(temp3);
           PRINTF("\r\n");
         }else{
           PRINTF(temp3);
           PRINTF(" Invalid msg\r\n");
         }
       }
   }
#endif

  if (xTaskCreate(task_steering_control, "task_steering_control", configMINIMAL_STACK_SIZE + 10, NULL, my_task_PRIORITY, NULL) != pdPASS)
  {
    PRINTF("Task creation failed!.\r\n");
    while (1);
  }

  vTaskStartScheduler();
#if 0


  Init_PWM_Servo();
  int arr[5] = {15, 30, 45, 60, 75};
  while(1) {
    int i=0;
    for(;i<5; i++) {
      //delay 0.5 second
      vTaskDelay( xDelay );
      PWM_Servo_Angle(arr[i]);
    }
  }
#endif
#if 0
  //Servo angles can be stored in a look-up table for steering the car.
  float table[4] = { 0.0,
     30.0,
     60.0,
     90.0
  };

  float Steering_Angle = table[1];
  PWM_Servo_Angle(Steering_Angle); // Call PWM_Servo_Angle function.

#endif

#if 0
  uint8_t ADDR[] = { 'n', 'R', 'F', '2', '4' }; // the address for RX pipe
  nRF24_DisableAA(0xFF); // disable ShockBurst
  nRF24_SetRFChannel(90); // set RF channel to 2490MHz
  nRF24_SetDataRate(nRF24_DR_2Mbps); // 2Mbit/s data rate
  nRF24_SetCRCScheme(nRF24_CRC_1byte); // 1-byte CRC scheme
  nRF24_SetAddrWidth(5); // address width is 5 bytes
  nRF24_SetAddr(nRF24_PIPE1, ADDR); // program pipe address
  nRF24_SetRXPipe(nRF24_PIPE1, nRF24_AA_OFF, 10); // enable RX pipe#1 with Auto-ACK: disabled, payload length: 10 bytes
  nRF24_SetOperationalMode(nRF24_MODE_RX); // switch transceiver to the RX mode
  nRF24_SetPowerMode(nRF24_PWR_UP); // wake-up transceiver (in case if it sleeping)
  // then pull CE pin to HIGH, and the nRF24 will start a receive...
#endif
#if 0
  ftm_config_t ftmConfigStruct;
    ftm_chnl_pwm_signal_param_t pwmParam;
    ftm_pwm_level_select_t pwmLevel = kFTM_HighTrue;

    /* Initialize FTM module. */
    FTM_GetDefaultConfig(&ftmConfigStruct);
    ftmConfigStruct.extTriggers = kFTM_Chnl0Trigger; /* Enable to output the trigger. */
    FTM_Init(FTM0, &ftmConfigStruct);

    /* Configure ftm params with frequency 24kHz */
    pwmParam.chnlNumber = kFTM_Chnl_0;
    pwmParam.level = pwmLevel;
    pwmParam.dutyCyclePercent = 15; /* Percent: 0 - 100. */
    pwmParam.firstEdgeDelayPercent = 0U;
    FTM_SetupPwm(FTM0, &pwmParam, 1U, kFTM_CenterAlignedPwm, 10000U, CLOCK_GetFreq(kCLOCK_CoreSysClk));


  //  FTM_SetupOutputCompare(FTM0, 0U, kFTM_ToggleOnMatch, 0xFFFF);
    FTM_StartTimer(FTM0, kFTM_SystemClock);

  //    while (1)
//    {
//        GETCHAR();
//        /*
//        * Start the FTM counter and finally trigger the ADC12's conversion.
//        * FTM_StartTimer() -> PDB PreTrigger -> ADC conversion done interrupt -> FTM_StopTimer().
//        */
//        FTM_StartTimer(FTM0, DEMO_FTM_COUNTER_CLOCK_SOURCE);
//        FTM_SetupOutputCompare(FTM0, 0U, kFTM_ToggleOnMatch, 0xFFFF);
//    }
#endif
#ifdef SPI
  //:TODO : change the SPI communication freq
  //:TODO : try to modify the communication timing (CS trigger before CLK)

  lpspi_rtos_handle_t spi0_handle;
  lpspi_master_config_t spi0_master_config;
  lpspi_transfer_t spi0_transfer;

  LPSPI_MasterGetDefaultConfig(&spi0_master_config);

  //initialize the SPI0 configuration
  spi0_master_config.pcsActiveHighOrLow = kLPSPI_PcsActiveLow;
  spi0_master_config.baudRate = 500000U;
  spi0_master_config.pcsToSckDelayInNanoSec = 1000000000 / spi0_master_config.baudRate * 2;
  spi0_master_config.lastSckToPcsDelayInNanoSec = 1000000000 / spi0_master_config.baudRate * 2;
  spi0_master_config.betweenTransferDelayInNanoSec = 1000000000 / spi0_master_config.baudRate * 2;
  spi0_master_config.whichPcs = kLPSPI_Pcs3;
  spi0_master_config.direction = kLPSPI_MsbFirst;

  spi0_transfer.txData = calloc(20, sizeof(int));
  spi0_transfer.txData[0] = 'f';
  spi0_transfer.txData[1] = 'a';
  spi0_transfer.txData[2] = 'f';
  spi0_transfer.txData[3] = 'a';
  spi0_transfer.rxData = malloc(sizeof(int) * 20);
  LPSPI_RTOS_Init(&spi0_handle, LPSPI0, &spi0_master_config, 3000000U);
//  for(int i=0;i<10;i++) LPSPI_RTOS_Transfer(&spi0_handle, &spi0_transfer);
  while(1) LPSPI_RTOS_Transfer(&spi0_handle, &spi0_transfer);
  LPSPI_RTOS_Deinit(&spi0_handle);

  lpi2c_rtos_handle_t i2c0_handle;
  lpi2c_master_config_t i2c0_master_config;
  lpi2c_master_transfer_t i2c0_transfer;
  LPI2C_MasterGetDefaultConfig(&i2c0_master_config);
  i2c0_transfer.flags = kLPI2C_TransferDefaultFlag;
  i2c0_master_config.baudRate_Hz = 100000U;
  i2c0_transfer.slaveAddress = 0x71;
  i2c0_transfer.direction = kLPI2C_Write;
  i2c0_transfer.subaddress = 0xfafa0000;
  i2c0_transfer.subaddressSize = 2;
  char i2c0_data[] = {'b', 'a', 'b', 'e'};
  i2c0_transfer.data = (void *) i2c0_data;
  i2c0_transfer.dataSize = 4 * sizeof(char);
  LPI2C_RTOS_Init(&i2c0_handle, LPI2C0, &i2c0_master_config, 1000000U);
//  for(int i=0;i<10;i++) LPI2C_RTOS_Transfer(&i2c0_handle, &i2c0_transfer);
  while(1) LPI2C_RTOS_Transfer(&i2c0_handle, &i2c0_transfer);
  LPI2C_RTOS_Deinit(&i2c0_handle);

    PRINTF("Hello World\n");
    if (xTaskCreate(my_task, "my_task", configMINIMAL_STACK_SIZE + 10, NULL, my_task_PRIORITY, NULL) != pdPASS)
  {
    PRINTF("Task creation failed!.\r\n");
    while (1);
  }

  vTaskStartScheduler();

    /* Force the counter to be placed into memory. */
    volatile static int i = 0 ;
    /* Enter an infinite loop, just incrementing a counter. */
    while(1) {
        i++ ;
    }
#endif
    return 0 ;
}


