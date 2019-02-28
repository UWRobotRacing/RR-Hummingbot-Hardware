/*
 * Copyright 2016-2018 NXP Semiconductor, Inc.
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
 * @file    Hummingbot_firmware.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKE14F16.h"
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "fsl_lpuart.h"


/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */


volatile uint32_t g_systickCounter;

void SysTick_Handler(void)
{
    if (g_systickCounter != 0U)
    {
        g_systickCounter--;
    }
}

void SysTick_DelayTicks(uint32_t n)
{
    g_systickCounter = n;
    while(g_systickCounter != 0U)
    {
    }
}

/*
 * @brief   Application entry point.
 */
int main(void) {

 /* Define the init structure for the output LED pin*/
	gpio_pin_config_t led_config = {
		kGPIO_DigitalOutput, 0,
	};

  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();

    /* Board pin init */
	BOARD_InitPins();
	BOARD_BootClockRUN();

  	/* Init FSL debug console. */
	BOARD_InitDebugConsole();


    /* Init output LED GPIO. */
     GPIO_PinInit(GPIOC, 12U, &led_config);
     GPIO_PinInit(GPIOC, 13U, &led_config);

     /* Init LPUART Clock. */
    // CLOCK_SetLpuartClock(1U);

	/* Set systick reload value to generate 1ms interrupt */
	if(SysTick_Config(SystemCoreClock / 1000U))
	{
		while(1)
		{
		}
	}

	/////////////////////////////////////////////////////
	//////////////* UART TEST CODE*/////////////////////
    ///////////////////////////////////////////////////
#if 1
	uint8_t ch = 0;
	lpuart_config_t lpuartConfig;
	uint8_t txbuff[] = "Hello \r\n";
	//uint8_t rxbuff[20] = {0};

	lpuartConfig.baudRate_Bps = 9600U;
	lpuartConfig.parityMode = kLPUART_ParityDisabled;
	lpuartConfig.stopBitCount = kLPUART_OneStopBit;
	lpuartConfig.txFifoWatermark = 0;
	lpuartConfig.rxFifoWatermark = 1;
	lpuartConfig.enableRx = true;
	lpuartConfig.enableTx = true;

	// TODO: optimize clock frequency
	//NOTE: clock frequency needs to match the clock register
	LPUART_Init(LPUART1, &lpuartConfig, 16000000U);

#endif

	 ////////////////////////////////////////////////////////////
	 ////////////////////////////////////////////////////////////


	// TODO: make printf work
	//PRINTF("Tsugumi was here!!\n");

    while(1) {

    	//delay 1 second
        SysTick_DelayTicks(1000U);

        //toggle LED
        GPIO_PortToggle(GPIOC, 1u << 12U);

        //delay 1 second
        SysTick_DelayTicks(1000U);

       // if (kStatus_LPUART_TxBusy)
        GPIO_PortToggle(GPIOC, 1u << 13U);

        //delay 1 second
        SysTick_DelayTicks(1000U);

        GPIO_PortToggle(GPIOC, 1u << 12U);
        GPIO_PortToggle(GPIOC, 1u << 13U);

        //UART transmit (polling)
        LPUART_WriteBlocking(LPUART1, txbuff, sizeof(txbuff)-1);
    }
    return 0 ;
}
