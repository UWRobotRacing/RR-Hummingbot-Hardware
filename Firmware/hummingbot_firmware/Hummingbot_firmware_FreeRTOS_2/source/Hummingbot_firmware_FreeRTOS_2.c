
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
#include "fsl_debug_console.h"
#include "fsl_device_registers.h"
#include "fsl_gpio.h"
#include "fsl_lpuart.h"
#include "fsl_lpspi.h"
#include "fsl_lpspi_freertos.h"
#include "fsl_lpi2c.h"
#include "fsl_lpi2c_freertos.h"
/* TODO: insert other include files here. */
#include "FreeRTOS.h"
#include "task.h"

/* TODO: insert other definitions and declarations here. */



#define my_task_PRIORITY (configMAX_PRIORITIES - 1)

static void my_task(void *pvParameters);

/*!
 * @brief Task responsible for printing of "Hello world." message.
 */

static void my_task(void *pvParameters)
{
    for (;;)
    {
        PRINTF("Hello World! In new task!\r\n");
//        vTaskSuspend(NULL);

    }
}
/*
 * @brief   Application entry point.
 */
int main(void) {
  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
	BOARD_InitDebugConsole();

	lpspi_rtos_handle_t spi0_handle;
	lpspi_master_config_t spi0_master_config;
	lpspi_transfer_t spi0_transfer;
	LPSPI_MasterGetDefaultConfig(&spi0_master_config);
	spi0_master_config.pcsActiveHighOrLow = kLPSPI_PcsActiveHigh;
	spi0_master_config.baudRate = 100000U;
	spi0_master_config.whichPcs = kLPSPI_Pcs3;
	spi0_master_config.direction = kLPSPI_MsbFirst;
	spi0_transfer.txData = calloc(20, sizeof(int));
	spi0_transfer.txData[0] = 'f';
	spi0_transfer.txData[1] = 'a';
	spi0_transfer.txData[2] = 'f';
	spi0_transfer.txData[3] = 'a';
	spi0_transfer.rxData = malloc(sizeof(int) * 20);
	LPSPI_RTOS_Init(&spi0_handle, LPSPI0, &spi0_master_config, 3000000U);
//	for(int i=0;i<10;i++) LPSPI_RTOS_Transfer(&spi0_handle, &spi0_transfer);
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
//	for(int i=0;i<10;i++) LPI2C_RTOS_Transfer(&i2c0_handle, &i2c0_transfer);
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
    return 0 ;
}


