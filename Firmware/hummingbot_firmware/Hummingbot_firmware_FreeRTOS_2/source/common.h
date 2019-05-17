/*
 * Common.h
 *
 *  Created on: Apr 30, 2019
 *      Author: jackxu
 */

#ifndef COMMON_H_
#define COMMON_H_

#include "fsl_gpio.h"
#include "fsl_lpspi_freertos.h"

typedef struct{
    GPIO_Type*    port;
    uint32_t      pin;
}pin_t;

#define SAME_GPIO(x, y) (((x).port==(y).port) && ((x).pin==(y).pin))

typedef struct{
    lpspi_rtos_handle_t   spi0_handle;
    lpspi_master_config_t spi0_master_config;
    lpspi_transfer_t      spi0_transfer;
}lpspi_t;


#endif /* COMMON_H_ */
