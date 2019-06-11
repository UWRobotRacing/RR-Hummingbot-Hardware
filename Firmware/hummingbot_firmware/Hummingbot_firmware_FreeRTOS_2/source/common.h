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

/*************************************
 ********* Macro Definitions **********
 *************************************/
#define SAME_GPIO(x, y)     (((x).port==(y).port) && ((x).pin==(y).pin))
#define LOW                 (0)
#define HIGH                (1)
/*************************************
 ********* Struct Definitions ********
 *************************************/
typedef struct{
    GPIO_Type*    port;
    uint32_t      pin;
}pin_t;

typedef struct{
    lpspi_rtos_handle_t   spi0_handle;
    lpspi_master_config_t spi0_master_config;
    lpspi_transfer_t      spi0_transfer;
}lpspi_t;

/*************************************
 ********* ENUM Definitions **********
 *************************************/


#endif /* COMMON_H_ */
