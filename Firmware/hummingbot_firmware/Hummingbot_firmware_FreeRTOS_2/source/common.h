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
typedef enum {
    HUMMING_STATUS_BIT_RF24_ALIVE        =0U,
    HUMMING_STATUS_BIT_RF24_COMM_STABLE  =1U,
    HUMMING_STATUS_BIT_RF24_ONLINE       =2U,
    HUMMING_STATUS_BIT_VC_ALIVE          =3U,
    HUMMING_STATUS_BIT_AUTO_MODE         =4U,
    HUMMING_STATUS_BIT_REMOTE_ESTOP      =5U,
    //--------UNUSED--------//
    HUMMING_STATUS_BIT_UNUSED6                =6U,
    HUMMING_STATUS_BIT_UNUSED7                =7U
} HUMMING_STATUS_BIT_E; //8 bit

#endif /* COMMON_H_ */
