/*
 * Common.h
 *
 *  Created on: Apr 30, 2019
 *      Author: jackxu
 */

#ifndef COMMON_H_
#define COMMON_H_

#include "fsl_gpio.h"

typedef struct{
    GPIO_Type*    port;
    uint32_t      pin;
}pin_t;

#define SAME_GPIO(x, y) (((x)->port==(y)->port) && ((x)->pin==(y)->pin))


#endif /* COMMON_H_ */
