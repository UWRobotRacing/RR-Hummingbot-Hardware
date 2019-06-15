/*
* Common definitions for configuring rf24 wireless modules btwn `Hummingboard` & `Controller`
*/
#ifndef RF24_COMMON_H
#define RF24_COMMON_H

//macro bit patterns
#define RF24_COMMON_MASK_UNIQUE_PATTERN   (0xA4) //8 bit for flags (6 pattern + 2 bit switch state)  101001XX
#define RF24_COMMON_MASK_BUFFER_FLAG      (0x000000FF)
#define RF24_COMMON_MASK_BUFFER_SPD       (0x000FFF00)
#define RF24_COMMON_MASK_BUFFER_STEER     (0xFFF00000)
#define RF24_COMMON_MASK_FLAG_ESTOP       (0x1)//01
#define RF24_COMMON_MASK_FLAG_AUTO        (0x2)//10
//macro access funcs
#define RF24_COMMON_GET_SPD(x)      (((x)&RF24_COMMON_MASK_BUFFER_SPD)>>8)
#define RF24_COMMON_GET_STEER(x)    (((x)&RF24_COMMON_MASK_BUFFER_STEER)>>20)
#define RF24_COMMON_GET_FLAG(x)     ((x)&RF24_COMMON_MASK_BUFFER_FLAG)
#define RF24_COMMON_CHECK_ESTOP_FLAG(x)   ((x)&RF24_COMMON_MASK_FLAG_ESTOP)
#define RF24_COMMON_CHECK_AUTO_FLAG(x)    ((x)&RF24_COMMON_MASK_FLAG_AUTO)
//macro default configs
#define RF24_COMMON_DEFAULT_CE_PORT       GPIOA
#define RF24_COMMON_DEFAULT_CE_PIN        11
#define RF24_COMMON_ADDRESS               "00101"//"00101"
#define RF24_COMMON_ADDRESS_SIZE          6

#endif
