#ifndef HUMMING_CONFIG_H_
#define HUMMING_CONFIG_H_

/*************************************
 ****** Servo Config Preference ******
 *************************************/
#define HUMMING_CONFIG_EXAMPLE_PWM_PERIOD             (3003) //us //333Hz
#define HUMMING_CONFIG_EXAMPLE_MAX_PW                 (2400) //us 
#define HUMMING_CONFIG_EXAMPLE_DEFAULT_PW             (1500) //us 
#define HUMMING_CONFIG_EXAMPLE_MIN_PW                 (600) //us 
#define HUMMING_CONFIG_EXAMPLE_GPIO_PORT         	  (GPIOE)
#define HUMMING_CONFIG_EXAMPLE_GPIO_PIN          	  (13U) 
// #define HUMMING_CONFIG_EXAMPLE_MIN_ANGLE         	  (60)
// #define HUMMING_CONFIG_EXAMPLE_MAX_ANGLE         	  (110)



// General Robot Preference
#define HUMMING_CONFIG_BOT_LOST_CONTROLLER_TIMEOUT_MS       (500) //ms
#define HUMMING_CONFIG_BOT_UNSTABLE_RF_COMM_MIN_CNTS        (10)  // continuous 10 counts will be regaed as unstable transmission
#define HUMMING_CONFIG_BOT_RF24_SEMAPHORE_LOCK_MAX_TICK     (100)  //max 100 ticks
#endif //(HUMMING_CONFIG_H_)