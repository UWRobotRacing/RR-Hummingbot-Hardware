#ifndef HUMMING_CONFIG_H_
#define HUMMING_CONFIG_H_

/*************************************
 ****** Servo Config Preference ******
 *************************************/
#define HUMMING_CONFIG_STEERING_SERVO_PWM_PERIOD             (3003) //us //333Hz
#define HUMMING_CONFIG_STEERING_SERVO_GPIO_PORT         	 (GPIOD)
#define HUMMING_CONFIG_STEERING_SERVO_GPIO_PIN          	 (7U)

#define HUMMING_CONFIG_THROTTLE_ESC_PWM_PERIOD                (3003) //us //333Hz
#define HUMMING_CONFIG_THROTTLE_ESC_GPIO_PORT                 (GPIOE)
#define HUMMING_CONFIG_THROTTLE_ESC_GPIO_PIN                  (13U) 

// General Robot Preference
#define HUMMING_CONFIG_BOT_LOST_CONTROLLER_TIMEOUT_MS       (500) //ms
#define HUMMING_CONFIG_BOT_UNSTABLE_RF_COMM_MIN_CNTS        (10)  // continuous 10 counts will be regaed as unstable transmission
#define HUMMING_CONFIG_BOT_RF24_SEMAPHORE_LOCK_MAX_TICK     (100)  //max 100 ticks
#endif //(HUMMING_CONFIG_H_)
