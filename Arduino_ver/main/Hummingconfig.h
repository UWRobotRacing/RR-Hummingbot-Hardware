#ifndef HUMMING_CONFIG_H_
#define HUMMING_CONFIG_H_

/*
---------------- VC ----
MEGA 2560:
   ESC -> 5
   SERVO -> 4
---------------- NRF24L01 ----
MEGA 2560:
   MISO -> 50
   MOSI -> 51
   SCK -> 52
   CE -> 7
   CSN -> 8
   GND -> GND
   VCC -> 3.3v
 */
/*************************************
 ****** Servo Config Preference ******
 *************************************/
#define HUMMING_CONFIG_STEERING_BEBUG_LED_GPIO_PIN  (40U)
#define HUMMING_CONFIG_STEERING_SERVO_GPIO_PIN      (4U)
#define HUMMING_CONFIG_THROTTLE_ESC_GPIO_PIN        (5U)
#define HUMMING_CONFIG_RF24_CE          	          (7U)
#define HUMMING_CONFIG_RF24_CSN                     (8U)
// General Robot Preference
#define HUMMING_CONFIG_BOT_LOST_CONTROLLER_TIMEOUT_MS       (500) //ms
#define HUMMING_CONFIG_BOT_UNSTABLE_RF_COMM_MIN_CNTS        (100)  // continuous 10 counts will be regaed as unstable transmission
#define HUMMING_CONFIG_BOT_RF24_SEMAPHORE_LOCK_MAX_TICK     (10)  //max 100 ticks

#endif //(HUMMING_CONFIG_H_)
