#ifndef SERVO_H_
#define SERVO_H_
#include "common.h"

/*******************************************************************************
 * typedef
 ******************************************************************************/
typedef struct{
    pin_t       gpio;
    uint16_t    refreshingPeriod;
    uint16_t    defaultPulseWidth_us; //for neutral position, ex: neutral position for a positional servo
    uint16_t    minPulseWidth_us;
    uint16_t    maxPulseWidth_us;
} SERVO_ServoConfig_S;

typedef enum 
{
	PWM_STATUS_IDLE,
	PWM_STATUS_INIT,
	PWM_STATUS_UPDATED,
	PWM_STATUS_ENABLED,
	PWM_STATUS_DISABLED,
	PWM_STATUS_COUNT,
  PWM_STATUS_UNKNOWN,
  PWM_STATUS_FAULT
} SERVO_PWM_STATUS_E; 
/*******************************************************************************
 * public function
 ******************************************************************************/
void SERVO_onDestroy(void);
bool SERVO_init(const SERVO_ServoConfig_S* configs, uint8_t size);

SERVO_PWM_STATUS_E SERVO_getStatus(uint8_t index);
bool SERVO_goDefault(uint8_t index);
bool SERVO_doStop(uint8_t index); //stop sending command
bool SERVO_requestStart(uint8_t index);
bool SERVO_write_us(uint8_t index, uint16_t pulseWidth_us);
bool SERVO_getNotifiedByNewTick(void);

#endif // (SERVO_H_)
