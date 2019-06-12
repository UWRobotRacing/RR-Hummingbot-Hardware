/*
 * servo.c
 *
 *  Created on: Apr 28, 2019
 *      Author: jackxu
 */
#include "vehicleController.h"

#include "../Servo/Servo.h"
#include "Hummingconfig.h"
#include "common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define SET_ERR_FLAG(err)         (m_data.errorFlags |=(1U<<err))
#define CLEAR_ERR_FLAG(err)       (m_data.errorFlags &=~(1U<<err))

/*******************************************************************************
 * typedef
 ******************************************************************************/
typedef struct{
    pulse_us_t   pw_us;//we are assuming neutral is 0 degree
    angle_deg_t  angle_deg;
} VC_rotation_pair_t;

typedef struct{
    pulse_us_t          pw_us;//we are assuming neutral is 0 degree
    speed_mm_per_s_t    speed_mm_per_s;
} VC_speed_pair_t;

typedef struct{
    VC_rotation_pair_t neutral;//we are assuming neutral is 0 degree
    VC_rotation_pair_t max;
    VC_rotation_pair_t min;
} VC_steerCalibration_S;

typedef struct{
    VC_speed_pair_t max_FWD_hardLimit;
    VC_speed_pair_t max_FWD_softLimit;
    VC_speed_pair_t min_FWD_starting; //due to friction, it requires certain power to move
    VC_speed_pair_t braking;  
} VC_throttleCalibration_S;

typedef enum{
    VC_CHANNEL_NAME_STEER,
    VC_CHANNEL_NAME_THROTTLE,
    VC_CHANNEL_NAME_COUNT,
} VC_channnelName_E;

typedef struct{
    SERVO_ServoConfig_S         deviceConfigs[VC_CHANNEL_NAME_COUNT];
    const VC_steerCalibration_S*     steering_config;
    const VC_throttleCalibration_S*  throttle_config;
    us_per_deg_t                us_per_deg;
    us_s_per_mm_t               us_s_per_mm;
    VC_state_E                  state;   
    uint16_t                    errorFlags;                
} VehicleController_data_S;


/*******************************************************************************
 * private variables
 ******************************************************************************/
static VehicleController_data_S    m_data = {0};
// TODO: please calibrate values
static const VC_steerCalibration_S    frsky_servo_calib ={
    .neutral = {
        .pw_us = 1500U,
        .angle_deg = 0,
    },
    .max = {
        .pw_us = 2200U,
        .angle_deg = 90,
    },
    .min = {
        .pw_us = 800U,
        .angle_deg = -90,
    },
};
// TODO: please calibrate these values
static const VC_throttleCalibration_S onyx_bldc_esc_calib ={
    .max_FWD_hardLimit = {
        .pw_us = 2000U,
        .speed_mm_per_s = 10,
    }, 
    .max_FWD_softLimit = {
        .pw_us = 1800U,
        .speed_mm_per_s = 8,
    }, 
    .min_FWD_starting = {
        .pw_us = 1300U,
        .speed_mm_per_s = 1,
    }, 
    .braking = {
        .pw_us = 1250,
        .speed_mm_per_s = 0,
    }, 
};
/*******************************************************************************
 * private function prototypes
 ******************************************************************************/
void VC_onDestroy(void)
{
    m_data.steering_config = NULL;
    m_data.throttle_config = NULL;
    // free all child modules
    SERVO_onDestroy();
    m_data.state = VC_STATE_DESTROYED;
}

void VC_Config(void)
{
    m_data.steering_config = &frsky_servo_calib;
    m_data.throttle_config = &onyx_bldc_esc_calib;
    // compute scale factor
    m_data.us_per_deg = (m_data.steering_config->max.pw_us - m_data.steering_config->min.pw_us)/
        (m_data.steering_config->max.angle_deg - m_data.steering_config->min.angle_deg);
    m_data.us_s_per_mm = (m_data.throttle_config->max_FWD_softLimit.pw_us - m_data.throttle_config->min_FWD_starting.pw_us)/
        (m_data.throttle_config->max_FWD_softLimit.speed_mm_per_s - m_data.throttle_config->min_FWD_starting.speed_mm_per_s);

    m_data.deviceConfigs[VC_CHANNEL_NAME_STEER].gpio.pin = HUMMING_CONFIG_EXAMPLE_GPIO_PIN;
    m_data.deviceConfigs[VC_CHANNEL_NAME_STEER].gpio.port= HUMMING_CONFIG_EXAMPLE_GPIO_PORT;
    m_data.deviceConfigs[VC_CHANNEL_NAME_STEER].refreshingPeriod = HUMMING_CONFIG_EXAMPLE_PWM_PERIOD;
    m_data.deviceConfigs[VC_CHANNEL_NAME_STEER].defaultPulseWidth_us =	HUMMING_CONFIG_EXAMPLE_DEFAULT_PW;
    m_data.deviceConfigs[VC_CHANNEL_NAME_STEER].minPulseWidth_us =	HUMMING_CONFIG_EXAMPLE_MIN_PW;
    m_data.deviceConfigs[VC_CHANNEL_NAME_STEER].maxPulseWidth_us =	HUMMING_CONFIG_EXAMPLE_MAX_PW;

    m_data.deviceConfigs[VC_CHANNEL_NAME_THROTTLE].gpio.pin = HUMMING_CONFIG_EXAMPLE_GPIO_PIN;
    m_data.deviceConfigs[VC_CHANNEL_NAME_THROTTLE].gpio.port= HUMMING_CONFIG_EXAMPLE_GPIO_PORT;
    m_data.deviceConfigs[VC_CHANNEL_NAME_THROTTLE].refreshingPeriod = HUMMING_CONFIG_EXAMPLE_PWM_PERIOD;
    m_data.deviceConfigs[VC_CHANNEL_NAME_THROTTLE].defaultPulseWidth_us =	HUMMING_CONFIG_EXAMPLE_DEFAULT_PW;
    m_data.deviceConfigs[VC_CHANNEL_NAME_THROTTLE].minPulseWidth_us =	HUMMING_CONFIG_EXAMPLE_MIN_PW;
    m_data.deviceConfigs[VC_CHANNEL_NAME_THROTTLE].maxPulseWidth_us =	HUMMING_CONFIG_EXAMPLE_MAX_PW;
    
    m_data.state = VC_STATE_CONFIGED;
}

bool VC_Init(void)
{
    if(m_data.state == VC_STATE_CONFIGED)
    {
	    SERVO_init(m_data.deviceConfigs, VC_CHANNEL_NAME_COUNT);
        m_data.state = VC_STATE_INITED;
    }
    return (m_data.state == VC_STATE_INITED);
}

bool VC_Begin(void)
{
    bool status = true;
    if(m_data.state == VC_STATE_INITED)
    {
        status &= SERVO_requestStart(VC_CHANNEL_NAME_STEER);
        status &= SERVO_requestStart(VC_CHANNEL_NAME_THROTTLE);
        if(status)
        {
            m_data.state = VC_STATE_IDLE;
        }
    }
    return (m_data.state == VC_STATE_IDLE);
}

void VC_dummyTestRun(void)
{
  uint32_t cnt = 0;
	uint32_t loop = 4U;
	uint32_t milisecondCounts = 0;
	uint32_t secondLoop = 10000U;//10us
	volatile uint16_t temp = 600U;
	bool increment = true;
    while (true)
    {
        if (SERVO_getNotifiedByNewTick())
        {
            milisecondCounts++;
            if (milisecondCounts >= secondLoop)
            {
                cnt++;
                if (cnt >= loop)
                {
                    cnt = 0;
                }
                milisecondCounts = 0U;
                if(temp>=2400)
                  increment = false;
                else if (temp<=600)
                  increment = true;
                temp += increment?(100):(-100);
                SERVO_write_us(0, temp);
            }
        }
        __WFI();
    }
}

/*******************************************************************************
 * public function
 ******************************************************************************/

/*******************************************************************************
 * private function
 ******************************************************************************/
