/*
 * servo.c
 *
 *  Created on: Apr 28, 2019
 *      Author: jackxu
 * 
 * TODO: 
 *  1. we can implement hard/soft/variable braking 
 *  2. we can implement ** sports mode **
 *  3. we can implement position control here if needed
 *  4. trajectory based control, a timing based queued speed and angle pairs
 *  5. fancy drifting control
 *  6. with encoder feedback, we can do active PID control
 * 
 */
#include "vehicleController.h"

#include "../Servo/Servo.h"
#include "Hummingconfig.h"
#include "common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define SET_ERR_FLAG(err)         (m_vc.errorFlags |=(1U<<err))
#define CLEAR_ERR_FLAG(err)       (m_vc.errorFlags &=~(1U<<err))
#define UPDATE_STATE(newState)    ((m_vc.state) = (newState))
#define VC_OK                     ((m_vc.state < VC_STATE_FAULT)&&(m_vc.state > VC_STATE_INITED))
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
    VC_rotation_pair_t neutral;
    VC_rotation_pair_t max;
    VC_rotation_pair_t min;
} VC_steerCalibration_S;

typedef struct{
    VC_speed_pair_t max_FWD_hardLimit; // for sports mode if needed, (unused)
    VC_speed_pair_t max_FWD_softLimit;
    VC_speed_pair_t min_FWD_starting; //due to friction, it requires certain power to move
    VC_speed_pair_t braking;  //TODO: might need a variable/hard/soft braking, TBD
} VC_throttleCalibration_S;

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
static VehicleController_data_S    m_vc = {0};
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


/*******************************************************************************
 * public function
 ******************************************************************************/

void VC_onDestroy(void)
{
    m_vc.steering_config = NULL;
    m_vc.throttle_config = NULL;
    // free all child modules
    SERVO_onDestroy();
    UPDATE_STATE(VC_STATE_DESTROYED);
}

void VC_Config(void)
{
    m_vc.steering_config = &frsky_servo_calib;
    m_vc.throttle_config = &onyx_bldc_esc_calib;
    // compute scale factor
    m_vc.us_per_deg = (m_vc.steering_config->max.pw_us - m_vc.steering_config->min.pw_us)/
        (m_vc.steering_config->max.angle_deg - m_vc.steering_config->min.angle_deg);
    m_vc.us_s_per_mm = (m_vc.throttle_config->max_FWD_softLimit.pw_us - m_vc.throttle_config->min_FWD_starting.pw_us)/
        (m_vc.throttle_config->max_FWD_softLimit.speed_mm_per_s - m_vc.throttle_config->min_FWD_starting.speed_mm_per_s);

    m_vc.deviceConfigs[VC_CHANNEL_NAME_STEERING].gpio.pin = HUMMING_CONFIG_EXAMPLE_GPIO_PIN;
    m_vc.deviceConfigs[VC_CHANNEL_NAME_STEERING].gpio.port= HUMMING_CONFIG_EXAMPLE_GPIO_PORT;
    m_vc.deviceConfigs[VC_CHANNEL_NAME_STEERING].refreshingPeriod = HUMMING_CONFIG_EXAMPLE_PWM_PERIOD;
    m_vc.deviceConfigs[VC_CHANNEL_NAME_STEERING].defaultPulseWidth_us =	m_vc.steering_config->neutral.pw_us; //default is neutral position
    m_vc.deviceConfigs[VC_CHANNEL_NAME_STEERING].minPulseWidth_us =	m_vc.steering_config->min.pw_us;
    m_vc.deviceConfigs[VC_CHANNEL_NAME_STEERING].maxPulseWidth_us =	m_vc.steering_config->max.pw_us;

    m_vc.deviceConfigs[VC_CHANNEL_NAME_THROTTLE].gpio.pin = HUMMING_CONFIG_EXAMPLE_GPIO_PIN;
    m_vc.deviceConfigs[VC_CHANNEL_NAME_THROTTLE].gpio.port= HUMMING_CONFIG_EXAMPLE_GPIO_PORT;
    m_vc.deviceConfigs[VC_CHANNEL_NAME_THROTTLE].refreshingPeriod = HUMMING_CONFIG_EXAMPLE_PWM_PERIOD;
    m_vc.deviceConfigs[VC_CHANNEL_NAME_THROTTLE].defaultPulseWidth_us =	m_vc.throttle_config->braking.speed_mm_per_s; // default is active braking
    m_vc.deviceConfigs[VC_CHANNEL_NAME_THROTTLE].minPulseWidth_us =	0; //make sure is 0, or you wont be able to stop with `write_us`
    m_vc.deviceConfigs[VC_CHANNEL_NAME_THROTTLE].maxPulseWidth_us =	m_vc.throttle_config->max_FWD_softLimit.speed_mm_per_s;
    
    UPDATE_STATE(VC_STATE_CONFIGED);
}

bool VC_Init(void)
{
    if(VC_STATE_CONFIGED == m_vc.state)
    {
	    SERVO_init(m_vc.deviceConfigs, VC_CHANNEL_NAME_COUNT);
        UPDATE_STATE(VC_STATE_INITED);
    }
    return (VC_STATE_INITED == m_vc.state);
}

bool VC_Begin(void)
{
    bool status = true;
    if(VC_STATE_INITED == m_vc.state)
    {
        status &= SERVO_requestStart(VC_CHANNEL_NAME_STEERING);
        status &= SERVO_requestStart(VC_CHANNEL_NAME_THROTTLE);
        if(status)
        {
           UPDATE_STATE(VC_STATE_IDLE);
        }
    }
    return (VC_STATE_IDLE == m_vc.state);
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

bool VC_requestSteering(angle_deg_t reqAng) 
{
    bool ret = false;
    pulse_us_t pulseWidth = 0; 
    if( (VC_OK) &&
        (reqAng <= m_vc.steering_config->max.angle_deg) &&
        (reqAng >= m_vc.steering_config->min.angle_deg))
    {
            // do conversion & offset here:
        int16_t pw_delta = (reqAng - (m_vc.steering_config->neutral.angle_deg))*m_vc.us_per_deg;
        pulseWidth = (pulse_us_t)((pw_delta) + (m_vc.steering_config->neutral.pw_us));
        if(SERVO_write_us(VC_CHANNEL_NAME_STEERING, pulseWidth))
        {
            ret = true;
        }   
    }
    return ret;
}

bool VC_requestThrottle(speed_mm_per_s_t reqSpd)
{
    bool ret = false;
    pulse_us_t pulseWidth = 0; 
    if((VC_OK) && reqSpd >= m_vc.throttle_config->min_FWD_starting.speed_mm_per_s)
    {
        if(reqSpd <= m_vc.throttle_config->max_FWD_softLimit.speed_mm_per_s)
        {
            // do conversion & offset here:
            int16_t pw_delta = (reqSpd - (m_vc.throttle_config->min_FWD_starting.speed_mm_per_s))*m_vc.us_s_per_mm;
            pulseWidth = (pulse_us_t)((pw_delta) + (m_vc.throttle_config->min_FWD_starting.pw_us));
            if(SERVO_write_us(VC_CHANNEL_NAME_THROTTLE, pulseWidth))
            {
                ret = true;
                UPDATE_STATE(VC_STATE_RUNNING);
            }
            else
            {
                SET_ERR_FLAG(VC_ERROR_FLAG_THROTTLE_ERR);
            }      
        }
        else
        {
            // keep current spd.
        }
    }
    else    // braking
    {
       if(SERVO_write_us(VC_CHANNEL_NAME_THROTTLE, m_vc.throttle_config->braking.pw_us))
        {
            ret = true;
            UPDATE_STATE(VC_STATE_IDLE);
        } 
        else
        {
            SET_ERR_FLAG(VC_ERROR_FLAG_THROTTLE_ERR);
        }
    }
    return ret;
}

bool VC_doBraking(angle_deg_t reqAng)
{
    bool ret = true;
    ret &= SERVO_goDefault(VC_CHANNEL_NAME_THROTTLE);
    if(reqAng == 0)
    {
       ret &= SERVO_goDefault(VC_CHANNEL_NAME_STEERING);
       UPDATE_STATE(VC_STATE_IDLE);
    }
    //in case we want to drift
    else
    {
      ret &= VC_requestSteering(reqAng);
    }
    if(!ret)
    {
        SET_ERR_FLAG(VC_ERROR_FLAG_UNABLE_BRAKING_ERR);
    }
    return ret;
}

// WARNING. this will not necessary do bbraking/straighten steering, all it does is to give up signalling(controlling) the servo/esc
bool VC_powerOff_FreeWheeling(VC_channnelName_E controller)
{
    bool ret = true;
    switch(controller)
    {
        case (VC_CHANNEL_NAME_STEERING):
        case (VC_CHANNEL_NAME_THROTTLE):
            ret &= SERVO_doStop(controller);
            break;

        case (VC_CHANNEL_NAME_COUNT):
        case (VC_CHANNEL_NAME_ALL):
        default:
            // drop both
            ret &= SERVO_doStop(VC_CHANNEL_NAME_STEERING);
            ret &= SERVO_doStop(VC_CHANNEL_NAME_THROTTLE); //will stop eventually
            break;
    }
    if(!ret)
    {
        SET_ERR_FLAG(VC_ERROR_FLAG_UNABLE_FREE_CONTROL_ERR);
    }
    else
    {
        UPDATE_STATE(VC_STATE_IDLE);        
    }
    return ret;
}

VC_state_E VC_getVehicleControllerState(void)
{
    return m_vc.state;
}

uint16_t VC_getErrorFlags(void)
{
    return m_vc.errorFlags;
}
/*******************************************************************************
 * private function
 ******************************************************************************/
