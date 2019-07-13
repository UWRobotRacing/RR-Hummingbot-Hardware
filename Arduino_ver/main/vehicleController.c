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

#include "Hummingconfig.h"
#include "Hummingbot_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define UPDATE_STATE(newState)    ((m_vc.state) = (newState))
/*******************************************************************************
 * typedef
 ******************************************************************************/
/* Defs for joystick controller mapping */
typedef struct{
    rf24_joystick_tik_t   val;//we are assuming neutral is 0 degree
    angle_deg_t     angle_deg;
} VC_rf24_joystick_steering_pair_t;

typedef struct{
    rf24_joystick_tik_t            val;//we are assuming neutral is 0 degree
    speed_cm_per_s_t    speed_cm_per_s;
} VC_rf24_joystick_throttle_pair_t;

typedef struct{
    VC_rf24_joystick_steering_pair_t steeringNeutral;
    VC_rf24_joystick_steering_pair_t steeringMax;
    VC_rf24_joystick_steering_pair_t steeringMin;
    VC_rf24_joystick_steering_pair_t steeringDeadband;
    VC_rf24_joystick_throttle_pair_t throttleMin;
    VC_rf24_joystick_throttle_pair_t throttleMax;
    VC_rf24_joystick_throttle_pair_t throttleBrakingMax;
    VC_rf24_joystick_throttle_pair_t throttleBrakingMin;
    VC_rf24_joystick_throttle_pair_t throttleDeadband;
} VC_rf24_joystick_configs_S;

/* Defs for vehicle controller*/
typedef struct{
    pulse_us_t   pw_us;//we are assuming neutral is 0 degree
    angle_deg_t  angle_deg;
} VC_rotation_pair_t;

typedef struct{
    pulse_us_t          pw_us;//we are assuming neutral is 0 degree
    speed_cm_per_s_t    speed_cm_per_s;
} VC_speed_pair_t;

typedef struct{
    VC_rotation_pair_t neutral;
    VC_rotation_pair_t max;
    VC_rotation_pair_t min;
} VC_steerCalibration_S;

typedef struct{
    VC_speed_pair_t max_FWD_softLimit;
    VC_speed_pair_t min_FWD_starting; //due to friction, it requires certain power to move
    VC_speed_pair_t braking;  //TODO: might need a variable/hard/soft braking, TBD
    VC_speed_pair_t min_REV_starting;
    VC_speed_pair_t max_REV_softLimit;
    pulse_us_t      min_pw_us;
    pulse_us_t      max_pw_us;
} VC_throttleCalibration_S;

typedef struct{
    const VC_steerCalibration_S*       steering_config;
    const VC_throttleCalibration_S*    throttle_config;
    const VC_rf24_joystick_configs_S*  joystick_config;
    // NOTE: since floating takes too long, we will use multiplier & divider to keep both accuracy and speed
    us_per_deg_t                us_per_deg_multiplier;
    us_per_deg_t                us_per_deg_divider;
    us_s_per_mm_t               us_s_per_mm_multiplier;
    us_s_per_mm_t               us_s_per_mm_divider;
    VC_state_E                  state;   
} VehicleController_data_S;

/*******************************************************************************
 * private variables
 ******************************************************************************/
static VehicleController_data_S    m_vc = {0};
// NOTE: please calibrate values
static const VC_steerCalibration_S    frsky_servo_calib ={
    .neutral = {
      .pw_us = 1400U,
      .angle_deg = 0,
    },
    .max = {
      //.pw_us = 1650U,
      .pw_us = 1600U,
      .angle_deg = 30,
    },
    .min = {
        //.pw_us = 1100U,
      .pw_us = 1200U,
      .angle_deg = -30,
    },
};
//NOTE: please calibrate these values figure out if 1540U is kinda freewheeling
static const VC_throttleCalibration_S onyx_bldc_esc_calib ={
    .max_FWD_softLimit = {
        .pw_us = 2000U,
        .speed_cm_per_s = 200,
    }, 
    .min_FWD_starting = {
        .pw_us = 1600U,
        .speed_cm_per_s = 10,
    }, 
    .braking = {
        .pw_us = 1540U,//800U,
        .speed_cm_per_s = 0,
    }, 
    .min_REV_starting = { //default min pwm to keep esc alive
        .pw_us = 1500U,//550U,
        .speed_cm_per_s = -10,
    },
    .max_REV_softLimit = { //default min pwm to keep esc alive
        .pw_us = 600U,//550U,
        .speed_cm_per_s = -200,
    },
    // PWM signal boundary
    .min_pw_us = 540U,
    .max_pw_us = 2000U,
};

static const VC_rf24_joystick_configs_S joystick_calib = {
    .steeringNeutral  = {
      .val        = 584,
      .angle_deg  = 0,
    },     
    .steeringMax      = { 
      .val        = 961, //+377
      .angle_deg  = 30, 
    }, 
    .steeringMin      = { 
      .val        = 108, //-476
      .angle_deg  = -30,
    }, 
    .steeringDeadband = { //every 10 tik, will be 1 degree
      .val        = 10,
      .angle_deg  = 1,
    },      
    .throttleMin  = {
      .val            = 108,
      .speed_cm_per_s = -200,
    },  
    .throttleMax      = { 
      .val            = 918, //+406
      .speed_cm_per_s = 200, //assume 200 cm/s TODO:TBD based on actual measurements
    },
    .throttleBrakingMin = { 
      .val            = 462, //512
      .speed_cm_per_s = 0,
    },
    .throttleBrakingMax = { 
      .val            = 562, //512
      .speed_cm_per_s = 0,
    },
    .throttleDeadband = {
      .val            = 10,
      .speed_cm_per_s = 5,//4, // 4cm/s every 10 rik changes
    },  
};
/*******************************************************************************
 * private function prototypes
 ******************************************************************************/


/*******************************************************************************
 * public function
 ******************************************************************************/

void VC_Config(void)
{
    m_vc.steering_config = &frsky_servo_calib;
    m_vc.throttle_config = &onyx_bldc_esc_calib;
    m_vc.joystick_config = &joystick_calib;

    // compute scale factor
    m_vc.us_per_deg_multiplier = (m_vc.steering_config->max.pw_us - m_vc.steering_config->min.pw_us);
    m_vc.us_per_deg_divider    = (m_vc.steering_config->max.angle_deg - m_vc.steering_config->min.angle_deg);
    m_vc.us_s_per_mm_multiplier = (m_vc.throttle_config->max_FWD_softLimit.pw_us - m_vc.throttle_config->min_FWD_starting.pw_us);
    m_vc.us_s_per_mm_divider    = (m_vc.throttle_config->max_FWD_softLimit.speed_cm_per_s - m_vc.throttle_config->min_FWD_starting.speed_cm_per_s);

    UPDATE_STATE(VC_STATE_IDLE);
}

pulse_us_t VC_requestSteering(angle_deg_t reqAng) 
{
    int32_t pw_delta = 0;
    pulse_us_t pulseWidth = 0; 
    if (reqAng > m_vc.steering_config->max.angle_deg)
    {
      reqAng = m_vc.steering_config->max.angle_deg;
    }
    else if (reqAng < m_vc.steering_config->min.angle_deg)
    {
      reqAng = m_vc.steering_config->min.angle_deg;
    }

    // do conversion & offset here:
    pw_delta = (reqAng - (m_vc.steering_config->neutral.angle_deg));
    pw_delta *= m_vc.us_per_deg_multiplier;
    pw_delta /= m_vc.us_per_deg_divider;
    pulseWidth = (pulse_us_t)((pw_delta) + (m_vc.steering_config->neutral.pw_us));

    return pulseWidth;
}

pulse_us_t VC_requestThrottle(speed_cm_per_s_t reqSpd)
{
    int32_t pw_delta = 0;
    pulse_us_t pulseWidth = 0; 
    if (reqSpd > m_vc.throttle_config->max_FWD_softLimit.speed_cm_per_s)
    {
      reqSpd = m_vc.throttle_config->max_FWD_softLimit.speed_cm_per_s;
    }
    else if (reqSpd < m_vc.throttle_config->max_REV_softLimit.speed_cm_per_s)
    {
      reqSpd = m_vc.throttle_config->max_REV_softLimit.speed_cm_per_s;
    }
    
    if (reqSpd >= m_vc.throttle_config->min_FWD_starting.speed_cm_per_s)
    {
      // do conversion & offset here:
      pw_delta = (reqSpd - (m_vc.throttle_config->min_FWD_starting.speed_cm_per_s));
      pw_delta *= m_vc.us_s_per_mm_multiplier;
      pw_delta /= m_vc.us_s_per_mm_divider;
      pulseWidth = (pulse_us_t)((pw_delta) + (m_vc.throttle_config->min_FWD_starting.pw_us));
      UPDATE_STATE(VC_STATE_RUNNING);
    }
    else if (reqSpd <= m_vc.throttle_config->min_REV_starting.speed_cm_per_s)
    {
      // do conversion & offset here:
      pw_delta = (reqSpd - (m_vc.throttle_config->min_REV_starting.speed_cm_per_s));
      pw_delta *= m_vc.us_s_per_mm_multiplier;
      pw_delta /= m_vc.us_s_per_mm_divider;
      pulseWidth = (pulse_us_t)((pw_delta) + (m_vc.throttle_config->min_REV_starting.pw_us));
      UPDATE_STATE(VC_STATE_REVERSING);
    }
    else // braking
    {
      pulseWidth = VC_doBraking();
      UPDATE_STATE(VC_STATE_IDLE);
    }
    return pulseWidth;
}

pulse_us_t VC_doBraking(void)
{
    return (m_vc.throttle_config->braking.pw_us);
}

void VC_joystick_control(rf24_joystick_tik_t steeringAxis, rf24_joystick_tik_t throttleAxis, angle_deg_t* out_convertedAng, speed_cm_per_s_t* out_convertedSpd, pulse_us_t *outAngPW, pulse_us_t *outSpdPW)
{
  rf24_joystick_tik_t delta = 0;
  angle_deg_t         steeringAng_req = 0;
  speed_cm_per_s_t    throttleSpd_req = 0;
  /// - steering mapping
  delta = steeringAxis - m_vc.joystick_config->steeringNeutral.val;
  steeringAng_req = delta*m_vc.joystick_config->steeringDeadband.angle_deg/m_vc.joystick_config->steeringDeadband.val;
  steeringAng_req += m_vc.joystick_config->steeringNeutral.angle_deg;
  // Bounding the angle
  if (steeringAng_req > m_vc.joystick_config->steeringMax.angle_deg)
  {
    steeringAng_req = m_vc.joystick_config->steeringMax.angle_deg;
  }
  else if (steeringAng_req < m_vc.joystick_config->steeringMin.angle_deg)
  {
    steeringAng_req = m_vc.joystick_config->steeringMin.angle_deg;
  }
  else
  {
    // do nothing
  }

  /// - throttle mapping
  if (throttleAxis > m_vc.joystick_config->throttleBrakingMax.val)
  {
    delta = throttleAxis - m_vc.joystick_config->throttleBrakingMax.val;
    throttleSpd_req = delta*m_vc.joystick_config->throttleDeadband.speed_cm_per_s/m_vc.joystick_config->throttleDeadband.val;
    throttleSpd_req += m_vc.joystick_config->throttleBrakingMax.speed_cm_per_s;
    //bounding the spd
    if (throttleSpd_req > m_vc.joystick_config->throttleMax.speed_cm_per_s)
    {
      throttleSpd_req = m_vc.joystick_config->throttleMax.speed_cm_per_s;
    }
    else
    {
      // do nothing
    }
  }
  else if (throttleAxis < m_vc.joystick_config->throttleBrakingMin.val)
  {
    // braking
    delta = m_vc.joystick_config->throttleBrakingMin.val - throttleAxis;
    throttleSpd_req = delta*m_vc.joystick_config->throttleDeadband.speed_cm_per_s/m_vc.joystick_config->throttleDeadband.val;
    throttleSpd_req = m_vc.joystick_config->throttleBrakingMin.speed_cm_per_s - throttleSpd_req;
    //bounding the spd
    if (throttleSpd_req < m_vc.joystick_config->throttleMin.speed_cm_per_s)
    {
      throttleSpd_req = m_vc.joystick_config->throttleMin.speed_cm_per_s;
    }
    else
    {
      // do nothing
    }
  }
  else
  {
    // do braking
    throttleSpd_req = 0U;
  }
  
  /// - output - NOTE: since the lower level will handle optimization, so we dont have to keep tracking the old values
  *out_convertedAng = steeringAng_req;
  *out_convertedSpd = throttleSpd_req;
  *outAngPW = VC_requestSteering(steeringAng_req);
  *outSpdPW = VC_requestThrottle(throttleSpd_req);
}

VC_state_E VC_getVehicleControllerState(void)
{
    return m_vc.state;
}

/*******************************************************************************
 * private function
 ******************************************************************************/
