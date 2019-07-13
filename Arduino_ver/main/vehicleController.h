#ifndef VEHICLE_CONTROLLER_H_
#define VEHICLE_CONTROLLER_H_


#include <stdint.h>
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif
/***********************************
 ********* Macro Definition **********
 ***********************************/
// define some common metric uint
typedef int16_t    angle_deg_t; 
typedef uint16_t   pulse_us_t;
typedef int16_t    speed_cm_per_s_t;
typedef int16_t    rf24_joystick_tik_t;
// some conversion unit
// NOTE: change to uint16_t if your processor does not have a FPU, might suffer trimming/truncation error
// NOTE*: please use uint16_t, since float still takes too long to compute
// NOTE: TODO: fixed float should be implemented in order to achieve high speed floating calc.
typedef uint16_t   us_per_deg_t;
typedef uint16_t   us_s_per_mm_t;

typedef enum{
    VC_STATE_UNDEFINED,
    VC_STATE_CONFIGED,
    VC_STATE_INITED,
    VC_STATE_IDLE,
    VC_STATE_RUNNING,
    VC_STATE_NEUTRAL,
    VC_STATE_REVERSING,
    VC_STATE_FAULT,
    VC_STATE_DESTROYED,
} VC_state_E;

typedef enum{
    VC_CHANNEL_NAME_STEERING,
    VC_CHANNEL_NAME_THROTTLE,
    VC_CHANNEL_NAME_COUNT,
    VC_CHANNEL_NAME_ALL,
} VC_channnelName_E;

/*******************************************************************************
 * Calibration Values
 ******************************************************************************/
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
    .neutral = {
        .pw_us = 1540U,//800U,
        .speed_cm_per_s = 0,
    }, 
    .braking = {
        .pw_us = 1000U,
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
 * public functions
 ******************************************************************************/
void VC_Config(void);
VC_state_E VC_getVehicleControllerState(void);

pulse_us_t VC_requestSteering(angle_deg_t reqAng);
pulse_us_t VC_requestThrottle(speed_cm_per_s_t reqSpd);
pulse_us_t VC_doBraking(bool isReversing);
void VC_joystick_control(rf24_joystick_tik_t steeringAxis, rf24_joystick_tik_t throttleAxis, angle_deg_t* out_convertedAng, speed_cm_per_s_t* out_convertedSpd, pulse_us_t *outAngPW, pulse_us_t *outSpdPW);
VC_state_E VC_getVehicleControllerState(void);

#ifdef __cplusplus
} // extern "C"
#endif
#endif //(VEHICLE_CONTROLLER_H_)
