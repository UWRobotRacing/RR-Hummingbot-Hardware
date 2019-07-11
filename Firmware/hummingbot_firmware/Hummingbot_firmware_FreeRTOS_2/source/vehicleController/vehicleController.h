#ifndef VEHICLE_CONTROLLER_H_
#define VEHICLE_CONTROLLER_H_
#include "common.h"

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
    VC_STATE_FAULT,
    VC_STATE_DESTROYED,
} VC_state_E;

typedef enum{
    VC_ERROR_FLAG_STEERING_ERR,
    VC_ERROR_FLAG_THROTTLE_ERR,
    VC_ERROR_FLAG_UNABLE_BRAKING_ERR,
    VC_ERROR_FLAG_UNABLE_FREE_CONTROL_ERR,
} VC_errorFlag_E;

typedef enum{
    VC_CHANNEL_NAME_STEERING,
    VC_CHANNEL_NAME_THROTTLE,
    VC_CHANNEL_NAME_COUNT,
    VC_CHANNEL_NAME_ALL,
} VC_channnelName_E;
/*******************************************************************************
 * public functions
 ******************************************************************************/
void VC_onDestroy(void);
void VC_Config(void);
bool VC_Init(void);
bool VC_Begin(void);

uint16_t VC_getErrorFlags(void);
VC_state_E VC_getVehicleControllerState(void);
void VC_dummyTestRun(void);

bool VC_requestSteering(angle_deg_t reqAng);
bool VC_requestThrottle(speed_cm_per_s_t reqSpd);
bool VC_doBraking(angle_deg_t reqAng);
bool VC_powerOff(VC_channnelName_E controller);
bool VC_do_FreeWheeling(VC_channnelName_E controller);
bool VC_joystick_control(rf24_joystick_tik_t steeringAxis, rf24_joystick_tik_t throttleAxis, angle_deg_t* out_convertedAng, speed_cm_per_s_t* out_convertedSpd);
pulse_us_t VC_getCurrentPulseWidth(VC_channnelName_E controller);

/* NOTE: PREFERRABLY: DO NOT USE RAW for main code, they are only meant for testing and calibration */ 
bool VC_requestThrottle_raw(pulse_us_t pw_us); // will be filtered by logic
bool VC_requestSteering_raw(pulse_us_t pw_us);
bool VC_requestPWM_force_raw(VC_channnelName_E controller, pulse_us_t pw_us); // only limited by the max/min in configuration
#endif //(VEHICLE_CONTROLLER_H_)
