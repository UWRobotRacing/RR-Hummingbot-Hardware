#ifndef VEHICLE_CONTROLLER_H_
#define VEHICLE_CONTROLLER_H_


#include <stdint.h>
#include <stdbool.h>
/***********************************
 ********* Macro Definition **********
 ***********************************/
// define some common metric uint
typedef int16_t    angle_deg_t; 
typedef int16_t    dist_mm_t; 
typedef uint16_t   pulse_us_t;
typedef int16_t    speed_mm_per_s_t;

// some conversion unit
typedef uint16_t   us_per_deg_t;
typedef uint16_t   us_s_per_mm_t;

typedef enum{
    VC_STATE_UNDEFINED,
    VC_STATE_CONFIGED,
    VC_STATE_INITED,
    VC_STATE_IDLE,
    VC_STATE_RUNNING,
    VC_STATE_FAULT,
    VC_STATE_DESTROYED,
} VC_state_E;

typedef enum{
    VC_ERROR_FLAG_STEERING_ERR,
    VC_ERROR_FLAG_THROTTLE_ERR,
} VC_errorFlag_E;
/*******************************************************************************
 * public functions
 ******************************************************************************/
void VC_onDestroy(void);
void VC_Config(void);
bool VC_Init(void);
bool VC_Begin(void);
void VC_dummyTestRun(void);

#endif //(VEHICLE_CONTROLLER_H_)
