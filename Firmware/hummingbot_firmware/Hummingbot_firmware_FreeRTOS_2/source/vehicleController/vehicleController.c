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
#include "fsl_ftm.h"

/*******************************************************************************
 * PREFERENCE
 ******************************************************************************/
#if (ENABLE_MOTOR_FEEDBACK)
  #define BOARD_FTM_BASEADDR                (FTM1)
  /* FTM channel used for input capture */
  #define BOARD_FTM_INPUT_CAPTURE_CHANNEL   (kFTM_Chnl_7)
  /* Interrupt number and interrupt handler for the FTM instance used */
  #define FTM_INTERRUPT_NUMBER              (FTM1_IRQn)
  #define FTM_INPUT_CAPTURE_HANDLER         (FTM1_IRQHandler)
  /* Interrupt to enable and flag to read; depends on the FTM channel pair used */
  #define FTM_CHANNEL_INTERRUPT_ENABLE      (kFTM_Chnl7InterruptEnable)
  #define FTM_CHANNEL_FLAG                  (kFTM_Chnl7Flag)
  /* Get source clock for FTM driver */
  #define FTM_SOURCE_CLOCK CLOCK_GetFreq    (kCLOCK_CoreSysClk)
  
  #define MIN_TICK_TO_BREAK              (60) //ms
  #define MIN_NUM_OF_TIMEOUT_TO_RUN      (10) //consecutive timeout
  #define BREAK_DURATION              (1) //100ms
  #define RETHROTTLE_PROTECTION       (10) //100ms = 1s
#endif   

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
    VC_rf24_joystick_throttle_pair_t throttleNeutral;
    VC_rf24_joystick_throttle_pair_t throttleMax;
    VC_rf24_joystick_throttle_pair_t throttleBraking;
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
    VC_speed_pair_t neutral;
    pulse_us_t      min_pw_us;
    pulse_us_t      max_pw_us;
} VC_throttleCalibration_S;

// Motor Feedback
#if (ENABLE_MOTOR_FEEDBACK)
 typedef struct{
  bool          ftmIsrFlag;
  ftm_config_t  ftmInfo;
  uint32_t      count;
  uint32_t      time_ms;
  uint32_t      timeout_count;
  uint32_t      turn_per_second;
 } VC_encoder_t;
#endif         

typedef struct{
    SERVO_ServoConfig_S         deviceConfigs[VC_CHANNEL_NAME_COUNT];
    const VC_steerCalibration_S*       steering_config;
    const VC_throttleCalibration_S*    throttle_config;
    const VC_rf24_joystick_configs_S*  joystick_config;
    // NOTE: since floating takes too long, we will use multiplier & divider to keep both accuracy and speed
    us_per_deg_t                us_per_deg_multiplier;
    us_per_deg_t                us_per_deg_divider;
    us_s_per_mm_t               us_s_per_mm_multiplier;
    us_s_per_mm_t               us_s_per_mm_divider;
    // some variables to avoid excess repeative operation
    speed_cm_per_s_t            current_throttle;
    angle_deg_t                 current_steering;
    bool                        isCurrentTrackingValsInvalid; //if so, current_x will no longer be valid, will force to retrack upon request() functions
    VC_state_E                  state;   
    uint16_t                    errorFlags;     
    // Motor Feedback
#if (ENABLE_MOTOR_FEEDBACK)
    VC_encoder_t                hallSensor;
    uint32_t                    prev_hallSensor_tick;
    uint32_t                    loop_tick;
    bool                        hacky_break;
#endif
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
        .pw_us = 540U,//800U,
        .speed_cm_per_s = -1,
    }, 
    .neutral = { //default min pwm to keep esc alive
        .pw_us = 1540U,//550U,
        .speed_cm_per_s = 0,
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
    .throttleNeutral  = {
      .val            = 512,
      .speed_cm_per_s = 0,
    },  
    .throttleMax      = { 
      .val            = 918, //+406
      .speed_cm_per_s = 200, //assume 200 cm/s TODO:TBD based on actual measurements
    },
    .throttleBraking  = { 
      .val            = 300,//108, // unused, because no reverse implemented
      .speed_cm_per_s = -1,
    },
    .throttleDeadband = {
      .val            = 10,
      .speed_cm_per_s = 5,//4, // 4cm/s every 10 rik changes
    },  
};
/*******************************************************************************
 * private function prototypes
 ******************************************************************************/
#if ENABLE_MOTOR_FEEDBACK
  static void initEncoders(void);
  static void configEncoders(void);
  void FTM_INPUT_CAPTURE_HANDLER(void);
#endif

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
    m_vc.joystick_config = &joystick_calib;

    // compute scale factor
    m_vc.us_per_deg_multiplier = (m_vc.steering_config->max.pw_us - m_vc.steering_config->min.pw_us);
    m_vc.us_per_deg_divider    = (m_vc.steering_config->max.angle_deg - m_vc.steering_config->min.angle_deg);
    m_vc.us_s_per_mm_multiplier = (m_vc.throttle_config->max_FWD_softLimit.pw_us - m_vc.throttle_config->min_FWD_starting.pw_us);
    m_vc.us_s_per_mm_divider    = (m_vc.throttle_config->max_FWD_softLimit.speed_cm_per_s - m_vc.throttle_config->min_FWD_starting.speed_cm_per_s);

    // TODO: need to map to appropriate gpios [Hummingconfig.h]
    m_vc.deviceConfigs[VC_CHANNEL_NAME_STEERING].gpio.pin = HUMMING_CONFIG_STEERING_SERVO_GPIO_PIN;
    m_vc.deviceConfigs[VC_CHANNEL_NAME_STEERING].gpio.port= HUMMING_CONFIG_STEERING_SERVO_GPIO_PORT;
    m_vc.deviceConfigs[VC_CHANNEL_NAME_STEERING].refreshingPeriod = HUMMING_CONFIG_STEERING_SERVO_PWM_PERIOD;
    m_vc.deviceConfigs[VC_CHANNEL_NAME_STEERING].defaultPulseWidth_us =	m_vc.steering_config->neutral.pw_us; //default is neutral position
    m_vc.deviceConfigs[VC_CHANNEL_NAME_STEERING].minPulseWidth_us =	m_vc.steering_config->min.pw_us;
    m_vc.deviceConfigs[VC_CHANNEL_NAME_STEERING].maxPulseWidth_us =	m_vc.steering_config->max.pw_us;

    m_vc.deviceConfigs[VC_CHANNEL_NAME_THROTTLE].gpio.pin = HUMMING_CONFIG_THROTTLE_ESC_GPIO_PIN;
    m_vc.deviceConfigs[VC_CHANNEL_NAME_THROTTLE].gpio.port= HUMMING_CONFIG_THROTTLE_ESC_GPIO_PORT;
    m_vc.deviceConfigs[VC_CHANNEL_NAME_THROTTLE].refreshingPeriod = HUMMING_CONFIG_THROTTLE_ESC_PWM_PERIOD;
    m_vc.deviceConfigs[VC_CHANNEL_NAME_THROTTLE].defaultPulseWidth_us =	m_vc.throttle_config->neutral.pw_us; // default is active braking
    m_vc.deviceConfigs[VC_CHANNEL_NAME_THROTTLE].minPulseWidth_us =	m_vc.throttle_config->min_pw_us; //make sure is 0, or you wont be able to stop with `write_us`
    m_vc.deviceConfigs[VC_CHANNEL_NAME_THROTTLE].maxPulseWidth_us =	m_vc.throttle_config->max_pw_us;
    
    // Configure encoder interrupt
   #if (ENABLE_MOTOR_FEEDBACK)
     configEncoders();
   #endif //(ENABLE_MOTOR_FEEDBACK)

    UPDATE_STATE(VC_STATE_CONFIGED);
}

bool VC_Init(void)
{
    if(VC_STATE_CONFIGED == m_vc.state)
    {
      SERVO_init(m_vc.deviceConfigs, VC_CHANNEL_NAME_COUNT);
 #if (ENABLE_MOTOR_FEEDBACK)
       initEncoders();
 #endif //(ENABLE_MOTOR_FEEDBACK)
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

#if (ENABLE_MOTOR_FEEDBACK)
  bool VC_getEncoderTimerValues(uint32_t* captureVal)
  {
    bool ret = false;
    if(m_vc.hallSensor.ftmIsrFlag)
    {
      *captureVal = BOARD_FTM_BASEADDR->CONTROLS[BOARD_FTM_INPUT_CAPTURE_CHANNEL].CnV;
      m_vc.hallSensor.ftmIsrFlag = false;
      ret = true;
    }
    return ret;
  }

  bool VC_getNewEncoderValues(uint32_t* captureVal)
  {
    bool isNewValue = false;
    isNewValue = m_vc.hallSensor.ftmIsrFlag;
    if(isNewValue)
    {
      m_vc.hallSensor.ftmIsrFlag = false;
    }
    *captureVal = (m_vc.hallSensor.count);
    return (isNewValue);
  }
  uint32_t VC_getMotorSpd(void)
  {
    return m_vc.hallSensor.turn_per_second;
  }
#endif //(ENABLE_MOTOR_FEEDBACK)

/* VC_getCurrentPulseWidth
 * @about return the actual pwm pulse output for given controller
 */
pulse_us_t VC_getCurrentPulseWidth(VC_channnelName_E controller)
{
    pulse_us_t ret = 0;
    switch(controller)
    {
      case (VC_CHANNEL_NAME_STEERING):
      case (VC_CHANNEL_NAME_THROTTLE):
        ret = SERVO_getCurrentPWM(controller);
        break;

      case (VC_CHANNEL_NAME_COUNT):
      case (VC_CHANNEL_NAME_ALL):
      default:
        //do nothing
        break;
    }
    return ret;
}

void VC_dummyTestRun(void) //DEPRECATED TODO:remove in the end
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
    int32_t pw_delta = 0;
    pulse_us_t pulseWidth = 0; 
    if( (reqAng != 0 && m_vc.current_steering == reqAng) && !m_vc.isCurrentTrackingValsInvalid)
    {
      ret = true; // avoid excess calculation
    }
    else if( (VC_OK) &&
        (reqAng <= m_vc.steering_config->max.angle_deg) &&
        (reqAng >= m_vc.steering_config->min.angle_deg))
    {
        // do conversion & offset here:
        pw_delta = (reqAng - (m_vc.steering_config->neutral.angle_deg));
        pw_delta *= m_vc.us_per_deg_multiplier;
        pw_delta /= m_vc.us_per_deg_divider;
        pulseWidth = (pulse_us_t)((pw_delta) + (m_vc.steering_config->neutral.pw_us));
        if(SERVO_write_us(VC_CHANNEL_NAME_STEERING, pulseWidth))
        {
          ret = true;
          m_vc.current_steering = reqAng;
        }   
    }
    else
    {
      //do nothing, FAILED
    }
    
    return ret;
}

bool VC_requestThrottle(speed_cm_per_s_t reqSpd)
{
    bool ret = false;
    int32_t pw_delta = 0;
    pulse_us_t pulseWidth = 0; 
    if((reqSpd != 0 && m_vc.current_throttle == reqSpd)  && !m_vc.isCurrentTrackingValsInvalid)
    {
      ret = true;
    }
    else if((VC_OK) && reqSpd >= m_vc.throttle_config->min_FWD_starting.speed_cm_per_s)
    {
        if(reqSpd <= m_vc.throttle_config->max_FWD_softLimit.speed_cm_per_s)
        {
            // do conversion & offset here:
            pw_delta = (reqSpd - (m_vc.throttle_config->min_FWD_starting.speed_cm_per_s));
            pw_delta *= m_vc.us_s_per_mm_multiplier;
            pw_delta /= m_vc.us_s_per_mm_divider;
            pulseWidth = (pulse_us_t)((pw_delta) + (m_vc.throttle_config->min_FWD_starting.pw_us));
            if(SERVO_write_us(VC_CHANNEL_NAME_THROTTLE, pulseWidth))
            {
              ret = true;
              m_vc.current_throttle = reqSpd;
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
    else if(reqSpd == 0)
    {
      if(SERVO_write_us(VC_CHANNEL_NAME_THROTTLE, m_vc.throttle_config->neutral.pw_us))
      {
          ret = true;
          m_vc.current_throttle = m_vc.throttle_config->neutral.speed_cm_per_s;
          UPDATE_STATE(VC_STATE_NEUTRAL);
      } 
      else
      {
          SET_ERR_FLAG(VC_ERROR_FLAG_THROTTLE_ERR);
      }
    }
    else // braking, reqSpd < 0
    {
      if(SERVO_write_us(VC_CHANNEL_NAME_THROTTLE, m_vc.throttle_config->braking.pw_us))
      {
          ret = true;
          m_vc.current_throttle = m_vc.throttle_config->braking.speed_cm_per_s;
          UPDATE_STATE(VC_STATE_IDLE);
      } 
      else
      {
          SET_ERR_FLAG(VC_ERROR_FLAG_THROTTLE_ERR);
      }
    }
    return ret;
}

bool VC_requestSteering_raw(pulse_us_t pw_us) 
{
    bool ret = false;
    if( (VC_OK) &&
        (pw_us <= m_vc.steering_config->max.pw_us) &&
        (pw_us >= m_vc.steering_config->min.pw_us))
    {
      if(SERVO_write_us(VC_CHANNEL_NAME_STEERING, pw_us))
      {
          ret = true;
      }   
    }
    m_vc.isCurrentTrackingValsInvalid = true;
    return ret;
}

bool VC_requestPWM_force_raw(VC_channnelName_E controller, pulse_us_t pw_us)
{
  bool ret = false;
  if(SERVO_write_us(controller, pw_us))
  {
      ret = true;
      UPDATE_STATE(VC_STATE_RUNNING);
  }
  else
  {
    if(VC_CHANNEL_NAME_THROTTLE == controller)
    {
      SET_ERR_FLAG(VC_ERROR_FLAG_THROTTLE_ERR);
    }
    else if(VC_CHANNEL_NAME_STEERING == controller)
    {
      SET_ERR_FLAG(VC_ERROR_FLAG_STEERING_ERR);
    }
    else
    {
      // do nothing
    }
    
  }      
  m_vc.isCurrentTrackingValsInvalid = true;
  return ret;
}

bool VC_requestThrottle_raw(pulse_us_t pw_us) 
{
    bool ret = false;
    if((VC_OK) && pw_us >= m_vc.throttle_config->min_FWD_starting.pw_us)
    {
        if(pw_us <= m_vc.throttle_config->max_FWD_softLimit.pw_us)
        {
            if(SERVO_write_us(VC_CHANNEL_NAME_THROTTLE, pw_us))
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
    m_vc.isCurrentTrackingValsInvalid = true;
    return ret;
}

bool VC_doBraking(angle_deg_t reqAng)
{
    bool ret = true;
    ret &= SERVO_write_us(VC_CHANNEL_NAME_THROTTLE, m_vc.throttle_config->braking.pw_us);
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
    m_vc.isCurrentTrackingValsInvalid = true;
    return ret;
}

bool VC_do_FreeWheeling(VC_channnelName_E controller)
{
    bool ret = true;
    switch(controller)
    {
        case (VC_CHANNEL_NAME_STEERING):
        case (VC_CHANNEL_NAME_THROTTLE):
            ret &= SERVO_goDefault(controller);
            break;

        case (VC_CHANNEL_NAME_COUNT):
        case (VC_CHANNEL_NAME_ALL):
        default:
            // drop both
            ret &= SERVO_goDefault(VC_CHANNEL_NAME_STEERING);
            ret &= SERVO_goDefault(VC_CHANNEL_NAME_THROTTLE); //will stop eventually
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
    m_vc.isCurrentTrackingValsInvalid = true;
    return ret;
}

// WARNING. all it does is to give up signalling(controlling) the servo/esc
bool VC_powerOff(VC_channnelName_E controller)
{
    bool ret = true;
    switch(controller)
    {
        case (VC_CHANNEL_NAME_STEERING):
        case (VC_CHANNEL_NAME_THROTTLE):
            ret &= SERVO_powerOff(controller);
            break;

        case (VC_CHANNEL_NAME_COUNT):
        case (VC_CHANNEL_NAME_ALL):
        default:
            // drop both
            ret &= SERVO_powerOff(VC_CHANNEL_NAME_STEERING);
            ret &= SERVO_powerOff(VC_CHANNEL_NAME_THROTTLE);
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
    m_vc.isCurrentTrackingValsInvalid = true;
    return ret;
}

bool VC_joystick_control(rf24_joystick_tik_t steeringAxis, rf24_joystick_tik_t throttleAxis, angle_deg_t* out_convertedAng, speed_cm_per_s_t* out_convertedSpd)
{
  bool ret = true;
  rf24_joystick_tik_t delta = 0;
  angle_deg_t         steeringAng_req = 0;
  speed_cm_per_s_t    throttleSpd_req = 0;
  // uint32_t hallSensor_timeout_tick = 0;

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
  delta = throttleAxis - m_vc.joystick_config->throttleNeutral.val;
  if (delta > 0) // only support FWD Motoring
  {
    throttleSpd_req = delta*m_vc.joystick_config->throttleDeadband.speed_cm_per_s/m_vc.joystick_config->throttleDeadband.val;
    throttleSpd_req += m_vc.joystick_config->throttleNeutral.speed_cm_per_s;
    //bounding the spd
    if (throttleSpd_req > m_vc.joystick_config->throttleMax.speed_cm_per_s)
    {
      throttleSpd_req = m_vc.joystick_config->throttleMax.speed_cm_per_s;
    }
    else
    {
      // do nothing
    }

    /// hacky self-looped feedback
#if (ENABLE_MOTOR_FEEDBACK && ENABLE_HACKY_TICK)
    if(m_vc.hacky_break)
    {
      m_vc.loop_tick ++;
      if(m_vc.loop_tick < BREAK_DURATION)
      {
        throttleSpd_req = m_vc.joystick_config->throttleBraking.speed_cm_per_s;
      }
      else if(m_vc.loop_tick < (BREAK_DURATION + RETHROTTLE_PROTECTION))
      {
        m_vc.hacky_break = false; // turn off hacky break
      }
    }
    else
    {
      hallSensor_timeout_tick = m_vc.hallSensor.timeout_count;
      if (hallSensor_timeout_tick > MIN_NUM_OF_TIMEOUT_TO_RUN)
      {
        m_vc.hacky_break = true;
        m_vc.loop_tick = 0;
      }
    }
#endif // (ENABLE_MOTOR_FEEDBACK && ENABLE_HACKY_TICK)

  }
  else if (throttleAxis < m_vc.joystick_config->throttleBraking.val)
  {
    // braking
    throttleSpd_req = m_vc.joystick_config->throttleBraking.speed_cm_per_s;
  }
  else
  {
    // do nothing
  }
  
  /// - output - NOTE: since the lower level will handle optimization, so we dont have to keep tracking the old values
  *out_convertedAng = steeringAng_req;
  *out_convertedSpd = throttleSpd_req;
  ret &= VC_requestSteering(steeringAng_req);
  ret &= VC_requestThrottle(throttleSpd_req);
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
#if (ENABLE_MOTOR_FEEDBACK)
static void configEncoders(void)
{
  m_vc.hallSensor.ftmIsrFlag = false;
  FTM_GetDefaultConfig(&m_vc.hallSensor.ftmInfo);
}

static void initEncoders(void)
{
  FTM_GetDefaultConfig(&m_vc.hallSensor.ftmInfo);
  /* Initialize FTM module */
  FTM_Init(BOARD_FTM_BASEADDR, &m_vc.hallSensor.ftmInfo);
  /* Setup dual-edge capture on a FTM channel pair */
  FTM_SetupInputCapture(BOARD_FTM_BASEADDR, BOARD_FTM_INPUT_CAPTURE_CHANNEL, kFTM_FallingEdge, 0);
  /* Set the timer to be in free-running mode */
  BOARD_FTM_BASEADDR->MOD = 0xFFFF;
  /* Enable channel interrupt when the second edge is detected */
  FTM_EnableInterrupts(BOARD_FTM_BASEADDR, FTM_CHANNEL_INTERRUPT_ENABLE);
  /* Enable at the NVIC */
  EnableIRQ(FTM_INTERRUPT_NUMBER);
  FTM_StartTimer(BOARD_FTM_BASEADDR, kFTM_SystemClock);
}

void FTM_INPUT_CAPTURE_HANDLER(void)
{
    m_vc.hallSensor.count ++;
    m_vc.hallSensor.time_ms = SERVO_TIMER_getCurrentTimeElapsed();
    uint32_t tps = 1000/m_vc.hallSensor.time_ms;
    m_vc.hallSensor.turn_per_second = (m_vc.hallSensor.turn_per_second*9)/10 + tps/10;
    if(m_vc.hallSensor.time_ms > MIN_TICK_TO_BREAK)
    {
      m_vc.hallSensor.timeout_count ++;
    }
    else
    {
      m_vc.hallSensor.timeout_count = 0;
    }

    if ((FTM_GetStatusFlags(BOARD_FTM_BASEADDR) & FTM_CHANNEL_FLAG) == FTM_CHANNEL_FLAG)
    {
        /* Clear interrupt flag.*/
        FTM_ClearStatusFlags(BOARD_FTM_BASEADDR, FTM_CHANNEL_FLAG);
    }
    m_vc.hallSensor.ftmIsrFlag = true;
    __DSB();
}
#endif //(ENABLE_MOTOR_FEEDBACK)

