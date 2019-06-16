/*
 * servo.c
 *
 *  Created on: Apr 28, 2019
 *      Author: jackxu
 */
#include "Servo.h"

#include <stdint.h>
#include <stdbool.h>

#include "fsl_ftm.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* FTM Hardware Config */
// The Flextimer instance/channel used for board 
#define SERVO_FTM_BASEADDR                      (FTM0)
// Interrupt number and interrupt handler for the FTM instance used 
#define SERVO_FTM_IRQ_NUM                       (FTM0_IRQn)
#define SERVO_FTM_HANDLER                       (FTM0_IRQHandler)
// Get source clock for FTM driver 
#define FTM_SOURCE_CLOCK                        (CLOCK_GetFreq(kCLOCK_CoreSysClk)/4)

/* PWM Settings */
#define SERVO_COMMON_FTM_PWM_PERIOD_US          (50U) // 50us per period, increase this, will reduce cpu load, but will reduce resolution

/* Servo Pereference */
#define SERVO_MAX_NUM_SERVO                     (5U)
/* State machine Helper functions */
#define SERVO_GO_TO_NEXT_STATE(index, newState) (m_servos.pwms[(index)].status = (newState))
#define SERVO_OUTPUT(index, logic)              (GPIO_PinWrite(m_servos.configs[(index)].gpio.port, m_servos.configs[(index)].gpio.pin, logic))
#define SERVO_UPDATE_PULSE_WIDTH(index)         (m_servos.pwms[(index)].current_pwm_width_us = m_servos.pwms[(index)].desired_pwm_width_us)
#define SERVO_PULSE_WIDTH_ISVALID(index)        (m_servos.pwms[(index)].current_pwm_width_us >= SERVO_COMMON_FTM_PWM_PERIOD_US)
#define SERVO_RESET_PWM_COUNTER(index)          (m_servos.pwms[(index)].counter_tick = 0)
#define SERVO_IS_END_OF_PULSE(index)            (m_servos.pwms[(index)].counter_tick*SERVO_COMMON_FTM_PWM_PERIOD_US >= (m_servos.pwms[(index)].current_pwm_width_us))
#define SERVO_IS_END_OF_PWM_PERIOD(index)       (m_servos.pwms[(index)].counter_tick*SERVO_COMMON_FTM_PWM_PERIOD_US >= (m_servos.configs[(index)].refreshingPeriod))
#define SERVO_IS_THERE_NEW_REQUEST(index)       (m_servos.pwms[(index)].current_pwm_width_us != m_servos.pwms[(index)].desired_pwm_width_us)

/*******************************************************************************
 * typedef
 ******************************************************************************/

/*******************************************************************************
 * typedef
 ******************************************************************************/
typedef struct{
    uint32_t            counter_tick;
    uint16_t            current_pwm_width_us;
    uint16_t            desired_pwm_width_us;
    SERVO_PWM_STATUS_E  status;
} SERVO_pwm_data_S;

typedef struct{
    const SERVO_ServoConfig_S* configs;
    bool                ftmIsrFlag;
    bool                configed;
    uint8_t             size;
    SERVO_pwm_data_S     pwms[SERVO_MAX_NUM_SERVO];
} SERVO_data_S;

/*******************************************************************************
 * private variables
 ******************************************************************************/
static volatile SERVO_data_S           m_servos = {0};

/*******************************************************************************
 * private function prototypes
 ******************************************************************************/
static inline void runStateMachine(uint8_t index);

/*******************************************************************************
 * public function
 ******************************************************************************/
void SERVO_onDestroy(void)
{
    m_servos.configs = NULL;
}

bool SERVO_init(const SERVO_ServoConfig_S* configs, uint8_t size)
{   
    int i = 0;
    gpio_pin_config_t servo_motor_gpio_config = {
        kGPIO_DigitalOutput, 0,
    };
    ftm_config_t ftmInfo;

    if(SERVO_MAX_NUM_SERVO > size)
    {  
        // copy device config
        m_servos.configs = configs;
        for(i = 0; i < size; i++)
        {
            // init pwms 
            m_servos.pwms[i].status = PWM_STATUS_IDLE;
            m_servos.pwms[i].counter_tick = 0;
            // init gpios
            GPIO_PinInit(m_servos.configs[i].gpio.port, m_servos.configs[i].gpio.pin, &servo_motor_gpio_config);
        }
        m_servos.size = size;

        // init FTM
        FTM_GetDefaultConfig(&ftmInfo);
        /* Divide FTM clock by 4 */
        ftmInfo.prescale = kFTM_Prescale_Divide_4;
        /* Initialize FTM module */
        FTM_Init(SERVO_FTM_BASEADDR, &ftmInfo);
        /*
        * Set timer period.
        */
        FTM_SetTimerPeriod(SERVO_FTM_BASEADDR, USEC_TO_COUNT(SERVO_COMMON_FTM_PWM_PERIOD_US, FTM_SOURCE_CLOCK)); // 10 usec
        FTM_EnableInterrupts(SERVO_FTM_BASEADDR, kFTM_TimeOverflowInterruptEnable);
        EnableIRQ(SERVO_FTM_IRQ_NUM);
        FTM_StartTimer(SERVO_FTM_BASEADDR, kFTM_SystemClock);

        m_servos.configed = true;
    }
    return (m_servos.configed);
}

bool SERVO_write_us(uint8_t index, uint16_t pulseWidth_us)
{
    bool ret = false;
    if((index < m_servos.size)
        &&(pulseWidth_us <= m_servos.configs[index].maxPulseWidth_us)
        &&(pulseWidth_us >= m_servos.configs[index].minPulseWidth_us))
    {
        m_servos.pwms[index].desired_pwm_width_us = pulseWidth_us;
        ret = true;
    }
    return ret;
}

bool SERVO_requestStart(uint8_t index)
{
    bool ret = false;
    if(index < m_servos.size)
    {
        // @ next tick, it will perform for this state
        SERVO_GO_TO_NEXT_STATE(index, PWM_STATUS_INIT);
        ret = true;
    }
    return ret;
}

bool SERVO_doStop(uint8_t index)
{
    bool ret = false;
    if(index < m_servos.size)
    {
        // shut down immediately
        SERVO_OUTPUT(index, LOW);
        SERVO_GO_TO_NEXT_STATE(index, PWM_STATUS_IDLE);
        ret = true;
    }
    return ret;
}

bool SERVO_goDefault(uint8_t index)
{
    bool ret = false;
    if(index < m_servos.size)
    {
        m_servos.pwms[index].desired_pwm_width_us = m_servos.configs[index].defaultPulseWidth_us;
        ret = true;
    }
    return ret;
}

SERVO_PWM_STATUS_E SERVO_getStatus(uint8_t index)
{
    SERVO_PWM_STATUS_E ret = PWM_STATUS_UNKNOWN;
    if(index < m_servos.size)
    {
        ret =  m_servos.pwms[index].status;
    }
    return ret;
}

bool SERVO_getNotifiedByNewTick(void)
{
    bool ret = false;
    if(m_servos.ftmIsrFlag)
    {
        ret = true;
        m_servos.ftmIsrFlag = false; //reset to get a new notification
    }
    return (ret);
}

/*******************************************************************************
 * private function
 ******************************************************************************/
void SERVO_FTM_HANDLER(void)
{
    /* Clear interrupt flag.*/
    FTM_ClearStatusFlags(SERVO_FTM_BASEADDR, kFTM_TimeOverflowFlag);
    m_servos.ftmIsrFlag = true;
    // run through TWO servo,
    /*NOTE:
     *  Do not use loop, it will not work,
     *  it's forbidden to stay too long in ISR.
     *
     */
    runStateMachine(0);
    runStateMachine(1);

    __DSB();
}

static inline void runStateMachine(uint8_t index)
{
    // out of boundary
    if(index >= m_servos.size)
        return;
    // ideally this should will not overflow, but in case, we will go to FALSE state
    m_servos.pwms[index].counter_tick ++;
    if(m_servos.pwms[index].counter_tick >= (0xFFFFFFFF))
    {
        SERVO_GO_TO_NEXT_STATE(index, PWM_STATUS_FAULT);
    }
    // state machine
    switch(m_servos.pwms[index].status)
    {
        case (PWM_STATUS_IDLE):
            //KEEP LOW
            SERVO_OUTPUT(index, LOW);
            SERVO_RESET_PWM_COUNTER(index);
            break;

        case (PWM_STATUS_INIT):
            //toggle High
            SERVO_UPDATE_PULSE_WIDTH(index);
            if(SERVO_PULSE_WIDTH_ISVALID(index))
            {
              SERVO_GO_TO_NEXT_STATE(index, PWM_STATUS_UPDATED);
            }
            break;

        case (PWM_STATUS_UPDATED):
            //toggle High
            SERVO_OUTPUT(index, HIGH);
            SERVO_RESET_PWM_COUNTER(index);
            SERVO_GO_TO_NEXT_STATE(index, PWM_STATUS_ENABLED);
            break;

        case (PWM_STATUS_ENABLED):
            if(SERVO_IS_END_OF_PULSE(index))
            {
                SERVO_OUTPUT(index, LOW);
                SERVO_GO_TO_NEXT_STATE(index, PWM_STATUS_DISABLED);
            }
            break;

        case (PWM_STATUS_DISABLED):
            if(SERVO_IS_END_OF_PWM_PERIOD(index))
            {
                if(SERVO_IS_THERE_NEW_REQUEST(index))
                {
                    SERVO_GO_TO_NEXT_STATE(index, PWM_STATUS_INIT);
                }
                else
                {
                    SERVO_GO_TO_NEXT_STATE(index, PWM_STATUS_UPDATED);
                }
            }
            break;

        case (PWM_STATUS_COUNT):
        case (PWM_STATUS_UNKNOWN):
        case (PWM_STATUS_FAULT):
        default:
            // Toggle Low
            SERVO_RESET_PWM_COUNTER(index);
            SERVO_OUTPUT(index, LOW);
            break;
    }
}

