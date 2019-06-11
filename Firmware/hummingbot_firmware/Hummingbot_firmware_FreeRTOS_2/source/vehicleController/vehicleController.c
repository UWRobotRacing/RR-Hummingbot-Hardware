// /*
//  * servo.c
//  *
//  *  Created on: Apr 28, 2019
//  *      Author: jackxu
//  */

// #include <stdint.h>
// #include <stdbool.h>

// #include "Servo.h"
// #include "common.h"

// /*******************************************************************************
//  * Definitions
//  ******************************************************************************/

// /*******************************************************************************
//  * Definitions
//  ******************************************************************************/
// /* FTM Hardware Config */
// // The Flextimer instance/channel used for board 
// #define SERVO_FTM_BASEADDR                      (FTM0)
// // Interrupt number and interrupt handler for the FTM instance used 
// #define SERVO_FTM_IRQ_NUM                       (FTM0_IRQn)
// #define SERVO_FTM_HANDLER                       (FTM0_IRQHandler)
// // Get source clock for FTM driver 
// #define FTM_SOURCE_CLOCK                        (CLOCK_GetFreq(kCLOCK_CoreSysClk)/4)

// /* PWM Settings */
// #define SERVO_COMMON_FTM_PWM_US_PER_TICK        (10U)
// #define REFRESHING_PERIOD                       (SERVO_COMMON_FTM_PWM_US_PER_TICK)

// /* Helper functions */
// #define GO_TO_NEXT_STATE(newState)              (PWM_Status = (newState))

// // #define PWM_SERVO_STEERING_REFRESHING_PERIOD    (3003) //us //333Hz

// /*******************************************************************************
//  * typedef
//  ******************************************************************************/

// /*******************************************************************************
//  * typedef
//  ******************************************************************************/
// typedef struct{
//     VC_ServoConfig_S    config,
//     uint16_t            requestedPulseWidth_us,
//     uint16_t            pulseWidth_us,
//     uint32_t            FTM_microsecondCounter_tick,
// } VC_Servo_S;

// typedef struct{
//     VC_Servo_S servo1;
//     VC_Servo_S servo2; 
// } VC_data_S;

// typedef struct{
//     bool        ftmIsrFlag,
//     uint32_t    milisecondCounts
// } VC_ftm_pwm_data_S;
// /*******************************************************************************
//  * private variables
//  ******************************************************************************/
// static volatile VC_data_S           VC_data;
// static volatile VC_ftm_pwm_data_S   VC_ftm_pwm_data =  {    .ftmIsrFlag         = false,
//                                                             .milisecondCounts   = 0
//                                                         };

// /*******************************************************************************
//  * private function prototypes
//  ******************************************************************************/


// /*******************************************************************************
//  * public function
//  ******************************************************************************/
// void VC_init(VC_ServoConfig_S servo1, VC_ServoConfig_S servo2)
// {
    
// }
// /*******************************************************************************
//  * private function
//  ******************************************************************************/