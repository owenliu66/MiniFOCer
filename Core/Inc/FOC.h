/*
 * FOC.h - library for implementing a FOC controller
 * Created by Yandong Liu, 20250510
*/

#ifndef FOC_H
#define FOC_H

#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "hrtim.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

// FOC variables
typedef struct {
    // Pad RAM
    volatile uint32_t pad1[64];

    // Physical variables
    volatile float U_current, V_current, W_current;
    volatile float motor_speed;
    volatile uint32_t F_sw;
    volatile int32_t motor_PhysPosition;
    volatile float motor_ElecPosition, motor_ElecPosition_next;
    volatile int32_t Encoder_os;
    volatile uint32_t motor_lastMeasTime;

    volatile int32_t temp_it, temp_it_next;
    volatile float sin_elec_position, cos_elec_position;
    volatile float sin_elec_position_next, cos_elec_position_next;
    volatile float I_a, I_b, I_q, I_d;
    volatile float I_q_avg, I_d_avg;
    volatile float cmd_q, cmd_d;
    volatile float cmd_a, cmd_b;
    volatile float integ_q, integ_d;
    volatile float Kp_Iq, Ki_Iq;
    volatile float Kp_Id, Ki_Id;
    volatile float I_d_err, I_q_err;
    volatile float TargetCurrent;
    volatile float TargetFieldWk;

    // SVPWM variables
    volatile uint8_t SVPWM_sector;
    volatile float SVPWM_mag, SVPWM_ang;
    volatile float SVPWM_Ti[4];
    volatile float SVPWM_Tb1, SVPWM_Tb2;
    volatile float SVPWM_beta;

    // Output variables
    volatile float duty_u, duty_v, duty_w;

    // Timer definitions
    uint32_t U_TIMER, V_TIMER, W_TIMER; 

    // Pad RAM
    volatile uint32_t pad2[64];
} FOC_data;

// FOC functions
void FOC_update(volatile FOC_data* self);


#endif