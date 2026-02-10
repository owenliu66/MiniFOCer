/*
 * FOC.c - library for implementing a FOC controller
 * Created by Yandong Liu, 20250510
*/

#include "FOC.h"    
#include "defines.h"
#include "FastMath.h"
#include "math.h"

extern void writePwm(uint32_t timer, int32_t duty);

extern volatile int16_t adc_data[64];
extern volatile int16_t adc_os[64];

extern const float math_LookupX[];
extern const float sin_LookupY[];
extern const float cos_LookupY[];
extern const uint32_t math_LookupSize;
extern const int32_t sin_LookupXl[];
extern const int32_t cos_LookupXl[];

volatile const uint8_t SVPWM_PermuataionMatrix[6][3] = {
	{ 1, 2, 0 },
	{ 3, 1, 0 },
	{ 0, 1, 2 },
	{ 0, 3, 1 },
	{ 2, 0, 1 },
	{ 1, 0, 3 }
};

void FOC_update(volatile FOC_data* self) {
    // FOC
    self->temp_it = self->motor_PhysPosition - self->Encoder_os;
    self->temp_it_next = self->temp_it;
    // compensate for the time delay if motor speed is high
    // if (fabsf(self->motor_speed) > ((300.0f/60.0f)*1e-6f*N_STEP_ENCODER * 0.02f)){
    //     uint32_t TIM2_CNT = TIM2->CNT;
    //     self->temp_it      -= ((int32_t)((TIM2_CNT - self->motor_lastMeasTime)) + (int32_t)(0UL << self->F_sw)) * (int32_t)(self->motor_speed);
    //     self->temp_it_next -= ((int32_t)((TIM2_CNT - self->motor_lastMeasTime)) + (int32_t)(19UL << self->F_sw)) * (int32_t)(self->motor_speed);
    // }
    self->temp_it *= N_POLES;
    self->temp_it %= N_STEP_ENCODER;
    if (self->temp_it < 0) self->temp_it += N_STEP_ENCODER;
    self->motor_ElecPosition = (float)(self->temp_it) / (float)N_STEP_ENCODER;
    self->temp_it_next *= N_POLES;
    self->temp_it_next %= N_STEP_ENCODER;
    if (self->temp_it_next < 0) self->temp_it_next += N_STEP_ENCODER;
    self->motor_ElecPosition_next = (float)(self->temp_it_next) / (float)N_STEP_ENCODER;
    // motor_ElecPosition = 0.0f;
    // self->sin_elec_position = sinf(self->motor_ElecPosition * PIx2);
    // self->cos_elec_position = cosf(self->motor_ElecPosition * PIx2);
    // self->sin_elec_position_next = sinf(self->motor_ElecPosition_next * PIx2);
    // self->cos_elec_position_next = cosf(self->motor_ElecPosition_next * PIx2);
    self->sin_elec_position = lookupTblf(math_LookupX, sin_LookupY, math_LookupSize, self->motor_ElecPosition);
    self->cos_elec_position = lookupTblf(math_LookupX, cos_LookupY, math_LookupSize, self->motor_ElecPosition);
    self->sin_elec_position_next = lookupTblf(math_LookupX, sin_LookupY, math_LookupSize, self->motor_ElecPosition_next);
    self->cos_elec_position_next = lookupTblf(math_LookupX, cos_LookupY, math_LookupSize, self->motor_ElecPosition_next);
    // self->sin_elec_position = flookupTbll(sin_LookupXl, sin_LookupY, math_LookupSize, self->temp_it);
    // self->cos_elec_position = flookupTbll(sin_LookupXl, cos_LookupY, math_LookupSize, self->temp_it);
    // self->sin_elec_position_next = flookupTbll(sin_LookupXl, sin_LookupY, math_LookupSize, self->temp_it_next);
    // self->cos_elec_position_next = flookupTbll(sin_LookupXl, cos_LookupY, math_LookupSize, self->temp_it_next);
    // Clarke transform
    self->I_a = self->U_current * 0.66666667f - self->V_current * 0.33333333f - self->W_current * 0.33333333f;
    self->I_b = sqrt3_1o * (self->V_current - self->W_current);
    // Park transform
    self->I_d = self->I_a * self->cos_elec_position + self->I_b * self->sin_elec_position;
    self->I_q = self->I_b * self->cos_elec_position - self->I_a * self->sin_elec_position;
    self->I_d_avg += (self->I_d - self->I_d_avg) * 0.001f;
    self->I_q_avg += (self->I_q - self->I_q_avg) * 0.001f;
    
    // Rev limiter
    // if (fabsf(self->motor_speed) > MAX_SPEED) self->isOverRev = 1;
    // if (fabsf(self->motor_speed) < MAX_SPEED_RECOV) self->isOverRev = 0;

    // PI controllers on Q and D
    self->I_d_err = self->TargetFieldWk - self->I_d;
    self->I_q_err = self->TargetCurrent - self->I_q;
    self->integ_d += self->I_d_err * self->Ki_Id * 25e-6f * ((float)(1U << self->F_sw));
    self->integ_d = (self->integ_d > MAX_CMD_D) ? MAX_CMD_D : self->integ_d;
    self->integ_d = (self->integ_d < MIN_CMD_D) ? MIN_CMD_D : self->integ_d;
    self->cmd_d = self->I_d_err * self->Kp_Id + self->integ_d;
    self->integ_q += self->I_q_err * self->Ki_Iq * 25e-6f * ((float)(1U << self->F_sw));
    self->integ_q = (self->integ_q > MAX_CMD_Q) ? MAX_CMD_Q : self->integ_q;
    self->integ_q = (self->integ_q < MIN_CMD_Q) ? MIN_CMD_Q : self->integ_q;
    self->cmd_q = self->I_q_err * self->Kp_Iq + self->integ_q;// + self->motor_speed / self->motor_kv / self->V_bus;

    // self->cmd_d = 0.0f;
    // self->cmd_q = 0.1f;

    // Inverse Park transform
    self->cmd_a = self->cmd_d * self->cos_elec_position_next - self->cmd_q * self->sin_elec_position_next;
    self->cmd_b = self->cmd_q * self->cos_elec_position_next + self->cmd_d * self->sin_elec_position_next;
    // Inverse Clarke transform
    self->duty_u = self->cmd_a;
    self->duty_v = (self->cmd_a * -0.5f + sqrt3o2 * self->cmd_b);
    self->duty_w = (self->cmd_a * -0.5f - sqrt3o2 * self->cmd_b);
    writePwm(self->U_TIMER, self->duty_u * 32000.0f + 32000U);
    writePwm(self->V_TIMER, self->duty_v * 32000.0f + 32000U);
    writePwm(self->W_TIMER, self->duty_w * 32000.0f + 32000U);

    // SVPWM generation
    // self->SVPWM_mag = hypotf(self->cmd_b, self->cmd_a);
    // self->SVPWM_ang = atan2f(self->cmd_b, self->cmd_a);
    // self->SVPWM_ang += PI;
    // self->SVPWM_sector = (uint8_t)(self->SVPWM_ang * PI_3o);
    // self->SVPWM_beta = self->SVPWM_ang - PIo3 * self->SVPWM_sector;
    // self->SVPWM_Tb1 = self->SVPWM_mag * sinf(PIo3 - self->SVPWM_beta);
    // self->SVPWM_Tb2 = self->SVPWM_mag * sinf(self->SVPWM_beta);
    // self->SVPWM_Ti[0] = (1.0f - self->SVPWM_Tb1 - self->SVPWM_Tb2) * 0.5f;
    // self->SVPWM_Ti[1] = self->SVPWM_Tb1 + self->SVPWM_Tb2 + self->SVPWM_Ti[0];
    // self->SVPWM_Ti[2] = self->SVPWM_Tb2 + self->SVPWM_Ti[0];
    // self->SVPWM_Ti[3] = self->SVPWM_Tb1 + self->SVPWM_Ti[0];
    // self->duty_u = self->SVPWM_Ti[SVPWM_PermuataionMatrix[self->SVPWM_sector][0]];
    // self->duty_v = self->SVPWM_Ti[SVPWM_PermuataionMatrix[self->SVPWM_sector][1]];
    // self->duty_w = self->SVPWM_Ti[SVPWM_PermuataionMatrix[self->SVPWM_sector][2]];
    // writePwm(U1_TIMER, self->duty_u * 64000.0f);
    // writePwm(V1_TIMER, self->duty_v * 64000.0f);
    // writePwm(W1_TIMER, self->duty_w * 64000.0f);
    // if (sendCANBus_flag == 0) sendCANBus_flag = 1;
    
    // TxHeaderIT.BitRateSwitch = FDCAN_BRS_OFF;
    // TxHeaderIT.DataLength = FDCAN_DLC_BYTES_8;
    // TxHeaderIT.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    // TxHeaderIT.FDFormat = FDCAN_CLASSIC_CAN;
    // TxHeaderIT.Identifier = CAN_DEBUG_ID;
    // TxHeaderIT.IdType = FDCAN_EXTENDED_ID;
    // TxHeaderIT.MessageMarker = 0;
    // TxHeaderIT.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    // TxHeaderIT.TxFrameType = FDCAN_DATA_FRAME;
    // uint32_t testtest = 0x0UL;
    // TxDataIT[0] = (uint8_t)0x00;
    // TxDataIT[1] = (uint8_t)0x00;
    // TxDataIT[2] = (uint8_t)0x00;
    // TxDataIT[3] = (uint8_t)0x00;
    // // memcpy(&TxDataIT[0], &testtest, 4);
    // uint16_t temp = motor_ElecPosition * 10000.0f;
    // memcpy(&TxDataIT[4], &temp, 2);
    // uint16_t temp2 = motor_PhysPosition;
    // memcpy(&TxDataIT[6], &temp2, 2);
    // HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeaderIT, TxDataIT);
}
