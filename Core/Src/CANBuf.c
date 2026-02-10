#include "CANBuf.h"
#include <stdint.h>
#include <string.h>
#include "defines.h"
#include "FOC.h"
#include "fdcan.h"
#include <math.h>

extern volatile FOC_data* FOC1;
extern volatile FOC_data* FOC2;
extern FDCAN_TxHeaderTypeDef TxHeader;
extern uint8_t TxData[64];
extern float V_drv;
extern uint8_t errors;

void enQueue(CAN_flagBuf* buf, uint32_t id) {
    if (buf->top - buf->bot < 255) {
        buf->id_buf[buf->top] = id;
        buf->top++;
    }
}

void deQueue(CAN_flagBuf* buf, FDCAN_HandleTypeDef* instance) {
    if (buf->top != buf->bot) {
        uint16_t data_u16[4];
        int16_t data_i16[4];
        TxHeader.Identifier = buf->id_buf[buf->bot];
        buf->bot++;
        switch (TxHeader.Identifier)
        {
        case CAN_STAT1_ID:
            data_i16[0] = FOC1->I_q_avg * 100.0f;
            data_i16[1] = FOC2->I_q_avg * 100.0f;
            data_i16[2] = FOC1->motor_speed * 60e6f / N_STEP_ENCODER;
            data_i16[3] = FOC2->motor_speed * 60e6f / N_STEP_ENCODER;
            memcpy(TxData, data_i16, 8);
            TxHeader.DataLength = FDCAN_DLC_BYTES_8;
            HAL_FDCAN_AddMessageToTxFifoQ(instance, &TxHeader, TxData);
            break;
        case CAN_STAT2_ID:
            data_i16[0] = FOC1->V_bus * 100.0f;
            data_i16[1] = V_drv * 100.0f;
            memcpy(TxData, data_i16, 4);
            TxData[4] = errors;
            TxHeader.DataLength = FDCAN_DLC_BYTES_5;
            HAL_FDCAN_AddMessageToTxFifoQ(instance, &TxHeader, TxData);
            break;
        case CAN_STAT3_ID:
            data_i16[0] = FOC1->cmd_q * 1e3f;
            data_i16[1] = FOC2->cmd_q * 1e3f;
            data_i16[2] = FOC1->cmd_d * 1e3f;
            data_i16[3] = FOC2->cmd_d * 1e3f;
            memcpy(TxData, data_i16, 8);
            TxHeader.DataLength = FDCAN_DLC_BYTES_8;
            HAL_FDCAN_AddMessageToTxFifoQ(instance, &TxHeader, TxData);
            break;
        case CAN_RDBK_ID:
            data_u16[0] = FOC1->Encoder_os;
            data_u16[1] = FOC2->Encoder_os;
            memcpy(TxData, data_u16, 4);
            TxData[4] = roundf(FOC1->motor_kv * 6e6 / N_STEP_ENCODER);
            TxData[5] = roundf(FOC2->motor_kv * 6e6 / N_STEP_ENCODER);
            TxHeader.DataLength = FDCAN_DLC_BYTES_6;
            HAL_FDCAN_AddMessageToTxFifoQ(instance, &TxHeader, TxData);
            break;
        default:
            break;
        }
    }
}