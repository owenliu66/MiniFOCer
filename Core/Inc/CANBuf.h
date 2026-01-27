#ifndef CANBUF_H
#define CANBUF_H

#include "fdcan.h"

typedef struct {
    uint8_t top, bot; // top points to next empty slot, bot points to oldest message
    uint32_t id_buf[256]; // circular buffer
} CAN_flagBuf;


void enQueue(CAN_flagBuf* buf, uint32_t id);
void deQueue(CAN_flagBuf* buf, FDCAN_HandleTypeDef* instance);

#endif