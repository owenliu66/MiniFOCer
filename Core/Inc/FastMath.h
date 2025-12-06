/*
 * FastMath.h - Fast math lookup tables
 * Created by Yandong Liu, 20250510
*/

#ifndef FASTMATH_H
#define FASTMATH_H

#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "hrtim.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"


int32_t lookupTbl(const int32_t* source, const int32_t* target, const uint32_t size, const int32_t value);
float lookupTblf(const float* source, const float* target, const uint32_t size, const float value);
float flookupTbll(const int32_t* source, const float* target, const uint32_t size, const int32_t value);

#endif
