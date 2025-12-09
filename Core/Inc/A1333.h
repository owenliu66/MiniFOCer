#ifndef A1333_H
#define A1333_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32g474xx.h"

typedef struct {
    uint16_t angle;
    float speed;
    uint32_t sampleTime;

    volatile uint32_t* micros;
    SPI_TypeDef* SPIx;
    GPIO_TypeDef* CS_Port;
    uint32_t CS_Pin;

    bool isUpdateOngoing;
} A1333_t;


// read a register from the A1333 sensor
uint16_t A1333_ReadReg(A1333_t* sensor, uint8_t reg);

// write a value to a register on the A1333 sensor
uint16_t A1333_WriteReg(A1333_t* sensor, uint8_t reg, uint8_t value);

// read a register immediately, reg sets the register read next time this is called
uint16_t A1333_ReadRegImmediate(A1333_t* sensor, uint8_t reg);

// initialize SPI and CS pin
void A1333_Init(A1333_t* sensor);

// read 15-bit angle from the A1333 sensor
uint16_t A1333_ReadAngle15(A1333_t* sensor);

// put this in the SPI interrupt handler
void A1333_SPI_IRQHandler(A1333_t* sensor);

// update angle and speed values
void A1333_Update(A1333_t* sensor);

#endif /* A1333_H */
