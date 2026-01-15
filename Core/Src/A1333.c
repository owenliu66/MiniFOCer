#include "A1333.h"
#include "spi.h"

int16_t compensation[] = {17, 21, 17, 25, 14, 9, 12, -9, -4, -13, -30, -26, -49, -56, -60, -81, -78, -101, -113, -115, -135, -143, -154, -167, -167, -183, -187, -183, -189, -179, -181, -184, -174, -177, -169, -162, -169, -152, -151, -146, -131, -137, -127, -111, -120, -101, -99, -96, -79, -86, -80, -71, -77, -66, -67, -73, -61, -73, -67, -62, -77, -71, -74, -86, -80, -91, -92, -91, -106, -98, -108, -112, -104, -111, -108, -100, -110, -101, -104, -106, -95, -96, -88, -80, -85, -71, -66, -64, -49, -56, -42, -30, -33, -19, -16};
uint16_t compensation_size = sizeof(compensation) / sizeof(compensation[0]);

uint16_t A1333_ReadReg(A1333_t* sensor, uint8_t reg) {
    reg &= 0x3F; // Ensure reg is 6 bits
    uint16_t sendData = ((reg & 0x3F) << 8);

    while (sensor->isUpdateOngoing);

    LL_GPIO_ResetOutputPin(sensor->CS_Port, sensor->CS_Pin);
    LL_SPI_TransmitData16(sensor->SPIx, sendData);
    while (!LL_SPI_IsActiveFlag_RXNE(sensor->SPIx));
    LL_GPIO_SetOutputPin(sensor->CS_Port, sensor->CS_Pin);
    uint16_t receiveData = LL_SPI_ReceiveData16(sensor->SPIx);

    LL_GPIO_ResetOutputPin(sensor->CS_Port, sensor->CS_Pin);
    LL_SPI_TransmitData16(sensor->SPIx, 0);
    while (!LL_SPI_IsActiveFlag_RXNE(sensor->SPIx));
    LL_GPIO_SetOutputPin(sensor->CS_Port, sensor->CS_Pin);
    receiveData = LL_SPI_ReceiveData16(SPI1);
    return receiveData;
}

uint16_t A1333_WriteReg(A1333_t* sensor, uint8_t reg, uint8_t value) {
    reg &= 0x3F; // Ensure reg is 6 bits
    uint16_t sendData = 1U << 14 | reg << 8 | value;

    while (sensor->isUpdateOngoing);

    LL_GPIO_ResetOutputPin(sensor->CS_Port, sensor->CS_Pin);
    LL_SPI_TransmitData16(sensor->SPIx, sendData);
    while (!LL_SPI_IsActiveFlag_RXNE(sensor->SPIx));
    LL_GPIO_SetOutputPin(sensor->CS_Port, sensor->CS_Pin);
    uint16_t receiveData = LL_SPI_ReceiveData16(sensor->SPIx);
    return receiveData;
}

uint16_t A1333_ReadRegImmediate(A1333_t* sensor, uint8_t reg) {
    reg &= 0x3F; // Ensure reg is 6 bits
    uint16_t sendData = ((reg & 0x3F) << 8);

    while (sensor->isUpdateOngoing);

    LL_GPIO_ResetOutputPin(sensor->CS_Port, sensor->CS_Pin);
    LL_SPI_TransmitData16(sensor->SPIx, sendData);
    while (!LL_SPI_IsActiveFlag_RXNE(sensor->SPIx));
    LL_GPIO_SetOutputPin(sensor->CS_Port, sensor->CS_Pin);
    uint16_t receiveData = LL_SPI_ReceiveData16(sensor->SPIx);
    return receiveData;
}

void A1333_Init(A1333_t* sensor) {
    LL_GPIO_SetOutputPin(sensor->CS_Port, sensor->CS_Pin);
    LL_SPI_Enable(sensor->SPIx);
    sensor->angle = 0;
    sensor->speed = 0;
    sensor->sampleTime = 0;
    sensor->isUpdateOngoing = false;
}

uint16_t A1333_ReadAngle15(A1333_t* sensor) {
    sensor->angle = A1333_ReadReg(sensor, 0x32);
    return sensor->angle;
}

void A1333_SPI_IRQHandler(A1333_t* sensor) {
    uint32_t sampleTime = *(sensor->micros) - 4; // 1us sensor delay + 3us SPI delay
    int32_t receivedData = LL_SPI_ReceiveData16(sensor->SPIx);
    LL_GPIO_SetOutputPin(sensor->CS_Port, sensor->CS_Pin);
    LL_SPI_DisableIT_RXNE(sensor->SPIx);
    sensor->isUpdateOngoing = false;
    
    // receivedData += compensation[receivedData * compensation_size / 32768];
    int32_t delta = receivedData - sensor->angle;
    if (delta > 16384) {
      delta -= 32768;
    } else if (delta < -16384) {
      delta += 32768;
    }
    // sampleTime = sampleTime > sensor->sampleTime ? sampleTime : sensor->sampleTime + 1;
    float newSpeed = (float)delta / (float)(sampleTime - sensor->sampleTime);
    sensor->speed += (newSpeed - sensor->speed) * 0.01f;
    // sensor->speed = newSpeed;
    sensor->sampleTime = sampleTime;
    sensor->angle = receivedData;
}

void A1333_Update(A1333_t* sensor) {
    while (sensor->isUpdateOngoing);
    __disable_irq();
    sensor->isUpdateOngoing = true;
    LL_GPIO_ResetOutputPin(sensor->CS_Port, sensor->CS_Pin);
    LL_SPI_EnableIT_RXNE(sensor->SPIx);
    LL_SPI_TransmitData16(sensor->SPIx, 0x32 << 8);
    __enable_irq();
}