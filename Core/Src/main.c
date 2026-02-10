/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "hrtim.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include "defines.h"
#include "FastMath.h"
#include "FOC.h"
#include "A1333.h"
#include "CANBuf.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  uint8_t n_poles;
  float kv, Res, Ind;
  uint16_t encoder_os;
} motor_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[64];
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[64];

// FOC variables
volatile FOC_data* FOC1;
volatile FOC_data* FOC2;

// Motor variables
float TargetCurrent1 = 2.0f, TargetCurrent2 = 2.0f;
float MaxCurrent = 10.0f, MaxAbsCurrent = 40.0f;
motor_t motor_1 = {
  .Ind = 120e-6,
  .Res = 0.26,
  .kv = 360,
  .n_poles = 7,
};
motor_t motor_2 = {
  .Ind = 120e-6,
  .Res = 0.26,
  .kv = 360,
  .n_poles = 7,
};

// Power status variables
float V_bus = 0, V_drv = 0;
float UVLO_thresh = 9.5, UVLO_hyst = 0.5;

// Encoders
A1333_t encoder_1 = {
  .SPIx = SPI3,
  .CS_Port = GPIOA,
  .CS_Pin = LL_GPIO_PIN_15,
  .micros = &(TIM2->CNT),
};
A1333_t encoder_2 = {
  .SPIx = SPI1,
  .CS_Port = GPIOD,
  .CS_Pin = LL_GPIO_PIN_2,
  .micros = &(TIM2->CNT),
};

// ADC variables
volatile int16_t adc_data[64];
volatile int16_t adc_os[64] = {0, 0, 0, 0, 0, 0, 0, 0};

// timing stuff
uint32_t micros = 0;
volatile bool wait = true;

// CANBus stuff
volatile uint8_t sendCANBus_flag = 0;
CAN_flagBuf CAN_TxBuffer = {0};

// For fun
bool audio_enable_1 = true, audio_enable_2 = true;

// Errors
uint8_t errors = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void resetGateDriver(uint8_t motor);
void disableGateDriver(uint8_t motor);
void writePwm(uint32_t timer, int32_t duty);
void measureEncoderOs(uint8_t motor, float TestCurrent);
void measureMotorKv(uint8_t motor, float TestCurrent);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  // SystemClock_Config();
  // LL_mDelay(1000);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  LL_SYSCFG_EnableAnalogBooster();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_HRTIM1_Init();
  MX_SPI3_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_FDCAN1_Init();
  MX_SPI1_Init();
  MX_ADC4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  disableGateDriver(3);

  // allocate memory on heap for FOC data
  FOC1 = (FOC_data*)malloc(sizeof(FOC_data));
  FOC2 = (FOC_data*)malloc(sizeof(FOC_data));
  if (FOC1 == NULL || FOC2 == NULL) {
    // Handle memory allocation failure
    disableGateDriver(3);
    while (1);
  }

  // Disable FPU lazy context save
  FPU->FPCCR &= ~FPU_FPCCR_LSPEN_Msk;

  // Init DWT delay
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;  // Reset counter
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // Enable microsecond counter
  LL_TIM_EnableCounter(TIM2);

  // Enable drive-enable timeout counters
  // LL_TIM_EnableIT_UPDATE(TIM6);
  // LL_TIM_EnableCounter(TIM6);
  // LL_TIM_EnableIT_UPDATE(TIM7);
  // LL_TIM_EnableCounter(TIM7);

  // Encoder setup
  A1333_Init(&encoder_1);
  A1333_Init(&encoder_2);

  // Enable ADC1 (motor 2 phase current sensors) with DMA
  LL_ADC_Enable(ADC1);
  while (!LL_ADC_IsActiveFlag_ADRDY(ADC1));
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&ADC1->DR);
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&adc_data[0]);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 2);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
  LL_ADC_REG_StartConversion(ADC1);

  // Enable ADC2 (motor 1 phase current sensors) with DMA
  LL_ADC_Enable(ADC2);
  while (!LL_ADC_IsActiveFlag_ADRDY(ADC2));
  LL_DMA_SetPeriphAddress(DMA2, LL_DMA_CHANNEL_1, (uint32_t)&ADC2->DR);
  LL_DMA_SetMemoryAddress(DMA2, LL_DMA_CHANNEL_1, (uint32_t)&adc_data[2]);
  LL_DMA_SetDataLength(DMA2, LL_DMA_CHANNEL_1, 2);
  LL_DMA_EnableChannel(DMA2, LL_DMA_CHANNEL_1);
  LL_ADC_REG_StartConversion(ADC2);

  // Enable ADC3 (bus and drive voltage, audio 1) with DMA
  LL_ADC_Enable(ADC3);
  while (!LL_ADC_IsActiveFlag_ADRDY(ADC3));
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t)&ADC3->DR);
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t)&adc_data[4]);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, 3);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
  LL_ADC_REG_StartConversion(ADC3);

  // Enable ADC4 (audio 2) with DMA
  LL_ADC_Enable(ADC4);
  while (!LL_ADC_IsActiveFlag_ADRDY(ADC4));
  LL_DMA_SetPeriphAddress(DMA2, LL_DMA_CHANNEL_2, (uint32_t)&ADC4->DR);
  LL_DMA_SetMemoryAddress(DMA2, LL_DMA_CHANNEL_2, (uint32_t)&adc_data[7]);
  LL_DMA_SetDataLength(DMA2, LL_DMA_CHANNEL_2, 1);
  LL_DMA_EnableChannel(DMA2, LL_DMA_CHANNEL_2);
  LL_ADC_REG_StartConversion(ADC4);

  // CANbus setup
  HAL_FDCAN_Start(&hfdcan1);
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.Identifier = CAN_STAT1_ID;
  TxHeader.IdType = FDCAN_EXTENDED_ID;
  TxHeader.MessageMarker = 0;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;

  // calculate DC bus and gate drive voltage
  LL_mDelay(10);
  V_bus = (float)adc_data[6] * 0.008f;
  V_drv = (float)adc_data[4] * 0.008f;

  // initialize FOC variables
  FOC1->integ_d = 0.0f;
  FOC1->integ_q = 0.0f;
  FOC1->motor_speed = 0.0f;
  FOC1->TargetCurrent = 0.0f;
  FOC1->TargetFieldWk = 0.0f;
  FOC1->U_TIMER = U1_TIMER;
  FOC1->V_TIMER = V1_TIMER;
  FOC1->W_TIMER = W1_TIMER;
  FOC1->motor_kv = 0.1911f; FOC1->V_bus = 14.8f;
  FOC2->integ_d = 0.0f;
  FOC2->integ_q = 0.0f;
  FOC2->motor_speed = 0.0f;
  FOC2->TargetCurrent = 0.0f;
  FOC2->TargetFieldWk = 0.0f;
  FOC2->U_TIMER = U2_TIMER;
  FOC2->V_TIMER = V2_TIMER;
  FOC2->W_TIMER = W2_TIMER;
  FOC2->motor_kv = 0.1911f; FOC2->V_bus = 14.8f;
  // FOC1->Kp_Iq = (80000 >> FOC1->F_sw) * motor_1.Ind / V_bus;
  // FOC1->Ki_Iq = (80000 >> FOC1->F_sw) * motor_1.Res / V_bus;
  // FOC2->Kp_Iq = (80000 >> FOC2->F_sw) * motor_2.Ind / V_bus;
  // FOC2->Ki_Iq = (80000 >> FOC2->F_sw) * motor_2.Res / V_bus;
  // FOC1->Kp_Id = (80000 >> FOC1->F_sw) * motor_1.Ind / V_bus;
  // FOC1->Ki_Id = (80000 >> FOC1->F_sw) * motor_1.Res / V_bus;
  // FOC2->Kp_Id = (80000 >> FOC2->F_sw) * motor_2.Ind / V_bus;
  // FOC2->Ki_Id = (80000 >> FOC2->F_sw) * motor_2.Res / V_bus;
  FOC1->Kp_Iq = 0.1f;
  FOC1->Ki_Iq = 100.0f;
  FOC2->Kp_Iq = 0.1f;
  FOC2->Ki_Iq = 100.0f;
  FOC1->Kp_Id = 0.1f;
  FOC1->Ki_Id = 100.0f;
  FOC2->Kp_Id = 0.1f;
  FOC2->Ki_Id = 100.0f;

  // measure ADC offset
  for (uint16_t i = 0; i < 100; i++){
    LL_mDelay(10);
    adc_os[0] = (adc_os[0]*3 + adc_data[0]) >> 2;
    adc_os[1] = (adc_os[1]*3 + adc_data[1]) >> 2;
    adc_os[2] = (adc_os[2]*3 + adc_data[2]) >> 2;
    adc_os[3] = (adc_os[3]*3 + adc_data[3]) >> 2;
    adc_os[4] = (adc_os[4]*3 + adc_data[4]) >> 2;
    adc_os[5] = (adc_os[5]*3 + adc_data[5]) >> 2;
    adc_os[6] = (adc_os[6]*3 + adc_data[6]) >> 2;
    adc_os[7] = (adc_os[7]*3 + adc_data[7]) >> 2;
  }

  // Enable HRTIM (gate drive signals)
  FOC1->F_sw = LL_HRTIM_TIM_GetPrescaler(HRTIM1, U1_TIMER);
  FOC2->F_sw = LL_HRTIM_TIM_GetPrescaler(HRTIM1, U2_TIMER);
  writePwm(U1_TIMER, 0);
  writePwm(V1_TIMER, 0);
  writePwm(W1_TIMER, 0);
  writePwm(U2_TIMER, 0);
  writePwm(V2_TIMER, 0);
  writePwm(W2_TIMER, 0);
  LL_HRTIM_EnableOutput(HRTIM1, 
    LL_HRTIM_OUTPUT_TA1|
    LL_HRTIM_OUTPUT_TB1|
    LL_HRTIM_OUTPUT_TC1|
    LL_HRTIM_OUTPUT_TD2|
    LL_HRTIM_OUTPUT_TE1|
    LL_HRTIM_OUTPUT_TF1
  );
  LL_HRTIM_TIM_CounterEnable(HRTIM1, LL_HRTIM_TIMER_MASTER
    |U1_TIMER|V1_TIMER|W1_TIMER
    |U2_TIMER|V2_TIMER|W2_TIMER
  );

  // enable HRTIM timer A interrupt(FOC calculations)
  LL_HRTIM_EnableIT_UPDATE(HRTIM1, LL_HRTIM_TIMER_A);

  // LL_mDelay(500);
  measureEncoderOs(3, 5);
  // measureMotorKv(3, 10);
  resetGateDriver(3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t lastCanSendTime = 0;
  while (1)
  {
    micros = TIM2->CNT;
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);

    // calculate DC bus and gate drive voltage
    V_bus = (float)adc_data[6] * 0.008f;
    V_drv = (float)adc_data[4] * 0.008f;

    // move encoder info to FOC struct
    __disable_irq();
    __ASM("nop");
    __ASM("nop");
    __ASM("nop");
    __ASM("nop");
    __ASM("nop");
    wait = true;
    FOC1->motor_PhysPosition = encoder_1.angle;
    FOC1->motor_lastMeasTime = encoder_1.sampleTime;
    FOC1->motor_speed = encoder_1.speed;
    FOC1->U_current = (adc_data[2] - adc_os[2]) * 0.040584415584415584f;
    FOC1->W_current = (adc_data[3] - adc_os[3]) * -0.040584415584415584f;
    FOC1->V_current = -FOC1->U_current - FOC1->W_current; // assuming balanced currents
    FOC1->TargetCurrent = -TargetCurrent1 - audio_enable_1 * (adc_data[7] - adc_os[7]) * 0.01f;
    FOC1->TargetFieldWk = 0.0f; // no field weakening
    FOC2->motor_PhysPosition = encoder_2.angle;
    FOC2->motor_lastMeasTime = encoder_2.sampleTime;
    FOC2->motor_speed = encoder_2.speed;
    FOC2->U_current = (adc_data[0] - adc_os[0]) * 0.040584415584415584f;
    FOC2->W_current = (adc_data[1] - adc_os[1]) * -0.040584415584415584f;
    FOC2->V_current = -FOC2->U_current - FOC2->W_current; // assuming balanced currents
    FOC2->TargetCurrent = TargetCurrent2 + audio_enable_2 * (adc_data[5] - adc_os[5]) * 0.01f;
    FOC2->TargetFieldWk = 0.0f; // no field weakening
    FOC1->V_bus = V_bus; FOC2->V_bus = V_bus;
    // Flush pipeline
    __ASM("nop");
    __ASM("nop");
    __ASM("nop");
    __ASM("nop");
    __ASM("nop");
    __enable_irq();

    // trigger rotor angle measurements
    A1333_Update(&encoder_1);
    A1333_Update(&encoder_2);

    // V_bus UVLO check
    if (V_bus > UVLO_thresh + UVLO_hyst) {
      errors &= !ERR_UVP;
    }
    else if (V_bus < UVLO_thresh - UVLO_hyst) {
      errors |= ERR_UVP;
      disableGateDriver(3);
    }

    // Motor 1 OCP check
    if (fabsf(FOC1->U_current) > MaxAbsCurrent || 
      fabsf(FOC1->V_current) > MaxAbsCurrent || 
      fabsf(FOC1->W_current) > MaxAbsCurrent) {
      errors |= ERR_M1_OCP;
      disableGateDriver(1);
    }
    // Motor 2 OCP check
    if (fabsf(FOC2->U_current) > MaxAbsCurrent || 
      fabsf(FOC2->V_current) > MaxAbsCurrent || 
      fabsf(FOC2->W_current) > MaxAbsCurrent) {
      errors |= ERR_M2_OCP;
      disableGateDriver(2);
    }

    // Rev limiter TODO: fix
    // if (fabsf(encoder_1.speed) > MAX_SPEED) disableGateDriver(1);
    // if (fabsf(encoder_1.speed) < MAX_SPEED_RECOV && errors == 0) resetGateDriver(1);

    if (micros - lastCanSendTime > 20000) {
      lastCanSendTime = micros;
      enQueue(&CAN_TxBuffer, CAN_STAT1_ID);
      enQueue(&CAN_TxBuffer, CAN_STAT2_ID);
      enQueue(&CAN_TxBuffer, CAN_STAT3_ID);
    }
    deQueue(&CAN_TxBuffer, &hfdcan1);

    while (wait && TIM2->CNT - micros < 50);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4)
  {
  }
  LL_PWR_EnableRange1BoostMode();
  LL_RCC_HSI_Enable();
   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  LL_RCC_HSI_SetCalibTrimming(64);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 20, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_EnableDomain_SYS();
  LL_RCC_PLL_Enable();
   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
  }

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }

  /* Insure 1us transition state at intermediate medium speed clock*/
  for (__IO uint32_t i = (170 >> 1); i !=0; i--);

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_SetSystemCoreClock(160000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void resetGateDriver(uint8_t motor){
  if ((motor & 0x1) != 0) LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_13);
  if ((motor & 0x2) != 0) LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_2);
}
void disableGateDriver(uint8_t motor){
  if ((motor & 0x1) != 0) LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_13);
  if ((motor & 0x2) != 0) LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_2);
}

void writePwm(uint32_t timer, int32_t duty){
  if (duty < 100) duty = 0;
  else if (duty > 63900) duty = 64000;
  // if (duty > 32000) {
  //   LL_HRTIM_TIM_SetCompare1(HRTIM1, timer, 0);
  //   LL_HRTIM_TIM_SetCompare3(HRTIM1, timer, duty);
  // }
  // else {
  //   LL_HRTIM_TIM_SetCompare1(HRTIM1, timer, 64000 - duty);
  //   LL_HRTIM_TIM_SetCompare3(HRTIM1, timer, 0);
  // }
  LL_HRTIM_TIM_SetCompare1(HRTIM1, timer, 0);
  LL_HRTIM_TIM_SetCompare3(HRTIM1, timer, duty);
}

void measureEncoderOs(uint8_t motor, float TestCurrent){
  if ((motor & 0x1) != 0) {
    disableGateDriver(2);
    resetGateDriver(1);
    FOC1->Encoder_os = 0;
    TIM2->CNT = 0;
    while (TIM2->CNT < 2000000) {
      micros = TIM2->CNT;
      A1333_Update(&encoder_1);
      __disable_irq();
      __ASM("nop");
      __ASM("nop");
      __ASM("nop");
      __ASM("nop");
      __ASM("nop");
      FOC1->motor_PhysPosition = 0;
      FOC1->TargetCurrent = 0;
      FOC1->TargetFieldWk = TestCurrent;
      FOC1->motor_speed = 0;
      FOC1->U_current = (adc_data[2] - adc_os[2]) * 0.040584415584415584f;
      FOC1->W_current = (adc_data[3] - adc_os[3]) * -0.040584415584415584f;
      FOC1->V_current = -FOC1->U_current - FOC1->W_current; // assuming balanced currents
      __ASM("nop");
      __ASM("nop");
      __ASM("nop");
      __ASM("nop");
      __ASM("nop");
      __enable_irq();
      while (TIM2->CNT - micros < 31);
    }
    motor_1.encoder_os = encoder_1.angle;
    FOC1->Encoder_os = encoder_1.angle;
  }
  if ((motor & 0x2) != 0){
    disableGateDriver(1);
    resetGateDriver(2);
    FOC2->Encoder_os = 0;
    uint16_t count = 0;
    uint32_t sum = 0;
    TIM2->CNT = 0;
    while (TIM2->CNT < 2000000) {
      micros = TIM2->CNT;
      A1333_Update(&encoder_2);
      __disable_irq();
      __ASM("nop");
      __ASM("nop");
      __ASM("nop");
      __ASM("nop");
      __ASM("nop");
      FOC2->motor_PhysPosition = 0;
      FOC2->TargetCurrent = 0;//TestCurrent * sinf((float)TIM2->CNT * 1e-6f * PIx2 * 1.0f);
      FOC2->TargetFieldWk = TestCurrent;
      FOC2->motor_speed = 0;
      FOC2->U_current = (adc_data[0] - adc_os[0]) * 0.040584415584415584f;
      FOC2->W_current = (adc_data[1] - adc_os[1]) * -0.040584415584415584f;
      FOC2->V_current = -FOC2->U_current - FOC2->W_current; // assuming balanced currents
      __ASM("nop");
      __ASM("nop");
      __ASM("nop");
      __ASM("nop");
      __ASM("nop");
      __enable_irq();
      while (TIM2->CNT - micros < 31);
      if (TIM2->CNT > 1000000) {
        count++;
        sum += encoder_2.angle;
      }
    }
    motor_2.encoder_os = encoder_2.angle;//sum / count;
    FOC2->Encoder_os = encoder_2.angle;//motor_2.encoder_os;
  }
  disableGateDriver(3);
}

void measureMotorKv(uint8_t motor, float TestCurrent){
  if ((motor & 0x1) != 0) {
    disableGateDriver(2);
    resetGateDriver(1);
    TIM2->CNT = 0;
    float speed = 0;
    while (TIM2->CNT < 1000000 || fabsf(encoder_1.speed - speed) > 0.01f) {
      micros = TIM2->CNT;
      speed = encoder_1.speed;
      A1333_Update(&encoder_1);

      // move encoder info to FOC struct
      __disable_irq();
      __ASM("nop");
      __ASM("nop");
      __ASM("nop");
      __ASM("nop");
      __ASM("nop");
      FOC1->motor_PhysPosition = encoder_1.angle;
      FOC1->motor_lastMeasTime = encoder_1.sampleTime;
      FOC1->motor_speed = encoder_1.speed;
      FOC1->U_current = (adc_data[2] - adc_os[2]) * 0.040584415584415584f;
      FOC1->W_current = (adc_data[3] - adc_os[3]) * -0.040584415584415584f;
      FOC1->V_current = -FOC1->U_current - FOC1->W_current; // assuming balanced currents
      FOC1->TargetCurrent = TestCurrent;
      FOC1->TargetFieldWk = 0.0f; // no field weakening
      // Flush pipeline
      __ASM("nop");
      __ASM("nop");
      __ASM("nop");
      __ASM("nop");
      __ASM("nop");
      __enable_irq();

      while (TIM2->CNT - micros < 31);
    }
    motor_1.kv = encoder_1.speed / V_bus;
    FOC1->motor_kv = motor_1.kv;
  }
  if ((motor & 0x2) != 0) {
    disableGateDriver(1);
    resetGateDriver(2);
    TIM2->CNT = 0;
    float speed = 0;
    while (TIM2->CNT < 1000000 || fabsf(encoder_2.speed - speed) > 0.01f) {
      micros = TIM2->CNT;
      speed = encoder_2.speed;
      A1333_Update(&encoder_2);

      // move encoder info to FOC struct
      __disable_irq();
      __ASM("nop");
      __ASM("nop");
      __ASM("nop");
      __ASM("nop");
      __ASM("nop");
      FOC2->motor_PhysPosition = encoder_2.angle;
      FOC2->motor_lastMeasTime = encoder_2.sampleTime;
      FOC2->motor_speed = encoder_2.speed;
      FOC2->U_current = (adc_data[0] - adc_os[1]) * 0.040584415584415584f;
      FOC2->W_current = (adc_data[0] - adc_os[1]) * -0.040584415584415584f;
      FOC2->V_current = -FOC2->U_current - FOC2->W_current; // assuming balanced currents
      FOC2->TargetCurrent = TestCurrent;
      FOC2->TargetFieldWk = 0.0f; // no field weakening
      // Flush pipeline
      __ASM("nop");
      __ASM("nop");
      __ASM("nop");
      __ASM("nop");
      __ASM("nop");
      __enable_irq();

      while (TIM2->CNT - micros < 31);
    }
    motor_2.kv = encoder_2.speed / V_bus;
    FOC2->motor_kv = motor_2.kv;
  }
  disableGateDriver(3);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
  {
    /* Retrieve Rx messages from RX FIFO0 */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
      Error_Handler();
    }
    if (RxHeader.Identifier == CAN_CMD_ID) {
      // motor current commands
      int16_t AC_current_raw[2];
      memcpy(&AC_current_raw[0], &RxData[0], 4);
      TargetCurrent1 = AC_current_raw[0] * 0.01f;
      TargetCurrent2 = AC_current_raw[1] * 0.01f;
      // Drive enable 2
      if ((RxData[4] & 0x1) != 0 && (errors & !ERR_M1_OCP) == 0) {
        resetGateDriver(2);
        LL_TIM_SetCounter(TIM7, 0); // reset drive-enable timeout counter
      }
      else {
        disableGateDriver(2);
      }
      // Drive enable 1
      if ((RxData[4] & 0x2) != 0 && (errors & !ERR_M2_OCP) == 0) {
        resetGateDriver(1);
        LL_TIM_SetCounter(TIM6, 0); // reset drive-enable timeout counter
      }
      else {
        disableGateDriver(1);
      }
      // Audio enable 2
      audio_enable_2 = (RxData[4] & 0x4) != 0;
      // Audio enable 1
      audio_enable_1 = (RxData[4] & 0x8) != 0;
    }
    else if (RxHeader.Identifier == CAN_CFG_ID) {
      MaxCurrent = RxData[0] * 0.2f;
      MaxAbsCurrent = RxData[1] * 0.2f;
      UVLO_thresh = RxData[2] * 0.1f;
      UVLO_hyst = RxData[3] * 0.01f;
    }
    else if (RxHeader.Identifier == CAN_SETUP1_ID) {
      motor_1.Res = RxData[0] * 1e-3f;
      motor_1.Ind = RxData[1] * 1e-6f;
      motor_2.Res = RxData[2] * 1e-3f;
      motor_2.Ind = RxData[3] * 1e-6f;
      motor_1.n_poles = RxData[4];
      motor_2.n_poles = RxData[5];
      FOC1->Kp_Iq = (80000 >> FOC1->F_sw) * motor_1.Ind / V_bus;
      FOC1->Ki_Iq = (80000 >> FOC1->F_sw) * motor_1.Res / V_bus;
      FOC2->Kp_Iq = (80000 >> FOC2->F_sw) * motor_2.Ind / V_bus;
      FOC2->Ki_Iq = (80000 >> FOC2->F_sw) * motor_2.Res / V_bus;
      FOC1->Kp_Id = (80000 >> FOC1->F_sw) * motor_1.Ind / V_bus;
      FOC1->Ki_Id = (80000 >> FOC1->F_sw) * motor_1.Res / V_bus;
      FOC2->Kp_Id = (80000 >> FOC2->F_sw) * motor_2.Ind / V_bus;
      FOC2->Ki_Id = (80000 >> FOC2->F_sw) * motor_2.Res / V_bus;
    }
    else if (RxHeader.Identifier == CAN_SETUP2_ID) {
      memcpy(&motor_1.encoder_os, &RxData[0], 2);
      memcpy(&motor_2.encoder_os, &RxData[2], 2);
      FOC1->Encoder_os = motor_1.encoder_os;
      FOC2->Encoder_os = motor_2.encoder_os;
      motor_1.kv = RxData[4] * 1.6666666666666667e-7f * N_STEP_ENCODER;
      motor_2.kv = RxData[5] * 1.6666666666666667e-7f * N_STEP_ENCODER;
      FOC1->motor_kv = motor_1.kv;
      FOC2->motor_kv = motor_2.kv;
      float TestCurrent1 = RxData[6] * 0.1f;
      float TestCurrent2 = RxData[7] * 0.1f;
      if (RxData[6] > 0) {
        measureEncoderOs(1, TestCurrent1);
        measureMotorKv(1, TestCurrent1);
      }
      if (RxData[7] > 0) {
        measureEncoderOs(2, TestCurrent2);
        measureMotorKv(2, TestCurrent2);
      }
      enQueue(&CAN_TxBuffer, CAN_RDBK_ID);
    }
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
