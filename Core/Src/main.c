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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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
volatile FOC_data* FOC;

// Motor variables
volatile bool SPI_Wait = false;
volatile uint8_t SPI_WaitState = 0;
volatile uint32_t SPI_buf = 0U;
int32_t motor_PhysPosition;
int32_t motor_lastPhysPosition;
volatile uint32_t motor_lastMeasTime, motor_lastMeasTime2;
uint32_t motor_last2MeasTime;
volatile float motor_speed = 0.0f; // encoder LSBs / us
volatile int32_t Encoder_os = 0;

// Encoders
A1333_t encoder_1 = {
  .SPIx = SPI1,
  .CS_Port = GPIOD,
  .CS_Pin = LL_GPIO_PIN_2,
  .micros = &(TIM2->CNT),
};

// ADC variables
volatile int16_t adc_data[64];
volatile int16_t adc_os[4] = {0, 0, 0, 0};

// timing stuff
uint32_t micros = 0, lastMicros = 0;
float dt_f = 33e-6f; // seconds
volatile int32_t dt_i = 50; // microseconds

// CANBus stuff
volatile uint8_t sendCANBus_flag = 0;

volatile bool init_complete = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void resetGateDriver();
void disableGateDriver();
void writePwm(uint32_t timer, int32_t duty);
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

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

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
  disableGateDriver();

  // allocate memory on heap for FOC data
  FOC = (FOC_data*)malloc(sizeof(FOC_data));
  if (FOC == NULL) {
    // Handle memory allocation failure
    disableGateDriver();
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

  // Encoder setup
  A1333_Init(&encoder_1);

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

  // initialize FOC variables
  FOC->Encoder_os = Encoder_os;
  FOC->Kp_Id = 0.1f; FOC->Ki_Id = 100.0f;
  FOC->Kp_Iq = 0.1f; FOC->Ki_Iq = 100.0f;
  FOC->integ_d = 0.0f;
  FOC->integ_q = 0.0f;
  FOC->motor_speed = 0.0f;
  FOC->TargetCurrent = 0.0f;
  FOC->TargetFieldWk = 0.0f;

  // measure ADC offset
  for (uint16_t i = 0; i < 100; i++){
    LL_mDelay(10);
    adc_os[0] = (adc_os[0]*3 + adc_data[0]) >> 2;
    adc_os[1] = (adc_os[1]*3 + adc_data[1]) >> 2;
    adc_os[2] = (adc_os[2]*3 + adc_data[2]) >> 2;
    adc_os[3] = (adc_os[3]*3 + adc_data[3]) >> 2;
  }

  // Enable HRTIM (gate drive signals)
  FOC->F_sw = LL_HRTIM_TIM_GetPrescaler(HRTIM1, LL_HRTIM_TIMER_A);
  writePwm(U1_TIMER, 0);
  writePwm(V1_TIMER, 0);
  writePwm(W1_TIMER, 0);
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
  resetGateDriver();

  TIM2->CNT = 0;
  while (TIM2->CNT < 2000000) {
    micros = TIM2->CNT;
    __disable_irq();
    __ASM("nop");
    __ASM("nop");
    __ASM("nop");
    __ASM("nop");
    __ASM("nop");
    FOC->motor_PhysPosition = 0;
    FOC->TargetCurrent = 5.0f * sinf((float)TIM2->CNT * 1e-6f * PIx2 * 131.0f) * (1.0f - fmin((float)TIM2->CNT * 1e-6f, 1.0f));
    FOC->TargetFieldWk = 5.0f;
    FOC->U_current = (adc_data[2] - adc_os[2]) * 0.040584415584415584f;
    FOC->W_current = (adc_data[3] - adc_os[3]) * -0.040584415584415584f;
    FOC->V_current = -FOC->U_current - FOC->W_current; // assuming balanced currents
    __ASM("nop");
    __ASM("nop");
    __ASM("nop");
    __ASM("nop");
    __ASM("nop");
    __enable_irq();
    SPI_Wait = true;
    A1333_Update(&encoder_1);
    uint32_t timeout = micros + 100;
    while (SPI_Wait) {
      if (TIM2->CNT >= timeout) {
        SPI_Wait = false;
      }
    }
    while (TIM2->CNT - micros < 31);
  }
  FOC->Encoder_os = encoder_1.angle;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t counter = 0;
  while (1)
  {
    micros = TIM2->CNT;
    dt_i = micros - lastMicros;
    dt_f = dt_i * 1e-6f;

    SPI_Wait = true;
    A1333_Update(&encoder_1);
    uint32_t timeout = micros + 100;
    while (SPI_Wait) {
      if (TIM2->CNT >= timeout) {
        SPI_Wait = false;
      }
    }

    // move encoder info to FOC struct
    __disable_irq();
    __ASM("nop");
    __ASM("nop");
    __ASM("nop");
    __ASM("nop");
    __ASM("nop");
    FOC->motor_PhysPosition = encoder_1.angle;
    FOC->motor_lastMeasTime = encoder_1.sampleTime;
    FOC->motor_speed = encoder_1.speed;
    FOC->U_current = (adc_data[2] - adc_os[2]) * 0.040584415584415584f;
    FOC->W_current = (adc_data[3] - adc_os[3]) * -0.040584415584415584f;
    FOC->V_current = -FOC->U_current - FOC->W_current; // assuming balanced currents
    FOC->TargetCurrent = 1.0f; // for testing
    FOC->TargetFieldWk = 0.0f; // for testing
    // Flush pipeline
    __ASM("nop");
    __ASM("nop");
    __ASM("nop");
    __ASM("nop");
    __ASM("nop");
    __enable_irq();

    while (TIM2->CNT - micros < 37);
    lastMicros = micros;
    counter++;
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
void resetGateDriver(){
  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_13);
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_2);
}
void disableGateDriver(){
  LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_13);
  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_2);
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
