/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "scheduler/tasks.h"

#include "drivers/nvic.h"
//
//#include "config/config.h"
//
#include "fc/init.h"
//#include "fc/rc_modes.h"
//#include "fc/rc_controls.h"
//#include "fc/core.h"
//
#include "flight/imu.h"
//#include "flight/pid_init.h"
#include "flight/position.h"

#include "sensors/sensors.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"
#include "sensors/gyro_init.h"
#include "sensors/adcinternal.h"
#include "sensors/battery.h"

#include "drivers/gps/gps.h"

#include "hw/timer.h"
//
#include "rx/rx.h"
#include "rx/crsf.h"

extern uint32_t cachedRccCsrValue;

void initialiseMemorySections(void);

void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
static void CPU_CACHE_Enable(void);
static void HandleStuckSysTick(void);

void hwInit(void);

void run(void);

int main(void)
{

  initialiseMemorySections();

  MPU_Config();

  /* Enable the CPU Cache */
  CPU_CACHE_Enable();

  HAL_Init();
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  // Configure NVIC preempt/priority groups
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITY_GROUPING);

  cachedRccCsrValue = RCC->RSR;

  __HAL_RCC_D2SRAM1_CLK_ENABLE();
  __HAL_RCC_D2SRAM2_CLK_ENABLE();
  __HAL_RCC_D2SRAM3_CLK_ENABLE();

  cycleCounterInit();

  hwInit();

//  TIM4->CCR1 = 21000;
//  TIM4->CCR2 = 21000;
//  TIM4->CCR3 = 21000;
//  TIM4->CCR4 = 21000;
//  HAL_Delay(7000);
//  TIM4->CCR1 = 10500;
//  TIM4->CCR2 = 10500;
//  TIM4->CCR3 = 10500;
//  TIM4->CCR4 = 10500;
//  HAL_Delay(8000);

  init();

  run();

  return 0;

}

void FAST_CODE run(void)
{
    while (true) {
        scheduler();
    }
}

void hwInit(void)
{
  #ifdef _USE_HW_RTC
//    rtcInit();
  #endif
  gpioInit();
//  flashInit();
  ledInit();
  MX_DMA_Init();
//  usbInit();
//  uartInit();
//  cliInit();
//  i2cInit();
  spiInit();
  adcInit();
//  timerInit();

//  if (sdInit() == true)
//  {
//    fatfsInit();
//  }
}

void initialiseMemorySections(void)
{
 #ifdef USE_FAST_DATA
   /* Load FAST_DATA variable initializers into DTCM RAM */
   extern uint8_t _sfastram_data;
   extern uint8_t _efastram_data;
   extern uint8_t _sfastram_idata;
   memcpy(&_sfastram_data, &_sfastram_idata, (size_t) (&_efastram_data - &_sfastram_data));

   extern uint32_t __fastram_bss_start__;
   extern uint32_t __fastram_bss_end__;
   uint32_t *p = &__fastram_bss_start__;
   while (p < &__fastram_bss_end__)
       *p++ = 0;

   /* Load DMA_DATA variable intializers into D2 RAM */
   extern uint8_t _sdmaram_bss;
   extern uint8_t _edmaram_bss;
   extern uint8_t _sdmaram_data;
   extern uint8_t _edmaram_data;
   extern uint8_t _sdmaram_idata;
   bzero(&_sdmaram_bss, (size_t) (&_edmaram_bss - &_sdmaram_bss));
   memcpy(&_sdmaram_data, &_sdmaram_idata, (size_t) (&_edmaram_data - &_sdmaram_data));
 #endif
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 480;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  HandleStuckSysTick();
  HAL_Delay(10);
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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

/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}

static void HandleStuckSysTick(void)
{
    uint32_t tickStart = HAL_GetTick();
    uint32_t tickEnd = 0;

    // H7 at 480Mhz requires a loop count of 160000. Double this for the timeout to be safe.
    int attemptsRemaining = 320000;
    while (((tickEnd = HAL_GetTick()) == tickStart) && --attemptsRemaining) {
    }

    if (tickStart == tickEnd) {
        systemResetWithoutDisablingCaches();
    }
}
