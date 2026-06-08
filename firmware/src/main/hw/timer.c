/*
 * timer.c
 *
 *  Created on: 2020. 12. 26.
 *      Author: WANG
 */


#include "timer.h"
#include "light_ws2811strip.h"
#include "fc/dshot_dpwm.h"

TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_ch1;
DMA_HandleTypeDef hdma_tim1_ch2;
DMA_HandleTypeDef hdma_tim1_ch3;
DMA_HandleTypeDef hdma_tim1_ch4;

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
DMA_HandleTypeDef hdma_tim8_ch4_trig_com;

static void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

bool timerInit(void)
{
	bool ret = true;

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 19;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */
	HAL_TIM_MspPostInit(&htim1);


	////////////////////////////////////////////////////////////////
//	TIM_ClockConfigTypeDef sClockSourceConfig1 = {0};
//	TIM_MasterConfigTypeDef sMasterConfig1 = {0};

	/* USER CODE BEGIN TIM5_Init 1 */

	/* USER CODE END TIM5_Init 1 */
//	htim5.Instance = TIM5;
//	htim5.Init.Prescaler = 83;
//	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
//	htim5.Init.Period = 4294967295;
//	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//	if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
//	{
//	Error_Handler();
//	}
//	sClockSourceConfig1.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//	if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig1) != HAL_OK)
//	{
//	Error_Handler();
//	}
//	sMasterConfig1.MasterOutputTrigger = TIM_TRGO_RESET;
//	sMasterConfig1.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig1) != HAL_OK)
//	{
//	Error_Handler();
//	}
//	HAL_TIM_Base_Init(&htim5);
//	HAL_TIM_Base_Start(&htim5);

	///////////////////////////////////////////////////////////////////////////
//	TIM_ClockConfigTypeDef sClockSourceConfig2 = {0};
//  TIM_MasterConfigTypeDef sMasterConfig2 = {0};
//  TIM_OC_InitTypeDef sConfigOC2 = {0};
//  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig2 = {0};


//  htim8.Instance = TIM8;
//  htim8.Init.Prescaler = 0;//(3)
//  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim8.Init.Period = 209; // 800kHz, (52) 1.25us period
//  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim8.Init.RepetitionCounter = 0;
//  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sClockSourceConfig2.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  sConfigOC2.OCMode = TIM_OCMODE_PWM1;
//  sConfigOC2.Pulse = 0;
//  sConfigOC2.OCPolarity = TIM_OCPOLARITY_HIGH;
//  sConfigOC2.OCFastMode = TIM_OCFAST_DISABLE;
//  sConfigOC2.OCIdleState = TIM_OCIDLESTATE_RESET;
//  sConfigOC2.OCNIdleState = TIM_OCNIDLESTATE_RESET;
//  if (HAL_TIM_OC_ConfigChannel(&htim8, &sConfigOC2, TIM_CHANNEL_4) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  HAL_TIM_MspPostInit(&htim8);

	return ret;
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspInit 0 */

  /* USER CODE END TIM1_MspInit 0 */
    /* TIM1 clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();

    /* TIM1 DMA Init */
    /* TIM1_CH1 Init */
    hdma_tim1_ch1.Instance = DMA2_Stream2;
    hdma_tim1_ch1.Init.Request = DMA_REQUEST_TIM1_CH1;
    hdma_tim1_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim1_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim1_ch1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim1_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim1_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim1_ch1.Init.Mode = DMA_NORMAL;
    hdma_tim1_ch1.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_tim1_ch1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_tim1_ch1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(tim_baseHandle,hdma[TIM_DMA_ID_CC1],hdma_tim1_ch1);

    /* TIM1_CH2 Init */
    hdma_tim1_ch2.Instance = DMA2_Stream3;
    hdma_tim1_ch2.Init.Request = DMA_REQUEST_TIM1_CH2;
    hdma_tim1_ch2.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim1_ch2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim1_ch2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim1_ch2.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim1_ch2.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim1_ch2.Init.Mode = DMA_NORMAL;
    hdma_tim1_ch2.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_tim1_ch2.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_tim1_ch2) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(tim_baseHandle,hdma[TIM_DMA_ID_CC2],hdma_tim1_ch2);

    /* TIM1_CH3 Init */
    hdma_tim1_ch3.Instance = DMA2_Stream4;
    hdma_tim1_ch3.Init.Request = DMA_REQUEST_TIM1_CH3;
    hdma_tim1_ch3.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim1_ch3.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim1_ch3.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim1_ch3.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim1_ch3.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim1_ch3.Init.Mode = DMA_NORMAL;
    hdma_tim1_ch3.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_tim1_ch3.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_tim1_ch3) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(tim_baseHandle,hdma[TIM_DMA_ID_CC3],hdma_tim1_ch3);

    /* TIM1_CH4 Init */
    hdma_tim1_ch4.Instance = DMA2_Stream5;
    hdma_tim1_ch4.Init.Request = DMA_REQUEST_TIM1_CH4;
    hdma_tim1_ch4.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim1_ch4.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim1_ch4.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim1_ch4.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim1_ch4.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim1_ch4.Init.Mode = DMA_NORMAL;
    hdma_tim1_ch4.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_tim1_ch4.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_tim1_ch4) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(tim_baseHandle,hdma[TIM_DMA_ID_CC4],hdma_tim1_ch4);

    /* TIM1 interrupt Init */
    HAL_NVIC_SetPriority(TIM1_UP_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
  /* USER CODE BEGIN TIM1_MspInit 1 */

  /* USER CODE END TIM1_MspInit 1 */
  }
//	else if(tim_baseHandle->Instance==TIM5)
//	{
//		/* USER CODE BEGIN TIM5_MspInit 0 */
//
//		/* USER CODE END TIM5_MspInit 0 */
//		/* TIM5 clock enable */
//		__HAL_RCC_TIM5_CLK_ENABLE();
//
//		/* TIM5 interrupt Init */
//		HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
//		HAL_NVIC_EnableIRQ(TIM5_IRQn);
//		/* USER CODE BEGIN TIM5_MspInit 1 */
//
//		/* USER CODE END TIM5_MspInit 1 */
//	}
//	else if(tim_baseHandle->Instance==TIM8)
//	  {
//	  /* USER CODE BEGIN TIM8_MspInit 0 */
//
//	  /* USER CODE END TIM8_MspInit 0 */
//      /* TIM8 clock enable */
//      __HAL_RCC_TIM8_CLK_ENABLE();
//
//      /* TIM8 DMA Init */
//      /* TIM8_CH4_TRIG_COM Init */
//      hdma_tim8_ch4_trig_com.Instance = DMA2_Stream7;
//      hdma_tim8_ch4_trig_com.Init.Channel = DMA_CHANNEL_7;
//      hdma_tim8_ch4_trig_com.Init.Direction = DMA_MEMORY_TO_PERIPH;
//      hdma_tim8_ch4_trig_com.Init.PeriphInc = DMA_PINC_DISABLE;
//      hdma_tim8_ch4_trig_com.Init.MemInc = DMA_MINC_ENABLE;
//      hdma_tim8_ch4_trig_com.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
//      hdma_tim8_ch4_trig_com.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
//      hdma_tim8_ch4_trig_com.Init.Mode = DMA_NORMAL;
//      hdma_tim8_ch4_trig_com.Init.Priority = DMA_PRIORITY_HIGH;
//      hdma_tim8_ch4_trig_com.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
//      if (HAL_DMA_Init(&hdma_tim8_ch4_trig_com) != HAL_OK)
//      {
//        Error_Handler();
//      }
//
//      /* Several peripheral DMA handle pointers point to the same DMA handle.
//       Be aware that there is only one stream to perform all the requested DMAs. */
//      __HAL_LINKDMA(tim_baseHandle,hdma[TIM_DMA_ID_CC4],hdma_tim8_ch4_trig_com);
//      //__HAL_LINKDMA(tim_baseHandle,hdma[TIM_DMA_ID_TRIGGER],hdma_tim8_ch4_trig_com);
//      //__HAL_LINKDMA(tim_baseHandle,hdma[TIM_DMA_ID_COMMUTATION],hdma_tim8_ch4_trig_com);
//
//	    /* TIM8 interrupt Init */
//	    HAL_NVIC_SetPriority(TIM8_CC_IRQn, 1, 2);
//	    HAL_NVIC_EnableIRQ(TIM8_CC_IRQn);
//	  }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {

  if (htim->Instance == TIM1) {
		motor_DMA_IRQHandler(htim);
  }

    if (htim->Instance == TIM8) {
        HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_4);
        WS2811_DMA_IRQHandler();
    }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(timHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspPostInit 0 */

  /* USER CODE END TIM1_MspPostInit 0 */

    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**TIM1 GPIO Configuration
    PE9     ------> TIM1_CH1
    PE11     ------> TIM1_CH2
    PE13     ------> TIM1_CH3
    PE14     ------> TIM1_CH4
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM1_MspPostInit 1 */

  /* USER CODE END TIM1_MspPostInit 1 */
  }
//  else if(timHandle->Instance==TIM8)
//    {
//    /* USER CODE BEGIN TIM8_MspPostInit 0 */
//
//    /* USER CODE END TIM8_MspPostInit 0 */
//
//      __HAL_RCC_GPIOC_CLK_ENABLE();
//      /**TIM8 GPIO Configuration
//      PC9     ------> TIM8_CH4
//      */
//      GPIO_InitStruct.Pin = GPIO_PIN_9;
//      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//      GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//      GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
//      HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//
//    /* USER CODE BEGIN TIM8_MspPostInit 1 */
//
//    /* USER CODE END TIM8_MspPostInit 1 */
//    }

}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspDeInit 0 */

  /* USER CODE END TIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();

    /* TIM1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);
  /* USER CODE BEGIN TIM1_MspDeInit 1 */

  /* USER CODE END TIM1_MspDeInit 1 */
  }
//	else if(tim_baseHandle->Instance==TIM5)
//	{
//		/* USER CODE BEGIN TIM5_MspDeInit 0 */
//
//		/* USER CODE END TIM5_MspDeInit 0 */
//		/* Peripheral clock disable */
//		__HAL_RCC_TIM5_CLK_DISABLE();
//
//		/* TIM5 interrupt Deinit */
//		HAL_NVIC_DisableIRQ(TIM5_IRQn);
//		/* USER CODE BEGIN TIM5_MspDeInit 1 */
//
//		/* USER CODE END TIM5_MspDeInit 1 */
//	}
//  else if(tim_baseHandle->Instance==TIM8)
//  {
//  /* USER CODE BEGIN TIM8_MspDeInit 0 */
//
//  /* USER CODE END TIM8_MspDeInit 0 */
//    /* Peripheral clock disable */
//    __HAL_RCC_TIM8_CLK_DISABLE();
//
//    /* TIM8 DMA DeInit */
//    HAL_DMA_DeInit(tim_baseHandle->hdma[TIM_DMA_ID_CC4]);
//    HAL_DMA_DeInit(tim_baseHandle->hdma[TIM_DMA_ID_TRIGGER]);
//    HAL_DMA_DeInit(tim_baseHandle->hdma[TIM_DMA_ID_COMMUTATION]);
//
//    /* TIM8 interrupt Deinit */
//    HAL_NVIC_DisableIRQ(TIM8_CC_IRQn);
//  /* USER CODE BEGIN TIM8_MspDeInit 1 */
//
//  /* USER CODE END TIM8_MspDeInit 1 */
//  }
}
