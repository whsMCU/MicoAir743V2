/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "utils.h"

#include "adc_impl.h"

#include "adc.h"

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc3;

//#define DEBUG_ADC_CHANNELS

adcConfig_t adcConfig;

#define ADC_BUF_LENGTH ADC_SOURCE_COUNT
#define ADC_BUF_BYTES (ADC_BUF_LENGTH * sizeof(uint16_t))
#define ADC_BUF_CACHE_ALIGN_BYTES  ((ADC_BUF_BYTES + 0x20) & ~0x1f)
#define ADC_BUF_CACHE_ALIGN_LENGTH (ADC_BUF_CACHE_ALIGN_BYTES / sizeof(uint16_t))

static volatile DMA_RAM uint16_t adcConversionBuffer[ADC_BUF_CACHE_ALIGN_LENGTH];

void adcConfig_Init(void)
{
    adcConfig.vbat.enabled = true;
    adcConfig.current.enabled = true;
    adcConfig.vrefIntCalibration = 0;
    adcConfig.tempSensorCalibration1 = 0;
    adcConfig.tempSensorCalibration2 = 0;
}

#ifdef USE_ADC_INTERNAL
//void adcInitInternalInjected(const adcConfig_t *config)
//{
//    ADC_TempSensorVrefintCmd(ENABLE);
//    ADC_InjectedDiscModeCmd(ADC1, DISABLE);
//    ADC_InjectedSequencerLengthConfig(ADC1, 2);
//    ADC_InjectedChannelConfig(ADC1, ADC_Channel_Vrefint, 1, ADC_SampleTime_480Cycles);
//    ADC_InjectedChannelConfig(ADC1, ADC_Channel_TempSensor, 2, ADC_SampleTime_480Cycles);
//
//    adcVREFINTCAL = config->vrefIntCalibration ? config->vrefIntCalibration : *(uint16_t *)VREFINT_CAL_ADDR;
//    adcTSCAL1 = config->tempSensorCalibration1 ? config->tempSensorCalibration1 : *(uint16_t *)TS_CAL1_ADDR;
//    adcTSCAL2 = config->tempSensorCalibration2 ? config->tempSensorCalibration2 : *(uint16_t *)TS_CAL2_ADDR;
//
//    adcTSSlopeK = (110 - 30) * 1000 / (adcTSCAL2 - adcTSCAL1);
//}

// Note on sampling time for temperature sensor and vrefint:
// Both sources have minimum sample time of 10us.
// With prescaler = 8:
// 168MHz : fAPB2 = 84MHz, fADC = 10.5MHz, tcycle = 0.090us, 10us = 105cycle < 144cycle
// 240MHz : fAPB2 = 120MHz, fADC = 15.0MHz, tcycle = 0.067usk 10us = 150cycle < 480cycle
// 480cycles@10.5MHz = 45.7us
// 480cycles@15.0MHz = 32us

//static bool adcInternalConversionInProgress = false;

bool adcInternalIsBusy(void)
{
  return false;
}

//void adcInternalStartConversion(void)
//{
//    ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);
//    ADC_SoftwareStartInjectedConv(ADC1);
//
//    adcInternalConversionInProgress = true;
//}
#endif

// H743, H735, H750 and H7A3 seems to use 16-bit precision value,
// while H723, H725 and H730 seems to use 12-bit precision value.
#if defined(STM32H743xx) || defined(STM32H750xx) || defined(STM32H7A3xx) || defined(STM32H7A3xxQ)
#define VREFINT_CAL_SHIFT 4
#elif defined(STM32H723xx) || defined(STM32H725xx) || defined(STM32H730xx) || defined(STM32H735xx)
#define VREFINT_CAL_SHIFT 0
#else
#error Unknown MCU
#endif

static void adcInitCalibrationValues(void)
{
    adcVREFINTCAL = *VREFINT_CAL_ADDR >> VREFINT_CAL_SHIFT;
    adcTSCAL1 = *TEMPSENSOR_CAL1_ADDR >> VREFINT_CAL_SHIFT;
    adcTSCAL2 = *TEMPSENSOR_CAL2_ADDR >> VREFINT_CAL_SHIFT;
    adcTSSlopeK = (TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP) * 1000 / (adcTSCAL2 - adcTSCAL1);
}

bool adcInit(void)
{

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};
  bool ret = true;

  adcInitCalibrationValues();

  hadc1.Instance 											= ADC1;
  hadc1.Init.ClockPrescaler 					= ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution 							= ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode 						= ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection 						= ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait 				= DISABLE;
  hadc1.Init.ContinuousConvMode 			= ENABLE;
  hadc1.Init.NbrOfConversion 					= 2;
  hadc1.Init.DiscontinuousConvMode 		= DISABLE;
  hadc1.Init.ExternalTrigConv 				= ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge 		= ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun 									= ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.LeftBitShift 						= ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode 				= DISABLE;
  hadc1.Init.Oversampling.Ratio 			= 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  // Execute calibration
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) {
    Error_Handler();
  }

  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_387CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }


  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adcConversionBuffer[0], 2);

  hadc3.Instance 											= ADC3;
  hadc3.Init.ClockPrescaler 					= ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution 							= ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode 						= ADC_SCAN_ENABLE;
  hadc3.Init.EOCSelection 						= ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait 				= DISABLE;
  hadc3.Init.ContinuousConvMode 			= ENABLE;
  hadc3.Init.NbrOfConversion 					= 3;
  hadc3.Init.DiscontinuousConvMode 		= DISABLE;
  hadc3.Init.ExternalTrigConv 				= ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge 		= ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc3.Init.Overrun 									= ADC_OVR_DATA_OVERWRITTEN;
  hadc3.Init.LeftBitShift 						= ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.OversamplingMode 				= DISABLE;
  hadc3.Init.Oversampling.Ratio 			= 1;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  // Execute calibration
  if (HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VBAT;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_ADC_Start_DMA(&hadc3, (uint32_t*)&adcConversionBuffer[4], 3);

  return ret;
}

#ifdef USE_ADC_INTERNAL

int32_t adcVREFINTCAL;      // ADC value (12-bit) of band gap with Vref = VREFINTCAL_VREF
int32_t adcTSCAL1;
int32_t adcTSCAL2;
int32_t adcTSSlopeK;

uint16_t adcInternalCompensateVref(uint16_t vrefAdcValue)
{
    // This is essentially a tuned version of
    // __HAL_ADC_CALC_VREFANALOG_VOLTAGE(vrefAdcValue, ADC_RESOLUTION_12B);
    return (uint16_t)((uint32_t)(adcVREFINTCAL * VREFINT_CAL_VREF) / vrefAdcValue);
}

int16_t adcInternalComputeTemperature(uint16_t tempAdcValue, uint16_t vrefValue)
{
    // This is essentially a tuned version of
    // __HAL_ADC_CALC_TEMPERATURE(vrefValue, tempAdcValue, ADC_RESOLUTION_12B);

    return ((((int32_t)((tempAdcValue * vrefValue) / TEMPSENSOR_CAL_VREFANALOG) - adcTSCAL1) * adcTSSlopeK) + 500) / 1000 + TEMPSENSOR_CAL1_TEMP;
}
#endif // USE_ADC_INTERNAL

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */

    /* ADC1 clock enable */
    __HAL_RCC_ADC12_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PC0     ------> ADC1_INP10
    PC1     ------> ADC1_INP11
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* ADC1 DMA Init */
    /* ADC1 Init */
    hdma_adc1.Instance 									= DMA2_Stream0;
    hdma_adc1.Init.Request 							= DMA_REQUEST_ADC1;
    hdma_adc1.Init.Direction 						= DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc 						= DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc 							= DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment 	= DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment 		= DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode 								= DMA_CIRCULAR;
    hdma_adc1.Init.Priority 						= DMA_PRIORITY_MEDIUM;
    hdma_adc1.Init.FIFOMode 						= DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc1);

    /* ADC1 interrupt Init */
    HAL_NVIC_SetPriority(ADC_IRQn, 1, 0);//
    HAL_NVIC_EnableIRQ(ADC_IRQn);
  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
  else if(adcHandle->Instance==ADC3)
  {
  /* USER CODE BEGIN ADC3_MspInit 0 */

  /* USER CODE END ADC3_MspInit 0 */

    /* ADC3 clock enable */
    __HAL_RCC_ADC3_CLK_ENABLE();

    /* ADC3 DMA Init */
    /* ADC3 Init */
    hdma_adc3.Instance 									= DMA2_Stream1;
    hdma_adc3.Init.Request 							= DMA_REQUEST_ADC3;
    hdma_adc3.Init.Direction 						= DMA_PERIPH_TO_MEMORY;
    hdma_adc3.Init.PeriphInc 						= DMA_PINC_DISABLE;
    hdma_adc3.Init.MemInc 							= DMA_MINC_ENABLE;
    hdma_adc3.Init.PeriphDataAlignment 	= DMA_PDATAALIGN_HALFWORD;
    hdma_adc3.Init.MemDataAlignment 		= DMA_MDATAALIGN_HALFWORD;
    hdma_adc3.Init.Mode 								= DMA_CIRCULAR;
    hdma_adc3.Init.Priority 						= DMA_PRIORITY_MEDIUM;
    if (HAL_DMA_Init(&hdma_adc3) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc3);

    /* ADC3 interrupt Init */
    HAL_NVIC_SetPriority(ADC3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(ADC3_IRQn);
  /* USER CODE BEGIN ADC3_MspInit 1 */

  /* USER CODE END ADC3_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC12_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PC0     ------> ADC1_INP10
    PC1     ------> ADC1_INP11
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_0|GPIO_PIN_1);

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit(adcHandle->DMA_Handle);

    /* ADC1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(ADC_IRQn);
  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
  else if(adcHandle->Instance==ADC3)
  {
  /* USER CODE BEGIN ADC3_MspDeInit 0 */

  /* USER CODE END ADC3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC3_CLK_DISABLE();

    /* ADC3 DMA DeInit */
    HAL_DMA_DeInit(adcHandle->DMA_Handle);

    /* ADC3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(ADC3_IRQn);
  /* USER CODE BEGIN ADC3_MspDeInit 1 */

  /* USER CODE END ADC3_MspDeInit 1 */
  }
}

void adcGetChannelValues(void)
{
    // Transfer values in conversion buffer into adcValues[]
    SCB_InvalidateDCache_by_Addr((uint32_t*)adcConversionBuffer, ADC_BUF_CACHE_ALIGN_BYTES);
    for (unsigned i = 0; i < ADC_EXTERNAL_COUNT; i++) {
            adcValues[i] = adcConversionBuffer[i];
    }
}

uint16_t adcInternalRead(adcSource_e source)
{
    switch (source) {
    case ADC_VREFINT:
    case ADC_TEMPSENSOR:
#if ADC_INTERNAL_VBAT4_ENABLED
    case ADC_VBAT4:
#endif
        //const unsigned dmaIndex = adcOperatingConfig[source].dmaIndex;
        return source < ADC_BUF_LENGTH ? adcConversionBuffer[source] : 0;
    default:
        return 0;
    }
}

//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
//{
//   /* Invalidate Data Cache to get the updated content of the SRAM on the second half of the ADC converted data buffer: 32 bytes */
//  SCB_InvalidateDCache_by_Addr((uint32_t *) &adcConversionBuffer[ADC_BUF_CACHE_ALIGN_LENGTH], ADC_BUF_CACHE_ALIGN_LENGTH);
//}
