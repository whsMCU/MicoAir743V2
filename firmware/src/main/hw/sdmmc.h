/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sdmmc.h
  * @brief   This file contains all the function prototypes for
  *          the sdmmc.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SDMMC_H__
#define __SDMMC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "hw.h"

extern SD_HandleTypeDef hsd1;

/* SDCARD pinouts
*
* SD CARD PINS
  _________________
 / 1 2 3 4 5 6 7 8 |  NR   |SDIO INTERFACE
/                  |       |NAME     STM32F746     DESCRIPTION
/ 9                 |       |         4-BIT  1-BIT
|                   |       |
|                   |   1   |CD/DAT3  PC11   -      Connector data line 3
|                   |   2   |CMD      PD2    PD2    Command/Response line
|                   |   3   |VSS1     GND    GND    GND
|   SD CARD Pinout  |   4   |VDD      3.3V   3.3V   3.3V Power supply
|                   |   5   |CLK      PC12   PC12   Clock
|                   |   6   |VSS2     GND    GND    GND
|                   |   7   |DAT0     PC8    PC8    Connector data line 0
|                   |   8   |DAT1     PC9    -      Connector data line 1
|___________________|   9   |DAT2     PC10   -      Connector data line 2

*/

bool MX_SDMMC1_SD_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __SDMMC_H__ */

