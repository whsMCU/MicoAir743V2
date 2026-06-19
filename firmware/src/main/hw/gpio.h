/*
 * gpio.h
 *
 *  Created on: 2021. 8. 14.
 *      Author: WANG
 */

#ifndef SRC_COMMON_HW_INCLUDE_GPIO_H_
#define SRC_COMMON_HW_INCLUDE_GPIO_H_

#include "hw.h"


#ifdef _USE_HW_GPIO


#define GPIO_MAX_CH     HW_GPIO_MAX_CH

enum
{
	BMI270_CS,
	BMI270_INT,
};

/** @defgroup GPIO_LL_EC_MODE Mode
  * @{
  */
#define LL_GPIO_MODE_INPUT                 (0x00000000U) /*!< Select input mode */
#define LL_GPIO_MODE_OUTPUT                GPIO_MODER_MODE0_0  /*!< Select output mode */
#define LL_GPIO_MODE_ALTERNATE             GPIO_MODER_MODE0_1  /*!< Select alternate function mode */
#define LL_GPIO_MODE_ANALOG                GPIO_MODER_MODE0    /*!< Select analog mode */
/**
  * @}
  */

/** @defgroup GPIO_LL_EC_OUTPUT Output Type
  * @{
  */
#define LL_GPIO_OUTPUT_PUSHPULL            (0x00000000U) /*!< Select push-pull as output type */
#define LL_GPIO_OUTPUT_OPENDRAIN           GPIO_OTYPER_OT0 /*!< Select open-drain as output type */
/**
  * @}
  */

/** @defgroup GPIO_LL_EC_SPEED Output Speed
  * @{
  */
#define LL_GPIO_SPEED_FREQ_LOW             (0x00000000U) /*!< Select I/O low output speed    */
#define LL_GPIO_SPEED_FREQ_MEDIUM          GPIO_OSPEEDR_OSPEED0_0 /*!< Select I/O medium output speed */
#define LL_GPIO_SPEED_FREQ_HIGH            GPIO_OSPEEDR_OSPEED0_1 /*!< Select I/O fast output speed   */
#define LL_GPIO_SPEED_FREQ_VERY_HIGH       GPIO_OSPEEDR_OSPEED0   /*!< Select I/O high output speed   */
/**
  * @}
  */
#define LL_GPIO_SPEED_LOW                  LL_GPIO_SPEED_FREQ_LOW
#define LL_GPIO_SPEED_MEDIUM               LL_GPIO_SPEED_FREQ_MEDIUM
#define LL_GPIO_SPEED_FAST                 LL_GPIO_SPEED_FREQ_HIGH
#define LL_GPIO_SPEED_HIGH                 LL_GPIO_SPEED_FREQ_VERY_HIGH


/** @defgroup GPIO_LL_EC_PULL Pull Up Pull Down
  * @{
  */
#define LL_GPIO_PULL_NO                    (0x00000000U) /*!< Select I/O no pull */
#define LL_GPIO_PULL_UP                    GPIO_PUPDR_PUPD0_0 /*!< Select I/O pull up */
#define LL_GPIO_PULL_DOWN                  GPIO_PUPDR_PUPD0_1 /*!< Select I/O pull down */
/**
  * @}
  */

/** @defgroup GPIO_LL_EC_AF Alternate Function
  * @{
  */
#define LL_GPIO_AF_0                       (0x0000000U) /*!< Select alternate function 0 */
#define LL_GPIO_AF_1                       (0x0000001U) /*!< Select alternate function 1 */
#define LL_GPIO_AF_2                       (0x0000002U) /*!< Select alternate function 2 */
#define LL_GPIO_AF_3                       (0x0000003U) /*!< Select alternate function 3 */
#define LL_GPIO_AF_4                       (0x0000004U) /*!< Select alternate function 4 */
#define LL_GPIO_AF_5                       (0x0000005U) /*!< Select alternate function 5 */
#define LL_GPIO_AF_6                       (0x0000006U) /*!< Select alternate function 6 */
#define LL_GPIO_AF_7                       (0x0000007U) /*!< Select alternate function 7 */
#define LL_GPIO_AF_8                       (0x0000008U) /*!< Select alternate function 8 */
#define LL_GPIO_AF_9                       (0x0000009U) /*!< Select alternate function 9 */
#define LL_GPIO_AF_10                      (0x000000AU) /*!< Select alternate function 10 */
#define LL_GPIO_AF_11                      (0x000000BU) /*!< Select alternate function 11 */
#define LL_GPIO_AF_12                      (0x000000CU) /*!< Select alternate function 12 */
#define LL_GPIO_AF_13                      (0x000000DU) /*!< Select alternate function 13 */
#define LL_GPIO_AF_14                      (0x000000EU) /*!< Select alternate function 14 */
#define LL_GPIO_AF_15                      (0x000000FU) /*!< Select alternate function 15 */

__STATIC_INLINE void GPIO_SetPinMode(GPIO_TypeDef *GPIOx, uint32_t Pin, uint32_t Mode)
{
  MODIFY_REG(GPIOx->MODER, ((Pin * Pin) * GPIO_MODER_MODE0), ((Pin * Pin) * Mode));
}

__STATIC_INLINE void GPIO_SetPinSpeed(GPIO_TypeDef *GPIOx, uint32_t Pin, uint32_t  Speed)
{
  MODIFY_REG(GPIOx->OSPEEDR, ((Pin * Pin) * GPIO_OSPEEDR_OSPEED0), ((Pin * Pin) * Speed));
}

__STATIC_INLINE void GPIO_SetPinPull(GPIO_TypeDef *GPIOx, uint32_t Pin, uint32_t Pull)
{
  MODIFY_REG(GPIOx->PUPDR, ((Pin * Pin) * GPIO_PUPDR_PUPD0), ((Pin * Pin) * Pull));
}

__STATIC_INLINE void GPIO_SetPinOutputType(GPIO_TypeDef *GPIOx, uint32_t PinMask, uint32_t OutputType)
{
  MODIFY_REG(GPIOx->OTYPER, PinMask, (PinMask * OutputType));
}

__STATIC_INLINE void GPIO_SetAFPin_0_7(GPIO_TypeDef *GPIOx, uint32_t Pin, uint32_t Alternate)
{
  MODIFY_REG(GPIOx->AFR[0], ((((Pin * Pin) * Pin) * Pin) * GPIO_AFRL_AFSEL0),
             ((((Pin * Pin) * Pin) * Pin) * Alternate));
}

__STATIC_INLINE void GPIO_SetAFPin_8_15(GPIO_TypeDef *GPIOx, uint32_t Pin, uint32_t Alternate)
{
  MODIFY_REG(GPIOx->AFR[1], (((((Pin >> 8U) * (Pin >> 8U)) * (Pin >> 8U)) * (Pin >> 8U)) * GPIO_AFRH_AFSEL8),
             (((((Pin >> 8U) * (Pin >> 8U)) * (Pin >> 8U)) * (Pin >> 8U)) * Alternate));
}

bool gpioInit(void);
bool gpioPinMode(uint8_t ch, uint8_t mode);
bool gpioPinMode_DSHOT(uint8_t ch, uint8_t mode);
void gpioPinWrite(uint8_t ch, bool value);
bool gpioPinRead(uint8_t ch);
void gpioPinToggle(uint8_t ch);


#endif

#endif /* SRC_COMMON_HW_INCLUDE_GPIO_H_ */
