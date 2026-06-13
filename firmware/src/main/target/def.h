/*
 * def.h
 *
 *  Created on: Dec 6, 2020
 *      Author: baram
 */

#ifndef SRC_COMMON_DEF_H_
#define SRC_COMMON_DEF_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <utils.h>

#define _DEF_LED1           0
#define _DEF_LED2           1
#define _DEF_LED3           2
#define _DEF_LED4           3

#define _DEF_USB            0
#define _DEF_UART1          1  // GPS
#define _DEF_UART2          2  //
#define _DEF_UART3          3  // Rangefinder, opticalflow
#define _DEF_UART4          4  // bluetooth
#define _DEF_UART5          5  //
#define _DEF_UART6          6  // Radio
#define _DEF_UART7          7  // ESC telemetry

#define _DEF_SPI1             0
#define _DEF_SPI2             1
#define _DEF_SPI3             2
#define _DEF_SPI4             3

#define _DEF_LOW              0
#define _DEF_HIGH             1

#define _DEF_INPUT            0
#define _DEF_INPUT_PULLUP     1
#define _DEF_INPUT_PULLDOWN   2
#define _DEF_INPUT_IT_RISING  3
#define _DEF_OUTPUT           4
#define _DEF_OUTPUT_PULLUP    5
#define _DEF_OUTPUT_PULLDOWN  6
#define _DEF_OUTPUT_AF_PP      7
#define _DEF_AVOID_GLITCH      8
#define _DEF_INPUT_AF_PP      9


#define MAX_SUPPORTED_MOTORS 8



#if !defined(ALT_HOLD_THROTTLE_NEUTRAL_ZONE)
  #define ALT_HOLD_THROTTLE_NEUTRAL_ZONE 40
#endif

/********************************************************************/
/****           altitude hold                                    ****/
/********************************************************************/

  /* uncomment to disable the altitude hold feature.
   * This is useful if all of the following apply
   * + you have a baro
   * + want altitude readout
   * + do not use altitude hold feature
   * + want to save memory space
   */
  //#define SUPPRESS_BARO_ALTHOLD

/* Natural alt change for rapid pilots. It's temporary switch OFF the althold when throttle stick is out of deadband defined with ALT_HOLD_THROTTLE_NEUTRAL_ZONE
 * but if it's commented: Smooth alt change routine is activated, for slow auto and aerophoto modes (in general solution from alexmos). It's slowly increase/decrease
 * altitude proportional to stick movement (+/-100 throttle gives about +/-50 cm in 1 second with cycle time about 3-4ms)
 */
//#define ALTHOLD_FAST_THROTTLE_CHANGE

#define USE_ACC
#define USE_BARO
#define USE_VARIO
#define USE_ADC
//#define USE_OSD
//#define USE_MAX7456
#define USE_MOTOR
#define USE_DMA_RAM

#define NAV_AUTO_MAG_DECLINATION_PRECISE

#define USE_MAG
#define USE_OPFLOW
#define USE_OPFLOW_MSP
#define USE_RANGEFINDER
#define USE_RANGEFINDER_MSP
#define USE_GPS
#define USE_GPS_UBLOX
#define USE_CRSF_LINK_STATISTICS
#define USE_RX_RSSI_DBM
#define USE_RX_LINK_QUALITY_INFO
//#define USE_GYRO_SLEW_LIMITER
#define USE_LATE_TASK_STATISTICS
#define USE_DSHOT
#define USE_DSHOT_TELEMETRY

#define USE_ITCM_RAM
#define USE_FAST_DATA
#define USE_DYN_NOTCH_FILTER

#define USE_LED_STRIP
#define USE_LED_STRIP_STATUS_MODE

#define USE_TELEMETRY
#define USE_TELEMETRY_CRSF
#define USE_MSP_OVER_TELEMETRY

#define USE_ESC_SENSOR

#define USE_ADC_INTERNAL
//#define USE_RC_SMOOTHING_FILTER
#define USE_PERSISTENT_STATS

#define USE_BOARD_INFO

#define USE_GPS_RESCUE

#define NOINLINE __attribute__((noinline))

#ifdef USE_ITCM_RAM
#if defined(ITCM_RAM_OPTIMISATION) && !defined(DEBUG)
#define FAST_CODE                   __attribute__((section(".tcm_code"))) __attribute__((optimize(ITCM_RAM_OPTIMISATION)))
#else
#define FAST_CODE                   __attribute__((section(".tcm_code")))
#endif
// If a particular target is short of ITCM RAM, defining FAST_CODE_PREF in the target.h file will
// cause functions decorated FAST_CODE_PREF to *not* go into ITCM RAM but if FAST_CODE_PREF is not
// defined for the target, FAST_CODE_PREF will become an alias to FAST_CODE (in the common post
// header file), and functions decorated with FAST_CODE_PREF *will* go into ITCM RAM.
#define FAST_CODE_NOINLINE          NOINLINE
#endif // USE_ITCM_RAM

#ifdef USE_FAST_DATA
#define FAST_DATA_ZERO_INIT         __attribute__ ((section(".fastram_bss"), aligned(4)))
#define FAST_DATA                   __attribute__ ((section(".fastram_data"), aligned(4)))
#else
#define FAST_DATA_ZERO_INIT
#define FAST_DATA
#endif  //USE_FAST_DATA

// DMA to/from any memory
#define DMA_DATA_ZERO_INIT          __attribute__ ((section(".dmaram_bss"), aligned(32)))
#define DMA_DATA                    __attribute__ ((section(".dmaram_data"), aligned(32)))
#define STATIC_DMA_DATA_AUTO        static DMA_DATA

#define PERSISTENT                  __attribute__ ((section(".persistent_data"), aligned(4)))

#ifdef USE_DMA_RAM
#if defined(STM32H7)
#define DMA_RAM 										__attribute__((section(".DMA_RAM"), aligned(32)))
#define DMA_RW_AXI 									__attribute__((section(".DMA_RW_AXI"), aligned(32)))
extern uint8_t _dmaram_start__;
extern uint8_t _dmaram_end__;
#elif defined(STM32G4)
#define DMA_RAM_R __attribute__((section(".DMA_RAM_R")))
#define DMA_RAM_W __attribute__((section(".DMA_RAM_W")))
#define DMA_RAM_RW __attribute__((section(".DMA_RAM_RW")))
#endif
#else
#define DMA_RAM
#define DMA_RW_AXI
#define DMA_RAM_R
#define DMA_RAM_W
#define DMA_RAM_RW
#endif

#define CACHE_LINE_SIZE 32
#define CACHE_LINE_MASK (CACHE_LINE_SIZE - 1)


#endif /* SRC_COMMON_DEF_H_ */
