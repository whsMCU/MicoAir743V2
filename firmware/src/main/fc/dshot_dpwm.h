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
 *
 * Author: jflyper
 */

#pragma once

#include "hw.h"

#include "drivers/dshot.h"
#include "drivers/motor_types.h"

#define MHZ_TO_HZ(x) ((x) * 1000000)

// Timer clock frequency for the dshot speeds
#define MOTOR_DSHOT600_HZ     MHZ_TO_HZ(12)
#define MOTOR_DSHOT300_HZ     MHZ_TO_HZ(6)
#define MOTOR_DSHOT150_HZ     MHZ_TO_HZ(3)

// These three constants are times in timer clock ticks, e.g. with a 6 MHz clock 20 ticks for bitlength = 300kHz bit rate
#define MOTOR_BIT_0           7
#define MOTOR_BIT_1           14
#define MOTOR_BITLENGTH       20

#define MOTOR_PROSHOT1000_HZ         MHZ_TO_HZ(24)
#define PROSHOT_BASE_SYMBOL          24 // 1uS
#define PROSHOT_BIT_WIDTH            3
#define MOTOR_NIBBLE_LENGTH_PROSHOT  (PROSHOT_BASE_SYMBOL * 4) // 4uS

#define DSHOT_TELEMETRY_DEADTIME_US   (30 + 5) // 30 to switch lines and 5 to switch lines back

typedef uint8_t loadDmaBufferFn(uint32_t *dmaBuffer, int stride, uint16_t packet);  // function pointer used to encode a digital motor value into the DMA buffer representation
extern FAST_DATA_ZERO_INIT loadDmaBufferFn *loadDmaBuffer;
uint8_t loadDmaBufferDshot(uint32_t *dmaBuffer, int stride, uint16_t packet);
uint8_t loadDmaBufferProshot(uint32_t *dmaBuffer, int stride, uint16_t packet);

uint32_t getDshotHz(motorProtocolTypes_e pwmProtocolType);

/* Motor DMA related, moved from pwm_output.h */

#define MAX_DMA_TIMERS        8

#define DSHOT_DMA_BUFFER_SIZE   18 /* resolution + frame reset (2us) */
#define PROSHOT_DMA_BUFFER_SIZE 6  /* resolution + frame reset (2us) */

#define GCR_TELEMETRY_INPUT_LEN MAX_GCR_EDGES

// For H7, DMA buffer is placed in a dedicated segment for coherency management
// XXX Do we need any special qualifier for AT32?
#if defined(STM32H7)
#define DSHOT_DMA_BUFFER_ATTRIBUTE DMA_RAM
#elif defined(STM32G4)
#define DSHOT_DMA_BUFFER_ATTRIBUTE DMA_RAM_W
#elif defined(STM32F7)
#define DSHOT_DMA_BUFFER_ATTRIBUTE FAST_DATA_ZERO_INIT
#else
#define DSHOT_DMA_BUFFER_ATTRIBUTE /* Empty */
#endif

#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7) || defined(STM32G4) || defined(AT32F435) || defined(APM32F4)
#define DSHOT_DMA_BUFFER_UNIT uint32_t
#else
#define DSHOT_DMA_BUFFER_UNIT uint8_t
#endif

#ifdef USE_DSHOT_TELEMETRY
STATIC_ASSERT(GCR_TELEMETRY_INPUT_LEN >= DSHOT_DMA_BUFFER_SIZE, dshotBufferSizeConstrait);
#define DSHOT_DMA_BUFFER_ALLOC_SIZE GCR_TELEMETRY_INPUT_LEN
#else
#define DSHOT_DMA_BUFFER_ALLOC_SIZE DSHOT_DMA_BUFFER_SIZE
#endif

extern DSHOT_DMA_BUFFER_UNIT dshotDmaBuffer[MAX_SUPPORTED_MOTORS][DSHOT_DMA_BUFFER_ALLOC_SIZE];
extern DSHOT_DMA_BUFFER_UNIT dshotDmaInputBuffer[MAX_SUPPORTED_MOTORS][DSHOT_DMA_BUFFER_ALLOC_SIZE];

#ifdef USE_DSHOT_DMAR
extern DSHOT_DMA_BUFFER_UNIT dshotBurstDmaBuffer[MAX_DMA_TIMERS][DSHOT_DMA_BUFFER_SIZE * 4];
#endif

typedef struct motorDmaOutput_s {
    dshotProtocolControl_t protocolControl;

    bool configured;

    TIM_HandleTypeDef *TimHandle;
    void *dmaResource;
    DMA_InitTypeDef dmaInitStruct;
    DMA_HandleTypeDef *dmaRef;


    uint32_t Channel;
    uint16_t DMA_Channel;
    uint16_t outputPeriod;
    uint8_t bufferSize;
    uint8_t io;

    uint8_t output;
    uint8_t index;

#ifdef USE_DSHOT_TELEMETRY
    volatile bool isInput;
    timeDelta_t dshotTelemetryDeadtimeUs;

    TIM_OC_InitTypeDef ocInitStruct;
		TIM_IC_InitTypeDef icInitStruct;

#endif // USE_DSHOT_TELEMETRY

    DSHOT_DMA_BUFFER_UNIT *dmaBuffer;
} motorDmaOutput_t;

motorDmaOutput_t *getMotorDmaOutput(unsigned index);

void pwmWriteDshotInt(uint8_t index, uint16_t value);
//bool pwmDshotMotorHardwareConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, uint8_t reorderedMotorIndex, motorProtocolTypes_e pwmProtocolType, uint8_t output);
#ifdef USE_DSHOT_TELEMETRY
bool pwmTelemetryDecode(void);
#endif
void pwmCompleteDshotMotorUpdate(void);

void motor_DMA_IRQHandler(DMA_HandleTypeDef *htim);

extern bool useBurstDshot;
