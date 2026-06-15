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

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#include "hw.h"

#ifdef USE_DSHOT

//#include "drivers/pwm_output.h"
#include "dshot_shared.h"
#include "drivers/dshot.h"
#include "dshot_dpwm.h"
#include "drivers/motor_impl.h"

#include "drivers/motor.h"

#include "drivers/dshot_command.h"

// XXX TODO: Share a single region among dshotDmaBuffer and dshotBurstDmaBuffer

DSHOT_DMA_BUFFER_ATTRIBUTE DSHOT_DMA_BUFFER_UNIT dshotDmaBuffer[MAX_SUPPORTED_MOTORS][DSHOT_DMA_BUFFER_ALLOC_SIZE];

#ifdef USE_DSHOT_DMAR
DSHOT_DMA_BUFFER_ATTRIBUTE DSHOT_DMA_BUFFER_UNIT dshotBurstDmaBuffer[MAX_DMA_TIMERS][DSHOT_DMA_BUFFER_SIZE * 4];
#endif

#ifdef USE_DSHOT_DMAR
FAST_DATA_ZERO_INIT bool useBurstDshot = false;
#endif
#ifdef USE_DSHOT_TELEMETRY
FAST_DATA_ZERO_INIT bool useDshotTelemetry = false;
#endif

FAST_DATA_ZERO_INIT loadDmaBufferFn *loadDmaBuffer;

FAST_CODE_NOINLINE uint8_t loadDmaBufferDshot(uint32_t *dmaBuffer, int stride, uint16_t packet)
{
    int i;
    for (i = 0; i < 16; i++) {
        dmaBuffer[i * stride] = (packet & 0x8000) ? MOTOR_BIT_1 : MOTOR_BIT_0;  // MSB first
        packet <<= 1;
    }
    dmaBuffer[i++ * stride] = 0;
    dmaBuffer[i++ * stride] = 0;

    return DSHOT_DMA_BUFFER_SIZE;
}

FAST_CODE_NOINLINE uint8_t loadDmaBufferProshot(uint32_t *dmaBuffer, int stride, uint16_t packet)
{
    int i;
    for (i = 0; i < 4; i++) {
        dmaBuffer[i * stride] = PROSHOT_BASE_SYMBOL + ((packet & 0xF000) >> 12) * PROSHOT_BIT_WIDTH;  // Most significant nibble first
        packet <<= 4;   // Shift 4 bits
    }
    dmaBuffer[i++ * stride] = 0;
    dmaBuffer[i++ * stride] = 0;

    return PROSHOT_DMA_BUFFER_SIZE;
}

uint32_t getDshotHz(motorProtocolTypes_e pwmProtocolType)
{
    switch (pwmProtocolType) {
    case(MOTOR_PROTOCOL_PROSHOT1000):
        return MOTOR_PROSHOT1000_HZ;
    case(MOTOR_PROTOCOL_DSHOT600):
        return MOTOR_DSHOT600_HZ;
    case(MOTOR_PROTOCOL_DSHOT300):
        return MOTOR_DSHOT300_HZ;
    default:
    case(MOTOR_PROTOCOL_DSHOT150):
        return MOTOR_DSHOT150_HZ;
    }
}

static void dshotPwmShutdown(void)
{
    // DShot signal is only generated if write to motors happen,
    // and that is prevented by enabled checking at write.
    // Hence there's no special processing required here.
    return;
}

static void dshotPwmDisableMotors(void)
{
    // No special processing required
    return;
}

static bool dshotPwmEnableMotors(void)
{
    // No special processing required
    return true;
}

static bool dshotPwmIsMotorEnabled(unsigned index)
{
	UNUSED(index);
    return true;
}

static IO_t pwmDshotGetMotorIO(unsigned index)
{
    return 0;
}

static FAST_CODE void dshotWriteInt(uint8_t index, uint16_t value)
{
    pwmWriteDshotInt(index, value);
}

static FAST_CODE void dshotWrite(uint8_t index, float value)
{
    pwmWriteDshotInt(index, lrintf(value));
}

static const motorVTable_t dshotPwmVTable = {
    .postInit = motorPostInitNull,
    .enable = dshotPwmEnableMotors,
    .disable = dshotPwmDisableMotors,
    .isMotorEnabled = dshotPwmIsMotorEnabled,
    .decodeTelemetry = pwmTelemetryDecode,
    .write = dshotWrite,
    .writeInt = dshotWriteInt,
    .updateComplete = pwmCompleteDshotMotorUpdate,
    //.convertExternalToMotor = dshotConvertFromExternal,
    //.convertMotorToExternal = dshotConvertToExternal,
    .shutdown = dshotPwmShutdown,
    .requestTelemetry = pwmDshotRequestTelemetry,
    //.isMotorIdle = pwmDshotIsMotorIdle,
    .getMotorIO = pwmDshotGetMotorIO,
};

bool dshotPwmDevInit(motorDevice_t *device, const motorConfig_t *motorConfig)
{
		device->vTable = &dshotPwmVTable;
    dshotMotorCount = device->count;
#ifdef USE_DSHOT_TELEMETRY
    useDshotTelemetry = motorConfig->useDshotTelemetry;
#endif

    switch (motorConfig->motorProtocol) {
    case MOTOR_PROTOCOL_PROSHOT1000:
        loadDmaBuffer = loadDmaBufferProshot;
        break;
    case MOTOR_PROTOCOL_DSHOT600:
    case MOTOR_PROTOCOL_DSHOT300:
    case MOTOR_PROTOCOL_DSHOT150:
        loadDmaBuffer = loadDmaBufferDshot;
#ifdef USE_DSHOT_DMAR
        useBurstDshot = motorConfig->useBurstDshot == DSHOT_DMAR_ON ||
            (motorConfig->useBurstDshot == DSHOT_DMAR_AUTO && !motorConfig->useDshotTelemetry);
#endif
        break;
    }

    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < dshotMotorCount; motorIndex++) {
    	dmaMotors[motorIndex].TimHandle = &htim1;
    	dmaMotors[motorIndex].configured = true;
      dmaMotors[motorIndex].dmaBuffer = &dshotDmaBuffer[motorIndex][0];
      dmaMotors[motorIndex].outputPeriod = htim1.Instance->ARR;
      dmaMotors[motorIndex].Channel = motorIndex * 4;
      dmaMotors[motorIndex].TimHandle->hdma[motorIndex + 1]->XferCpltCallback = motor_DMA_IRQHandler;
      dmaMotors[motorIndex].dmaRef = dmaMotors[motorIndex].TimHandle->hdma[motorIndex + 1];
      dmaMotors[motorIndex].io = motorIndex + 3;
      dmaMotors[motorIndex].index = 4 - motorIndex;
      TIM_CCxChannelCmd(dmaMotors[motorIndex].TimHandle->Instance, dmaMotors[motorIndex].Channel, TIM_CCx_ENABLE);

    }
    return true;
}

void pwmDshotSetDirectionOutput(
    motorDmaOutput_t * const motor
#ifndef USE_DSHOT_TELEMETRY
    , LL_TIM_OC_InitTypeDef* pOcInit, LL_DMA_InitTypeDef* pDmaInit
#endif
)
{
#ifdef USE_DSHOT_TELEMETRY
		TIM_OC_InitTypeDef* pOcInit = &motor->ocInitStruct;
		DMA_InitTypeDef* pDmaInit = &motor->dmaInitStruct;
#endif

    HAL_DMA_DeInit(motor->dmaRef);
    motor->TimHandle->hdma[motor->io - 2]->XferCpltCallback = motor_DMA_IRQHandler;

#ifdef USE_DSHOT_TELEMETRY
    motor->isInput = false;
#endif

    __HAL_TIM_DISABLE_OCxPRELOAD(motor->TimHandle, motor->Channel);
    HAL_TIM_OC_ConfigChannel(motor->TimHandle, pOcInit, motor->Channel);
    __HAL_TIM_ENABLE_OCxPRELOAD(motor->TimHandle, motor->Channel);

    motor->dmaInitStruct.Direction = DMA_MEMORY_TO_PERIPH;

    motor->dmaRef->Init = *pDmaInit;
    HAL_DMA_Init(motor->dmaRef);

    
    // __HAL_LINKDMA(motor->TimHandle, hdma[motor->index + 1], *motor->dmaRef);
    //motor->TimHandle->hdma[motor->io - 2] = motor->dmaRef;
    //motor->dmaRef->Parent = motor->TimHandle;

    __HAL_DMA_ENABLE_IT(motor->dmaRef, DMA_IT_TC);
}

#ifdef USE_DSHOT_TELEMETRY
FAST_CODE static void pwmDshotSetDirectionInput(
    motorDmaOutput_t * const motor
)
{
		DMA_InitTypeDef* pDmaInit = &motor->dmaInitStruct;

    TIM_TypeDef *timer = motor->TimHandle->Instance;

    HAL_DMA_DeInit(motor->dmaRef);
    motor->TimHandle->hdma[motor->io - 2]->XferCpltCallback = motor_DMA_IRQHandler;

    motor->isInput = true;
    if (!inputStampUs) {
        inputStampUs = micros();
    }
    SET_BIT(motor->TimHandle->Instance->CR1, TIM_CR1_ARPE); // Only update the period once all channels are done

    timer->ARR = 0xffffffff;

#ifdef STM32H7
    // Configure pin as GPIO output to avoid glitch during timer configuration
    gpioPinMode(motor->io + 3, _DEF_AVOID_GLITCH);
#endif

    HAL_TIM_IC_ConfigChannel(motor->TimHandle, &motor->icInitStruct, motor->Channel);

#ifdef STM32H7
    // Configure pin back to timer
    gpioPinMode(motor->io + 3, _DEF_INPUT_AF_PP);
#endif

    motor->dmaInitStruct.Direction = DMA_PERIPH_TO_MEMORY;
    motor->dmaRef->Init = *pDmaInit;
    HAL_DMA_Init(motor->dmaRef);
}
#endif

volatile uint32_t motor_update_time[4], motor_update_time_temp[4];

FAST_CODE void pwmCompleteDshotMotorUpdate(void)
{
    /* If there is a dshot command loaded up, time it correctly with motor update*/
    if (!dshotCommandQueueEmpty() && !dshotCommandOutputIsEnabled(dshotMotorCount)) {
        return;
    }

    for (int i = 0; i < dshotMotorCount; i++) {
#ifdef USE_DSHOT_DMAR
        if (useBurstDshot) {
            xLL_EX_DMA_SetDataLength(dmaMotorTimers[i].dmaBurstRef, dmaMotorTimers[i].dmaBurstLength);
            xLL_EX_DMA_EnableResource(dmaMotorTimers[i].dmaBurstRef);

            /* configure the DMA Burst Mode */
            LL_TIM_ConfigDMABurst(dmaMotorTimers[i].timer, LL_TIM_DMABURST_BASEADDR_CCR1, LL_TIM_DMABURST_LENGTH_4TRANSFERS);
            /* Enable the TIM DMA Request */
            LL_TIM_EnableDMAReq_UPDATE(dmaMotorTimers[i].timer);
        } else
#endif
        {
        	HAL_DMA_Start_IT(dmaMotors[i].TimHandle->hdma[i + 1],	(uint32_t)dmaMotors[i].dmaBuffer,
    											(uint32_t)&dmaMotors[i].TimHandle->Instance->CCR1 + dmaMotors[i].Channel,
    											dmaMotors[i].bufferSize);
        }
    }
    for (int i = 0; i < dshotMotorCount; i++) {
    	motor_update_time_temp[i] = micros();

    	CLEAR_BIT(dmaMotors[i].TimHandle->Instance->CR1, TIM_CR1_ARPE); //TIM_AUTORELOAD_PRELOAD_DISABLE
    	//__HAL_TIM_SET_COUNTER(dmaMotors[i].TimHandle, 0);
      __HAL_TIM_ENABLE_DMA(dmaMotors[i].TimHandle, 1 << (9 + i));
    }
}

FAST_CODE static void motor_IRQHandler(motorDmaOutput_t * const motor)
{
		#ifdef USE_DSHOT_TELEMETRY
						if (!motor->isInput) {
								dshotDMAHandlerCycleCounters.irqAt = getCycleCounter();
		#endif
		#ifdef USE_DSHOT_DMAR
								if (useBurstDshot) {
										xLL_EX_DMA_DisableResource(motor->timerHardware->dmaTimUPRef);
										LL_TIM_DisableDMAReq_UPDATE(motor->timerHardware->tim);
								} else
		#endif
								{
										__HAL_DMA_DISABLE(motor->dmaRef);
								}

		#ifdef USE_DSHOT_TELEMETRY
								if (useDshotTelemetry) {
//										pwmDshotSetDirectionInput(motor);
//										__HAL_DMA_SET_COUNTER(motor->dmaRef, GCR_TELEMETRY_INPUT_LEN);
//										__HAL_DMA_ENABLE(motor->dmaRef);
//										__HAL_TIM_ENABLE_DMA(motor->TimHandle, 7680);
										dshotDMAHandlerCycleCounters.changeDirectionCompletedAt = getCycleCounter();
								}
						}
		#endif
}

FAST_CODE void motor_DMA_IRQHandler(DMA_HandleTypeDef *hdma)
{
	TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

	if (hdma == htim->hdma[TIM_DMA_ID_CC1])
	{
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC1);
		motorDmaOutput_t * const motor = &dmaMotors[0];
		motor_IRQHandler(motor);
		motor_update_time[0] = micros()-motor_update_time_temp[0];
	}
	else if(hdma == htim->hdma[TIM_DMA_ID_CC2])
	{
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC2);
		motorDmaOutput_t * const motor = &dmaMotors[1];
		motor_IRQHandler(motor);
		motor_update_time[1] = micros()-motor_update_time_temp[1];
	}
	else if(hdma == htim->hdma[TIM_DMA_ID_CC3])
	{
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC3);
		motorDmaOutput_t * const motor = &dmaMotors[2];
		motor_IRQHandler(motor);
		motor_update_time[2] = micros()-motor_update_time_temp[2];
	}
	else if(hdma == htim->hdma[TIM_DMA_ID_CC4])
	{
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC4);
		motorDmaOutput_t * const motor = &dmaMotors[3];
		motor_IRQHandler(motor);
		motor_update_time[3] = micros()-motor_update_time_temp[3];
	}
}

#endif // USE_DSHOT
