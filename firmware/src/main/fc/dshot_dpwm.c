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
    //.decodeTelemetry = pwmTelemetryDecode,
    .write = dshotWrite,
    .writeInt = dshotWriteInt,
    .updateComplete = pwmCompleteDshotMotorUpdate,
    //.convertExternalToMotor = dshotConvertFromExternal,
    //.convertMotorToExternal = dshotConvertToExternal,
    .shutdown = dshotPwmShutdown,
    //.requestTelemetry = pwmDshotRequestTelemetry,
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
    }

    return true;
}

uint32_t motor_update_time[4], motor_update_time_temp[4];

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
        	//dmaMotors[i].TimHandle->Instance->ARR = dmaMotors[i].outputPeriod;
        	motor_update_time_temp[i] = micros();

          /* Enable channel DMA requests */
          HAL_TIM_PWM_Start_DMA(dmaMotors[i].TimHandle, dmaMotors[i].Channel,
																(uint32_t *)dmaMotors[i].dmaBuffer, dmaMotors[i].bufferSize);

          /* Reset timer counter */
          __HAL_TIM_SET_COUNTER(dmaMotors[i].TimHandle, 0);
        }
    }
}

FAST_CODE void motor_DMA_IRQHandler(TIM_HandleTypeDef *htim)
{
  switch(htim->Channel)
  {
      case HAL_TIM_ACTIVE_CHANNEL_1:
          HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_1);
          motor_update_time[0] = micros()-motor_update_time_temp[0];
          break;

      case HAL_TIM_ACTIVE_CHANNEL_2:
          HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_2);
          motor_update_time[1] = micros()-motor_update_time_temp[1];
          break;

      case HAL_TIM_ACTIVE_CHANNEL_3:
          HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_3);
          motor_update_time[2] = micros()-motor_update_time_temp[2];
          break;

      case HAL_TIM_ACTIVE_CHANNEL_4:
          HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_4);
          motor_update_time[3] = micros()-motor_update_time_temp[3];
          break;

      case HAL_TIM_ACTIVE_CHANNEL_CLEARED:
      default:
					break;
  }

//    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF)) {
//        motorDmaOutput_t * const motor = &dmaMotors[descriptor->userParam];
//#ifdef USE_DSHOT_TELEMETRY
//        if (!motor->isInput) {
//            dshotDMAHandlerCycleCounters.irqAt = getCycleCounter();
//#endif
//#ifdef USE_DSHOT_DMAR
//            if (useBurstDshot) {
//                xLL_EX_DMA_DisableResource(motor->timerHardware->dmaTimUPRef);
//                LL_TIM_DisableDMAReq_UPDATE(motor->timerHardware->tim);
//            } else
//#endif
//            {
//                xLL_EX_DMA_DisableResource(motor->dmaRef);
//                LL_EX_TIM_DisableIT(motor->timerHardware->tim, motor->timerDmaSource);
//            }
//
//#ifdef USE_DSHOT_TELEMETRY
//            if (useDshotTelemetry) {
//                pwmDshotSetDirectionInput(motor);
//                xLL_EX_DMA_SetDataLength(motor->dmaRef, GCR_TELEMETRY_INPUT_LEN);
//                xLL_EX_DMA_EnableResource(motor->dmaRef);
//                LL_EX_TIM_EnableIT(motor->timerHardware->tim, motor->timerDmaSource);
//                dshotDMAHandlerCycleCounters.changeDirectionCompletedAt = getCycleCounter();
//            }
//        }
//#endif
//        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
//    }
}

#endif // USE_DSHOT
