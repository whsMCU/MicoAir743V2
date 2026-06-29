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
#include <string.h>

#include "hw.h"

#include "common/maths.h"

#include "flight/failsafe.h"
#include "flight/pid.h"

#include "fc/runtime_config.h"

#include "drivers/dshot.h"
#include "drivers/motor.h"
#include "drivers/dshot_command.h"

#include "rx/rx.h"

motorConfig_t motorConfig;

float FAST_DATA_ZERO_INIT motor[MAX_SUPPORTED_MOTORS];

static FAST_DATA_ZERO_INIT motorDevice_t motorDevice;

static bool motorProtocolEnabled = false;
static bool motorProtocolDshot = false;

unsigned short LF, LR, RR, RF;

//TIM4->CCR1 // RR
//TIM4->CCR2 // RF
//TIM4->CCR3 // LR
//TIM4->CCR4 // LF

//ROLL
//angle : +, gyro : +, rx : +

//PITCH
//angle : +, gyro : +, rx : +

//YAW
//angle : +, gyro : -, rx : +  //gyro mul negative sign

void motorConfig_Init(void)
{
  motorConfig.motorProtocol = MOTOR_PROTOCOL_DSHOT600;
  motorConfig.minthrottle = 1050;
  motorConfig.maxthrottle = 2000;
  motorConfig.mincommand = 1000;
  motorConfig.digitalIdleOffsetValue = 550;
  motorConfig.motorPoleCount = 14;   // Most brushes motors that we use are 14 poles
  motorConfig.motorCount = 4;
  motorConfig.useDshotTelemetry = true;
  motorConfig.useDshotEdt = DSHOT_EDT_FORCE;
}


bool isMotorProtocolEnabled(void)
{
    return motorProtocolEnabled;
}

bool isMotorProtocolDshot(void)
{
    return motorProtocolDshot;
}

bool isMotorProtocolBidirDshot(void)
{
    return isMotorProtocolDshot() && useDshotTelemetry;
}

unsigned motorDeviceCount(void)
{
    return motorDevice.count;
}

const motorVTable_t *motorGetVTable(void)
{
    return motorDevice.vTable;
}

void motorDevInit(void)
{
		bool success = false;

    motorDevice.count = motorConfig.motorCount;
    success = dshotPwmDevInit(&motorDevice, &motorConfig);

    // if the VTable has been populated, the device is initialized.
    if (success) {
			motorDevice.initialized = true;
			motorDevice.motorEnableTimeMs = 0;
			motorDevice.enabled = false;
    }
}

void motorShutdown(void)
{
  motorDevice.vTable->shutdown();
  motorDevice.enabled = false;
  motorDevice.motorEnableTimeMs = 0;
  motorDevice.initialized = false;
}

extern float applyCommand[4];

FAST_CODE void motorWriteAll(void)
{
  if(ARMING_FLAG(ARMED))
  {
    if(failsafeFlags == 0)
    {
      if(rcData[THROTTLE] > 1030)
      {
        motor[R_R] = RR > 21000 ? 21000 : RR < 11000 ? 11000 : RR;
        motor[R_F] = RF > 21000 ? 21000 : RF < 11000 ? 11000 : RF;
        motor[L_R] = LR > 21000 ? 21000 : LR < 11000 ? 11000 : LR;
        motor[L_F] = LF > 21000 ? 21000 : LF < 11000 ? 11000 : LF;
      }
      else
      {
        motor[R_R] = 11000;
        motor[R_F] = 11000;
        motor[L_R] = 11000;
        motor[L_F] = 11000;
      }
    }
    else
    {
      motor[R_R] = 10500;
      motor[R_F] = 10500;
      motor[L_R] = 10500;
      motor[L_F] = 10500;
    }
  }
  else
  {
    motor[R_R] = 10500;
    motor[R_F] = 10500;
    motor[L_R] = 10500;
    motor[L_F] = 10500;
  }

    if (motorDevice.enabled) {
    // Perform the decode of the last data received
    // New data will be received once the send of motor data, triggered above, completes
#if defined(USE_DSHOT) && defined(USE_DSHOT_TELEMETRY)
    if (motorDevice.vTable->decodeTelemetry) {
            motorDevice.vTable->decodeTelemetry();
    }
#endif

    // Update the motor data
    for (int i = 0; i < motorDevice.count; i++) {
    	motor[i] = (motor[i] == 10500) ? DSHOT_CMD_MOTOR_STOP : scaleRangef(motor[i], 10500 + 1, 21000, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE);
      motorDevice.vTable->write(i, motor[i]);
    }

    // Trigger the transmission of the motor data
    motorDevice.vTable->updateComplete();
  }
}

void motorRequestTelemetry(unsigned index)
{
    if (index >= motorDevice.count) {
        return;
    }

    if (motorDevice.vTable->requestTelemetry) {
        motorDevice.vTable->requestTelemetry(index);
    }
}

void motorDisable(void)
{
  motorDevice.vTable->disable();
  motorDevice.enabled = false;
  motorDevice.motorEnableTimeMs = 0;
}

void motorEnable(void)
{
  if (motorDevice.initialized && motorDevice.vTable->enable())
  {
      motorDevice.enabled = true;
      motorDevice.motorEnableTimeMs = millis();
  }
}

bool motorIsEnabled(void)
{
    return motorDevice.enabled;
}

bool motorIsMotorEnabled(unsigned index)
{
    return motorDevice.vTable->isMotorEnabled(index);
}

uint8_t getMotorCount(void)
{
    return motorConfig.motorCount;
}

bool motorIsMotorIdle(unsigned index)
{
    return motorDevice.vTable->isMotorIdle ? motorDevice.vTable->isMotorIdle(index) : false;
}

#ifdef USE_DSHOT
timeMs_t motorGetMotorEnableTimeMs(void)
{
    return motorDevice.motorEnableTimeMs;
}
#endif

/* functions below for empty methods and no active motors */
void motorPostInitNull(void)
{
}

static bool motorEnableNull(void)
{
    return false;
}

static void motorDisableNull(void)
{
}

static bool motorIsEnabledNull(unsigned index)
{
    UNUSED(index);
    return false;
}

bool motorDecodeTelemetryNull(void)
{
    return true;
}

void motorWriteNull(uint8_t index, float value)
{
    UNUSED(index);
    UNUSED(value);
}

static void motorWriteIntNull(uint8_t index, uint16_t value)
{
    UNUSED(index);
    UNUSED(value);
}

void motorUpdateCompleteNull(void)
{
}

static void motorShutdownNull(void)
{
}

static float motorConvertFromExternalNull(uint16_t value)
{
    UNUSED(value);
    return 0.0f ;
}

static uint16_t motorConvertToExternalNull(float value)
{
    UNUSED(value);
    return 0;
}

static const motorVTable_t motorNullVTable = {
    .postInit = motorPostInitNull,
    .enable = motorEnableNull,
    .disable = motorDisableNull,
    .isMotorEnabled = motorIsEnabledNull,
    .decodeTelemetry = motorDecodeTelemetryNull,
    .write = motorWriteNull,
    .writeInt = motorWriteIntNull,
    .updateComplete = motorUpdateCompleteNull,
    .convertExternalToMotor = motorConvertFromExternalNull,
    .convertMotorToExternal = motorConvertToExternalNull,
    .shutdown = motorShutdownNull,
    .requestTelemetry = NULL,
    .isMotorIdle = NULL,
    .getMotorIO = NULL,
};

void motorNullDevInit(motorDevice_t *device)
{
    device->vTable = &motorNullVTable;
    device->count = 0;
}
