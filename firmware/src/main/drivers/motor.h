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

#include "common/time.h"
#include "common/axis.h"
#include "drivers/motor_types.h"

//TODO: DMAR is platform specific. This should be moved to platform specific code.
typedef enum {
    DSHOT_DMAR_OFF,
    DSHOT_DMAR_ON,
    DSHOT_DMAR_AUTO
} dshotDmar_e;

typedef enum {
    DSHOT_BITBANG_OFF,
    DSHOT_BITBANG_ON,
    DSHOT_BITBANG_AUTO,
} dshotBitbangMode_e;

typedef enum {
    DSHOT_TELEMETRY_OFF,
    DSHOT_TELEMETRY_ON,
} dshotTelemetry_e;

typedef enum {
    DSHOT_EDT_OFF = 0,
    DSHOT_EDT_ON = 1,
    DSHOT_EDT_FORCE = 2,
} dshotEdt_e;

typedef struct motorConfig_s {
		uint8_t  motorProtocol;                 // Pwm Protocol
    uint16_t digitalIdleOffsetValue;        // Idle value for DShot protocol, full motor output = 10000
    uint16_t minthrottle;                   // Set the minimum throttle command sent to the ESC (Electronic Speed Controller). This is the minimum value that allow motors to run at a idle speed.
    uint16_t maxthrottle;                   // This is the maximum value for the ESCs at full power this value can be increased up to 2000
    uint16_t mincommand;                    // This is the value for the ESCs when they are not armed. In some cases, this value must be lowered down to 900 for some specific ESCs
    uint8_t motorPoleCount;                // Magnetic poles in the motors for calculating actual RPM from eRPM provided by ESC telemetry
    uint8_t motorCount;
    uint8_t  useDshotTelemetry;
    uint8_t  useDshotEdt;
} motorConfig_t;

typedef enum {
  R_R,
  R_F,
  L_R,
  L_F
}motor_e;

extern motorConfig_t motorConfig;
extern float motor[MAX_SUPPORTED_MOTORS];
extern unsigned short LF, LR, RR, RF;

void motorConfig_Init(void);

timeMs_t motorGetMotorEnableTimeMs(void);

bool isMotorProtocolDshot(void);
bool isMotorProtocolBidirDshot(void);
bool isMotorProtocolEnabled(void);

void motorDevInit(void);
unsigned motorDeviceCount(void);
const motorVTable_t *motorGetVTable(void);

void motorWriteAll(void);
void motorDisable(void);
void motorEnable(void);
bool motorIsEnabled(void);
void motorShutdown(void); // Replaces stopPwmAllMotors
uint8_t getMotorCount(void);
