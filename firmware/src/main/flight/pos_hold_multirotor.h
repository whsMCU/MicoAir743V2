/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#ifndef USE_WING
#ifdef USE_POSITION_HOLD

#include "common/time.h"

#define POSHOLD_TASK_RATE_HZ 100 // hz

typedef enum {
    POSHOLD_SOURCE_AUTO = 0,
    POSHOLD_SOURCE_GPS_ONLY,
    POSHOLD_SOURCE_OPTICALFLOW_ONLY
} posHoldSource_e;

typedef struct posHoldConfig_s {
    uint8_t deadband;
    uint8_t positionSource;              // Position source selection
    uint8_t opticalflowQualityMin;       // Minimum optical flow quality threshold
    uint16_t opticalflowMaxRange;        // Maximum altitude for optical flow (cm)
} posHoldConfig_t;

extern posHoldConfig_t posHoldConfig;

void posHoldInit(void);
void posHoldConfig_Init(void);
void updatePosHold(timeUs_t currentTimeUs);
bool isAutopilotInControl(void);

bool posHoldFailure(void);
bool posHoldReady(void);

#endif // USE_POSITION_HOLD
#endif // !USE_WING
