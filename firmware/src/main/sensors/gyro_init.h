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

#pragma once

#include "hw.h"

#include "sensors/sensors.h"
#include "sensors/gyro.h"

#include "drivers/accgyro/accgyro.h"

typedef struct gyroConfig_s {
    uint8_t gyroMovementCalibrationThreshold; // people keep forgetting that moving model while init results in wrong gyro offsets. and then they never reset gyro. so this is now on by default.
    uint8_t gyro_hardware_lpf;                // gyro DLPF setting
    uint8_t gyro_high_fsr;

    uint16_t gyro_lpf1_static_hz;
    uint16_t gyro_lpf2_static_hz;

    uint16_t gyro_soft_notch_hz_1;
    uint16_t gyro_soft_notch_cutoff_1;
    uint16_t gyro_soft_notch_hz_2;
    uint16_t gyro_soft_notch_cutoff_2;
    int16_t gyro_offset_yaw;
    uint8_t checkOverflow;

    // Lowpass primary/secondary
    uint8_t gyro_lpf1_type;
    uint8_t gyro_lpf2_type;

    uint8_t yaw_spin_recovery;
    int16_t yaw_spin_threshold;

    uint16_t gyroCalibrationDuration;   // Gyro calibration duration in 1/100 second

    uint16_t gyro_lpf1_dyn_min_hz;
    uint16_t gyro_lpf1_dyn_max_hz;

    uint8_t gyro_filter_debug_axis;

    uint8_t gyrosDetected; // What gyros should be shown as part of the cli status command.
    uint8_t gyro_lpf1_dyn_expo; // set the curve for dynamic gyro lowpass filter
    uint8_t simplified_gyro_filter;
    uint8_t simplified_gyro_filter_multiplier;

    uint8_t gyro_enabled_bitmask;
} gyroConfig_t;

extern gyroConfig_t gyroConfig;

void gyroConfig_init(void);
void gyroSetTargetLooptime(uint8_t pidDenom);
void gyroPreInit(void);
bool gyroInit(void);
void gyroInitFilters(void);
struct mpuDetectionResult_s;
const struct mpuDetectionResult_s *gyroMpuDetectionResult(void);
int16_t gyroRateDps(int axis);
uint8_t gyroReadRegister(uint8_t whichSensor, uint8_t reg);
