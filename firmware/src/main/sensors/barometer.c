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

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "build/debug.h"

#include "common/maths.h"

#include "fc/runtime_config.h"

#include "sensors/barometer.h"
#include "sensors/sensors.h"

#include "drivers/barometer/barometer.h"
#include "drivers/barometer/barometer_dps310.h"

#include "scheduler/scheduler.h"

#include "navigation/navigation.h"

#ifdef USE_BARO_DPS310

#ifdef _USE_HW_CLI
static void cliDps310(cli_args_t *args);
#endif


baro_t baro;                        // barometer access functions

barometerConfig_t barometerConfig;

void barometerConfig_Init(void)
{
    barometerConfig.baro_sample_count = 21;
    barometerConfig.baro_noise_lpf = 600;
    barometerConfig.baro_cf_vel = 985;
    barometerConfig.baro_hardware = BARO_DEFAULT;

    // For backward compatibility; ceate a valid default value for bus parameters
    //
    // 1. If DEFAULT_BARO_xxx is defined, use it.
    // 2. Determine default based on USE_BARO_xxx
    //   a. Precedence is in the order of popularity; BMP388, BMP280, MS5611 then BMP085, then
    //   b. If SPI variant is specified, it is likely onboard, so take it.

	#define DEFAULT_BARO_DPS310

    barometerConfig.baro_busType = 1;//;BUS_TYPE_I2C;
    barometerConfig.baro_i2c_device = 0;//I2C_DEV_TO_CFG(BARO_I2C_INSTANCE);
    barometerConfig.baro_i2c_address = 0;

}

#define NUM_CALIBRATION_CYCLES   100        // 10 seconds init_delay + 100 * 25 ms = 12.5 seconds before valid baro altitude
#define NUM_GROUND_LEVEL_CYCLES   10        // calibrate baro to new ground level (10 * 25 ms = ~250 ms non blocking)

static uint16_t calibrationCycles = 0;      // baro calibration = get new ground pressure value
static uint16_t calibrationCycleCount = 0;
static float baroGroundAltitude = 0.0f;
static bool baroCalibrated = false;
static bool baroReady = false;

void Baro_Init(void)
{

	dps310Detect(&baro.dev);

	sensorsSet(SENSOR_BARO);

  	#ifdef _USE_HW_CLI
		cliAdd("dps310", cliDps310);
	#endif
}

bool baroIsCalibrated(void)
{
    return baroCalibrated;
}

void baroStartCalibration(void)
{
    if (detectedSensors[SENSOR_INDEX_BARO] == BARO_VIRTUAL) {
        baroCalibrated = true;
        return;
    }
    baroGroundAltitude = 0;
    baroCalibrated = false;
    calibrationCycles = NUM_CALIBRATION_CYCLES;
    calibrationCycleCount = 0;
}

void baroSetGroundLevel(void)
{
    baroGroundAltitude = 0;
    baroCalibrated = false;
    calibrationCycles = NUM_GROUND_LEVEL_CYCLES;
    calibrationCycleCount = 0;
}

#define PRESSURE_SAMPLES_MEDIAN 3

static int32_t applyBarometerMedianFilter(int32_t newPressureReading)
{
    static int32_t barometerFilterSamples[PRESSURE_SAMPLES_MEDIAN];
    static int currentFilterSampleIndex = 0;
    static bool medianFilterReady = false;
    int nextSampleIndex;

    nextSampleIndex = (currentFilterSampleIndex + 1);
    if (nextSampleIndex == PRESSURE_SAMPLES_MEDIAN) {
        nextSampleIndex = 0;
        medianFilterReady = true;
    }

    barometerFilterSamples[currentFilterSampleIndex] = newPressureReading;
    currentFilterSampleIndex = nextSampleIndex;

    if (medianFilterReady)
        return quickMedianFilter3(barometerFilterSamples);
    else
        return newPressureReading;
}

static uint32_t recalculateBarometerTotal(uint32_t pressureTotal, int32_t newPressureReading)
{
    static int32_t barometerSamples[BARO_SAMPLE_COUNT_MAX + 1];
    static int currentSampleIndex = 0;
    int nextSampleIndex;

    // store current pressure in barometerSamples
    if (currentSampleIndex >= barometerConfig.baro_sample_count) {
        nextSampleIndex = 0;
        baroReady = true;
    } else {
        nextSampleIndex = (currentSampleIndex + 1);
    }
    barometerSamples[currentSampleIndex] = applyBarometerMedianFilter(newPressureReading);

    // recalculate pressure total
    pressureTotal += barometerSamples[currentSampleIndex];
    pressureTotal -= barometerSamples[nextSampleIndex];

    currentSampleIndex = nextSampleIndex;

    return pressureTotal;
}

typedef enum {
    BARO_STATE_TEMPERATURE_READ = 0,
    BARO_STATE_TEMPERATURE_SAMPLE,
    BARO_STATE_PRESSURE_START,
    BARO_STATE_PRESSURE_READ,
    BARO_STATE_PRESSURE_SAMPLE,
    BARO_STATE_TEMPERATURE_START,
    BARO_STATE_COUNT
} barometerState_e;

bool isBaroReady(void)
{
    return baroReady;
}

static float pressureToAltitude(const float pressure)
{
    return (1.0f - powf(pressure / 101325.0f, 0.190295f)) * 4433000.0f;
}

void taskUpdateBaro(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    const uint32_t newDeadline = baroUpdate(currentTimeUs);
    if (newDeadline != 0) {
        rescheduleTask(TASK_SELF, newDeadline);
    }
    updatePositionEstimator_BaroTopic(currentTimeUs);
}

static void performBaroCalibrationCycle(const float altitude);

uint32_t baroUpdate(timeUs_t currentTimeUs)
{
  static timeUs_t baroStateDurationUs[BARO_STATE_COUNT];
    static barometerState_e state = BARO_STATE_PRESSURE_START;
    static timeUs_t cycleStartUs = 0;
    barometerState_e oldState = state;
    timeUs_t executeTimeUs;
    timeUs_t sleepTime = 1000; // Wait 1ms between states


    if (busBusy()){
        // If the bus is busy, simply return to have another go later
      schedulerIgnoreTaskStateTime();
      return sleepTime;
    }

    switch (state) {
        default:
        case BARO_STATE_TEMPERATURE_START:
        		cycleStartUs = currentTimeUs;
            baro.dev.start_ut(&baro.dev);
            state = BARO_STATE_TEMPERATURE_READ;
            sleepTime = baro.dev.ut_delay;
            break;

        case BARO_STATE_TEMPERATURE_READ:
            if (baro.dev.read_ut(&baro.dev)) {
                state = BARO_STATE_TEMPERATURE_SAMPLE;
            } else {
                // No action was taken as the read has not completed
                schedulerIgnoreTaskExecTime();
            }
            break;

        case BARO_STATE_TEMPERATURE_SAMPLE:
            if (baro.dev.get_ut(&baro.dev)) {
                state = BARO_STATE_PRESSURE_START;
            } else {
                // No action was taken as the read has not completed
                schedulerIgnoreTaskExecTime();
            }
            break;

        case BARO_STATE_PRESSURE_START:
            baro.dev.start_up(&baro.dev);
            state = BARO_STATE_PRESSURE_READ;
            sleepTime = baro.dev.up_delay;
            break;

        case BARO_STATE_PRESSURE_READ:
            if (baro.dev.read_up(&baro.dev)) {
                state = BARO_STATE_PRESSURE_SAMPLE;
            } else {
                // No action was taken as the read has not completed
                schedulerIgnoreTaskExecTime();
            }
            break;

        case BARO_STATE_PRESSURE_SAMPLE:
            if (!baro.dev.get_up(&baro.dev)) {
                // No action was taken as the read has not completed
                schedulerIgnoreTaskExecTime();
                break;
            }

            // update baro data
            baro.dev.calculate(&baro.pressure, &baro.temperature);

            // If baro.pressure is invalid then skip altitude counting / call of calibration cycle
            if (baro.pressure > 0) {
                const float altitude = pressureToAltitude(baro.pressure);
                if (baroIsCalibrated()) {
                    // zero baro altitude
                    baro.altitude = altitude - baroGroundAltitude;
                } else {
                    // establish stable baroGroundAltitude value to zero baro altitude with
                    performBaroCalibrationCycle(altitude);
                    baro.altitude = 0.0f;
                }
            } else {
                // return 0 during calibration, reuse last value otherwise
                if (!baroIsCalibrated()) {
                    baro.altitude = 0.0f;
                }
            }

            if (debugMode == DEBUG_BARO) {
                DEBUG_SET(DEBUG_BARO, 1, lrintf(baro.pressure / 100.0f));   // hPa
                DEBUG_SET(DEBUG_BARO, 2, baro.temperature);                 // c°C
                DEBUG_SET(DEBUG_BARO, 3, lrintf(baro.altitude));            // cm
            }

            if (baro.dev.combined_read) {
                state = BARO_STATE_PRESSURE_START;
            } else {
                state = BARO_STATE_TEMPERATURE_START;
                // Pace cycle restart so the full cycle completes no faster
                // than TASK_BARO_RATE_HZ. Pacing only bites when the natural
                // cycle would be faster than the target; otherwise sleepTime
                // is left at the default 1ms set above.
                const timeUs_t targetCycleUs = 1000000U / TASK_BARO_RATE_HZ;
                const timeUs_t cycleElapsedUs = currentTimeUs - cycleStartUs;
                if (cycleElapsedUs < targetCycleUs) {
                    sleepTime = targetCycleUs - cycleElapsedUs;
                }
            }
            break;
    }

    // Where we are using a state machine call schedulerIgnoreTaskExecRate() for all states bar one
    if (state != BARO_STATE_PRESSURE_START) {
        schedulerIgnoreTaskExecRate();
    }

    executeTimeUs = micros() - currentTimeUs;

    if (executeTimeUs > baroStateDurationUs[oldState]) {
        baroStateDurationUs[oldState] = executeTimeUs;
    }

    schedulerSetNextStateTime(baroStateDurationUs[state]);

    return sleepTime;
}

float getBaroAltitude(void)
{
    return baro.altitude;
}

static void performBaroCalibrationCycle(const float altitude)
{
    baroGroundAltitude += altitude;
    calibrationCycleCount++;

    if (calibrationCycleCount >= calibrationCycles) {
        baroGroundAltitude /= calibrationCycleCount;  // simple average
        baroCalibrated = true;
        calibrationCycleCount = 0;
    }
}

#ifdef _USE_HW_CLI
void cliDps310(cli_args_t *args)
{
  bool ret = false;

if (args->argc == 1 && args->isStr(0, "baro_show") == true)
{
    uint32_t pre_time;
    pre_time = millis();
    while(cliKeepLoop())
    {
        if (millis()-pre_time >= 1000)
        {
            pre_time = millis();
            // cliPrintf("acc x: %d, y: %d, z: %d\n\r", x, y, z);
        }
    }
    ret = true;
    }

  if (args->argc == 3 && args->isStr(0, "mem_read") == true)
  {
    uint8_t ch;
    uint8_t addr;

    ch   = (uint8_t)args->getData(1);
    addr = (uint8_t)args->getData(2);
    addr |= 0x80;

    ret = true;
  }

    if (args->argc == 4 && args->isStr(0, "mem_write") == true)
  {
    uint8_t ch;
    uint8_t addr;

    ch     = (uint8_t)args->getData(1);
    addr   = (uint8_t)args->getData(2);

    ret = true;
  }

  if (ret != true)
  {
    cliPrintf("dsp310 baro_show \n\r");
    cliPrintf("dsp310 mem_read ch0:1, addr \n\r");
    cliPrintf("dsp310 mem_write ch0:1, addr data \n\r");
  }
}
#endif

#endif /* BARO */
