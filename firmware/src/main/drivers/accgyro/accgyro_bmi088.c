/*
 * This file is part of INAV.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */

#include <stdbool.h>
#include <stdint.h>

#include "hw.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

//#include "drivers/system.h"
//#include "drivers/time.h"
//#include "drivers/io.h"
//#include "drivers/bus.h"

#include "sensors/gyro.h"
#include "drivers/sensor.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_bmi088.h"


#if defined(USE_IMU_BMI088)

/*
  device registers, names follow datasheet conventions, with REGA_
  prefix for accel, and REGG_ prefix for gyro
 */
#define REGA_CHIPID        0x00
#define REGA_ERR_REG       0x02
#define REGA_STATUS        0x03
#define REGA_X_LSB         0x12
#define REGA_INT_STATUS_1  0x1D
#define REGA_TEMP_LSB      0x22
#define REGA_TEMP_MSB      0x23
#define REGA_CONF          0x40
#define REGA_RANGE         0x41
#define REGA_PWR_CONF      0x7C
#define REGA_PWR_CTRL      0x7D
#define REGA_SOFTRESET     0x7E
#define REGA_FIFO_CONFIG0  0x48
#define REGA_FIFO_CONFIG1  0x49
#define REGA_FIFO_DOWNS    0x45
#define REGA_FIFO_DATA     0x26
#define REGA_FIFO_LEN0     0x24
#define REGA_FIFO_LEN1     0x25

#define REGG_CHIPID        0x00
#define REGG_RATE_X_LSB    0x02
#define REGG_INT_CTRL      0x15
#define REGG_INT_STATUS_1  0x0A
#define REGG_INT_STATUS_2  0x0B
#define REGG_INT_STATUS_3  0x0C
#define REGG_FIFO_STATUS   0x0E
#define REGG_RANGE         0x0F
#define REGG_BW            0x10
#define REGG_LPM1          0x11
#define REGG_RATE_HBW      0x13
#define REGG_BGW_SOFTRESET 0x14
#define REGG_FIFO_CONFIG_1 0x3E
#define REGG_FIFO_DATA     0x3F

// BMI088 register reads are 16bits with the first byte a "dummy" value 0
// that must be ignored. The result is in the second byte.
static uint8_t bmi088RegisterRead(uint8_t dev, uint8_t registerId)
{
    uint8_t data[2] = { 0, 0 };

    if (spiReadRegMskBufRB(dev, registerId, data, 2)) {
        return data[1];
    } else {
        return 0;
    }
}

static void bmi088RegisterWrite(uint8_t dev, uint8_t registerId, uint8_t value, uint8_t delayMs)
{
    spiWriteReg(dev, registerId, value);
    if (delayMs) {
        delay(delayMs);
    }
}

static void bmi088GyroInit(void)
{
    //busSetSpeed(gyro->busDev, BUS_SPEED_INITIALIZATION);

    // Soft reset
		bmi088RegisterWrite(BMI088_GYRO, REGG_BGW_SOFTRESET, 0xB6, 1);
    delay(100);

    // ODR 2kHz, BW 532Hz
    bmi088RegisterWrite(BMI088_GYRO, REGG_BW, 0x81, 1);
    delay(1);

    // Enable sampling
    bmi088RegisterWrite(BMI088_GYRO, REGG_INT_CTRL, 0x80, 1);
    delay(1);

    //busSetSpeed(gyro->busDev, BUS_SPEED_FAST);
}

static void bmi088AccInit(void)
{
    //busSetSpeed(acc->busDev, BUS_SPEED_INITIALIZATION);

    // Soft reset
		bmi088RegisterWrite(BMI088_ACCEL, REGA_SOFTRESET, 0xB6, 1);
    delay(100);

    // Active mode
		bmi088RegisterWrite(BMI088_ACCEL, REGA_PWR_CONF, 0, 1);
    delay(100);

    // ACC ON
		bmi088RegisterWrite(BMI088_ACCEL, REGA_PWR_CTRL, 0x04, 1);
    delay(100);

    // OSR4, ODR 1600Hz
		bmi088RegisterWrite(BMI088_ACCEL, REGA_CONF, 0x8C, 1);
    delay(1);

    // Range 12g
		bmi088RegisterWrite(BMI088_ACCEL, REGA_RANGE, 0x02, 1);
    delay(1);

    //busSetSpeed(acc->busDev, BUS_SPEED_STANDARD);

    bmi088.acc_1G = 2048;
}

bool bmi088GyroRead(imu_t *gyro)
{
    uint8_t gyroRaw[6];

    if (spiReadRegMskBufRB(BMI088_GYRO, REGG_RATE_X_LSB, gyroRaw, 6)) {
    		gyro->gyroADCRaw[X] = (float) int16_val_little_endian(gyroRaw, 0);
    		gyro->gyroADCRaw[Y] = (float) int16_val_little_endian(gyroRaw, 1);
    		gyro->gyroADCRaw[Z] = (float) int16_val_little_endian(gyroRaw, 2);
        return true;
    }

    return false;
}

bool bmi088AccRead(imu_t *acc)
{
    uint8_t buffer[7];
    if (spiReadRegMskBufRB(BMI088_ACCEL, REGA_STATUS, buffer, 2) && (buffer[1] & 0x80) && spiReadRegMskBufRB(BMI088_ACCEL, REGA_X_LSB, buffer, 7)) {
      // first byte is discarded, see datasheet
    	acc->accADCRaw[X] = (float)(((int16_t)(buffer[2] << 8) | buffer[1]) * 3 / 4);
    	acc->accADCRaw[Y] = (float)(((int16_t)(buffer[4] << 8) | buffer[3]) * 3 / 4);
    	acc->accADCRaw[Z] = (float)(((int16_t)(buffer[6] << 8) | buffer[5]) * 3 / 4);
      return true;
    }

    return false;
}

static bool gyroDeviceDetect(void)
{
    uint8_t attempts;

    //busSetSpeed(busDev, BUS_SPEED_INITIALIZATION);

    for (attempts = 0; attempts < 5; attempts++) {
        uint8_t chipId;

        delay(100);

        chipId = bmi088RegisterRead(BMI088_GYRO, REGG_CHIPID);

        if (chipId == 0x0F) {
            return true;
        }
    }

    return false;
}

static bool accDeviceDetect(void)
{
    uint8_t attempts;

    //busSetSpeed(busDev, BUS_SPEED_INITIALIZATION);

    for (attempts = 0; attempts < 5; attempts++) {
        uint8_t chipId;

        delay(100);
        chipId = bmi088RegisterRead(BMI088_ACCEL, REGA_CHIPID);

        if (chipId == 0x1E) {
            return true;
        }
    }

    return false;
}

bool bmi088_Init(void)
{
  bool ret = false;

  ret = gyroDeviceDetect();
  if(ret == true){
    bmi088GyroInit();
    bmi088.scale = 1.0f / 16.4f; // 16.4 dps/lsb

  }

  ret = accDeviceDetect();
  if(ret == true){
    bmi088AccInit();
  }
    //gyroInit();

    return ret;
}

#endif /* USE_IMU_BMI088 */
