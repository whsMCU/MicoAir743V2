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
#define REGA_INT1_IO		   0x53
#define REGA_INT2_IO		   0x54
#define REGA_INT_MAP	     0x58
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
#define REGG_INT_IO		     0x16
#define REGG_INT_MAP	     0x18
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

    if(dev == BMI088_ACCEL)
    {
      if (spiReadRegMskBufRB(dev, registerId, data, 2)) {
          return data[1];
      } else {
          return 0;
      }
    }else
    {
			if (spiReadRegMskBufRB(dev, registerId, data, 2)) {
					return data[0];
			} else {
					return 0;
			}
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
    // Soft reset
		bmi088RegisterWrite(BMI088_GYRO, REGG_BGW_SOFTRESET, 0xB6, 100);

    // ODR 2kHz, BW 532Hz
    bmi088RegisterWrite(BMI088_GYRO, REGG_BW, 0x00, 1);

    // INT_IO_CONFIG
    bmi088RegisterWrite(BMI088_GYRO, REGG_INT_IO, 0x01, 1);

    // INT_IO_MAP
    bmi088RegisterWrite(BMI088_GYRO, REGG_INT_MAP, 0x01, 1);

    // Enable sampling
    bmi088RegisterWrite(BMI088_GYRO, REGG_INT_CTRL, 0x80, 1);
}

static void bmi088AccInit(void)
{
    // Soft reset
		bmi088RegisterWrite(BMI088_ACCEL, REGA_SOFTRESET, 0xB6, 100);

    // Active mode
		bmi088RegisterWrite(BMI088_ACCEL, REGA_PWR_CONF, 0, 100);

    // ACC ON
		bmi088RegisterWrite(BMI088_ACCEL, REGA_PWR_CTRL, 0x04, 100);

    // OSR4, ODR 1600Hz
		bmi088RegisterWrite(BMI088_ACCEL, REGA_CONF, 0x8C, 1);


    // Range 12g
		bmi088RegisterWrite(BMI088_ACCEL, REGA_RANGE, 0x02, 1);

    // INT_IO_CONFIG
    bmi088RegisterWrite(BMI088_ACCEL, REGA_INT1_IO, 0x0A, 1);

    // INT_IO_MAP
    bmi088RegisterWrite(BMI088_ACCEL, REGA_INT_MAP, 0x04, 1);

    bmi088.acc.acc_1G = 2048;
}

bool bmi088GyroRead(imu_t *gyro)
{
    uint8_t gyroRaw[6];

    if (spiReadRegMskBufRB(BMI088_GYRO, REGG_RATE_X_LSB, gyroRaw, 6)) {
    		gyro->gyro.gyroADCRaw[X] = (float) int16_val_little_endian(gyroRaw, 0);
    		gyro->gyro.gyroADCRaw[Y] = (float) int16_val_little_endian(gyroRaw, 1);
    		gyro->gyro.gyroADCRaw[Z] = (float) int16_val_little_endian(gyroRaw, 2);
        return true;
    }

    return false;
}

bool bmi088AccRead(imu_t *acc)
{
    uint8_t buffer[7];
    if (spiReadRegMskBufRB(BMI088_ACCEL, REGA_X_LSB, buffer, 7)) {
      // first byte is discarded, see datasheet
    	acc->acc.accADCRaw[X] = (float)(((int16_t)(buffer[2] << 8) | buffer[1]) * 3 / 4);
    	acc->acc.accADCRaw[Y] = (float)(((int16_t)(buffer[4] << 8) | buffer[3]) * 3 / 4);
    	acc->acc.accADCRaw[Z] = (float)(((int16_t)(buffer[6] << 8) | buffer[5]) * 3 / 4);
      return true;
    }

    return false;
}

static bool gyroDeviceDetect(void)
{
    uint8_t attempts;

    for (attempts = 0; attempts < 5; attempts++) {
        uint8_t chipId;

        delay(100);

        chipId = bmi088RegisterRead(BMI088_GYRO, REGG_CHIPID);
        //spiReadRegMskBufRB(BMI088_GYRO, REGG_CHIPID, chipId, 2);

        if (chipId == 0x0F) {
            return true;
        }
    }

    return false;
}

static bool accDeviceDetect(void)
{
    uint8_t attempts;

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

	gpioPinWrite(BMI080_ACCEL_CS, _DEF_HIGH);
	gpioPinWrite(BMI080_GYRO_CS, _DEF_HIGH);

  ret = gyroDeviceDetect();
  if(ret == true){
    bmi088GyroInit();
    bmi088.gyro.scale = 1.0f / 16.4f; // 16.4 dps/lsb

  }

  ret = accDeviceDetect();
  if(ret == true){
    bmi088AccInit();
  }
    //gyroInit();

    return ret;
}

void bmi088_AccData_ready(void)
{
	static uint32_t pre_time = 0;
	bmi088.acc.exit_callback_dt = micros() - pre_time;
	pre_time = micros();
	// Ideally we'd use a timer to capture such information, but unfortunately the port used for EXTI interrupt does
	// not have an associated timer
	uint32_t nowCycles = micros();

	bmi088.acc.LastEXTI = nowCycles;

	bmi088.acc.dataReady = true;
	bmi088.acc.detectedEXTI++;
}

void bmi088_GyroData_ready(void)
{
	static uint32_t pre_time = 0;
	bmi088.gyro.exit_callback_dt = micros() - pre_time;
	pre_time = micros();
	// Ideally we'd use a timer to capture such information, but unfortunately the port used for EXTI interrupt does
	// not have an associated timer
	uint32_t nowCycles = micros();
	bmi088.gyro.gyroLastEXTI = nowCycles;

	bmi088.gyro.dataReady = true;
	bmi088.gyro.detectedEXTI++;
}

#endif /* USE_IMU_BMI088 */
