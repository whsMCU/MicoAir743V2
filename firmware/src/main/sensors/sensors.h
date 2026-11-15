/*
 * sensor.h
 *
 *  Created on: 2020. 12. 20.
 *      Author: WANG
 */

#ifndef SRC_COMMON_CORE_SENSOR_H_
#define SRC_COMMON_CORE_SENSOR_H_

#include "hw.h"

#define GYRO_COUNT 2

typedef enum {
    SENSOR_INDEX_GYRO = 0,
    SENSOR_INDEX_ACC,
    SENSOR_INDEX_BARO,
    SENSOR_INDEX_MAG,
    SENSOR_INDEX_RANGEFINDER,
    SENSOR_INDEX_OPTICALFLOW,
    SENSOR_INDEX_COUNT
} sensorIndex_e;

extern uint8_t detectedSensors[SENSOR_INDEX_COUNT];
extern uint8_t detectedGyros[GYRO_COUNT];

typedef struct int16_flightDynamicsTrims_s {
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    int16_t calibrationCompleted;
} flightDynamicsTrims_def_t;

typedef union flightDynamicsTrims_u {
    int16_t raw[4];
    flightDynamicsTrims_def_t values;
} flightDynamicsTrims_t;

typedef enum {
    SENSOR_GYRO = 1 << 0, // always present
    SENSOR_ACC = 1 << 1,
    SENSOR_BARO = 1 << 2,
    SENSOR_MAG = 1 << 3,
    SENSOR_SONAR = 1 << 4,
    SENSOR_RANGEFINDER = 1 << 4,
    SENSOR_GPS = 1 << 5,
    SENSOR_GPSMAG = 1 << 6,
    SENSOR_OPTICALFLOW = 1 << 7
} sensors_e;

// Type of accelerometer used/detected
// Acc hardware types were updated in PR #14087 (removed ACC_ADXL345, ACC_MMA8452, ACC_BMA280, ACC_LSM303DLHC)
typedef enum {
    ACC_DEFAULT,
    ACC_NONE,
    ACC_MPU6050,
    ACC_MPU6000,
    ACC_MPU6500,
    ACC_MPU9250,
    ACC_ICM20601,
    ACC_ICM20602,
    ACC_ICM20608G,
    ACC_ICM20649,
    ACC_ICM20689,
    ACC_ICM42605,
    ACC_ICM42688P,
    ACC_BMI160,
    ACC_BMI270,
    ACC_LSM6DSO,
    ACC_LSM6DSV16X,
    ACC_IIM42653,
    ACC_ICM45605,
    ACC_ICM45686,
    ACC_ICM40609D,
    ACC_IIM42652,
    ACC_LSM6DSK320X,
    ACC_ICM42622P,
    ACC_ICM42686P,
    ACC_ICM56686,
    ACC_VIRTUAL,
    ACC_HARDWARE_COUNT
} accelerationSensor_e;

bool Sensor_Init(void);



#endif /* SRC_COMMON_CORE_SENSOR_H_ */
