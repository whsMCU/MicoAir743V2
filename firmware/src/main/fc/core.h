/*
 * core.h
 *
 *  Created on: 2026. 10. 12.
 *      Author: 왕학승
 */

#ifndef SRC_MAIN_FC_CORE_H_
#define SRC_MAIN_FC_CORE_H_

#include "hw.h"

#include "common/time.h"

#if defined(USE_GPS) || defined(USE_MAG)
extern int16_t magHold;
#endif

typedef struct throttleCorrectionConfig_s {
    uint16_t throttle_correction_angle;     // the angle when the throttle correction is maximal. in 0.1 degres, ex 225 = 22.5 ,30.0, 450 = 45.0 deg
    uint8_t throttle_correction_value;      // the correction that will be applied at throttle_correction_angle.
} throttleCorrectionConfig_t;

typedef enum {
    LAUNCH_CONTROL_DISABLED = 0,
    LAUNCH_CONTROL_ACTIVE,
    LAUNCH_CONTROL_TRIGGERED,
} launchControlState_e;

typedef enum {
    LAUNCH_CONTROL_MODE_NORMAL = 0,
    LAUNCH_CONTROL_MODE_PITCHONLY,
    LAUNCH_CONTROL_MODE_FULL,
    LAUNCH_CONTROL_MODE_COUNT // must be the last element
} launchControlMode_e;

typedef enum {
    DISARM_REASON_ARMING_DISABLED   = 0,
    DISARM_REASON_FAILSAFE          = 1,
    DISARM_REASON_THROTTLE_TIMEOUT  = 2,
    DISARM_REASON_STICKS            = 3,
    DISARM_REASON_SWITCH            = 4,
    DISARM_REASON_CRASH_PROTECTION  = 5,
    DISARM_REASON_RUNAWAY_TAKEOFF   = 6,
    DISARM_REASON_GPS_RESCUE        = 7,
    DISARM_REASON_SERIAL_COMMAND    = 8,
    DISARM_REASON_LANDING           = 9,
    DISARM_REASON_CRASHFLIP         = 10,
#ifdef UNIT_TEST
    DISARM_REASON_SYSTEM            = 255,
#endif
} flightLogDisarmReason_e;

#ifdef USE_LAUNCH_CONTROL
#define LAUNCH_CONTROL_THROTTLE_TRIGGER_MAX 90
extern const char * const osdLaunchControlModeNames[LAUNCH_CONTROL_MODE_COUNT];
#endif

extern throttleCorrectionConfig_t throttleCorrectionConfig;

void throttleCorrectionConfig_Init(void);

void disarm(flightLogDisarmReason_e reason);
void tryArm(void);

void taskGyroSample(timeUs_t currentTimeUs);

bool gyroFilterReady(void);

bool pidLoopReady(void);

void taskFiltering(timeUs_t currentTimeUs);

void taskMainPidLoop(timeUs_t currentTimeUs);

bool areSticksActive(uint8_t stickPercentLimit);

#endif /* SRC_MAIN_FC_CORE_H_ */
