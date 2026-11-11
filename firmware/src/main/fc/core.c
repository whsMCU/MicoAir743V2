/*
 * core.c
 *
 *  Created on: 2026. 10. 12.
 *      Author: 왕학승
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "build/debug.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "flight/pid.h"
#include "drivers/dshot.h"

#include "sensors/gyro.h"

static FAST_DATA_ZERO_INIT uint8_t pidUpdateCounter;

void disarm(flightLogDisarmReason_e reason)
{

    if (!wasLastDisarmUserRequested()) {
        // Non-user disarm, clear the user-initated flag in rc_controls.c
        clearWasLastDisarmUserRequested();
    }

    if (ARMING_FLAG(ARMED)) {
        if (!crashFlipModeActive) {
            ENABLE_ARMING_FLAG(WAS_EVER_ARMED);
        }
        DISABLE_ARMING_FLAG(ARMED); // disarm now
        lastDisarmTimeUs = micros();

#ifdef USE_OSD
        if (IS_RC_MODE_ACTIVE(BOXCRASHFLIP) || isLaunchControlActive()) {
            osdSuppressStats(true);
        }
#endif

#ifdef USE_BLACKBOX
        flightLogEvent_disarm_t eventData;
        eventData.reason = reason;
        blackboxLogEvent(FLIGHT_LOG_EVENT_DISARM, (flightLogEventData_t*)&eventData);

        if (blackboxConfig()->device && blackboxConfig()->mode != BLACKBOX_MODE_ALWAYS_ON) { // Close the log upon disarm except when logging mode is ALWAYS ON
            blackboxFinish();
        }
#else
        UNUSED(reason);
#endif

        BEEP_OFF;

#ifdef USE_PERSISTENT_STATS
        if (!crashFlipModeActive) {
            statsOnDisarm();
        }
#endif
    // Terminate crashflip mode in any disarm
    if (crashFlipModeActive) {
        crashFlipModeActive = false;
    }

        // always set motor direction to normal on disarming
#ifdef USE_DSHOT
        setMotorSpinDirection(DSHOT_CMD_SPIN_DIRECTION_NORMAL);
#endif

        // make disarming beeps, but not if ARMING_DISABLED (RUNAWAY_TAKEOFF or CRASH_DETECTED)
        if (!(getArmingDisableFlags() & (ARMING_DISABLED_RUNAWAY_TAKEOFF | ARMING_DISABLED_CRASH_DETECTED))) {
            beeper(BEEPER_DISARMING);
        }
    }
}

void tryArm(void)
{
    if (armingConfig()->gyro_cal_on_first_arm) {
        gyroStartCalibration(true);
    }


    updateArmingStatus();
    // set or clear armingDisabled flags, while arming is requested, whether armed or disarmed,


    if (!isArmingDisabled()) {
        if (ARMING_FLAG(ARMED)) {
            return;
            // don't allow arming if there are active armingDisabled flags or already armed
        }
        const timeUs_t currentTimeUs = micros();

#ifdef USE_DSHOT
// Handle DShot factors that should delay arming
timeDelta_t beaconTimeDiff = cmpTimeUs(currentTimeUs, getLastDshotBeaconCommandTimeUs());
if (beaconTimeDiff < DSHOT_BEACON_GUARD_DELAY_US && beaconTimeDiff >= 0) {
    if (tryingToArm == ARMING_DELAYED_DISARMED) {
        #ifdef USE_LAUNCH_CONTROL
        if (canUseLaunchControl()) {
            tryingToArm = ARMING_DELAYED_LAUNCH_CONTROL;
        } else
        #endif
        {
            tryingToArm = ARMING_DELAYED_NORMAL;
        }
    }
    return;
}

if (isMotorProtocolDshot()) {
    #if defined(USE_ESC_SENSOR) && defined(USE_DSHOT_TELEMETRY)
    // Try to activate extended DSHOT telemetry only if no esc sensor exists and dshot telemetry is active
    if (!featureIsEnabled(FEATURE_ESC_SENSOR) && useDshotTelemetry) {
        dshotCleanTelemetryData();
        if (motorConfig()->dev.useDshotEdt) {
            dshotCommandWrite(ALL_MOTORS, getMotorCount(), DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE, DSHOT_CMD_TYPE_INLINE);
        }
    }
    #endif

    crashFlipModeActive = IS_RC_MODE_ACTIVE(BOXCRASHFLIP);
    setMotorSpinDirection(crashFlipModeActive ? DSHOT_CMD_SPIN_DIRECTION_REVERSED : DSHOT_CMD_SPIN_DIRECTION_NORMAL);
}
#endif // USE_DSHOT

#ifdef USE_LAUNCH_CONTROL
        if (!crashFlipModeActive && (canUseLaunchControl() || (tryingToArm == ARMING_DELAYED_LAUNCH_CONTROL))) {
            if (launchControlState == LAUNCH_CONTROL_DISABLED) {  // only activate if it hasn't already been triggered
                launchControlState = LAUNCH_CONTROL_ACTIVE;
            }
        }
#endif

#ifdef USE_OSD
        osdSuppressStats(false);
#endif
#ifdef USE_RPM_LIMIT
        mixerResetRpmLimiter();
#endif
        ENABLE_ARMING_FLAG(ARMED);  // ***ARM NOW ***

#ifdef USE_RC_STATS
        NotifyRcStatsArming();
#endif

        resetTryingToArm();

#ifdef USE_ACRO_TRAINER
        pidAcroTrainerInit();
#endif // USE_ACRO_TRAINER

        if (isModeActivationConditionPresent(BOXPREARM)) {
            ENABLE_ARMING_FLAG(WAS_ARMED_WITH_PREARM);
        }
        imuQuaternionHeadfreeOffsetSet();

#if defined(USE_DYN_NOTCH_FILTER)
        resetMaxFFT();
#endif

        disarmAt = currentTimeUs + armingConfig()->auto_disarm_delay * 1000 * 1000;   // start disarm timeout, will be extended when throttle is nonzero

        lastArmingDisabledReason = 0;

#ifdef USE_GPS
        //beep to indicate arming
        if (featureIsEnabled(FEATURE_GPS)) {
            GPS_reset_home_position();
            canUseGPSHeading = false; // block use of GPS Heading in position hold after each arm, until quad can set IMU to GPS COG
            if (STATE(GPS_FIX) && gpsSol.numSat >= gpsRescueConfig()->minSats) {
                beeper(BEEPER_ARMING_GPS_FIX);
            } else {
                beeper(BEEPER_ARMING_GPS_NO_FIX);
            }
        } else {
            beeper(BEEPER_ARMING);
        }
#else
        beeper(BEEPER_ARMING);
#endif

#ifdef USE_PERSISTENT_STATS
        statsOnArm();
#endif

#ifdef USE_RUNAWAY_TAKEOFF
        runawayTakeoffDeactivateUs = 0;
        runawayTakeoffAccumulatedUs = 0;
        runawayTakeoffTriggerUs = 0;
#endif
    } else {
       resetTryingToArm();
        if (!isFirstArmingGyroCalibrationRunning()) {
            int armingDisabledReason = ffs(getArmingDisableFlags());
            if (lastArmingDisabledReason != armingDisabledReason) {
                lastArmingDisabledReason = armingDisabledReason;

                beeperWarningBeeps(armingDisabledReason);
            }
        }
    }
}

FAST_CODE void taskGyroSample(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    gyroUpdate(currentTimeUs);
    if (pidUpdateCounter % activePidLoopDenom == 0) {
        pidUpdateCounter = 0;
    }
    pidUpdateCounter++;
}

FAST_CODE bool gyroFilterReady(void)
{
    if (pidUpdateCounter % activePidLoopDenom == 0) {
        return true;
    } else {
        return false;
    }
}

FAST_CODE bool pidLoopReady(void)
{
    if ((pidUpdateCounter % activePidLoopDenom) == (activePidLoopDenom / 2)) {
        return true;
    }
    return false;
}

FAST_CODE void taskFiltering(timeUs_t currentTimeUs)
{
		#ifdef USE_DSHOT_TELEMETRY
				updateDshotTelemetry();  // decode and update Dshot telemetry
		#endif

    gyroFiltering(currentTimeUs);

}

// Function for loop trigger
FAST_CODE void taskMainPidLoop(timeUs_t currentTimeUs)
{
	// DEBUG_PIDLOOP, timings for:
	// 0 - gyroUpdate()
	// 1 - subTaskPidController()
	// 2 - subTaskMotorUpdate()
	// 3 - subTaskPidSubprocesses()
	//DEBUG_SET(DEBUG_PIDLOOP, 0, micros() - currentTimeUs);

	//subTaskRcCommand(currentTimeUs);
	subTaskPidController(currentTimeUs);
	//subTaskMotorUpdate(currentTimeUs);
	//subTaskPidSubprocesses(currentTimeUs);

	//DEBUG_SET(DEBUG_CYCLETIME, 0, getTaskDeltaTimeUs(TASK_SELF));
	//DEBUG_SET(DEBUG_CYCLETIME, 1, getAverageSystemLoadPercent());
}

#if defined(USE_RUNAWAY_TAKEOFF) || defined(USE_GPS_RESCUE)
// determine if the R/P/Y stick deflection exceeds the specified limit - integer math is good enough here.
bool areSticksActive(uint8_t stickPercentLimit)
{
    for (int axis = FD_ROLL; axis <= FD_YAW; axis ++) {
        if (getRcDeflectionAbs(axis) * 100.f >= stickPercentLimit) {
            return true;
        }
    }
    return false;
}
#endif
