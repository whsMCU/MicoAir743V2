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

#include "hw.h"

//#include "blackbox/blackbox.h"
//#include "blackbox/blackbox_fielddefs.h"

#include "build/debug.h"

//#include "cli/cli.h"

//#include "cms/cms.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"
#include "common/utils.h"

//#include "config/config.h"
#include "config/feature.h"

#include "drivers/dshot.h"
#include "drivers/dshot_command.h"
//#include "drivers/light_led.h"
#include "drivers/motor.h"
//#include "drivers/sound_beeper.h"
//#include "drivers/system.h"
//#include "drivers/time.h"
//#include "drivers/transponder_ir.h"

#include "fc/controlrate_profile.h"
#include "fc/rc.h"
#include "fc/rc_adjustments.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "fc/stats.h"

#include "flight/failsafe.h"
#include "flight/gps_rescue.h"
#include "flight/alt_hold.h"
#include "flight/pos_hold.h"

#if ENABLE_FLIGHT_PLAN && !defined(USE_WING)
#include "flight/flight_plan_capture.h"
#endif
#include "flight/flight_plan_nav.h"

#ifdef USE_OSD_NAV_MAP
#include "flight/nav_trail.h"
#endif

#if defined(USE_DYN_NOTCH_FILTER)
#include "flight/dyn_notch_filter.h"
#endif

#include "flight/imu.h"
//#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/position.h"
#include "flight/rpm_filter.h"
//#include "flight/servos.h"

//#include "io/beeper.h"
#include "io/gps.h"
//#include "io/pidaudio.h"
//#include "io/serial.h"
//#include "io/statusindicator.h"
//#include "io/transponder_ir.h"
//#include "io/vtx_control.h"
//#include "io/vtx_rtc6705.h"

#include "msp/msp_serial.h"

#include "osd/osd.h"

//#include "pg/motor.h"
//#include "pg/pg.h"
//#include "pg/pg_ids.h"
//#include "pg/rx.h"

//#include "rx/rc_stats.h"
#include "rx/rx.h"

#include "scheduler/scheduler.h"

//#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"

#include "telemetry/telemetry.h"

#include "core.h"

enum {
    ALIGN_GYRO = 0,
    ALIGN_ACCEL = 1,
    ALIGN_MAG = 2
};

enum {
    ARMING_DELAYED_DISARMED = 0,
    ARMING_DELAYED_NORMAL = 1,
    ARMING_DELAYED_CRASHFLIP = 2,
    ARMING_DELAYED_LAUNCH_CONTROL = 3,
};

#define GYRO_WATCHDOG_DELAY 80 //  delay for gyro sync

#ifdef USE_RUNAWAY_TAKEOFF
#define RUNAWAY_TAKEOFF_PIDSUM_THRESHOLD         600   // The pidSum threshold required to trigger - corresponds to a pidSum value of 60% (raw 600) in the blackbox viewer
#define RUNAWAY_TAKEOFF_ACTIVATE_DELAY           75000 // (75ms) Time in microseconds where pidSum is above threshold to trigger
#define RUNAWAY_TAKEOFF_DEACTIVATE_STICK_PERCENT 15    // 15% - minimum stick deflection during deactivation phase
#define RUNAWAY_TAKEOFF_DEACTIVATE_PIDSUM_LIMIT  100   // 10.0% - pidSum limit during deactivation phase
#define RUNAWAY_TAKEOFF_GYRO_LIMIT_RP            15    // Roll/pitch 15 deg/sec threshold to prevent triggering during bench testing without props
#define RUNAWAY_TAKEOFF_GYRO_LIMIT_YAW           50    // Yaw 50 deg/sec threshold to prevent triggering during bench testing without props
#define RUNAWAY_TAKEOFF_HIGH_THROTTLE_PERCENT    75    // High throttle limit to accelerate deactivation (halves the deactivation delay)

#define DEBUG_RUNAWAY_TAKEOFF_ENABLED_STATE      0
#define DEBUG_RUNAWAY_TAKEOFF_ACTIVATING_DELAY   1
#define DEBUG_RUNAWAY_TAKEOFF_DEACTIVATING_DELAY 2
#define DEBUG_RUNAWAY_TAKEOFF_DEACTIVATING_TIME  3

#define DEBUG_RUNAWAY_TAKEOFF_TRUE  1
#define DEBUG_RUNAWAY_TAKEOFF_FALSE 0
#endif

#if defined(USE_GPS) || defined(USE_MAG)
int16_t magHold;
#endif

static FAST_DATA_ZERO_INIT uint8_t pidUpdateCounter;

static bool crashFlipModeActive = false;
static timeUs_t disarmAt;     // Time of automatic disarm when "Don't spin the motors when armed" is enabled and auto_disarm_delay is nonzero
static int lastArmingDisabledReason = 0;
static timeUs_t lastDisarmTimeUs;
static int tryingToArm = ARMING_DELAYED_DISARMED;

#ifdef USE_RUNAWAY_TAKEOFF
static timeUs_t runawayTakeoffDeactivateUs = 0;
static timeUs_t runawayTakeoffAccumulatedUs = 0;
static bool runawayTakeoffCheckDisabled = false;
static timeUs_t runawayTakeoffTriggerUs = 0;
static bool runawayTakeoffTemporarilyDisabled = false;
#endif

#ifdef USE_LAUNCH_CONTROL
static launchControlState_e launchControlState = LAUNCH_CONTROL_DISABLED;

const char * const osdLaunchControlModeNames[] = {
    "NORMAL",
    "PITCHONLY",
    "FULL"
};
#endif

throttleCorrectionConfig_t throttleCorrectionConfig;

void throttleCorrectionConfig_Init(void)
{
	throttleCorrectionConfig.throttle_correction_value = 0;      // could 10 with althold or 40 for fpv
	throttleCorrectionConfig.throttle_correction_angle = 800;     // could be 80.0 deg with atlhold or 45.0 for fpv
}

#ifdef USE_DSHOT
static void setMotorSpinDirection(uint8_t spinDirection)
{
    if (isMotorProtocolDshot()) { //    if (isMotorProtocolDshot() && !featureIsEnabled(FEATURE_3D)) {
        dshotCommandWrite(ALL_MOTORS, getMotorCount(), spinDirection, DSHOT_CMD_TYPE_INLINE);
    }
}
#endif

void updateArmingStatus(void)
{

    if (ARMING_FLAG(ARMED)) {
        LED0_ON;

#ifdef USE_DSHOT
// --- handle crashFlip behaviours while armed ---
if (crashFlipModeActive) {
    if (!IS_RC_MODE_ACTIVE(BOXCRASHFLIP)) {
        // Pilot has reverted the crash flip switch while crashflip is active and craft is  armed
        if (!mixerConfig()->crashflip_auto_rearm) {
            // we are in manual re-arm mode:  block arming until manual re-arm:
            setArmingDisabled(ARMING_DISABLED_CRASHFLIP); // block tryArm() until user disarms manually
            clearWasLastDisarmUserRequested();
            // tell disarm() that this was not a user generated disarm
            // also clear the flag in rc_controls so that it will only be true when the pilot makes a new user manual disarm
            disarm(DISARM_REASON_CRASHFLIP); // stop the motors, revert crashflipMode and set motor direction normal
        } else {
            // we are in auto re-arm mode, terminate crashflip mode and set motor direction normal
            setMotorSpinDirection(DSHOT_CMD_SPIN_DIRECTION_NORMAL);
            crashFlipModeActive = false;
        }
    }
}
#endif // USE_DSHOT
    } else {
        // arming switch on, but not yet armed; currently DISARMED
        // identify things that should delay, or prevent, arming, then arm

        // Check if the power-on arming grace time has elapsed
        bool graceTimeElapsed =
            (getArmingDisableFlags() & ARMING_DISABLED_BOOT_GRACE_TIME)
            && (millis() >= systemConfig()->powerOnArmingGraceTime * 1000);

#ifdef USE_DSHOT
        // With DSHOT, we also require DSHOT to be ready
        graceTimeElapsed = graceTimeElapsed && (!isMotorProtocolDshot() || dshotStreamingCommandsAreEnabled());
#endif

        if (graceTimeElapsed) {
            unsetArmingDisabled(ARMING_DISABLED_BOOT_GRACE_TIME);
        }
        // If switch is used for arming then check it is not defaulting to on when the RX link recovers from a fault
        if (!isUsingSticksForArming()) {
            static bool hadRx = false;
            const bool haveRx = isRxReceivingSignal();

            const bool justGotRxBack = !hadRx && haveRx;

            if (justGotRxBack && IS_RC_MODE_ACTIVE(BOXARM)) {
                // If the RX has just started to receive a signal again and the arm switch is on, apply arming restriction
                setArmingDisabled(ARMING_DISABLED_NOT_DISARMED);
            } else if (haveRx && !IS_RC_MODE_ACTIVE(BOXARM)) {
                // If RX signal is OK and the arm switch is off, remove arming restriction
                unsetArmingDisabled(ARMING_DISABLED_NOT_DISARMED);
            }
        hadRx = haveRx;
        }

        if (getArmingDisableFlags() & ARMING_DISABLED_CRASHFLIP) {
             // if, while disarmed, arming was blocked due to crashflip, eg in manual mode and the crashflip switch was reverted, watch for a manual user disarm
             if (wasLastDisarmUserRequested()) {
             // if there has been a new manual user disarm since the last time its flag was cleared, eg since the crashflip switch was reverted in manual mode
                unsetArmingDisabled(ARMING_DISABLED_CRASHFLIP);
                // clear the armingDisabled block to permit re-arming
            }
        }

        if (IS_RC_MODE_ACTIVE(BOXFAILSAFE)) {
            setArmingDisabled(ARMING_DISABLED_BOXFAILSAFE);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_BOXFAILSAFE);
        }

        if (IS_RC_MODE_ACTIVE(BOXALTHOLD)) {
            setArmingDisabled(ARMING_DISABLED_ALTHOLD);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_ALTHOLD);
        }

        if (IS_RC_MODE_ACTIVE(BOXPOSHOLD)) {
            setArmingDisabled(ARMING_DISABLED_POSHOLD);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_POSHOLD);
        }

#if ENABLE_FLIGHT_PLAN && !defined(USE_WING)
        if (IS_RC_MODE_ACTIVE(BOXAUTOPILOT)) {
            setArmingDisabled(ARMING_DISABLED_AUTOPILOT);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_AUTOPILOT);
        }
#endif

        if (calculateThrottleStatus() != THROTTLE_LOW) {
            setArmingDisabled(ARMING_DISABLED_THROTTLE);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_THROTTLE);
        }

        if (!isUpright() && !IS_RC_MODE_ACTIVE(BOXCRASHFLIP)) {
            setArmingDisabled(ARMING_DISABLED_ANGLE);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_ANGLE);
        }

#if defined(USE_LATE_TASK_STATISTICS)
        if ((getCpuPercentageLate() > schedulerConfig.cpuLatePercentageLimit)) {
            setArmingDisabled(ARMING_DISABLED_LOAD);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_LOAD);
        }
#endif // USE_LATE_TASK_STATISTICS

        if (isCalibrating()) {
            setArmingDisabled(ARMING_DISABLED_CALIBRATING);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_CALIBRATING);
        }

        if (isModeActivationConditionPresent(BOXPREARM)) {
            if (IS_RC_MODE_ACTIVE(BOXPREARM) && (!ARMING_FLAG(WAS_ARMED_WITH_PREARM) || armingConfig()->prearm_allow_rearm) ) {
                unsetArmingDisabled(ARMING_DISABLED_NOPREARM);
            } else {
                setArmingDisabled(ARMING_DISABLED_NOPREARM);
            }
        }

#ifdef USE_GPS_RESCUE
        if (gpsRescueIsConfigured()) {
            if (gpsRescueConfig.allowArmingWithoutFix || (STATE(GPS_FIX) && (gpsSol.numSat >= gpsRescueConfig()->minSats)) ||
            ARMING_FLAG(WAS_EVER_ARMED) || IS_RC_MODE_ACTIVE(BOXCRASHFLIP)) {
                unsetArmingDisabled(ARMING_DISABLED_GPS);
            } else {
                setArmingDisabled(ARMING_DISABLED_GPS);
            }
            if (IS_RC_MODE_ACTIVE(BOXGPSRESCUE)) {
                setArmingDisabled(ARMING_DISABLED_RESC);
            } else {
                unsetArmingDisabled(ARMING_DISABLED_RESC);
            }
        }
#endif

#ifdef USE_DSHOT_TELEMETRY
        // If Dshot Telemetry is enabled and any motor isn't providing telemetry, then disable arming
        if (useDshotTelemetry && !isDshotTelemetryActive()) {
            setArmingDisabled(ARMING_DISABLED_DSHOT_TELEM);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_DSHOT_TELEM);
        }
#endif

#ifdef USE_DSHOT_BITBANG
        if (isDshotBitbangActive(&motorConfig()->dev) && dshotBitbangGetStatus() != DSHOT_BITBANG_STATUS_OK) {
            setArmingDisabled(ARMING_DISABLED_DSHOT_BITBANG);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_DSHOT_BITBANG);
        }
#endif

        if (IS_RC_MODE_ACTIVE(BOXPARALYZE)) {
            setArmingDisabled(ARMING_DISABLED_PARALYZE);
        }

#ifdef USE_ACC
        if (accNeedsCalibration()) {
            setArmingDisabled(ARMING_DISABLED_ACC_CALIBRATION);
        } else {
            unsetArmingDisabled(ARMING_DISABLED_ACC_CALIBRATION);
        }
#endif

        if (!isMotorProtocolEnabled()) {
            setArmingDisabled(ARMING_DISABLED_MOTOR_PROTOCOL);
        }

        if (!isUsingSticksForArming()) {
            if (!IS_RC_MODE_ACTIVE(BOXARM)) {
#ifdef USE_RUNAWAY_TAKEOFF
                unsetArmingDisabled(ARMING_DISABLED_RUNAWAY_TAKEOFF);
#endif
                unsetArmingDisabled(ARMING_DISABLED_CRASH_DETECTED);
            }

            /* Ignore ARMING_DISABLED_CALIBRATING if we are going to calibrate gyro on first arm */
            bool ignoreGyro = armingConfig.gyro_cal_on_first_arm
                && !(getArmingDisableFlags() & ~(ARMING_DISABLED_ARM_SWITCH | ARMING_DISABLED_CALIBRATING));

            /* Ignore ARMING_DISABLED_THROTTLE (once arm switch is on) if we are in 3D mode */
            bool ignoreThrottle = featureIsEnabled(FEATURE_3D)
                 && !IS_RC_MODE_ACTIVE(BOX3D)
                 && !flight3DConfig.switched_mode3d
                 && !(getArmingDisableFlags() & ~(ARMING_DISABLED_ARM_SWITCH | ARMING_DISABLED_THROTTLE));

            // If arming is disabled and the ARM switch is on
            if (isArmingDisabled()
                && !ignoreGyro
                && !ignoreThrottle
                && IS_RC_MODE_ACTIVE(BOXARM)) {
                setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
            } else if (!IS_RC_MODE_ACTIVE(BOXARM)) {
                unsetArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
            }
        }

        if (isArmingDisabled()) {
            warningLedFlash();
        } else {
            warningLedDisable();
        }

        warningLedUpdate();
    }
}

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

        //BEEP_OFF;

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
//        if (!(getArmingDisableFlags() & (ARMING_DISABLED_RUNAWAY_TAKEOFF | ARMING_DISABLED_CRASH_DETECTED))) {
//            beeper(BEEPER_DISARMING);
//        }
    }
}

void tryArm(void)
{
    if (armingConfig.gyro_cal_on_first_arm) {
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

#ifdef USE_DSHOT_
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

        disarmAt = currentTimeUs + armingConfig.auto_disarm_delay * 1000 * 1000;   // start disarm timeout, will be extended when throttle is nonzero

        lastArmingDisabledReason = 0;

#ifdef USE_GPS
        //beep to indicate arming
        if (featureIsEnabled(FEATURE_GPS)) {
            GPS_reset_home_position();
            canUseGPSHeading = false; // block use of GPS Heading in position hold after each arm, until quad can set IMU to GPS COG
            if (STATE(GPS_FIX) && gpsSol.numSat >= gpsRescueConfig.minSats) {
                //beeper(BEEPER_ARMING_GPS_FIX);
            } else {
                //beeper(BEEPER_ARMING_GPS_NO_FIX);
            }
        } else {
            //beeper(BEEPER_ARMING);
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
