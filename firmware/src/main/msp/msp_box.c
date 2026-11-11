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
#include <string.h>

#include "hw.h"

#include "common/bitarray.h"
#include "common/streambuf.h"
#include "common/utils.h"

#include "config/feature.h"

//#include "config/config.h"
#include "fc/runtime_config.h"

//#include "flight/mixer.h"
#include "flight/pid.h"

#include "sensors/sensors.h"

#include "telemetry/telemetry.h"

//#include "pg/piniobox.h"

#include "msp_box.h"

#include "navigation/navigation.h"

#define BOX_SUFFIX ';'
#define BOX_SUFFIX_LEN 1

// permanent IDs must uniquely identify BOX meaning, DO NOT REUSE THEM!
static const box_t boxes[CHECKBOX_ITEM_COUNT] = {
    { .boxId = BOXARM, .boxName = "ARM", .permanentId = 0 },
    { .boxId = BOXANGLE, .boxName = "ANGLE", .permanentId = 1 },
    { .boxId = BOXHORIZON, .boxName = "HORIZON", .permanentId = 2 },
    { .boxId = BOXALTHOLD, .boxName = "ALTHOLD", .permanentId = 3 },
    { .boxId = BOXANTIGRAVITY, .boxName = "ANTI GRAVITY", .permanentId = 4 },
    { .boxId = BOXMAG, .boxName = "MAG", .permanentId = 5 },
    { .boxId = BOXHEADFREE, .boxName = "HEADFREE", .permanentId = 6 },
    { .boxId = BOXHEADADJ, .boxName = "HEADADJ", .permanentId = 7 },
    { .boxId = BOXCAMSTAB, .boxName = "CAMSTAB", .permanentId = 8 },
//    { .boxId = BOXCAMTRIG, .boxName = "CAMTRIG", .permanentId = 9 },
//    { .boxId = BOXGPSHOME, .boxName = "GPS HOME", .permanentId = 10 },
    { .boxId = BOXPOSHOLD, .boxName = "POS HOLD", .permanentId = 11 },
    { .boxId = BOXPASSTHRU, .boxName = "PASSTHRU", .permanentId = 12 },
    { .boxId = BOXBEEPERON, .boxName = "BEEPER", .permanentId = 13 },
//    { .boxId = BOXLEDMAX, .boxName = "LEDMAX", .permanentId = 14 }, (removed)
    { .boxId = BOXLEDLOW, .boxName = "LEDLOW", .permanentId = 15 },
//    { .boxId = BOXLLIGHTS, .boxName = "LLIGHTS", .permanentId = 16 }, (removed)
    { .boxId = BOXCALIB, .boxName = "CALIB", .permanentId = 17 },
//    { .boxId = BOXGOV, .boxName = "GOVERNOR", .permanentId = 18 }, (removed)
    { .boxId = BOXOSD, .boxName = "OSD DISABLE", .permanentId = 19 },
    { .boxId = BOXTELEMETRY, .boxName = "TELEMETRY", .permanentId = 20 },
//    { .boxId = BOXGTUNE, .boxName = "GTUNE", .permanentId = 21 }, (removed)
//    { .boxId = BOXRANGEFINDER, .boxName = "RANGEFINDER", .permanentId = 22 }, (removed)
    { .boxId = BOXSERVO1, .boxName = "SERVO1", .permanentId = 23 },
    { .boxId = BOXSERVO2, .boxName = "SERVO2", .permanentId = 24 },
    { .boxId = BOXSERVO3, .boxName = "SERVO3", .permanentId = 25 },
    { .boxId = BOXBLACKBOX, .boxName = "BLACKBOX", .permanentId = 26 },
    { .boxId = BOXFAILSAFE, .boxName = "FAILSAFE", .permanentId = 27 },
    { .boxId = BOXAIRMODE, .boxName = "AIR MODE", .permanentId = 28 },
    { .boxId = BOX3D, .boxName = "3D DISABLE", .permanentId = 29},
    { .boxId = BOXFPVANGLEMIX, .boxName = "FPV ANGLE MIX", .permanentId = 30},
    { .boxId = BOXBLACKBOXERASE, .boxName = "BLACKBOX ERASE", .permanentId = 31 },
    { .boxId = BOXCAMERA1, .boxName = "CAMERA CONTROL 1", .permanentId = 32},
    { .boxId = BOXCAMERA2, .boxName = "CAMERA CONTROL 2", .permanentId = 33},
    { .boxId = BOXCAMERA3, .boxName = "CAMERA CONTROL 3", .permanentId = 34 },
    { .boxId = BOXCRASHFLIP, .boxName = "FLIP OVER AFTER CRASH", .permanentId = 35 },
    { .boxId = BOXPREARM, .boxName = "PREARM", .permanentId = 36 },
    { .boxId = BOXBEEPGPSCOUNT, .boxName = "GPS BEEP SATELLITE COUNT", .permanentId = 37 },
//    { .boxId = BOX3DONASWITCH, .boxName = "3D ON A SWITCH", .permanentId = 38 }, (removed)
    { .boxId = BOXVTXPITMODE, .boxName = "VTX PIT MODE", .permanentId = 39 },
    { .boxId = BOXUSER1, .boxName = BOX_USER1_NAME, .permanentId = 40 }, // may be overridden by modeActivationConfig
    { .boxId = BOXUSER2, .boxName = BOX_USER2_NAME, .permanentId = 41 },
    { .boxId = BOXUSER3, .boxName = BOX_USER3_NAME, .permanentId = 42 },
    { .boxId = BOXUSER4, .boxName = BOX_USER4_NAME, .permanentId = 43 },
    { .boxId = BOXPIDAUDIO, .boxName = "PID AUDIO", .permanentId = 44 },
    { .boxId = BOXPARALYZE, .boxName = "PARALYZE", .permanentId = 45 },
    { .boxId = BOXGPSRESCUE, .boxName = "GPS RESCUE", .permanentId = 46 },
    { .boxId = BOXACROTRAINER, .boxName = "ACRO TRAINER", .permanentId = 47 },
    { .boxId = BOXVTXCONTROLDISABLE, .boxName = "VTX CONTROL DISABLE", .permanentId = 48},
    { .boxId = BOXLAUNCHCONTROL, .boxName = "LAUNCH CONTROL", .permanentId = 49 },
    { .boxId = BOXMSPOVERRIDE, .boxName = "MSP OVERRIDE", .permanentId = 50},
    { .boxId = BOXSTICKCOMMANDDISABLE, .boxName = "STICK COMMANDS DISABLE", .permanentId = 51},
    { .boxId = BOXBEEPERMUTE, .boxName = "BEEPER MUTE", .permanentId = 52},
    { .boxId = BOXREADY, .boxName = "READY", .permanentId = 53},
    { .boxId = BOXLAPTIMERRESET, .boxName = "LAP TIMER RESET", .permanentId = 54},
    { .boxId = BOXCHIRP, .boxName = "CHIRP", .permanentId = 55},
    { .boxId = BOXAUTOPILOT, .boxName = "AUTOPILOT", .permanentId = 56},
    { .boxId = BOXWPCAPTURE, .boxName = "WP CAPTURE", .permanentId = 57}
};

// mask of enabled IDs, calculated on startup based on enabled features. boxId_e is used as bit index

static boxBitmask_t activeBoxIds;

const box_t *findBoxByActiveBoxId(boxId_e boxId)
{
  for (unsigned i = 0; i < ARRAYLEN(boxes); i++) {
      const box_t *candidate = &boxes[i];
      if (candidate->boxId == boxId)
          return candidate;
  }
  return NULL;
}

const box_t *findBoxByPermanentId(uint8_t permenantId)
{
  for (unsigned i = 0; i < ARRAYLEN(boxes); i++) {
      const box_t *candidate = &boxes[i];
      if (candidate->permanentId == permanentId)
          return candidate;
  }
  return NULL;
}

bool serializeBoxNamesReply(sbuf_t *dst)
{
    // First run of the loop - calculate total length of the reply
    int replyLengthTotal = 0;
    for (int i = 0; i < activeBoxIdCount; i++) {
        const box_t *box = findBoxByActiveBoxId(activeBoxIds[i]);
        if (box) {
            replyLengthTotal += strlen(box->boxName) + BOX_SUFFIX_LEN;
        }
    }

    // Check if we have enough space to send a reply
    if (sbufBytesRemaining(dst) < replyLengthTotal) {
        return false;
    }

    for (int i = 0; i < activeBoxIdCount; i++) {
        const int activeBoxId = activeBoxIds[i];
        const box_t *box = findBoxByActiveBoxId(activeBoxId);
        if (box) {
            const int len = strlen(box->boxName);
            sbufWriteData(dst, box->boxName, len);
            sbufWriteU8(dst, BOX_SUFFIX);
        }
    }

    return true;
}

void serializeBoxReply(sbuf_t *dst)
{
    for (int i = 0; i < activeBoxIdCount; i++) {
        const box_t *box = findBoxByActiveBoxId(activeBoxIds[i]);
        if (!box) {
            continue;
        }
        sbufWriteU8(dst, box->permanentId);
    }
}

void initActiveBoxIds(void)
{
    // calculate used boxes based on features and fill availableBoxes[] array
    memset(activeBoxIds, 0xFF, sizeof(activeBoxIds));

    RESET_BOX_ID_COUNT;
    ADD_ACTIVE_BOX(BOXARM);
    ADD_ACTIVE_BOX(BOXPREARM);

    if (sensors(SENSOR_ACC) && STATE(ALTITUDE_CONTROL)) {
        ADD_ACTIVE_BOX(BOXANGLE);
        ADD_ACTIVE_BOX(BOXHORIZON);
        ADD_ACTIVE_BOX(BOXTURNASSIST);
    }

//    if (!feature(FEATURE_AIRMODE) && STATE(ALTITUDE_CONTROL)) {
//        ADD_ACTIVE_BOX(BOXAIRMODE);
//    }

    ADD_ACTIVE_BOX(BOXHEADINGHOLD);

    //Camstab mode is enabled always
    ADD_ACTIVE_BOX(BOXCAMSTAB);

    if (STATE(MULTIROTOR)) {
        if ((sensors(SENSOR_ACC) || sensors(SENSOR_MAG))) {
            ADD_ACTIVE_BOX(BOXHEADFREE);
            ADD_ACTIVE_BOX(BOXHEADADJ);
        }
        if (sensors(SENSOR_BARO) && sensors(SENSOR_RANGEFINDER)) {
            ADD_ACTIVE_BOX(BOXSURFACE);
        }
        ADD_ACTIVE_BOX(BOXFPVANGLEMIX);
    }

    bool navReadyAltControl = sensors(SENSOR_BARO);
//#ifdef USE_GPS
//    navReadyAltControl = navReadyAltControl || (feature(FEATURE_GPS) && (STATE(AIRPLANE) || positionEstimationConfig()->use_gps_no_baro));
//
//    const bool navFlowDeadReckoning = sensors(SENSOR_OPTICALFLOW) && sensors(SENSOR_ACC) && positionEstimationConfig()->allow_dead_reckoning;
//    bool navReadyPosControl = sensors(SENSOR_ACC) && feature(FEATURE_GPS);
//    if (STATE(MULTIROTOR)) {
//        navReadyPosControl = navReadyPosControl && getHwCompassStatus() != HW_SENSOR_NONE;
//    }
//
//    if (STATE(ALTITUDE_CONTROL) && navReadyAltControl && (navReadyPosControl || navFlowDeadReckoning)) {
//        ADD_ACTIVE_BOX(BOXNAVPOSHOLD);
//        if (STATE(AIRPLANE)) {
//            ADD_ACTIVE_BOX(BOXLOITERDIRCHN);
//        }
//    }
//
//    if (navReadyPosControl) {
//        if (!STATE(ALTITUDE_CONTROL) || (STATE(ALTITUDE_CONTROL) && navReadyAltControl)) {
//            ADD_ACTIVE_BOX(BOXNAVRTH);
//            ADD_ACTIVE_BOX(BOXNAVWP);
//            ADD_ACTIVE_BOX(BOXHOMERESET);
//            ADD_ACTIVE_BOX(BOXGCSNAV);
//            ADD_ACTIVE_BOX(BOXPLANWPMISSION);
//#ifdef USE_MULTI_MISSION
//            ADD_ACTIVE_BOX(BOXCHANGEMISSION);
//#endif
//        }
//
//        if (STATE(AIRPLANE)) {
//            ADD_ACTIVE_BOX(BOXNAVCRUISE);
//            ADD_ACTIVE_BOX(BOXNAVCOURSEHOLD);
//            ADD_ACTIVE_BOX(BOXSOARING);
//        }
//    }
//
//#ifdef USE_MR_BRAKING_MODE
//    if (mixerConfig()->platformType == PLATFORM_MULTIROTOR) {
//        ADD_ACTIVE_BOX(BOXBRAKING);
//    }
//#endif
//#endif  // GPS
    if (STATE(ALTITUDE_CONTROL) && navReadyAltControl) {
        ADD_ACTIVE_BOX(BOXNAVALTHOLD);
    }

//    if (STATE(AIRPLANE) || STATE(ROVER) || STATE(BOAT)) {
//        ADD_ACTIVE_BOX(BOXMANUAL);
//    }
//
//    if (STATE(AIRPLANE)) {
//        if (!feature(FEATURE_FW_LAUNCH)) {
//           ADD_ACTIVE_BOX(BOXNAVLAUNCH);
//        }
//
//        if (!feature(FEATURE_FW_AUTOTRIM)) {
//            ADD_ACTIVE_BOX(BOXAUTOTRIM);
//        }

#if defined(USE_AUTOTUNE_FIXED_WING)
        ADD_ACTIVE_BOX(BOXAUTOTUNE);
#endif
//        if (sensors(SENSOR_BARO)) {
//            ADD_ACTIVE_BOX(BOXAUTOLEVEL);
//        }
//    }

    /*
     * FLAPERON mode active only in case of airplane and custom airplane. Activating on
     * flying wing can cause bad thing
     */
//    if (STATE(FLAPERON_AVAILABLE)) {
//        ADD_ACTIVE_BOX(BOXFLAPERON);
//    }

//    ADD_ACTIVE_BOX(BOXBEEPERON);

#ifdef USE_LIGHTS
    ADD_ACTIVE_BOX(BOXLIGHTS);
#endif

#ifdef USE_LED_STRIP
    if (featureIsEnabled(FEATURE_LED_STRIP)) {
        ADD_ACTIVE_BOX(BOXLEDLOW);
    }
#endif

    ADD_ACTIVE_BOX(BOXOSD);

#ifdef USE_TELEMETRY
    if (featureIsEnabled(FEATURE_TELEMETRY) && telemetryConfig.telemetry_switch) {
        ADD_ACTIVE_BOX(BOXTELEMETRY);
    }
#endif

#ifdef USE_BLACKBOX
    if (feature(FEATURE_BLACKBOX)) {
        ADD_ACTIVE_BOX(BOXBLACKBOX);
    }
#endif

    ADD_ACTIVE_BOX(BOXKILLSWITCH);
    ADD_ACTIVE_BOX(BOXFAILSAFE);

#ifdef USE_RCDEVICE
    ADD_ACTIVE_BOX(BOXCAMERA1);
    ADD_ACTIVE_BOX(BOXCAMERA2);
    ADD_ACTIVE_BOX(BOXCAMERA3);
#endif

#ifdef USE_PINIOBOX
    // USER modes are only used for PINIO at the moment
    ADD_ACTIVE_BOX(BOXUSER1);
    ADD_ACTIVE_BOX(BOXUSER2);
    ADD_ACTIVE_BOX(BOXUSER3);
    ADD_ACTIVE_BOX(BOXUSER4);
#endif

#if defined(USE_OSD) && defined(OSD_LAYOUT_COUNT)
#if OSD_LAYOUT_COUNT > 0
    ADD_ACTIVE_BOX(BOXOSDALT1);
#if OSD_LAYOUT_COUNT > 1
    ADD_ACTIVE_BOX(BOXOSDALT2);
#if OSD_LAYOUT_COUNT > 2
    ADD_ACTIVE_BOX(BOXOSDALT3);
#endif
#endif
#endif
#endif

#if defined(USE_RX_MSP) && defined(USE_MSP_RC_OVERRIDE)
    ADD_ACTIVE_BOX(BOXMSPRCOVERRIDE);
#endif

#ifdef USE_DSHOT
//    if(STATE(MULTIROTOR) && isMotorProtocolDshot()) {
//        ADD_ACTIVE_BOX(BOXTURTLE);
//    }
#endif
}

#define IS_ENABLED(mask) ((mask) == 0 ? 0 : 1)
#define CHECK_ACTIVE_BOX(condition, index)    do { if (IS_ENABLED(condition)) { activeBoxes[index] = 1; } } while(0)

void packBoxModeFlags(boxBitmask_t * mspBoxModeFlags)
{
    uint8_t activeBoxes[CHECKBOX_ITEM_COUNT];
    ZERO_FARRAY(activeBoxes);

    // Serialize the flags in the order we delivered them, ignoring BOXNAMES and BOXINDEXES
    // Requires new Multiwii protocol version to fix
    // It would be preferable to setting the enabled bits based on BOXINDEX.
    CHECK_ACTIVE_BOX(IS_ENABLED(FLIGHT_MODE(ANGLE_MODE)),               BOXANGLE);
    CHECK_ACTIVE_BOX(IS_ENABLED(FLIGHT_MODE(HORIZON_MODE)),             BOXHORIZON);
    CHECK_ACTIVE_BOX(IS_ENABLED(FLIGHT_MODE(HEADING_MODE)),             BOXHEADINGHOLD);
    CHECK_ACTIVE_BOX(IS_ENABLED(FLIGHT_MODE(HEADFREE_MODE)),            BOXHEADFREE);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXHEADADJ)),         BOXHEADADJ);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXCAMSTAB)),         BOXCAMSTAB);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXFPVANGLEMIX)),     BOXFPVANGLEMIX);
    CHECK_ACTIVE_BOX(IS_ENABLED(FLIGHT_MODE(MANUAL_MODE)),              BOXMANUAL);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXBEEPERON)),        BOXBEEPERON);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXLEDLOW)),          BOXLEDLOW);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXLIGHTS)),          BOXLIGHTS);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXOSD)),             BOXOSD);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXTELEMETRY)),       BOXTELEMETRY);
    CHECK_ACTIVE_BOX(IS_ENABLED(ARMING_FLAG(ARMED)),                    BOXARM);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXBLACKBOX)),        BOXBLACKBOX);
    CHECK_ACTIVE_BOX(IS_ENABLED(FLIGHT_MODE(FAILSAFE_MODE)),            BOXFAILSAFE);
    CHECK_ACTIVE_BOX(IS_ENABLED(FLIGHT_MODE(NAV_ALTHOLD_MODE)),         BOXNAVALTHOLD);
    CHECK_ACTIVE_BOX(IS_ENABLED(FLIGHT_MODE(NAV_POSHOLD_MODE)),         BOXNAVPOSHOLD);
    CHECK_ACTIVE_BOX(IS_ENABLED(FLIGHT_MODE(NAV_COURSE_HOLD_MODE)),     BOXNAVCOURSEHOLD);
    CHECK_ACTIVE_BOX(IS_ENABLED(FLIGHT_MODE(NAV_COURSE_HOLD_MODE)) && IS_ENABLED(FLIGHT_MODE(NAV_ALTHOLD_MODE)), BOXNAVCRUISE);
    CHECK_ACTIVE_BOX(IS_ENABLED(FLIGHT_MODE(NAV_RTH_MODE)),             BOXNAVRTH);
    CHECK_ACTIVE_BOX(IS_ENABLED(FLIGHT_MODE(NAV_WP_MODE)),              BOXNAVWP);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXAIRMODE)),         BOXAIRMODE);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXGCSNAV)),          BOXGCSNAV);
    CHECK_ACTIVE_BOX(IS_ENABLED(FLIGHT_MODE(FLAPERON)),                 BOXFLAPERON);
    CHECK_ACTIVE_BOX(IS_ENABLED(FLIGHT_MODE(TURN_ASSISTANT)),           BOXTURNASSIST);
    CHECK_ACTIVE_BOX(IS_ENABLED(FLIGHT_MODE(NAV_LAUNCH_MODE)),          BOXNAVLAUNCH);
    CHECK_ACTIVE_BOX(IS_ENABLED(FLIGHT_MODE(AUTO_TUNE)),                BOXAUTOTUNE);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXAUTOTRIM)),        BOXAUTOTRIM);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXKILLSWITCH)),      BOXKILLSWITCH);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXHOMERESET)),       BOXHOMERESET);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXCAMERA1)),         BOXCAMERA1);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXCAMERA2)),         BOXCAMERA2);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXCAMERA3)),         BOXCAMERA3);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXOSDALT1)),         BOXOSDALT1);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXOSDALT2)),         BOXOSDALT2);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXOSDALT3)),         BOXOSDALT3);
    CHECK_ACTIVE_BOX(IS_ENABLED(navigationTerrainFollowingEnabled()),   BOXSURFACE);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXBRAKING)),         BOXBRAKING);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXUSER1)),           BOXUSER1);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXUSER2)),           BOXUSER2);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXUSER3)),           BOXUSER3);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXUSER4)),           BOXUSER4);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXLOITERDIRCHN)),    BOXLOITERDIRCHN);
#if defined(USE_RX_MSP) && defined(USE_MSP_RC_OVERRIDE)
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXMSPRCOVERRIDE)),   BOXMSPRCOVERRIDE);
#endif
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXAUTOLEVEL)),       BOXAUTOLEVEL);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXPLANWPMISSION)),   BOXPLANWPMISSION);
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXSOARING)),         BOXSOARING);
#ifdef USE_MULTI_MISSION
    CHECK_ACTIVE_BOX(IS_ENABLED(IS_RC_MODE_ACTIVE(BOXCHANGEMISSION)),   BOXCHANGEMISSION);
#endif

    memset(mspBoxModeFlags, 0, sizeof(boxBitmask_t));
    for (uint32_t i = 0; i < activeBoxIdCount; i++) {
        if (activeBoxes[activeBoxIds[i]]) {
            bitArraySet(mspBoxModeFlags->bits, i);
        }
    }
}

uint16_t packSensorStatus(void)
{
    // Sensor bits
    uint16_t sensorStatus =
            IS_ENABLED(sensors(SENSOR_GYRO))        << 0 |
            IS_ENABLED(sensors(SENSOR_ACC))         << 1 |
            IS_ENABLED(sensors(SENSOR_BARO))        << 2 |
            IS_ENABLED(sensors(SENSOR_MAG))         << 3 |
            IS_ENABLED(sensors(SENSOR_SONAR))       << 4 |
            IS_ENABLED(sensors(SENSOR_RANGEFINDER)) << 4 |
            IS_ENABLED(sensors(SENSOR_GPS))         << 5 |
            IS_ENABLED(sensors(SENSOR_GPSMAG))      << 6 |
            IS_ENABLED(sensors(SENSOR_OPTICALFLOW)) << 7;

    // Hardware failure indication bit
//    if (!isHardwareHealthy()) {
//        sensorStatus |= 1 << 15;        // Bit 15 of sensor bit field indicates hardware failure
//    }

    return sensorStatus;
}
