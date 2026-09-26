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
 *
 * Author: jflyper
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#ifdef USE_DSHOT

#include "build/debug.h"

#include "common/maths.h"
#include "common/time.h"

#include "config/feature.h"

#include "drivers/motor.h"
#include "drivers/timer.h"

#include "drivers/dshot_dpwm.h" // for motorDmaOutput_t, should be gone
#include "drivers/dshot_command.h"
#include "drivers/time.h"
#include "drivers/pwm_output.h" // for PWM_TYPE_* and others

#include "fc/rc_controls.h" // for flight3DConfig_t

#include "rx/rx.h"

#include "dshot.h"

void dshotInitEndpoints(const motorConfig_t *motorConfig, float outputLimit, float *outputLow, float *outputHigh, float *disarm, float *deadbandMotor3dHigh, float *deadbandMotor3dLow) {
    float outputLimitOffset = DSHOT_RANGE * (1 - outputLimit);
    *disarm = DSHOT_CMD_MOTOR_STOP;
    if (featureIsEnabled(FEATURE_3D)) {
        *outputLow = DSHOT_MIN_THROTTLE + getDigitalIdleOffset(motorConfig) * (DSHOT_3D_FORWARD_MIN_THROTTLE - 1 - DSHOT_MIN_THROTTLE);
        *outputHigh = DSHOT_MAX_THROTTLE - outputLimitOffset / 2;
        *deadbandMotor3dHigh = DSHOT_3D_FORWARD_MIN_THROTTLE + getDigitalIdleOffset(motorConfig) * (DSHOT_MAX_THROTTLE - DSHOT_3D_FORWARD_MIN_THROTTLE);
        *deadbandMotor3dLow = DSHOT_3D_FORWARD_MIN_THROTTLE - 1 - outputLimitOffset / 2;
    } else {
        *outputLow = DSHOT_MIN_THROTTLE + getDigitalIdleOffset(motorConfig) * DSHOT_RANGE;
        *outputHigh = DSHOT_MAX_THROTTLE - outputLimitOffset;
    }
}

float dshotConvertFromExternal(uint16_t externalValue)
{
    float motorValue;

    externalValue = constrain(externalValue, PWM_RANGE_MIN, PWM_RANGE_MAX);

    if (featureIsEnabled(FEATURE_3D)) {
        if (externalValue == PWM_RANGE_MIDDLE) {
            motorValue = DSHOT_CMD_MOTOR_STOP;
        } else if (externalValue < PWM_RANGE_MIDDLE) {
            motorValue = scaleRangef(externalValue, PWM_RANGE_MIN, PWM_RANGE_MIDDLE - 1, DSHOT_3D_FORWARD_MIN_THROTTLE - 1, DSHOT_MIN_THROTTLE);
        } else {
            motorValue = scaleRangef(externalValue, PWM_RANGE_MIDDLE + 1, PWM_RANGE_MAX, DSHOT_3D_FORWARD_MIN_THROTTLE, DSHOT_MAX_THROTTLE);
        }
    } else {
        motorValue = (externalValue == PWM_RANGE_MIN) ? DSHOT_CMD_MOTOR_STOP : scaleRangef(externalValue, PWM_RANGE_MIN + 1, PWM_RANGE_MAX, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE);
    }

    return motorValue;
}

uint16_t dshotConvertToExternal(float motorValue)
{
    float externalValue;

    if (featureIsEnabled(FEATURE_3D)) {
        if (motorValue == DSHOT_CMD_MOTOR_STOP || motorValue < DSHOT_MIN_THROTTLE) {
            externalValue = PWM_RANGE_MIDDLE;
        } else if (motorValue <= DSHOT_3D_FORWARD_MIN_THROTTLE - 1) {
            externalValue = scaleRangef(motorValue, DSHOT_MIN_THROTTLE, DSHOT_3D_FORWARD_MIN_THROTTLE - 1, PWM_RANGE_MIDDLE - 1, PWM_RANGE_MIN);
        } else {
            externalValue = scaleRangef(motorValue, DSHOT_3D_FORWARD_MIN_THROTTLE, DSHOT_MAX_THROTTLE, PWM_RANGE_MIDDLE + 1, PWM_RANGE_MAX);
        }
    } else {
        externalValue = (motorValue < DSHOT_MIN_THROTTLE) ? PWM_RANGE_MIN : scaleRangef(motorValue, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE, PWM_RANGE_MIN + 1, PWM_RANGE_MAX);
    }

    return lrintf(externalValue);
}

FAST_DATA_ZERO_INIT PifDshot dshotPif;

STATIC_ASSERT(MAX_SUPPORTED_MOTORS <= PIF_DSHOT_MAX_MOTORS, pifDshotMotorCount);
STATIC_ASSERT(ALL_MOTORS == PIF_DSHOT_ALL_MOTORS, pifDshotAllMotors);

// Default to an 8 kHz (125 us) loop until the PID loop is initialised, so
// that a command queued before it still gets sensible delays.
static uint32_t dshotPidLoopTimeUs = 125;

void dshotSetPidLoopTime(uint32_t pidLoopTime)
{
    // readEEPROM() runs pidInit() through activateConfig() before the gyro
    // is initialised, with a loop time of 0. Kept out, or dshotPifInit()
    // would refuse it and leave the board without motors.
    if (!pidLoopTime) {
        return;
    }

    dshotPidLoopTimeUs = pidLoopTime;
    pifDshot_SetCyclePeriod(&dshotPif, pidLoopTime);
}

bool dshotPifInit(uint8_t motorCount, bool bidirectional, PifActDshotWrite actWrite)
{
    return pifDshot_Init(&dshotPif, PIF_ID_AUTO, motorCount, bidirectional, dshotPidLoopTimeUs, actWrite);
}

void dshotRequestTelemetry(uint8_t motorIndex)
{
    if (motorIndex < dshotPif._motor_count) {
        dshotPif._motor[motorIndex].request_telemetry = true;
    }
}

#ifdef USE_DSHOT_TELEMETRY
FAST_DATA_ZERO_INIT dshotTelemetryState_t dshotTelemetryState;

uint16_t getDshotTelemetry(uint8_t index)
{
    return dshotTelemetryState.motorState[index].telemetryValue;
}

bool dshotTelemetryReceive(uint8_t motorIndex, uint32_t gcr)
{
    if (gcr == PIF_DSHOT_GCR_NONE) {
        // An ESC too busy to answer is not an error in the answer.
        return false;
    }

    dshotTelemetryState.readCount++;

    const bool valid = pifDshot_PutGcr(&dshotPif, motorIndex, gcr);
    if (valid) {
        // Kept in units of 100 eRPM, which is what the RPM filter, the OSD
        // and MSP read.
        const uint16_t value = (dshotPif._motor[motorIndex]._erpm + 50) / 100;
        dshotTelemetryState.motorState[motorIndex].telemetryValue = value;
        dshotTelemetryState.motorState[motorIndex].telemetryActive = true;
        if (motorIndex < 4) {
            DEBUG_SET(DEBUG_DSHOT_RPM_TELEMETRY, motorIndex, value);
        }
    } else {
        dshotTelemetryState.invalidPacketCount++;
    }

#ifdef USE_DSHOT_TELEMETRY_STATS
    updateDshotTelemetryQuality(&dshotTelemetryQuality[motorIndex], valid, millis());
#endif

    return valid;
}

#endif

#ifdef USE_DSHOT_TELEMETRY_STATS
FAST_DATA_ZERO_INIT dshotTelemetryQuality_t dshotTelemetryQuality[MAX_SUPPORTED_MOTORS];

void updateDshotTelemetryQuality(dshotTelemetryQuality_t *qualityStats, bool packetValid, timeMs_t currentTimeMs)
{
    uint8_t statsBucketIndex = (currentTimeMs / DSHOT_TELEMETRY_QUALITY_BUCKET_MS) % DSHOT_TELEMETRY_QUALITY_BUCKET_COUNT;
    if (statsBucketIndex != qualityStats->lastBucketIndex) {
        qualityStats->packetCountSum -= qualityStats->packetCountArray[statsBucketIndex];
        qualityStats->invalidCountSum -= qualityStats->invalidCountArray[statsBucketIndex];
        qualityStats->packetCountArray[statsBucketIndex] = 0;
        qualityStats->invalidCountArray[statsBucketIndex] = 0;
        qualityStats->lastBucketIndex = statsBucketIndex;
    }
    qualityStats->packetCountSum++;
    qualityStats->packetCountArray[statsBucketIndex]++;
    if (!packetValid) {
        qualityStats->invalidCountSum++;
        qualityStats->invalidCountArray[statsBucketIndex]++;
    }
}
#endif // USE_DSHOT_TELEMETRY_STATS

#endif // USE_DSHOT

// temporarily here, needs to be moved during refactoring
void validateAndfixMotorOutputReordering(uint8_t *array, const unsigned size)
{
    bool invalid = false;

    for (unsigned i = 0; i < size; i++) {
        if (array[i] >= size) {
            invalid = true;
            break;
        }
    }

    int valuesAsIndexes[size];

    for (unsigned i = 0; i < size; i++) {
        valuesAsIndexes[i] = -1;
    }

    if (!invalid) {
        for (unsigned i = 0; i < size; i++) {
            if (-1 != valuesAsIndexes[array[i]]) {
                invalid = true;
                break;
            }

            valuesAsIndexes[array[i]] = array[i];
        }
    }

    if (invalid) {
        for (unsigned i = 0; i < size; i++) {
            array[i] = i;
        }
    }
}
