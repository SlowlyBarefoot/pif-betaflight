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

#include "platform.h"

#ifdef USE_DSHOT

#include "common/time.h"
#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/motor.h"
#include "drivers/time.h"
#include "drivers/timer.h"

#include "drivers/dshot.h"
#include "drivers/dshot_dpwm.h"
#include "drivers/pwm_output.h"

#include "dshot_command.h"

#define DSHOT_PROTOCOL_DETECTION_DELAY_MS 3000

// Longest a blocking command may take: the 10 ms before it, 10 repeats 1 ms
// apart and the 100 ms after a beacon come to about 120 ms, so this is only
// hit when the motor driver never runs the queue.
#define DSHOT_BLOCKING_TIMEOUT_MS 1000

// The queue, its timing and the frames of a command are PIF's (pif_dshot, in
// dshotPif). What is left here is when Betaflight allows a command at all,
// and running a blocking one while nothing else updates the motors.

bool dshotStreamingCommandsAreEnabled(void)
{
    return motorIsEnabled() && motorGetMotorEnableTimeMs() && millis() > motorGetMotorEnableTimeMs() + DSHOT_PROTOCOL_DETECTION_DELAY_MS;
}

static bool dshotCommandsAreEnabled(dshotCommandType_e commandType)
{
    bool ret = false;

    switch (commandType) {
    case DSHOT_CMD_TYPE_BLOCKING:
        ret = !motorIsEnabled();

        break;
    case DSHOT_CMD_TYPE_INLINE:
        ret = dshotStreamingCommandsAreEnabled();

        break;
    default:

        break;
    }

    return ret;
}

void dshotCommandWrite(uint8_t index, uint8_t motorCount, uint8_t command, dshotCommandType_e commandType)
{
    UNUSED(motorCount); // dshotPif knows how many motors there are

    if (!isMotorProtocolDshot() || !dshotCommandsAreEnabled(commandType) || (command > DSHOT_MAX_COMMAND)) {
        return;
    }

    if (commandType == DSHOT_CMD_TYPE_BLOCKING) {
        // The motors are disabled, so the PID loop does not update them and
        // the queue is run from here instead, one motor update per PID loop
        // time, which is what its delays are counted in. Motors the command
        // is not for are sent DSHOT_CMD_MOTOR_STOP, and the command waits for
        // every motor to be at 0 before it goes.
        pifDshot_SetThrottle(&dshotPif, PIF_DSHOT_ALL_MOTORS, DSHOT_CMD_MOTOR_STOP);
        if (!pifDshot_Command(&dshotPif, index, command)) {
            return;
        }

        const timeMs_t timeoutMs = millis() + DSHOT_BLOCKING_TIMEOUT_MS;
        while (pifDshot_IsCommandBusy(&dshotPif) && cmp32(timeoutMs, millis()) > 0) {
            delayMicroseconds(dshotPif._cycle_us);
#ifdef USE_DSHOT_TELEMETRY
            timeUs_t timeoutUs = micros() + 1000;
            while (!motorGetVTable().updateStart() &&
                   cmpTimeUs(timeoutUs, micros()) > 0);
#endif
            motorGetVTable().updateComplete();
        }
    } else if (commandType == DSHOT_CMD_TYPE_INLINE) {
        pifDshot_Command(&dshotPif, index, command);
    }
}
#endif // USE_DSHOT
