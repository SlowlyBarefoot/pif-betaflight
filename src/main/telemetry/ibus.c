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

/*
 * FlySky iBus telemetry implementation by CraigJPerry.
 * Unit tests and some additions by Unitware
 *
 * Many thanks to Dave Borthwick's iBus telemetry dongle converter for
 * PIC 12F1572 (also distributed under GPLv3) which was referenced to
 * clarify the protocol.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(USE_TELEMETRY_IBUS)

#include "scheduler/scheduler.h"

#include "telemetry/ibus.h"
#include "telemetry/ibus_shared.h"
#include "telemetry/telemetry.h"

#include "rc/pif_rc_ibus.h"


#define IBUS_TASK_PERIOD_US (1000)

#define IBUS_UART_MODE     (MODE_RXTX)
#define IBUS_BAUDRATE      (115200)


static serialPort_t *ibusSerialPort = NULL;
static const serialPortConfig_t *ibusSerialPortConfig;

static bool ibusTelemetryEnabled = false;
static portSharing_e ibusPortSharing;

static PifRcIbus s_ibus;


void initIbusTelemetry(void)
{
    ibusSerialPortConfig = findSerialPortConfig(FUNCTION_TELEMETRY_IBUS);
    ibusPortSharing = determinePortSharing(ibusSerialPortConfig, FUNCTION_TELEMETRY_IBUS);
    ibusTelemetryEnabled = false;
}


bool checkIbusTelemetryState(void)
{
    bool newTelemetryEnabledValue = telemetryDetermineEnabledState(ibusPortSharing);

    if (newTelemetryEnabledValue == ibusTelemetryEnabled) {
        return false;
    }

    if (newTelemetryEnabledValue) {
        rescheduleTask(TASK_TELEMETRY, IBUS_TASK_PERIOD_US);
        return configureIbusTelemetryPort();
    } else {
        freeIbusTelemetryPort();
    }

    return true;
}


bool configureIbusTelemetryPort(void)
{
    if (!ibusSerialPortConfig) {
        return false;
    }

    if (isSerialPortShared(ibusSerialPortConfig, FUNCTION_RX_SERIAL, FUNCTION_TELEMETRY_IBUS)) {
        // serialRx will open port and handle telemetry
        return false;
    }

    ibusSerialPort = openSerialPort(ibusSerialPortConfig->identifier, FUNCTION_TELEMETRY_IBUS, NULL, NULL, IBUS_BAUDRATE, IBUS_UART_MODE, SERIAL_BIDIR | (telemetryConfig()->telemetry_inverted ? SERIAL_INVERTED : SERIAL_NOT_INVERTED) | SERIAL_PIF);

    if (!ibusSerialPort) {
        return false;
    }

    if (!pifRcIbus_Init(&s_ibus, PIF_ID_AUTO)) return false;
    s_ibus.evt_telemetry = respondToIbusRequest;
    pifRcIbus_AttachUart(&s_ibus, &ibusSerialPort->uart);

    initSharedIbusTelemetry(ibusSerialPort);
    ibusTelemetryEnabled = true;
    return true;
}


void freeIbusTelemetryPort(void)
{
    pifRcIbus_Clear(&s_ibus);
    closeSerialPort(ibusSerialPort);
    ibusSerialPort = NULL;
    ibusTelemetryEnabled = false;
}

#endif
