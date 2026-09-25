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

#include "platform.h"

#if defined(USE_TELEMETRY_IBUS)

#include "common/utils.h"

#include "drivers/serial.h"
#include "drivers/time.h"

#include "io/serial.h"

#include "scheduler/scheduler.h"

#include "telemetry/ibus.h"
#include "telemetry/ibus_shared.h"
#include "telemetry/telemetry.h"

#include "communication/pif_uart.h"
#include "rc/pif_rc_ibus.h"


#define IBUS_TASK_PERIOD_US (1000)

#define IBUS_UART_MODE     (MODE_RXTX)
#define IBUS_BAUDRATE      (115200)

// How long after the echo of a reply is due its bytes are still taken as the
// echo. The receiver leaves several ms between requests, so what is read later
// than this is a request, which is what a transceiver that does not listen to
// itself looks like.
#define IBUS_ECHO_WINDOW_US (2 * IBUS_TASK_PERIOD_US)


// The requests are read from the port in handleIbusTelemetry() and parsed by
// pif_rc_ibus, which then answers them through ibusUart. That PifUart has no
// buffers and no tasks: pifUart_SendTxData() calls act_send_data, which writes
// to the port.
static PifRcIbus ibusRc;
static PifUart ibusUart;

static serialPort_t *ibusSerialPort = NULL;
static const serialPortConfig_t *ibusSerialPortConfig;

/* The sent bytes will be echoed back since Tx and Rx are wired together, this counter
 * will keep track of how many rx chars that shall be discarded */
static uint8_t outboundBytesToIgnoreOnRxCount = 0;
static timeUs_t echoEndUs;

static bool ibusTelemetryEnabled = false;
static portSharing_e ibusPortSharing;


// act_send_data of ibusUart, called from pifRcIbus_SendTelemetry().
static uint16_t ibusUartSendData(PifUart *uart, uint8_t *data, uint16_t size)
{
    echoEndUs = micros() + (timeUs_t)size * uart->_transfer_time + IBUS_ECHO_WINDOW_US;
    outboundBytesToIgnoreOnRxCount += size;

    serialWriteBuf(ibusSerialPort, data, size);
    return size;
}


void initIbusTelemetry(void)
{
    ibusSerialPortConfig = findSerialPortConfig(FUNCTION_TELEMETRY_IBUS);
    ibusPortSharing = determinePortSharing(ibusSerialPortConfig, FUNCTION_TELEMETRY_IBUS);
    ibusTelemetryEnabled = false;
}


void handleIbusTelemetry(void)
{
    if (!ibusTelemetryEnabled) {
        return;
    }

    while (serialRxBytesWaiting(ibusSerialPort) > 0) {
        uint8_t c = serialRead(ibusSerialPort);

        if (outboundBytesToIgnoreOnRxCount) {
            if (cmpTimeUs(micros(), echoEndUs) <= 0) {
                outboundBytesToIgnoreOnRxCount--;
                continue;
            }
            outboundBytesToIgnoreOnRxCount = 0;
        }

        if (pifRcIbus_ParsingPacket(&ibusRc, c) == IBUS_FRAME_TELEMETRY) {
            pifRcIbus_SendTelemetry(&ibusRc, ibusRc._tlm_command, ibusRc._tlm_address);
        }
    }
}


bool checkIbusTelemetryState(void)
{
    bool newTelemetryEnabledValue = telemetryDetermineEnabledState(ibusPortSharing);

    if (newTelemetryEnabledValue == ibusTelemetryEnabled) {
        return false;
    }

    if (newTelemetryEnabledValue) {
        rescheduleTask(TASK_TELEMETRY, IBUS_TASK_PERIOD_US);
        configureIbusTelemetryPort();
    } else {
        freeIbusTelemetryPort();
    }

    return true;
}


void configureIbusTelemetryPort(void)
{
    if (!ibusSerialPortConfig) {
        return;
    }

    if (isSerialPortShared(ibusSerialPortConfig, FUNCTION_RX_SERIAL, FUNCTION_TELEMETRY_IBUS)) {
        // serialRx will open port and handle telemetry
        return;
    }

    if (!pifRcIbus_Init(&ibusRc, PIF_ID_AUTO) || !pifUart_Init(&ibusUart, PIF_ID_AUTO, IBUS_BAUDRATE)) {
        return;
    }
    ibusUart.act_send_data = ibusUartSendData;
    pifRcIbus_AttachUart(&ibusRc, &ibusUart);
    ibusRc.evt_telemetry = respondToIbusRequest;

    ibusSerialPort = openSerialPort(ibusSerialPortConfig->identifier, FUNCTION_TELEMETRY_IBUS, NULL, NULL, IBUS_BAUDRATE, IBUS_UART_MODE, SERIAL_BIDIR | (telemetryConfig()->telemetry_inverted ? SERIAL_INVERTED : SERIAL_NOT_INVERTED));

    if (!ibusSerialPort) {
        return;
    }

    initSharedIbusTelemetry();
    outboundBytesToIgnoreOnRxCount = 0;
    ibusTelemetryEnabled = true;
}


void freeIbusTelemetryPort(void)
{
    if (ibusSerialPort) {
        closeSerialPort(ibusSerialPort);
    }
    pifRcIbus_Clear(&ibusRc);
    ibusSerialPort = NULL;
    ibusTelemetryEnabled = false;
}

#endif
