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
 * Driver for IBUS (Flysky) receiver
 *   - initial implementation for MultiWii by Cesco/Pl¸schi
 *   - implementation for BaseFlight by Andreas (fiendie) Tacke
 *   - ported to CleanFlight by Konstantin (digitalentity) Sharlaimov
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#ifdef USE_SERIALRX_IBUS

#include "pg/rx.h"

#include "common/utils.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/time.h"

#include "io/serial.h"

#ifdef USE_TELEMETRY
#include "telemetry/telemetry.h"
#endif

#include "rx/rx.h"
#include "rx/ibus.h"
#include "telemetry/ibus.h"
#include "telemetry/ibus_shared.h"

#include "rc/pif_rc_ibus.h"

#define IBUS_BAUDRATE 115200

static bool ibusFrameDone = false;
static uint32_t ibusChannelData[PIF_IBUS_EXP_CHANNEL_COUNT];

static timeUs_t lastFrameTimeUs = 0;

static PifRcIbus s_ibus;


static void _evtIbusReceive(PifRc* p_owner, uint16_t* channel, PifIssuerP p_issuer)
{
    (void)p_owner;
    (void)p_issuer;

	for (int i = 0; i < PIF_IBUS_EXP_CHANNEL_COUNT; i++) {
		ibusChannelData[i] = channel[i];
	}
    lastFrameTimeUs = microsISR();
    ibusFrameDone = true;
}

static uint8_t ibusFrameStatus(rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxRuntimeState);

    uint8_t frameStatus = RX_FRAME_PENDING;

    if (!ibusFrameDone) {
        return frameStatus;
    }

    ibusFrameDone = false;

    if (s_ibus._model == IBUS_MODEL_IA6 || s_ibus._length == IBUS_FRAME_SIZE) {
        frameStatus = RX_FRAME_COMPLETE;
        rxRuntimeState->lastRcFrameTimeUs = lastFrameTimeUs;
    }

    return frameStatus;
}


static float ibusReadRawRC(const rxRuntimeState_t *rxRuntimeState, uint8_t chan)
{
    UNUSED(rxRuntimeState);
    return ibusChannelData[chan];
}

bool ibusInit(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxConfig);

    rxRuntimeState->channelCount = PIF_IBUS_EXP_CHANNEL_COUNT;
    rxRuntimeState->rxRefreshRate = 20000; // TODO - Verify speed

    rxRuntimeState->rcReadRawFn = ibusReadRawRC;
    rxRuntimeState->rcFrameStatusFn = ibusFrameStatus;
    rxRuntimeState->rcFrameTimeUsFn = rxFrameTimeUs;

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        return false;
    }

#ifdef USE_TELEMETRY
    bool portShared = isSerialPortShared(portConfig, FUNCTION_RX_SERIAL, FUNCTION_TELEMETRY_IBUS);
#else
    bool portShared = false;
#endif


    serialPort_t *ibusPort = openSerialPort(portConfig->identifier,
        FUNCTION_RX_SERIAL,
        NULL,
        NULL,
        IBUS_BAUDRATE,
        portShared ? MODE_RXTX : MODE_RX,
        (rxConfig->serialrx_inverted ? SERIAL_INVERTED : 0) | (rxConfig->halfDuplex || portShared ? SERIAL_BIDIR : 0) | SERIAL_PIF
        );

#if defined(USE_TELEMETRY) && defined(USE_TELEMETRY_IBUS)
    if (portShared) {
        initSharedIbusTelemetry(ibusPort);
    }
#endif

    if (!pifRcIbus_Init(&s_ibus, PIF_ID_AUTO)) return FALSE;
    pifRc_AttachEvtReceive(&s_ibus.parent, _evtIbusReceive, NULL);
#if defined(USE_TELEMETRY) && defined(USE_TELEMETRY_IBUS)
    s_ibus.evt_telemetry = respondToIbusRequest;
#endif    
    pifRcIbus_AttachUart(&s_ibus, &ibusPort->uart);

    return ibusPort != NULL;
}

#endif //SERIAL_RX
