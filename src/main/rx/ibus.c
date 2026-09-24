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

#include "drivers/serial_pif.h"

#include "rc/pif_rc_ibus.h"

#define IBUS_BAUDRATE 115200

// The first 14 channels are 12 bit; the upper nibble of each carries a third of
// one of the 4 channels that follow, which pif_rc_ibus has already put
// together.
#define IBUS_CHANNEL_MASK 0x0FFF

// Framing, checksums and the telemetry replies of a shared port are done by
// pif_rc_ibus, from the PifUart RX task of ibusSerialPif. ibusDataReceive()
// only takes the channels of a servo frame.
static serialPif_t ibusSerialPif;
static PifRcIbus ibusRc;

static bool ibusFrameDone = false;
static uint32_t ibusChannelData[PIF_IBUS_EXP_CHANNEL_COUNT];

static timeUs_t lastFrameTimeUs = 0;

static void ibusDataReceive(PifRc *rc, uint16_t *channel, PifIssuerP issuer)
{
    UNUSED(rc);
    UNUSED(issuer);

    int i;
    for (i = 0; i < PIF_IBUS_CHANNEL_COUNT; i++) {
        ibusChannelData[i] = channel[i] & IBUS_CHANNEL_MASK;
    }
    for (; i < PIF_IBUS_EXP_CHANNEL_COUNT; i++) {
        ibusChannelData[i] = channel[i];
    }

    // The frame ended with the last byte the ISR saw, not when the RX task got
    // round to it.
    lastFrameTimeUs = serialPifLastRxTimeUs(&ibusSerialPif);
    ibusFrameDone = true;
}

static uint8_t ibusFrameStatus(rxRuntimeState_t *rxRuntimeState)
{
    if (!ibusFrameDone) {
        return RX_FRAME_PENDING;
    }

    ibusFrameDone = false;
    rxRuntimeState->lastRcFrameTimeUs = lastFrameTimeUs;

    return RX_FRAME_COMPLETE;
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

    if (!pifRcIbus_Init(&ibusRc, PIF_ID_AUTO)) {
        return false;
    }
    pifRc_AttachEvtReceive(&ibusRc.parent, ibusDataReceive, NULL);

    serialPort_t *ibusPort = serialPifOpen(&ibusSerialPif, portConfig->identifier,
        FUNCTION_RX_SERIAL,
        IBUS_BAUDRATE,
        portShared ? MODE_RXTX : MODE_RX,
        (rxConfig->serialrx_inverted ? SERIAL_INVERTED : 0) | (rxConfig->halfDuplex || portShared ? SERIAL_BIDIR : 0)
        );
    if (!ibusPort) {
        return false;
    }

#if defined(USE_TELEMETRY) && defined(USE_TELEMETRY_IBUS)
    if (portShared) {
        initSharedIbusTelemetry();
        ibusRc.evt_telemetry = respondToIbusRequest;
    }
#endif

    // Last, since this is what starts the PifUart tasks.
    pifRcIbus_AttachUart(&ibusRc, &ibusSerialPif.uart);

    return true;
}

#endif //SERIAL_RX
