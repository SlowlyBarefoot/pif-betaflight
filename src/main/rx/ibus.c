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
#include <string.h>

#include "platform.h"

#ifdef USE_SERIALRX_IBUS

#include "pg/rx.h"

#include "common/utils.h"

#include "drivers/serial.h"
#include "drivers/time.h"

#include "io/serial.h"

#ifdef USE_TELEMETRY
#include "telemetry/telemetry.h"
#endif

#include "rx/rx.h"
#include "rx/ibus.h"
#include "telemetry/ibus.h"
#include "telemetry/ibus_shared.h"

#include "communication/pif_uart.h"
#include "rc/pif_rc_ibus.h"

#define IBUS_BAUDRATE 115200

// The first 14 channels are 12 bit; the upper nibble of each carries a third of
// one of the 4 channels that follow, which pif_rc_ibus has already put
// together.
#define IBUS_CHANNEL_MASK 0x0FFF

// How long after the last byte of a reply has gone out its echo may still
// arrive before the rest of it is given up on, so that a transceiver that does
// not listen to itself does not lose the next frame. Same as the frame gap of
// the native iBUS code.
#define IBUS_ECHO_GAP_US 500

// The bytes are parsed by pif_rc_ibus in the receive ISR, as the native code
// framed them there, so the frame time is that of its last byte. The channels
// of a servo frame are taken over by ibusFrameStatus(). A sensor request on a
// shared port is answered from ibusFrameStatus() too, so the sensors are not
// read and the port is not written from the ISR.
static PifRcIbus ibusRc;

static serialPort_t *ibusPort;

#if defined(USE_TELEMETRY) && defined(USE_TELEMETRY_IBUS)
// pif_rc_ibus sends the replies through a PifUart. It has no buffers and no
// tasks: pifUart_SendTxData() calls act_send_data, which writes to ibusPort.
static PifUart ibusUart;

static volatile bool ibusTelemetryPending = false;
static volatile uint8_t ibusTelemetryCommand;
static volatile uint8_t ibusTelemetryAddress;
#endif

// Echo of the last reply still to be dropped on a SERIAL_BIDIR port, and the
// time after which whatever is left of it is not coming.
static volatile uint8_t rxBytesToIgnore = 0;
static volatile timeUs_t echoEndUs;

static volatile bool ibusFrameDone = false;
static uint16_t ibusFrameChannels[PIF_IBUS_EXP_CHANNEL_COUNT];
static uint32_t ibusChannelData[PIF_IBUS_EXP_CHANNEL_COUNT];

static timeUs_t ibusFrameTimeUs = 0;
static timeUs_t lastFrameTimeUs = 0;

// evt_receive of ibusRc, called from the ISR through pifRcIbus_ParsingPacket().
static void ibusChannelReceive(PifRc *rc, uint16_t *channel, PifIssuerP issuer)
{
    UNUSED(rc);
    UNUSED(issuer);

    memcpy(ibusFrameChannels, channel, sizeof(ibusFrameChannels));
}

// Receive ISR callback
static void ibusDataReceive(uint16_t c, void *data)
{
    UNUSED(data);

    const timeUs_t now = microsISR();

    if (rxBytesToIgnore) {
        if (cmpTimeUs(now, echoEndUs) <= 0) {
            rxBytesToIgnore--;
            return;
        }
        rxBytesToIgnore = 0;
    }

    switch (pifRcIbus_ParsingPacket(&ibusRc, c)) {
    case IBUS_FRAME_SERVO:
        ibusFrameTimeUs = now;
        ibusFrameDone = true;
        break;

#if defined(USE_TELEMETRY) && defined(USE_TELEMETRY_IBUS)
    case IBUS_FRAME_TELEMETRY:
        ibusTelemetryCommand = ibusRc._tlm_command;
        ibusTelemetryAddress = ibusRc._tlm_address;
        ibusTelemetryPending = true;
        break;
#endif

    default:
        break;
    }
}

#if defined(USE_TELEMETRY) && defined(USE_TELEMETRY_IBUS)
// act_send_data of ibusUart, called from pifRcIbus_SendTelemetry().
static uint16_t ibusUartSendData(PifUart *uart, uint8_t *data, uint16_t size)
{
    // Set before the first byte goes out, so the ISR knows about its echo.
    // A reply is only sent once its request has been received in full, so
    // no echo of an earlier reply is still being counted down here.
    echoEndUs = micros() + (timeUs_t)size * uart->_transfer_time + IBUS_ECHO_GAP_US;
    rxBytesToIgnore = size;

    serialWriteBuf(ibusPort, data, size);
    return size;
}
#endif

static uint8_t ibusFrameStatus(rxRuntimeState_t *rxRuntimeState)
{
#if defined(USE_TELEMETRY) && defined(USE_TELEMETRY_IBUS)
    if (ibusTelemetryPending) {
        ibusTelemetryPending = false;
        pifRcIbus_SendTelemetry(&ibusRc, ibusTelemetryCommand, ibusTelemetryAddress);
    }
#endif

    if (!ibusFrameDone) {
        return RX_FRAME_PENDING;
    }

    ibusFrameDone = false;

    for (int i = 0; i < PIF_IBUS_EXP_CHANNEL_COUNT; i++) {
        ibusChannelData[i] = i < PIF_IBUS_CHANNEL_COUNT ? ibusFrameChannels[i] & IBUS_CHANNEL_MASK : ibusFrameChannels[i];
    }
    lastFrameTimeUs = ibusFrameTimeUs;
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

    // Before the port is opened, since the ISR parses into it from then on.
    if (!pifRcIbus_Init(&ibusRc, PIF_ID_AUTO)) {
        return false;
    }
    pifRc_AttachEvtReceive(&ibusRc.parent, ibusChannelReceive, NULL);

#if defined(USE_TELEMETRY) && defined(USE_TELEMETRY_IBUS)
    if (portShared) {
        if (!pifUart_Init(&ibusUart, PIF_ID_AUTO, IBUS_BAUDRATE)) {
            return false;
        }
        ibusUart.act_send_data = ibusUartSendData;
        pifRcIbus_AttachUart(&ibusRc, &ibusUart);

        initSharedIbusTelemetry();
        ibusRc.evt_telemetry = respondToIbusRequest;
    }
#endif

    rxBytesToIgnore = 0;
    ibusPort = openSerialPort(portConfig->identifier,
        FUNCTION_RX_SERIAL,
        ibusDataReceive,
        NULL,
        IBUS_BAUDRATE,
        portShared ? MODE_RXTX : MODE_RX,
        (rxConfig->serialrx_inverted ? SERIAL_INVERTED : 0) | (rxConfig->halfDuplex || portShared ? SERIAL_BIDIR : 0)
        );

    return ibusPort != NULL;
}

#endif //SERIAL_RX
