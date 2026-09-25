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

#include "platform.h"

#include "build/debug.h"

#include "cli/cli.h"

#include "common/streambuf.h"
#include "common/utils.h"

#include "drivers/system.h"

#include "io/displayport_msp.h"

#include "msp/msp.h"

#include "msp_serial.h"

#include "protocol/pif_msp_v2.h"

// A frame holds the payload and at most PIF_MSP_V2_MAX_OVERHEAD bytes of
// header and checksums, and a PifRingBuffer keeps one byte unused.
#define MSP_PORT_ANSWER_SIZE (MSP_PORT_OUTBUF_SIZE + PIF_MSP_V2_MAX_OVERHEAD + 1)

// The frames are parsed and built by PIF's pif_msp_v2. It gets the received
// bytes from mspSerialProcess() rather than through a PifUart, since the port
// can be the USB VCP and the non-MSP bytes are looked at here. A frame it
// builds goes to its answer buffer and is written to the port straight away,
// so a reply is out before its post process function runs.
typedef struct mspPort_s {
    struct serialPort_s *port; // null when port unused.
    timeMs_t lastActivityMs;
    mspPendingSystemRequest_e pendingRequest;
    PifMspV2 msp;
    PifMspPacket *packet;      // the packet pifMspV2_ParsingPacket() has just completed
    bool sharedWithTelemetry;
    mspDescriptor_t descriptor;
    uint8_t inBuf[MSP_PORT_INBUF_SIZE];
} mspPort_t;

static mspPort_t mspPorts[MAX_MSP_PORT_COUNT];

// Answer buffers of the ports, by port index. In CCM on the F4, where the
// main RAM has no room for them; they are only copied to the serial port,
// never read by DMA.
static FAST_DATA_ZERO_INIT uint8_t mspAnswerBuffers[MAX_MSP_PORT_COUNT][MSP_PORT_ANSWER_SIZE];

// evt_receive of the PifMspV2 of a port.
static void mspSerialReceive(PifMsp *msp, PifMspPacket *packet, PifIssuerP issuer)
{
    UNUSED(msp);

    ((mspPort_t *)issuer)->packet = packet;
}

static bool resetMspPort(mspPort_t *mspPortToReset, uint8_t *answerBuffer, serialPort_t *serialPort, bool sharedWithTelemetry)
{
    memset(mspPortToReset, 0, sizeof(mspPort_t));

    PifMspV2 *msp = &mspPortToReset->msp;
    if (!pifMspV2_Init(msp, NULL, PIF_ID_AUTO)
        || !pifMsp_AssignRxBuffer(&msp->_msp, sizeof(mspPortToReset->inBuf), mspPortToReset->inBuf)
        || !pifMsp_AssignAnswerBuffer(&msp->_msp, MSP_PORT_ANSWER_SIZE, answerBuffer)) {
        pifMspV2_Clear(msp);
        return false;
    }
    pifMsp_AttachEvtReceive(&msp->_msp, mspSerialReceive, NULL, mspPortToReset);

    mspPortToReset->port = serialPort;
    mspPortToReset->sharedWithTelemetry = sharedWithTelemetry;
    mspPortToReset->descriptor = mspDescriptorAlloc();
    return true;
}

static void releaseMspPort(mspPort_t *mspPortToRelease)
{
    pifMspV2_Clear(&mspPortToRelease->msp);
    closeSerialPort(mspPortToRelease->port);
    memset(mspPortToRelease, 0, sizeof(mspPort_t));
}

void mspSerialAllocatePorts(void)
{
    uint8_t portIndex = 0;
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_MSP);
    while (portConfig && portIndex < MAX_MSP_PORT_COUNT) {
        mspPort_t *mspPort = &mspPorts[portIndex];

        if (mspPort->port) {
            portIndex++;
            continue;
        }

        serialPort_t *serialPort = openSerialPort(portConfig->identifier, FUNCTION_MSP, NULL, NULL, baudRates[portConfig->msp_baudrateIndex], MODE_RXTX, SERIAL_NOT_INVERTED);
        if (serialPort) {
            bool sharedWithTelemetry = isSerialPortShared(portConfig, FUNCTION_MSP, TELEMETRY_PORT_FUNCTIONS_MASK);
            if (resetMspPort(mspPort, mspAnswerBuffers[portIndex], serialPort, sharedWithTelemetry)) {
                portIndex++;
            } else {
                closeSerialPort(serialPort);
            }
        }

        portConfig = findNextSerialPortConfig(FUNCTION_MSP);
    }
}

void mspSerialReleasePortIfAllocated(serialPort_t *serialPort)
{
    for (uint8_t portIndex = 0; portIndex < MAX_MSP_PORT_COUNT; portIndex++) {
        mspPort_t *candidateMspPort = &mspPorts[portIndex];
        if (candidateMspPort->port == serialPort) {
            releaseMspPort(candidateMspPort);
        }
    }
}

#if defined(USE_TELEMETRY)
void mspSerialReleaseSharedTelemetryPorts(void) {
    for (uint8_t portIndex = 0; portIndex < MAX_MSP_PORT_COUNT; portIndex++) {
        mspPort_t *candidateMspPort = &mspPorts[portIndex];
        if (candidateMspPort->sharedWithTelemetry) {
            releaseMspPort(candidateMspPort);
        }
    }
}
#endif

// Sends a frame through the answer buffer of the port. Returns the number of
// bytes written, 0 if the frame was not sent.
static int mspSerialSendFrame(mspPort_t *msp, PifMspVersion mspVersion, uint8_t direction, uint8_t flags, uint16_t cmd, uint8_t *data, int dataLen)
{
    // We are allowed to send out the response if
    //  a) TX buffer is completely empty (we are talking to well-behaving party that follows request-response scheduling;
    //     this allows us to transmit jumbo frames bigger than TX buffer (serialWriteBuf will block, but for jumbo frames we don't care)
    //  b) Response fits into TX buffer
    const int totalFrameLength = pifMspV2_GetFrameSize(mspVersion, dataLen);
    if (!isSerialTransmitBufferEmpty(msp->port) && ((int)serialTxBytesFree(msp->port) < totalFrameLength)) {
        return 0;
    }

    if (!pifMspV2_MakePacket(&msp->msp, mspVersion, direction, flags, cmd, data, dataLen)) {
        return 0;
    }

    // Transmit frame. It may wrap around the end of the answer buffer.
    uint8_t *frame;
    uint16_t length;
    serialBeginWrite(msp->port);
    while ((length = pifMsp_GetAnswer(&msp->msp._msp, &frame))) {
        serialWriteBuf(msp->port, frame, length);
        pifMsp_RemoveAnswer(&msp->msp._msp, length);
    }
    serialEndWrite(msp->port);

    return totalFrameLength;
}

static int mspSerialEncode(mspPort_t *msp, mspPacket_t *packet, PifMspVersion mspVersion)
{
    return mspSerialSendFrame(msp, mspVersion, packet->result == MSP_RESULT_ERROR ? '!' : '>', packet->flags, packet->cmd,
        sbufPtr(&packet->buf), sbufBytesRemaining(&packet->buf));
}

static mspPostProcessFnPtr mspSerialProcessReceivedCommand(mspPort_t *msp, PifMspPacket *packet, mspProcessCommandFnPtr mspProcessCommandFn)
{
    static uint8_t mspSerialOutBuf[MSP_PORT_OUTBUF_SIZE];

    mspPacket_t reply = {
        .buf = { .ptr = mspSerialOutBuf, .end = ARRAYEND(mspSerialOutBuf), },
        .cmd = -1,
        .flags = 0,
        .result = 0,
        .direction = MSP_DIRECTION_REPLY,
    };
    uint8_t *outBufHead = reply.buf.ptr;

    mspPacket_t command = {
        .buf = { .ptr = packet->p_data, .end = packet->p_data + packet->data_count, },
        .cmd = packet->command,
        .flags = packet->flags,
        .result = 0,
        .direction = MSP_DIRECTION_REQUEST,
    };

    mspPostProcessFnPtr mspPostProcessFn = NULL;
    const mspResult_e status = mspProcessCommandFn(msp->descriptor, &command, &reply, &mspPostProcessFn);

    if (status != MSP_RESULT_NO_REPLY) {
        sbufSwitchToReader(&reply.buf, outBufHead); // change streambuf direction
        mspSerialEncode(msp, &reply, packet->version);
    }

    return mspPostProcessFn;
}

static void mspEvaluateNonMspData(mspPort_t * mspPort, uint8_t receivedChar)
{
   if (receivedChar == serialConfig()->reboot_character) {
        mspPort->pendingRequest = MSP_PENDING_BOOTLOADER_ROM;
#ifdef USE_CLI
   } else if (receivedChar == '#') {
        mspPort->pendingRequest = MSP_PENDING_CLI;
#endif
#if defined(USE_FLASH_BOOT_LOADER)
   } else if (receivedChar == 'F') {
        mspPort->pendingRequest = MSP_PENDING_BOOTLOADER_FLASH;
#endif
    }
}

static void mspProcessPendingRequest(mspPort_t * mspPort)
{
    // If no request is pending or 100ms guard time has not elapsed - do nothing
    if ((mspPort->pendingRequest == MSP_PENDING_NONE) || (millis() - mspPort->lastActivityMs < 100)) {
        return;
    }

    switch(mspPort->pendingRequest) {
    case MSP_PENDING_BOOTLOADER_ROM:
        systemResetToBootloader(BOOTLOADER_REQUEST_ROM);

        break;
#if defined(USE_FLASH_BOOT_LOADER)
    case MSP_PENDING_BOOTLOADER_FLASH:
        systemResetToBootloader(BOOTLOADER_REQUEST_FLASH);

        break;
#endif
#ifdef USE_CLI
    case MSP_PENDING_CLI:
        cliEnter(mspPort->port);
        break;
#endif

    default:
        break;
    }
}

static void mspSerialProcessReceivedReply(PifMspPacket *packet, mspProcessReplyFnPtr mspProcessReplyFn)
{
    mspPacket_t reply = {
        .buf = {
            .ptr = packet->p_data,
            .end = packet->p_data + packet->data_count,
        },
        .cmd = packet->command,
        .result = 0,
    };

    mspProcessReplyFn(&reply);
}

/*
 * Process MSP commands from serial ports configured as MSP ports.
 *
 * Called periodically by the scheduler.
 */
void mspSerialProcess(mspEvaluateNonMspData_e evaluateNonMspData, mspProcessCommandFnPtr mspProcessCommandFn, mspProcessReplyFnPtr mspProcessReplyFn)
{
    for (uint8_t portIndex = 0; portIndex < MAX_MSP_PORT_COUNT; portIndex++) {
        mspPort_t * const mspPort = &mspPorts[portIndex];
        if (!mspPort->port) {
            continue;
        }

        mspPostProcessFnPtr mspPostProcessFn = NULL;

        if (serialRxBytesWaiting(mspPort->port)) {
            // There are bytes incoming - abort pending request
            mspPort->lastActivityMs = millis();
            mspPort->pendingRequest = MSP_PENDING_NONE;

            while (serialRxBytesWaiting(mspPort->port)) {
                const uint8_t c = serialRead(mspPort->port);
                const PifMspFrame frame = pifMspV2_ParsingPacket(&mspPort->msp, c);

                if (frame == MF_OTHER && evaluateNonMspData == MSP_EVALUATE_NON_MSP_DATA) {
                    mspEvaluateNonMspData(mspPort, c);
                }

                if (frame == MF_PACKET) {
                    if (mspPort->packet->type == MPT_COMMAND) {
                        mspPostProcessFn = mspSerialProcessReceivedCommand(mspPort, mspPort->packet, mspProcessCommandFn);
                    } else {
                        mspSerialProcessReceivedReply(mspPort->packet, mspProcessReplyFn);
                    }

                    break; // process one command at a time so as not to block.
                }
            }

            if (mspPostProcessFn) {
                waitForSerialPortToFinishTransmitting(mspPort->port);
                mspPostProcessFn(mspPort->port);
            }
        } else {
            mspProcessPendingRequest(mspPort);
        }
    }
}

bool mspSerialWaiting(void)
{
    for (uint8_t portIndex = 0; portIndex < MAX_MSP_PORT_COUNT; portIndex++) {
        mspPort_t * const mspPort = &mspPorts[portIndex];
        if (!mspPort->port) {
            continue;
        }

        if (serialRxBytesWaiting(mspPort->port)) {
            return true;
        }
    }
    return false;
}

void mspSerialInit(void)
{
    memset(mspPorts, 0, sizeof(mspPorts));
    mspSerialAllocatePorts();
}

int mspSerialPush(serialPortIdentifier_e port, uint8_t cmd, uint8_t *data, int datalen, mspDirection_e direction)
{
    int ret = 0;

    for (int portIndex = 0; portIndex < MAX_MSP_PORT_COUNT; portIndex++) {
        mspPort_t * const mspPort = &mspPorts[portIndex];

        // XXX Kludge!!! Avoid zombie VCP port (avoid VCP entirely for now)
        if (!mspPort->port
#ifndef USE_MSP_PUSH_OVER_VCP
            || mspPort->port->identifier == SERIAL_PORT_USB_VCP
#endif
            || (port != SERIAL_PORT_ALL && mspPort->port->identifier != port)) {
            continue;
        }

        mspPacket_t push = {
            .buf = { .ptr = data, .end = data + datalen, },
            .cmd = cmd,
            .result = 0,
            .direction = direction,
        };

        ret = mspSerialEncode(mspPort, &push, MV_V1);
    }
    return ret; // return the number of bytes written
}


uint32_t mspSerialTxBytesFree(void)
{
    uint32_t ret = UINT32_MAX;

    for (int portIndex = 0; portIndex < MAX_MSP_PORT_COUNT; portIndex++) {
        mspPort_t * const mspPort = &mspPorts[portIndex];
        if (!mspPort->port) {
            continue;
        }

        // XXX Kludge!!! Avoid zombie VCP port (avoid VCP entirely for now)
        if (mspPort->port->identifier == SERIAL_PORT_USB_VCP) {
            continue;
        }

        const uint32_t bytesFree = serialTxBytesFree(mspPort->port);
        if (bytesFree < ret) {
            ret = bytesFree;
        }
    }

    return ret;
}
