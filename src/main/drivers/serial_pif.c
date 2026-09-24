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

#include "common/time.h"

#include "drivers/serial.h"
#include "drivers/time.h"

#include "io/serial.h"

#include "serial_pif.h"

// How long after the last byte has gone out its echo may still arrive before
// the rest of it is given up on. Same as the frame gap of the native iBUS code.
#define SERIAL_PIF_ECHO_GAP_US      500

// Runs in the ISR of the port.
static void serialPifDataReceive(uint16_t c, void *data)
{
    serialPif_t *pif = data;
    const timeUs_t now = microsISR();

    if (pif->echoBytesToIgnore) {
        if (cmpTimeUs(now, pif->echoEndUs) <= 0) {
            pif->echoBytesToIgnore--;
            return;
        }
        // The rest of the echo is not coming, which is what a transceiver
        // that does not listen to itself looks like.
        pif->echoBytesToIgnore = 0;
    }

    pif->lastRxTimeUs = now;
    pifUart_PutRxByte(&pif->uart, c);
}

// Called from the PifUart TX task when its buffer holds data. Everything
// queued is moved to the port at once.
static BOOL serialPifActStartTransfer(PifUart *uart)
{
    serialPif_t *pif = (serialPif_t *)uart;
    uint8_t data;

    if (pif->options & SERIAL_BIDIR) {
        const uint16_t count = pifUart_GetFillSizeOfTxBuffer(uart);

        // Set before the first byte goes out, so the ISR knows about its echo.
        // A reply is only sent once its request has been received in full, so
        // no echo of an earlier reply is still being counted down here.
        pif->echoEndUs = micros() + (timeUs_t)count * uart->_transfer_time + SERIAL_PIF_ECHO_GAP_US;
        pif->echoBytesToIgnore = count;
    }

    while (pifUart_GetTxByte(uart, &data) & PIF_UART_SEND_DATA_STATE_DATA) {
        serialWrite(pif->port, data);
    }

    // FALSE keeps the PifUart TX state idle: the buffer has already been
    // emptied, and there is no TX-complete interrupt here to set it back.
    return FALSE;
}

serialPort_t *serialPifOpen(serialPif_t *pif, serialPortIdentifier_e identifier, serialPortFunction_e function,
    uint32_t baudRate, portMode_e mode, portOptions_e options)
{
    pif->port = NULL;
    pif->options = options;
    pif->lastRxTimeUs = 0;
    pif->echoBytesToIgnore = 0;
    pif->echoEndUs = 0;

    if (!pifUart_Init(&pif->uart, PIF_ID_AUTO, baudRate)) {
        return NULL;
    }

    if (mode & MODE_RX) {
        if (!pifUart_AssignRxBuffer(&pif->uart, sizeof(pif->rxBuffer), pif->rxBuffer)
            || !pifUart_AttachRxTask(&pif->uart, PIF_ID_AUTO, TM_EXTERNAL, 0, "SerialRx")) {
            pifUart_Clear(&pif->uart);
            return NULL;
        }
    }

    if (mode & MODE_TX) {
        if (!pifUart_AssignTxBuffer(&pif->uart, sizeof(pif->txBuffer), pif->txBuffer)
            || !pifUart_AttachTxTask(&pif->uart, PIF_ID_AUTO, TM_EXTERNAL, 0, "SerialTx")) {
            pifUart_Clear(&pif->uart);
            return NULL;
        }
        pif->uart.act_start_transfer = serialPifActStartTransfer;
    }

    pif->port = openSerialPort(identifier, function, serialPifDataReceive, pif, baudRate, mode, options);
    if (!pif->port) {
        pifUart_Clear(&pif->uart);
        return NULL;
    }

    return pif->port;
}

void serialPifClose(serialPif_t *pif)
{
    if (!pif->port) {
        return;
    }

    // The rxCallback goes first, so the ISR stops feeding the buffers that
    // pifUart_Clear() releases.
    closeSerialPort(pif->port);
    pifUart_Clear(&pif->uart);
    pif->port = NULL;
}
