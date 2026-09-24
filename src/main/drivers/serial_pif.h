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
 * serial_pif.h - a PifUart on top of a Betaflight serial port.
 *
 * PIF protocol drivers (pif_rc_ibus, ...) talk to a PifUart. This opens a
 * Betaflight serial port with openSerialPort() and puts a PifUart in front of
 * it, so the serial driver itself stays as it is and the bridge works on every
 * UART, soft serial and SITL port.
 *
 * Receive: the port's rxCallback pushes every byte into the PifUart RX buffer
 * from the ISR, which triggers the PifUart RX task; the protocol driver parses
 * the bytes from that task. Transmit: the protocol driver sends with
 * pifUart_SendTxData(), which queues the bytes in the PifUart TX buffer and
 * triggers the PifUart TX task, and that task hands them to serialWrite().
 *
 * On a SERIAL_BIDIR (half-duplex) port the bytes sent are received back. The
 * bridge drops that echo before it reaches the PifUart, as the native iBUS
 * code did with its bytes-to-ignore counters.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "drivers/serial.h"
#include "drivers/time.h"

#include "io/serial.h"

#include "communication/pif_uart.h"

#define SERIAL_PIF_RX_BUFFER_SIZE   64
#define SERIAL_PIF_TX_BUFFER_SIZE   64

typedef struct serialPif_s {
    // Must stay the first member: the PifUart callbacks get the PifUart and
    // cast it back to the serialPif_t.
    PifUart uart;
    serialPort_t *port;
    portOptions_e options;

    // Written by the rxCallback ISR.
    volatile timeUs_t lastRxTimeUs;
    // Echo of the last transmission still to be dropped on a SERIAL_BIDIR
    // port, and the time after which whatever is left of it is not coming.
    volatile uint8_t echoBytesToIgnore;
    volatile timeUs_t echoEndUs;

    uint8_t rxBuffer[SERIAL_PIF_RX_BUFFER_SIZE];
    uint8_t txBuffer[SERIAL_PIF_TX_BUFFER_SIZE];
} serialPif_t;

// Opens the serial port and brings up pif->uart with an RX task for MODE_RX
// and a TX task for MODE_TX, both started once a client is attached with
// pifUart_AttachClient(). Returns the port, or NULL if either the port or the
// PifUart could not be set up; nothing is left open in that case.
serialPort_t *serialPifOpen(serialPif_t *pif, serialPortIdentifier_e identifier, serialPortFunction_e function,
    uint32_t baudRate, portMode_e mode, portOptions_e options);

// Closes the port and releases the PifUart tasks and buffers. Does nothing if
// pif is not open.
void serialPifClose(serialPif_t *pif);

// Time of the last byte received, taken in the ISR. The PifUart RX task runs
// later than that, so a driver that timestamps frames should use this.
static inline timeUs_t serialPifLastRxTimeUs(const serialPif_t *pif)
{
    return pif->lastRxTimeUs;
}
