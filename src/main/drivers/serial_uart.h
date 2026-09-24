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

#pragma once

#include "drivers/dma.h" // For dmaResource_t

#include "communication/pif_uart.h"

// Since serial ports can be used for any function these buffer sizes should be equal
// The two largest things that need to be sent are: 1, MSP responses, 2, UBLOX SVINFO packet.

// Size must be a power of two due to various optimizations which use 'and' instead of 'mod'
// Various serial routines return the buffer occupied size as uint8_t which would need to be extended in order to
// increase size further.

typedef enum {
    UARTDEV_1 = 0,
    UARTDEV_2 = 1,
    UARTDEV_3 = 2,
    UARTDEV_4 = 3,
    UARTDEV_5 = 4,
    UARTDEV_6 = 5,
    UARTDEV_7 = 6,
    UARTDEV_8 = 7,
    UARTDEV_9 = 8,
    UARTDEV_10 = 9,
    LPUARTDEV_1 = 10,
} UARTDevice_e;

typedef struct uartPort_s {
    serialPort_t port;

#ifdef USE_DMA
#ifdef USE_HAL_DRIVER
    DMA_HandleTypeDef rxDMAHandle;
    DMA_HandleTypeDef txDMAHandle;
#endif

    dmaResource_t *rxDMAResource;
    dmaResource_t *txDMAResource;
    uint32_t rxDMAChannel;
    uint32_t txDMAChannel;

    uint32_t rxDMAIrq;
    uint32_t txDMAIrq;

    // Buffer index the RX DMA had written up to when the PifUart RX head was
    // last moved after it. The DMA writes the RX buffer memory from index 0,
    // see uartRxDmaStart().
    uint32_t rxDMAPos;
    // Bytes the running TX DMA transfer takes from the PifUart TX buffer. They
    // stay in the buffer until the transfer is over, and are removed by the
    // next uartTryStartTxDMA().
    uint16_t txDMALength;

    uint32_t txDMAPeripheralBaseAddr;
    uint32_t rxDMAPeripheralBaseAddr;
#endif // USE_DMA

#ifdef USE_HAL_DRIVER
    // All USARTs can also be used as UART, and we use them only as UART.
    UART_HandleTypeDef Handle;
#endif
    USART_TypeDef *USARTx;

    // Holds the RX and TX buffers of the port in place of the head and tail
    // indices of serialPort_t. The buffers are the static uartNRxBuffer and
    // uartNTxBuffer arrays; only the PifRingBuffer objects come from the PIF
    // heap, once per UART on its first uartOpen(). No PifUart task is attached,
    // so the IRQ handler and the DMA move the bytes themselves.
    PifUart uart;
} uartPort_t;

void uartPinConfigure(const serialPinConfig_t *pSerialPinConfig);
serialPort_t *uartOpen(UARTDevice_e device, serialReceiveCallbackPtr rxCallback, void *rxCallbackData, uint32_t baudRate, portMode_e mode, portOptions_e options);
