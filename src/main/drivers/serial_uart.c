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
 * Authors:
 * jflyper - Refactoring, cleanup and made pin-configurable
 * Dominic Clifton - Serial port abstraction, Separation of common STM32 code for cleanflight, various cleanups.
 * Hamasaki/Timecop - Initial baseflight code
*/

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_UART

#include "build/build_config.h"

#include "common/utils.h"

#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "drivers/rcc.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"

#include "pg/serial_uart.h"

#if defined(STM32H7)
#define UART_TX_BUFFER_ATTRIBUTE DMA_RAM            // D2 SRAM
#define UART_RX_BUFFER_ATTRIBUTE DMA_RAM            // D2 SRAM
#elif defined(STM32G4)
#define UART_TX_BUFFER_ATTRIBUTE DMA_RAM_W          // SRAM MPU NOT_BUFFERABLE
#define UART_RX_BUFFER_ATTRIBUTE DMA_RAM_R          // SRAM MPU NOT CACHABLE
#elif defined(STM32F7)
#define UART_TX_BUFFER_ATTRIBUTE FAST_DATA_ZERO_INIT // DTCM RAM
#define UART_RX_BUFFER_ATTRIBUTE FAST_DATA_ZERO_INIT // DTCM RAM
#elif defined(STM32F4) || defined(STM32F3) || defined(STM32F1)
#define UART_TX_BUFFER_ATTRIBUTE                    // NONE
#define UART_RX_BUFFER_ATTRIBUTE                    // NONE
#else
#error Undefined UART_{TX,RX}_BUFFER_ATTRIBUTE for this MCU
#endif

#define UART_BUFFERS(n) \
    UART_BUFFER(UART_TX_BUFFER_ATTRIBUTE, n, T); \
    UART_BUFFER(UART_RX_BUFFER_ATTRIBUTE, n, R); struct dummy_s

#define LPUART_BUFFERS(n) \
    LPUART_BUFFER(UART_TX_BUFFER_ATTRIBUTE, n, T); \
    LPUART_BUFFER(UART_RX_BUFFER_ATTRIBUTE, n, R); struct dummy_s

#ifdef USE_UART1
UART_BUFFERS(1);
#endif

#ifdef USE_UART2
UART_BUFFERS(2);
#endif

#ifdef USE_UART3
UART_BUFFERS(3);
#endif

#ifdef USE_UART4
UART_BUFFERS(4);
#endif

#ifdef USE_UART5
UART_BUFFERS(5);
#endif

#ifdef USE_UART6
UART_BUFFERS(6);
#endif

#ifdef USE_UART7
UART_BUFFERS(7);
#endif

#ifdef USE_UART8
UART_BUFFERS(8);
#endif

#ifdef USE_UART9
UART_BUFFERS(9);
#endif

#ifdef USE_UART10
UART_BUFFERS(10);
#endif

#ifdef USE_LPUART1
LPUART_BUFFERS(1);
#endif

#undef UART_BUFFERS

// Brings up the PifUart of the port on the first open of the UART, over the
// static buffers of the UART, and empties its buffers on every later one.
static bool uartInitPifUart(uartDevice_t *uartdev, uint32_t baudRate)
{
    const uartHardware_t *hardware = uartdev->hardware;
    PifUart *uart = &uartdev->port.uart;

    if (uart->_p_rx_buffer) {
        pifRingBuffer_Empty(uart->_p_rx_buffer);
        pifRingBuffer_Empty(uart->_p_tx_buffer);
        return pifUart_ChangeBaudrate(uart, baudRate);
    }

    if (!pifUart_Init(uart, PIF_ID_AUTO, baudRate)
        || !pifUart_AssignRxBuffer(uart, hardware->rxBufferSize, (uint8_t *)hardware->rxBuffer)
        || !pifUart_AssignTxBuffer(uart, hardware->txBufferSize, (uint8_t *)hardware->txBuffer)) {
        pifUart_Clear(uart);
        return false;
    }

    return true;
}

serialPort_t *uartOpen(UARTDevice_e device, serialReceiveCallbackPtr rxCallback, void *rxCallbackData, uint32_t baudRate, portMode_e mode, portOptions_e options)
{
    uartDevice_t *uartdev = uartDevmap[device];

    if (!uartdev || !uartInitPifUart(uartdev, baudRate)) {
        return NULL;
    }

    uartPort_t *uartPort = serialUART(device, baudRate, mode, options);

    if (!uartPort)
        return (serialPort_t *)uartPort;

#ifdef USE_DMA
    uartPort->txDMALength = 0;
#endif

    // common serial initialisation code should move to serialPort::init()
    // callback works for IRQ-based RX ONLY
    uartPort->port.rxCallback = rxCallback;
    uartPort->port.rxCallbackData = rxCallbackData;
    uartPort->port.mode = mode;
    uartPort->port.baudRate = baudRate;
    uartPort->port.options = options;

    uartReconfigure(uartPort);

    return (serialPort_t *)uartPort;
}

static void uartSetBaudRate(serialPort_t *instance, uint32_t baudRate)
{
    uartPort_t *uartPort = (uartPort_t *)instance;
    uartPort->port.baudRate = baudRate;
    pifUart_ChangeBaudrate(&uartPort->uart, baudRate);
    uartReconfigure(uartPort);
}

static void uartSetMode(serialPort_t *instance, portMode_e mode)
{
    uartPort_t *uartPort = (uartPort_t *)instance;
    uartPort->port.mode = mode;
    uartReconfigure(uartPort);
}

#ifdef USE_DMA
void uartResetRxDmaBuffer(uartPort_t *uartPort)
{
    PifRingBuffer *rxBuffer = uartPort->uart._p_rx_buffer;

    // A circular DMA always starts at the base of its memory, so the ring
    // buffer has to start at index 0 as well. Re-initialising it over the
    // same memory is the only way PifRingBuffer offers to get there.
    pifRingBuffer_InitStatic(rxBuffer, rxBuffer->_id, rxBuffer->_size, (uint8_t *)uartPort->port.rxBuffer);
    pifRingBuffer_SetName(rxBuffer, "RB");
    uartPort->rxDMAPos = 0;
}

// Moves the PifUart RX head up to where the RX DMA has written. Only runs in
// the context that reads the port, which is also the only one that touches
// the RX buffer when the port receives by DMA.
static void uartSyncRxDma(uartPort_t *uartPort)
{
    const uint32_t size = uartPort->port.rxBufferSize;
#ifdef USE_HAL_DRIVER
    const uint32_t counter = __HAL_DMA_GET_COUNTER(uartPort->Handle.hdmarx);
#else
    const uint32_t counter = xDMA_GetCurrDataCounter(uartPort->rxDMAResource);
#endif

    // The counter runs down from the buffer size and is reloaded after 1.
    const uint32_t pos = (size - counter) % size;
    const uint32_t count = (pos + size - uartPort->rxDMAPos) % size;

    if (count) {
        pifRingBuffer_MoveHead(uartPort->uart._p_rx_buffer, count);
        uartPort->rxDMAPos = pos;
    }
}
#endif

static uint32_t uartTotalRxBytesWaiting(const serialPort_t *instance)
{
    uartPort_t *uartPort = (uartPort_t *)instance;

#ifdef USE_DMA
    if (uartPort->rxDMAResource) {
        uartSyncRxDma(uartPort);
    }
#endif

    return pifUart_GetFillSizeOfRxBuffer(&uartPort->uart);
}

static uint32_t uartTotalTxBytesFree(const serialPort_t *instance)
{
    uartPort_t *uartPort = (uartPort_t *)instance;

    // The bytes of a running TX DMA transfer are still in the buffer, so
    // they are counted here without having to ask the DMA.
    return (uartPort->port.txBufferSize - 1) - pifUart_GetFillSizeOfTxBuffer(&uartPort->uart);
}

static bool isUartTransmitBufferEmpty(const serialPort_t *instance)
{
    uartPort_t *uartPort = (uartPort_t *)instance;

    return pifUart_GetFillSizeOfTxBuffer(&uartPort->uart) == 0;
}

static uint8_t uartRead(serialPort_t *instance)
{
    uint8_t ch = 0;
    uartPort_t *uartPort = (uartPort_t *)instance;

#ifdef USE_DMA
    if (uartPort->rxDMAResource) {
        uartSyncRxDma(uartPort);
    }
#endif

    pifRingBuffer_GetByte(uartPort->uart._p_rx_buffer, &ch);

    return ch;
}

static void uartWrite(serialPort_t *instance, uint8_t ch)
{
    uartPort_t *uartPort = (uartPort_t *)instance;

    // A full buffer drops the byte instead of overwriting the oldest ones.
    pifUart_SendTxData(&uartPort->uart, &ch, 1);

#ifdef USE_DMA
    if (uartPort->txDMAResource) {
        uartTryStartTxDMA(uartPort);
    } else
#endif
    {
#ifdef USE_HAL_DRIVER
        __HAL_UART_ENABLE_IT(&uartPort->Handle, UART_IT_TXE);
#else
        USART_ITConfig(uartPort->USARTx, USART_IT_TXE, ENABLE);
#endif
    }
}

const struct serialPortVTable uartVTable[] = {
    {
        .serialWrite = uartWrite,
        .serialTotalRxWaiting = uartTotalRxBytesWaiting,
        .serialTotalTxFree = uartTotalTxBytesFree,
        .serialRead = uartRead,
        .serialSetBaudRate = uartSetBaudRate,
        .isSerialTransmitBufferEmpty = isUartTransmitBufferEmpty,
        .setMode = uartSetMode,
        .setCtrlLineStateCb = NULL,
        .setBaudRateCb = NULL,
        .writeBuf = NULL,
        .beginWrite = NULL,
        .endWrite = NULL,
    }
};

#ifdef USE_DMA
void uartConfigureDma(uartDevice_t *uartdev)
{
    uartPort_t *uartPort = &(uartdev->port);
    const uartHardware_t *hardware = uartdev->hardware;

#ifdef USE_DMA_SPEC
    UARTDevice_e device = hardware->device;
    const dmaChannelSpec_t *dmaChannelSpec;

    if (serialUartConfig(device)->txDmaopt != DMA_OPT_UNUSED) {
        dmaChannelSpec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_UART_TX, device, serialUartConfig(device)->txDmaopt);
        if (dmaChannelSpec) {
            uartPort->txDMAResource = dmaChannelSpec->ref;
            uartPort->txDMAChannel = dmaChannelSpec->channel;
        }
    }

    if (serialUartConfig(device)->rxDmaopt != DMA_OPT_UNUSED) {
        dmaChannelSpec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_UART_RX, device, serialUartConfig(device)->txDmaopt);
        if (dmaChannelSpec) {
            uartPort->rxDMAResource = dmaChannelSpec->ref;
            uartPort->rxDMAChannel = dmaChannelSpec->channel;
        }
    }
#else
    // Non USE_DMA_SPEC does not support configurable ON/OFF of UART DMA

    if (hardware->rxDMAResource) {
        uartPort->rxDMAResource = hardware->rxDMAResource;
        uartPort->rxDMAChannel = hardware->rxDMAChannel;
    }

    if (hardware->txDMAResource) {
        uartPort->txDMAResource = hardware->txDMAResource;
        uartPort->txDMAChannel = hardware->txDMAChannel;
    }
#endif

    if (uartPort->txDMAResource) {
        dmaIdentifier_e identifier = dmaGetIdentifier(uartPort->txDMAResource);
        if (dmaAllocate(identifier, OWNER_SERIAL_TX, RESOURCE_INDEX(hardware->device))) {
            dmaEnable(identifier);
            dmaSetHandler(identifier, uartDmaIrqHandler, hardware->txPriority, (uint32_t)uartdev);
            uartPort->txDMAPeripheralBaseAddr = (uint32_t)&UART_REG_TXD(hardware->reg);
        }
    }

    if (uartPort->rxDMAResource) {
        dmaIdentifier_e identifier = dmaGetIdentifier(uartPort->rxDMAResource);
        if (dmaAllocate(identifier, OWNER_SERIAL_RX, RESOURCE_INDEX(hardware->device))) {
            dmaEnable(identifier);
            uartPort->rxDMAPeripheralBaseAddr = (uint32_t)&UART_REG_RXD(hardware->reg);
        }
    }
}
#endif

#define UART_IRQHandler(type, number, dev)                    \
    FAST_IRQ_HANDLER void type ## number ## _IRQHandler(void)                  \
    {                                                         \
        uartPort_t *uartPort = &(uartDevmap[UARTDEV_ ## dev]->port); \
        uartIrqHandler(uartPort);                                    \
    }

#ifdef USE_UART1
UART_IRQHandler(USART, 1, 1) // USART1 Rx/Tx IRQ Handler
#endif

#ifdef USE_UART2
UART_IRQHandler(USART, 2, 2) // USART2 Rx/Tx IRQ Handler
#endif

#ifdef USE_UART3
UART_IRQHandler(USART, 3, 3) // USART3 Rx/Tx IRQ Handler
#endif

#ifdef USE_UART4
UART_IRQHandler(UART, 4, 4)  // UART4 Rx/Tx IRQ Handler
#endif

#ifdef USE_UART5
UART_IRQHandler(UART, 5, 5)  // UART5 Rx/Tx IRQ Handler
#endif

#ifdef USE_UART6
UART_IRQHandler(USART, 6, 6) // USART6 Rx/Tx IRQ Handler
#endif

#ifdef USE_UART7
UART_IRQHandler(UART, 7, 7)  // UART7 Rx/Tx IRQ Handler
#endif

#ifdef USE_UART8
UART_IRQHandler(UART, 8, 8)  // UART8 Rx/Tx IRQ Handler
#endif

#ifdef USE_UART9
UART_IRQHandler(LPUART, 1, 9) // UART9 (implemented with LPUART1) Rx/Tx IRQ Handler
#endif

#endif // USE_UART
