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

#include "build/atomic.h"

#ifdef USE_SPI

#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/nvic.h"

#include "bus_spi_pif.h"

// spiSequence() reads the segments of a transfer until it is over, and links
// a transfer queued behind it through its terminating segment, so a transfer
// that does not wait needs segments that outlive the call. Each device slot of
// a port gets its own, because spiSequence() drops a segment list that is
// already queued, and pending marks them as still in use.
//
// A transfer may be started from an interrupt (a gyro EXTI handler) as well as
// from a task, so a slot is released and claimed with interrupts masked.
typedef struct spiPifAsync_s {
    PifSpiDevice *device;
    bool pending;
    busSegment_t segments[2];
} spiPifAsync_t;

// The callback a client attached with spiPifAttachTransferDone().
typedef struct spiPifDone_s {
    PifSpiDevice *device;
    spiPifTransferDoneFn fn;
    void *arg;
} spiPifDone_t;

static PifSpiPort spiPifPorts[SPIDEV_COUNT];
static bool spiPifPortReady[SPIDEV_COUNT];
static spiPifAsync_t spiPifAsync[SPIDEV_COUNT][SPI_PIF_DEVICE_COUNT];
static spiPifDone_t spiPifDone[SPIDEV_COUNT][SPI_PIF_DEVICE_COUNT];
// Slot of the transfer that holds the port, the only one that may end it.
static spiPifAsync_t * volatile spiPifActiveSlot[SPIDEV_COUNT];

static const extDevice_t *spiPifExtDevice(const PifSpiDevice *pDevice)
{
    return (const extDevice_t *)pDevice->parent._p_client;
}

// A free bus has finished every transfer queued on it, so none of the
// segments of its port are in use any more.
static void spiPifReleaseIfIdle(const PifSpiDevice *pDevice, const extDevice_t *dev)
{
    if (spiIsBusy(dev)) {
        return;
    }

    spiPifAsync_t *pAsync = spiPifAsync[pDevice->_p_port - spiPifPorts];

    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        // Checked again, as a transfer may have been started meanwhile
        if (!spiIsBusy(dev)) {
            for (int i = 0; i < SPI_PIF_DEVICE_COUNT; i++) {
                pAsync[i].pending = false;
            }
        }
    }
}

// Segment callback of a transfer that does not wait, called by the SPI driver
// when it is over, from the DMA interrupt when it used DMA. The slot is not
// released here: the driver still reads its terminating segment after this.
// pifSpiPort_sigEndTransfer() does not say which transfer it ends, so a
// transfer PIF has already given up on past the device timeout must not end
// the one that holds the port now.
static busStatus_e spiPifTransferDone(uint32_t arg)
{
    spiPifAsync_t *pSlot = (spiPifAsync_t *)arg;
    PifSpiDevice *pDevice = pSlot->device;
    const unsigned port = pDevice->_p_port - spiPifPorts;

    if (spiPifActiveSlot[port] != pSlot) {
        return BUS_READY;
    }
    spiPifActiveSlot[port] = NULL;

    pifSpiPort_sigEndTransfer(pDevice->_p_port, TRUE);

    for (int i = 0; i < SPI_PIF_DEVICE_COUNT; i++) {
        const spiPifDone_t *pDone = &spiPifDone[port][i];
        if (pDone->device == pDevice) {
            if (pDone->fn) {
                pDone->fn(pDone->arg);
            }
            break;
        }
    }

    return BUS_READY;
}

// A plain transfer with no register address, which pif_max7456 uses for the
// row brightness writes and the 0xFF that ends an auto increment write. It
// waits behind a DMA transfer still on the bus instead of failing, and then
// for its own transfer to finish, as spiReadWriteBuf() does, so the buffers
// may be on the caller's stack.
static void spiPifActTransfer(PifSpiDevice *pDevice, uint8_t *pWrite, uint8_t *pRead, size_t size)
{
    const extDevice_t *dev = spiPifExtDevice(pDevice);

    if (!dev || size == 0 || (!pWrite && !pRead)) {
        return;
    }

    spiReadWriteBuf(dev, pWrite, pRead, size);
}

// The same transfer without the wait, which pif_max7456 uses to send the
// screen by DMA. It is queued behind a transfer still on the bus, as
// spiSequence() does, and act_is_busy reports it until the bus is free. It
// fails while the previous transfer of the same device may still be queued.
// When it is over, pifSpiPort_sigEndTransfer() is called for the port
// through a segment callback, which takes over the callbackArg of the device's
// extDevice_t, so that device must not also run Betaflight segments with a
// callback of their own. It may be called from an interrupt.
static BOOL spiPifActStartTransfer(PifSpiDevice *pDevice, uint8_t *pWrite, uint8_t *pRead, size_t size)
{
    const extDevice_t *dev = spiPifExtDevice(pDevice);

    if (!dev || size == 0 || size > INT32_MAX || (!pWrite && !pRead)) {
        return FALSE;
    }

    spiPifReleaseIfIdle(pDevice, dev);

    // Take the slot this device used last, or else one that is not in use. A
    // slot left behind by a removed device is taken over once it is free.
    spiPifAsync_t *pAsync = spiPifAsync[pDevice->_p_port - spiPifPorts];
    spiPifAsync_t *pSlot = NULL;
    bool claimed = false;

    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        for (int i = 0; i < SPI_PIF_DEVICE_COUNT; i++) {
            if (pAsync[i].device == pDevice) {
                pSlot = &pAsync[i];
                break;
            }
            if (!pSlot && !pAsync[i].pending) {
                pSlot = &pAsync[i];
            }
        }

        if (pSlot && !pSlot->pending) {
            pSlot->device = pDevice;
            pSlot->pending = true;
            claimed = true;
        }
    }

    if (!claimed) {
        return FALSE;
    }

    pSlot->segments[0] = (busSegment_t){.u.buffers = {pWrite, pRead}, (int)size, true, spiPifTransferDone};
    pSlot->segments[1] = (busSegment_t){.u.link = {NULL, NULL}, 0, true, NULL};

    ((extDevice_t *)dev)->callbackArg = (uint32_t)pSlot;
    spiPifActiveSlot[pDevice->_p_port - spiPifPorts] = pSlot;

    spiSequence(dev, &pSlot->segments[0]);

    return TRUE;
}

// Betaflight's SPI register API always sends a one byte register address. No
// max_transfer_size is set, so PIF never splits a transfer; were it set, each
// piece would come with its own register address.

// The register address is sent with bit 7 set, as busReadRegisterBuffer()
// does, which is the read flag of the chips on these buses.
static BOOL spiPifActRead(PifSpiDevice *pDevice, uint32_t iaddr, uint8_t isize, uint8_t *pData, size_t size)
{
    const extDevice_t *dev = spiPifExtDevice(pDevice);

    if (!dev || isize != 1 || size > UINT8_MAX) {
        return FALSE;
    }

    return spiReadRegMskBufRB(dev, iaddr, pData, size) ? TRUE : FALSE;
}

// The register address is sent with bit 7 clear, as busWriteRegister() does.
static BOOL spiPifActWrite(PifSpiDevice *pDevice, uint32_t iaddr, uint8_t isize, uint8_t *pData, size_t size)
{
    const extDevice_t *dev = spiPifExtDevice(pDevice);

    if (!dev || isize != 1) {
        return FALSE;
    }

    if (size == 1) {
        return spiWriteRegRB(dev, iaddr & 0x7f, pData[0]) ? TRUE : FALSE;
    }

    // spiWriteRegBuf() has no busy check of its own.
    if (spiIsBusy(dev)) {
        return FALSE;
    }
    spiWriteRegBuf(dev, iaddr & 0x7f, pData, size);
    return TRUE;
}

static BOOL spiPifActIsBusy(PifSpiDevice *pDevice)
{
    const extDevice_t *dev = spiPifExtDevice(pDevice);

    if (!dev) {
        return FALSE;
    }

    spiPifReleaseIfIdle(pDevice, dev);

    return spiIsBusy(dev) ? TRUE : FALSE;
}

bool spiPifAttachTransferDone(PifSpiDevice *pDevice, spiPifTransferDoneFn fn, void *arg)
{
    if (!pDevice) {
        return false;
    }

    spiPifDone_t *pDones = spiPifDone[pDevice->_p_port - spiPifPorts];
    spiPifDone_t *pDone = NULL;

    for (int i = 0; i < SPI_PIF_DEVICE_COUNT; i++) {
        if (pDones[i].device == pDevice) {
            pDone = &pDones[i];
            break;
        }
        if (!pDone && !pDones[i].device) {
            pDone = &pDones[i];
        }
    }

    if (!pDone) {
        return false;
    }

    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        pDone->device = fn ? pDevice : NULL;
        pDone->fn = fn;
        pDone->arg = arg;
    }

    return true;
}

PifSpiPort *spiPifPort(SPIDevice device)
{
    if (device == SPIINVALID || device >= SPIDEV_COUNT) {
        return NULL;
    }

    PifSpiPort *pPort = &spiPifPorts[device];

    if (!spiPifPortReady[device]) {
        if (!pifSpiPort_Init(pPort, PIF_ID_AUTO, SPI_PIF_DEVICE_COUNT)) {
            return NULL;
        }
        pPort->act_transfer = spiPifActTransfer;
        pPort->act_start_transfer = spiPifActStartTransfer;
        pPort->act_read = spiPifActRead;
        pPort->act_write = spiPifActWrite;
        pPort->act_is_busy = spiPifActIsBusy;
        spiPifPortReady[device] = true;
    }

    return pPort;
}

#endif // USE_SPI
