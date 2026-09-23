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

#ifdef USE_SPI

#include "drivers/bus.h"
#include "drivers/bus_spi.h"

#include "bus_spi_pif.h"

static PifSpiPort spiPifPorts[SPIDEV_COUNT];
static bool spiPifPortReady[SPIDEV_COUNT];

static const extDevice_t *spiPifExtDevice(const PifSpiDevice *pDevice)
{
    return (const extDevice_t *)pDevice->_p_client;
}

// Betaflight's SPI register API always sends a one byte register address, and
// no max_transfer_size is set, so PIF never splits a transfer into pieces that
// continue without one.

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

    return (dev && spiIsBusy(dev)) ? TRUE : FALSE;
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
        pPort->act_read = spiPifActRead;
        pPort->act_write = spiPifActWrite;
        pPort->act_is_busy = spiPifActIsBusy;
        spiPifPortReady[device] = true;
    }

    return pPort;
}

#endif // USE_SPI
