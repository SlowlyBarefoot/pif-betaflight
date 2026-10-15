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
 * bus_spi_pif.h - a PifSpiPort for each Betaflight SPI bus.
 *
 * PIF sensor drivers talk to their chip through a PifSpiPort. This gives each
 * SPIDevice one on top of drivers/bus_spi.h, so a PIF driver shares the bus
 * with the Betaflight drivers on it. A SPI bus carries several chips told
 * apart by their chip select, so each PifSpiDevice on the port is handed the
 * extDevice_t of its chip as p_client, and every transfer goes through that
 * extDevice_t. Clients reach a device through &device->parent with the
 * pifDeviceReg_ functions. Register reads and writes block until the transfer
 * is over, like spiReadRegMskBufRB() and spiWriteReg(), and fail rather than
 * wait when a DMA transfer still holds the bus. A plain pifSpiDevice_Transfer()
 * waits for the bus instead, like spiReadWriteBuf(), so it always goes through.
 * pifSpiDevice_StartTransfer() queues the transfer with spiSequence() and
 * returns at once. When it is over, pifSpiPort_sigEndTransfer() is called from
 * the SPI DMA interrupt (or from spiSequence() itself when the bus does not use
 * DMA), followed by the callback attached with spiPifAttachTransferDone(). PIF
 * keeps the port held until pifDeviceReg_CheckTransfer() reports the transfer
 * over, so the client asks it, from that callback or from its own task, before
 * the next transfer on the port. pifSpiDevice_StartTransfer() may be called
 * from an interrupt. It takes over the callbackArg of the device's extDevice_t.
 */

#pragma once

#include "drivers/bus_spi.h"

#include "communication/pif_spi.h"

// Number of PifSpiDevice slots on each port.
#define SPI_PIF_DEVICE_COUNT    2

// Returns the PifSpiPort of a SPI bus, initialising it on first use, or NULL
// if the bus is invalid or the port could not be allocated. The bus itself
// must already have been brought up with spiInit().
PifSpiPort *spiPifPort(SPIDevice device);

typedef void (*spiPifTransferDoneFn)(void *arg);

// Attaches the callback that is called, with arg, when a transfer begun with
// pifSpiDevice_StartTransfer() on the device is over. It is called from the
// SPI DMA interrupt, after pifSpiPort_sigEndTransfer(), so it has to be short;
// it may call pifDeviceReg_CheckTransfer() to give the port back at once. A
// NULL callback detaches it. Returns false when every slot of the port is
// taken by another device.
bool spiPifAttachTransferDone(PifSpiDevice *pDevice, spiPifTransferDoneFn fn, void *arg);
