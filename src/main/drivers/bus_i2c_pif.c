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

#ifdef USE_I2C

#include "drivers/bus_i2c.h"

#include "bus_i2c_pif.h"

static PifI2cPort i2cPifPorts[I2CDEV_COUNT];
static bool i2cPifPortReady[I2CDEV_COUNT];

static I2CDevice i2cPifPortDevice(const PifI2cDevice *pDevice)
{
    return (I2CDevice)(pDevice->_p_port - i2cPifPorts);
}

// Betaflight's I2C API sends a one byte register address, or none when it is
// given 0xFF (the F4 driver and the HAL one both take it so), so a transfer
// with any other internal address size cannot be expressed. With no register
// address (isize 0) only pData goes over the bus: the commands of pif_ms5611
// and the probe of pifI2cPort_ScanAddress() are sent so. Register 0xFF itself
// cannot be reached, so a transfer to it fails rather than going out as one
// with no register address.
#define I2C_PIF_NO_REGISTER     0xFF

static bool i2cPifRegister(uint32_t iaddr, uint8_t isize, uint8_t *pReg)
{
    if (isize == 0) {
        *pReg = I2C_PIF_NO_REGISTER;
        return true;
    }
    if (isize == 1 && iaddr != I2C_PIF_NO_REGISTER) {
        *pReg = iaddr;
        return true;
    }
    return false;
}

// Reads only start the transfer and answer IR_WAIT; i2cPifActCheck() then
// reports the end of it. PIF's blocking reads poll that in their wait loop,
// and pifI2cDevice_StartRead() lets a driver poll it from its own task instead,
// so pData must stay valid until the transfer is over either way.
static PifI2cReturn i2cPifActRead(PifI2cDevice *pDevice, uint32_t iaddr, uint8_t isize, uint8_t *pData, size_t size)
{
    uint8_t reg;

    if (!i2cPifRegister(iaddr, isize, &reg) || size == 0 || size > UINT8_MAX) {
        return IR_ERROR;
    }

    return i2cReadBuffer(i2cPifPortDevice(pDevice), pDevice->addr, reg, size, pData) ? IR_WAIT : IR_ERROR;
}

// The I2C drivers have no completion callback to call
// pifI2cPort_sigEndTransfer() from, so the port is polled instead.
static PifI2cReturn i2cPifActCheck(PifI2cDevice *pDevice)
{
    bool error = false;

    if (i2cBusy(i2cPifPortDevice(pDevice), &error)) {
        return IR_WAIT;
    }

    return error ? IR_ERROR : IR_COMPLETE;
}

// Writes run like reads: pData is the caller's buffer, which a blocking
// pifI2cDevice_Write() keeps valid until the transfer is over. A write with no
// register address has to carry at least one byte, since the drivers do not
// send an address alone.
static PifI2cReturn i2cPifActWrite(PifI2cDevice *pDevice, uint32_t iaddr, uint8_t isize, uint8_t *pData, size_t size)
{
    uint8_t reg;

    if (!i2cPifRegister(iaddr, isize, &reg) || (isize == 0 && size == 0) || size > UINT8_MAX) {
        return IR_ERROR;
    }

    return i2cWriteBuffer(i2cPifPortDevice(pDevice), pDevice->addr, reg, size, pData) ? IR_WAIT : IR_ERROR;
}

// Called by PIF when a transfer outlives pDevice->timeout, which is 10 ms by
// default like I2C_TIMEOUT_US. The transfer is still holding the peripheral,
// so reset it as i2cWait() would have.
static void i2cPifActRecover(PifI2cDevice *pDevice)
{
    i2cRecover(i2cPifPortDevice(pDevice));
}

PifI2cPort *i2cPifPort(I2CDevice device)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        return NULL;
    }

    PifI2cPort *pPort = &i2cPifPorts[device];

    if (!i2cPifPortReady[device]) {
        if (!pifI2cPort_Init(pPort, PIF_ID_AUTO, I2C_PIF_DEVICE_COUNT)) {
            return NULL;
        }
        pPort->act_read = i2cPifActRead;
        pPort->act_write = i2cPifActWrite;
        pPort->act_check = i2cPifActCheck;
        pPort->act_recover = i2cPifActRecover;
        i2cPifPortReady[device] = true;
    }

    return pPort;
}

#endif // USE_I2C
