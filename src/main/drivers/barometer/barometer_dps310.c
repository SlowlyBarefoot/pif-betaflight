/*
 * This file is part of Cleanflight, Betaflight and INAV.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 *
 * Copyright: INAVFLIGHT OU
 */

// See datasheet at https://www.infineon.com/dgdl/Infineon-DPS310-DataSheet-v01_02-EN.pdf?fileId=5546d462576f34750157750826c42242


#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/bus.h"
#include "drivers/bus_i2c_pif.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_spi_pif.h"
#include "drivers/time.h"
#include "drivers/barometer/barometer.h"
#include "drivers/barometer/barometer_dps310.h"
#include "drivers/resource.h"

#include "pif/pif_linker.h"

#include "sensor/pif_dps310_i2c.h"
#include "sensor/pif_dps310_spi.h"

// 10 MHz max SPI frequency
#define DPS310_MAX_SPI_CLK_HZ 10000000

#if defined(USE_BARO) && defined(USE_BARO_DPS310)

// The chip is driven by PIF's pif_dps310. pifDps310I2c_Init() and
// pifDps310Spi_Init() reset it, read the calibration coefficients and set up
// continuous pressure and temperature measurement at 32 Hz with 16 times
// oversampling, as the native driver did. The samples are then read by a PIF
// task of pif_dps310's own and handed to baro->evt_read, so TASK_BARO and the
// start/read/get functions of baroDev_t are not used for this chip.
static PifDps310 dps310;

// Read period of the PIF task: 20 Hz, the rate the native driver was sampled
// at through TASK_BARO (45 ms up_delay plus the 1 ms steps around it).
#define DPS310_READ_PERIOD_MS       50

#define DETECTION_MAX_RETRY_COUNT   5
static bool deviceDetect(extDevice_t *dev)
{
    for (int retry = 0; retry < DETECTION_MAX_RETRY_COUNT; retry++) {
        delay(100);

        switch (dev->bus->busType) {
#ifdef USE_I2C
        case BUS_TYPE_I2C: {
            PifI2cPort *port = i2cPifPort(dev->bus->busType_u.i2c.device);
            if (port && pifDps310I2c_Detect(port, dev->busType_u.i2c.address, NULL)) {
                return true;
            }
            break;
        }
#endif
#ifdef USE_BARO_SPI_DPS310
        case BUS_TYPE_SPI: {
            PifSpiPort *port = spiPifPort(spiDeviceByInstance(dev->bus->busType_u.spi.instance));
            if (port && pifDps310Spi_Detect(port, dev)) {
                return true;
            }
            break;
        }
#endif
        default:
            return false;
        }
    }

    return false;
}

static bool deviceConfigure(extDevice_t *dev)
{
    switch (dev->bus->busType) {
#ifdef USE_I2C
    case BUS_TYPE_I2C: {
        PifI2cPort *port = i2cPifPort(dev->bus->busType_u.i2c.device);
        return port && pifDps310I2c_Init(&dps310, PIF_ID_AUTO, port, dev->busType_u.i2c.address, NULL);
    }
#endif
#ifdef USE_BARO_SPI_DPS310
    case BUS_TYPE_SPI: {
        PifSpiPort *port = spiPifPort(spiDeviceByInstance(dev->bus->busType_u.spi.instance));
        return port && pifDps310Spi_Init(&dps310, PIF_ID_AUTO, port, dev);
    }
#endif
    default:
        return false;
    }
}

// Undoes deviceConfigure(): removes the PIF task, if any, and gives the
// PifI2cDevice or PifSpiDevice slot back to its port.
static void deviceUnconfigure(const extDevice_t *dev)
{
    switch (dev->bus->busType) {
#ifdef USE_I2C
    case BUS_TYPE_I2C:
        pifDps310I2c_Clear(&dps310);
        break;
#endif
#ifdef USE_BARO_SPI_DPS310
    case BUS_TYPE_SPI:
        pifDps310Spi_Clear(&dps310);
        break;
#endif
    default:
        break;
    }
}

static void deviceInit(const extDevice_t *dev, resourceOwner_e owner)
{
#ifdef USE_BARO_SPI_DPS310
    if (dev->bus->busType == BUS_TYPE_SPI) {
        IOHi(dev->busType_u.spi.csnPin); // Disable
        IOInit(dev->busType_u.spi.csnPin, owner, 0);
        IOConfigGPIO(dev->busType_u.spi.csnPin, IOCFG_OUT_PP);
        spiSetClkDivisor(dev, spiCalculateDivider(DPS310_MAX_SPI_CLK_HZ));
    }
#else
    UNUSED(dev);
    UNUSED(owner);
#endif
}

static void deviceDeInit(const extDevice_t *dev)
{
#ifdef USE_BARO_SPI_DPS310
    if (dev->bus->busType == BUS_TYPE_SPI) {
        spiPreinitByIO(dev->busType_u.spi.csnPin);
    }
#else
    UNUSED(dev);
#endif
}

bool baroDPS310Detect(baroDev_t *baro)
{
    extDevice_t *dev = &baro->dev;
    bool defaultAddressApplied = false;

    // pif_dps310 waits 40 ms on pif's 1 ms clock after the soft reset, which
    // only advances once pifLinker_Init() has succeeded, and its samples only
    // reach Betaflight through evt_read.
    if (!pifLinker_IsReady() || !baro->evt_read) {
        return false;
    }

    deviceInit(&baro->dev, OWNER_BARO_CS);

    if ((dev->bus->busType == BUS_TYPE_I2C) && (dev->busType_u.i2c.address == 0)) {
        // Default address for DPS310
        dev->busType_u.i2c.address = DPS310_I2C_ADDR(0);
        defaultAddressApplied = true;
    }

    if (!deviceDetect(dev)) {
        deviceDeInit(dev);
        if (defaultAddressApplied) {
            dev->busType_u.i2c.address = 0;
        }
        return false;
    }

    if (!deviceConfigure(dev)) {
        deviceDeInit(dev);
        return false;
    }

    busDeviceRegister(dev);

    if (!pifDps310_AttachTaskForReading(&dps310, PIF_ID_AUTO, DPS310_READ_PERIOD_MS, baro->evt_read, TRUE)) {
        deviceUnconfigure(dev);
        deviceDeInit(dev);
        return false;
    }

    return true;
}

#endif
