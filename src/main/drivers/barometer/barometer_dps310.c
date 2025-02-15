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
#include <string.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"
#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_impl.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "drivers/time.h"
#include "drivers/barometer/barometer.h"
#include "drivers/barometer/barometer_dps310.h"
#include "drivers/resource.h"

#include "sensors/barometer.h"

#include "sensor/pif_dps310_i2c.h"
#include "sensor/pif_dps310_spi.h"

// 10 MHz max SPI frequency
#define DPS310_MAX_SPI_CLK_HZ 10000000

#if defined(USE_BARO) && defined(USE_BARO_DPS310)

static PifDps310 dps310;

#define DETECTION_MAX_RETRY_COUNT   5
static bool deviceDetect(extDevice_t *dev, PifI2cPort *p_i2c_port)
{
    (void)dev;

    for (int retry = 0; retry < DETECTION_MAX_RETRY_COUNT; retry++) {
        delay(100);

        if (dev->bus->busType == BUS_TYPE_I2C) {
            if (pifDps310I2c_Detect(p_i2c_port, dev->busType_u.i2c.address, NULL)) return true;
        }
        else if (dev->bus->busType == BUS_TYPE_SPI) {
            if (pifDps310Spi_Detect(dev->busType_u.spi.p_spi_port, dev)) return true;
        }
    };

    return false;
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
    uint8_t disallow_yield_id;
    PifI2cPort *p_i2c_port;

    if (!baro->evt_read) return false;

    deviceInit(&baro->dev, OWNER_BARO_CS);

    if ((dev->bus->busType == BUS_TYPE_I2C) && (dev->busType_u.i2c.address == 0)) {
        // Default address for DPS310
        dev->busType_u.i2c.address = DPS310_I2C_ADDR(0);
        defaultAddressApplied = true;
        p_i2c_port = &i2cDevice[I2C_CFG_TO_DEV(barometerConfig()->baro_i2c_device)].i2c_port;
    }

    if (!deviceDetect(dev, p_i2c_port)) {
        deviceDeInit(dev);
        if (defaultAddressApplied) {
            dev->busType_u.i2c.address = 0;
        }
        return false;
    }

    if (dev->bus->busType == BUS_TYPE_I2C) {
        if (!pifDps310I2c_Init(&dps310, PIF_ID_AUTO, p_i2c_port, dev->busType_u.i2c.address, NULL)) {
            deviceDeInit(dev);
            return false;
        }
        disallow_yield_id = DISALLOW_YIELD_ID_I2C;
    }
#ifdef USE_BARO_SPI_DPS310
    else if (dev->bus->busType == BUS_TYPE_SPI) {
        if (!pifDps310Spi_Init(&dps310, PIF_ID_AUTO, dev->busType_u.spi.p_spi_port, dev)) {
            deviceDeInit(dev);
            return false;
        }
        disallow_yield_id = DISALLOW_YIELD_ID_SPI;
    }
#endif

    busDeviceRegister(dev);

    if (!pifDps310_AddTaskForReading(&dps310, 50, baro->evt_read, TRUE)) {   // 50ms : 20hz update rate (20hz LPF on acc)
        return false;
    }
    dps310._p_task->disallow_yield_id = disallow_yield_id;

    return true;
}

#endif
