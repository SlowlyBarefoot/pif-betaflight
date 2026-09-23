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

#include <math.h>

#include "platform.h"

#ifdef USE_MAG_QMC5883

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/bus_i2c_pif.h"
#include "drivers/sensor.h"

#include "pif/pif_linker.h"

#include "sensor/pif_qmc5883.h"

#include "compass.h"
#include "compass_qmc5883l.h"

// The chip is driven by PIF's pif_qmc5883, which attaches itself as the
// magnetometer of the shared g_imu_sensor. The samples are read raw with
// pifQmc5883_ReadMagAsync() so that Betaflight's own mag alignment in
// sensors/compass.c stays the only one applied.
static PifQmc5883 qmc5883;

static bool qmc5883lInit(magDev_t *magDev)
{
    extDevice_t *dev = &magDev->dev;

    busDeviceRegister(dev);

    PifI2cPort *port = i2cPifPort(dev->bus->busType_u.i2c.device);
    if (!port) {
        return false;
    }

    if (!pifQmc5883_Init(&qmc5883, PIF_ID_AUTO, port, NULL, &g_imu_sensor)) {
        return false;
    }

    return pifQmc5883_SetControl1(&qmc5883, QMC5883_MODE_CONTIMUOUS | QMC5883_ODR_200HZ | QMC5883_OSR_512 | QMC5883_RNG_8G);
}

static bool qmc5883lRead(magDev_t *magDev, int16_t *magData)
{
    UNUSED(magDev);

    // One step per call, as the original state machine did: start the STATUS
    // read, then the data read if DRDY is set, then hand the sample over.
    // compassUpdate() calls again 1 ms later while this returns false.
    return pifQmc5883_ReadMagAsync(&qmc5883, magData);
}

bool qmc5883lDetect(magDev_t *magDev)
{
    extDevice_t *dev = &magDev->dev;

    // pifQmc5883_Detect() waits 20 ms on pif's 1 ms clock after the soft
    // reset, which only advances once pifLinker_Init() has succeeded.
    if (dev->bus->busType != BUS_TYPE_I2C || !pifLinker_IsReady()) {
        return false;
    }

    if (dev->busType_u.i2c.address == 0) {
        dev->busType_u.i2c.address = QMC5883_I2C_ADDR;
    }

    // pifQmc5883_Detect() probes the fixed QMC5883_I2C_ADDR.
    if (dev->busType_u.i2c.address != QMC5883_I2C_ADDR) {
        return false;
    }

    PifI2cPort *port = i2cPifPort(dev->bus->busType_u.i2c.device);
    if (port && pifQmc5883_Detect(port, NULL)) {
        magDev->init = qmc5883lInit;
        magDev->read = qmc5883lRead;
        return true;
    }

    return false;
}
#endif
