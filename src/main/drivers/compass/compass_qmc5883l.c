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
#include "drivers/bus_i2c_impl.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

#include "compass.h"
#include "compass_qmc5883l.h"

#include "sensor/pif_qmc5883.h"


static PifQmc5883 qmc5883;


static bool qmc5883lInit(magDev_t *magDev)
{
    extDevice_t *dev = &magDev->dev;

    busDeviceRegister(dev);

    if (!pifQmc5883_Init(&qmc5883, PIF_ID_AUTO, &i2cDevice[dev->bus->busType_u.i2c.device].i2c_port, &g_imu_sensor)) return false;

    pifQmc5883_SetControl1(&qmc5883, QMC5883_MODE_CONTIMUOUS | QMC5883_ODR_200HZ | QMC5883_RNG_8G | QMC5883_OSR_512);

    return true;
}

static bool qmc5883lRead(magDev_t *magDev, int16_t *magData)
{
    float buf[3];

    UNUSED(magDev);

    if (!pifImuSensor_ReadRawMag(&g_imu_sensor, buf)) return false;

    magData[X] = (int16_t)buf[0];
    magData[Y] = (int16_t)buf[1];
    magData[Z] = (int16_t)buf[2];

    return true;
}

bool qmc5883lDetect(magDev_t *magDev)
{

    extDevice_t *dev = &magDev->dev;

    if (dev->bus->busType == BUS_TYPE_I2C && dev->busType_u.i2c.address == 0) {
        dev->busType_u.i2c.address = QMC5883_I2C_ADDR;
    }

    if (pifQmc5883_Detect(&i2cDevice[dev->bus->busType_u.i2c.device].i2c_port)) {
        magDev->init = qmc5883lInit;
        magDev->read = qmc5883lRead;
        return true;
    }

    return false;
}
#endif
