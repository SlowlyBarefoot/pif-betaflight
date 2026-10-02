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
#include <stdlib.h>

#include "platform.h"

#if defined(USE_ACC_MPU6050) || defined(USE_GYRO_MPU6050)

#include "build/debug.h"

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_pif.h"
#include "drivers/exti.h"
#include "drivers/nvic.h"
#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "pif/pif_linker.h"

#include "sensor/pif_mpu60x0_i2c.h"

#include "accgyro.h"
#include "accgyro_mpu.h"
#include "accgyro_mpu6050.h"

// MPU6050, Standard address 0x68
// MPU_INT on PB13 on rev4 Naze32 hardware

// The chip is driven by PIF's pif_mpu60x0. mpuDetect() still tells the MPU
// family apart by WHO_AM_I, and pifMpu60x0I2c_Init() then adds the chip to
// the PifI2cPort of its bus, resets it and registers it on g_imu_sensor. The
// register set up after that stays here, since it follows Betaflight's gyro
// settings, and the samples are read with pifMpu60x0_ReadGyro() and
// pifMpu60x0_ReadAccel(), which block on the bus as busReadRegisterBuffer()
// did. The data ready interrupt is still set up by mpuGyroInit().
static PifMpu60x0 mpu6050;

static void mpu6050RegisterWrite(PifMpu60x0Reg registerId, uint8_t value)
{
    (mpu6050._fn.write_byte)(mpu6050._fn.p_device, registerId, value);
}

static void mpu6050AccInit(accDev_t *acc)
{
    switch (acc->mpuDetectionResult.resolution) {
        case MPU_HALF_RESOLUTION:
            acc->acc_1G = 256 * 4;
            break;
        case MPU_FULL_RESOLUTION:
            acc->acc_1G = 512 * 4;
            break;
    }
}

static bool mpu6050AccRead(accDev_t *acc)
{
    return pifMpu60x0_ReadAccel(&mpu6050, acc->ADCRaw);
}

static bool mpu6050GyroRead(gyroDev_t *gyro)
{
    return pifMpu60x0_ReadGyro(&mpu6050, gyro->gyroADCRaw);
}

bool mpu6050AccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != MPU_60x0 || !mpu6050._p_i2c) {
        return false;
    }

    acc->initFn = mpu6050AccInit;
    acc->readFn = mpu6050AccRead;
    acc->revisionCode = (acc->mpuDetectionResult.resolution == MPU_HALF_RESOLUTION ? 'o' : 'n'); // es/non-es variance between MPU6050 sensors, half of the naze boards are mpu6000ES.

    return true;
}

static void mpu6050GyroInit(gyroDev_t *gyro)
{
    mpuGyroInit(gyro);

    // The reset and its 100ms delay were done by pifMpu60x0I2c_Init() in
    // mpu6050GyroDetect()

    mpu6050RegisterWrite(MPU60X0_REG_PWR_MGMT_1, MPU60X0_CLKSEL_PLL_ZGYRO); //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
    mpu6050RegisterWrite(MPU60X0_REG_SMPLRT_DIV, gyro->mpuDividerDrops); //SMPLRT_DIV    -- SMPLRT_DIV = 0  Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    delay(15); //PLL Settling time when changing CLKSEL is max 10ms.  Use 15ms to be sure
    // CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; DLPF_CFG = 1 => ACC bandwidth = 184Hz  GYRO bandwidth = 188Hz
    // gyroSetSampleRate() reads an I2C MPU6050 at 1kHz, which is the gyro output rate with DLPF_CFG 1 to 6.
    // DLPF_CFG 0 would leave it at 8kHz with a 256Hz filter, so the 1kHz reads would alias the rest.
    mpu6050RegisterWrite(MPU60X0_REG_CONFIG, MPU60X0_DLPF_CFG_A184HZ_G188HZ);
    pifMpu60x0_SetGyroConfig(&mpu6050, MPU60X0_FS_SEL_2000DPS);   //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec

    // ACC Init stuff.
    // Accel scale 16g (2048 LSB/g)
    pifMpu60x0_SetAccelConfig(&mpu6050, MPU60X0_AFS_SEL_16G);

    // INT_LEVEL_HIGH, INT_OPEN_DIS, LATCH_INT_DIS, INT_RD_CLEAR_DIS, FSYNC_INT_LEVEL_HIGH, FSYNC_INT_DIS, I2C_BYPASS_EN, CLOCK_DIS
    // I2C_BYPASS_EN puts the auxiliary bus on the main one, where the HMC5883L of a GY-86 is.
    mpu6050RegisterWrite(MPU60X0_REG_INT_PIN_CFG, MPU60X0_I2C_BYPASS_EN(1));

#ifdef USE_MPU_DATA_READY_SIGNAL
    mpu6050RegisterWrite(MPU60X0_REG_INT_ENABLE, MPU60X0_DATA_RDY_EN(1));
#endif
}

bool mpu6050GyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != MPU_60x0) {
        return false;
    }

    // pif_mpu60x0 waits on pif's 1 ms clock, which only advances once
    // pifLinker_Init() has succeeded.
    if (!pifLinker_IsReady()) {
        return false;
    }

    // Adds the chip to the PifI2cPort of its bus and resets it. Done here
    // rather than in initFn so that running out of PifI2cDevice slots fails
    // detection instead of leaving a gyro that cannot be read.
    PifI2cPort *port = i2cPifPort(gyro->dev.bus->busType_u.i2c.device);
    if (mpu6050._p_i2c) {
        pifMpu60x0I2c_Clear(&mpu6050);
    }
    if (!port || !pifMpu60x0I2c_Init(&mpu6050, PIF_ID_AUTO, port, gyro->dev.busType_u.i2c.address, NULL, &g_imu_sensor)) {
        return false;
    }

    gyro->initFn = mpu6050GyroInit;
    gyro->readFn = mpu6050GyroRead;

    gyro->scale = GYRO_SCALE_2000DPS;

    return true;
}
#endif
