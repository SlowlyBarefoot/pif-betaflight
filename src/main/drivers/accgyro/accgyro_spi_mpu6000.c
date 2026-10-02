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
 * Dominic Clifton - Cleanflight implementation
 * John Ihlein - Initial FF32 code
*/

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(USE_GYRO_SPI_MPU6000) || defined(USE_ACC_SPI_MPU6000)

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_spi_mpu6000.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_spi_pif.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/time.h"
#include "drivers/sensor.h"
#include "drivers/system.h"

#include "pif/pif_linker.h"

#include "sensor/pif_mpu60x0_spi.h"

// 20 MHz max SPI frequency
#define MPU6000_MAX_SPI_CLK_HZ 20000000

#define MPU6000_SHORT_THRESHOLD         82  // Any interrupt interval less than this will be recognised as the short interval of ~79us

// Need to see at least this many interrupts during initialisation to confirm EXTI connectivity
#define GYRO_EXTI_DETECT_THRESHOLD 1000

// Accelerometer, temperature and gyro in one burst, after the register address
#define MPU6000_BURST_SIZE      (MPU60X0_REG_GYRO_XOUT_H - MPU60X0_REG_ACCEL_XOUT_H + 7)
// Index of the gyro X axis in the 16 bit words of rxBuf after a burst
#define MPU6000_BURST_GYRO_INDEX (((MPU60X0_REG_GYRO_XOUT_H - MPU60X0_REG_ACCEL_XOUT_H) >> 1) + 1)

// The chip is driven by PIF's pif_mpu60x0. pifMpu60x0Spi_Detect() checks
// WHO_AM_I and the product ID, and pifMpu60x0Spi_Init() adds the chip to the
// PifSpiPort of its bus, resets it and registers it on g_imu_sensor. The
// register set up after that and the gyro and accelerometer reads stay here,
// since they follow Betaflight's gyro modes; they go through the PifSpiDevice
// of the chip. The EXTI triggered DMA read is started from the ISR with
// pifSpiDevice_StartTransfer(), and its completion is reported through the
// event attached with pifSpiDevice_AttachEvtTransferDone().
//
// A board may carry two of them (gyro_to_use = BOTH), so there is one
// PifMpu60x0 per gyro, found by the extDevice_t it was handed as p_client.
// Both register on the one g_imu_sensor, so it reads the last one set up.
static PifMpu60x0 mpu6000[2];

static PifMpu60x0 *mpu6000Find(const extDevice_t *dev)
{
    for (unsigned i = 0; i < ARRAYLEN(mpu6000); i++) {
        if (mpu6000[i]._p_spi && mpu6000[i]._p_spi->_p_client == dev) {
            return &mpu6000[i];
        }
    }
    return NULL;
}

static PifMpu60x0 *mpu6000FindFree(const extDevice_t *dev)
{
    PifMpu60x0 *mpu = mpu6000Find(dev);
    if (mpu) {
        pifMpu60x0Spi_Clear(mpu);
        return mpu;
    }
    for (unsigned i = 0; i < ARRAYLEN(mpu6000); i++) {
        if (!mpu6000[i]._p_spi) {
            return &mpu6000[i];
        }
    }
    return NULL;
}

static PifSpiPort *mpu6000PifPort(const extDevice_t *dev)
{
    return spiPifPort(spiDeviceByInstance(dev->bus->busType_u.spi.instance));
}

static void mpu6000RegisterWrite(PifMpu60x0 *mpu, PifMpu60x0Reg registerId, uint8_t value)
{
    (mpu->_fn.write_byte)(mpu->_fn.p_device, registerId, value);
    delayMicroseconds(15);
}

/*
 * Gyro interrupt service routine
 */
#ifdef USE_GYRO_EXTI
// Called in ISR context
// Gyro read has just completed
static void mpu6000TransferDone(PifIssuerP p_issuer)
{
    gyroDev_t *gyro = (gyroDev_t *)p_issuer;
    int32_t gyroDmaDuration = cmpTimeCycles(getCycleCounter(), gyro->gyroLastEXTI);

    if (gyroDmaDuration > gyro->gyroDmaMaxDuration) {
        gyro->gyroDmaMaxDuration = gyroDmaDuration;
    }

    gyro->dataReady = true;
}

static void mpu6000ExtiHandler(extiCallbackRec_t *cb)
{
    gyroDev_t *gyro = container_of(cb, gyroDev_t, exti);

    // Ideally we'd use a timer to capture such information, but unfortunately the port used for EXTI interrupt does
    // not have an associated timer
    uint32_t nowCycles = getCycleCounter();
    int32_t gyroLastPeriod = cmpTimeCycles(nowCycles, gyro->gyroLastEXTI);
    // This detects the short (~79us) EXTI interval of an MPU6xxx gyro
    if ((gyro->gyroShortPeriod == 0) || (gyroLastPeriod < gyro->gyroShortPeriod)) {
        gyro->gyroSyncEXTI = gyro->gyroLastEXTI + gyro->gyroDmaMaxDuration;
    }
    gyro->gyroLastEXTI = nowCycles;

    if (gyro->gyroModeSPI == GYRO_EXTI_INT_DMA) {
        PifMpu60x0 *mpu = mpu6000Find(&gyro->dev);
        if (mpu) {
            pifSpiDevice_StartTransfer(mpu->_p_spi, gyro->dev.txBuf, &gyro->dev.rxBuf[1], MPU6000_BURST_SIZE);
        }
    }

    gyro->detectedEXTI++;
}

static void mpu6000IntExtiInit(gyroDev_t *gyro)
{
    if (gyro->mpuIntExtiTag == IO_TAG_NONE) {
        return;
    }

    const IO_t mpuIntIO = IOGetByTag(gyro->mpuIntExtiTag);

#ifdef ENSURE_MPU_DATA_READY_IS_LOW
    uint8_t status = IORead(mpuIntIO);
    if (status) {
        return;
    }
#endif

    IOInit(mpuIntIO, OWNER_GYRO_EXTI, 0);
    EXTIHandlerInit(&gyro->exti, mpu6000ExtiHandler);
    EXTIConfig(mpuIntIO, &gyro->exti, NVIC_PRIO_MPU_INT_EXTI, IOCFG_IN_FLOATING, BETAFLIGHT_EXTI_TRIGGER_RISING);
    EXTIEnable(mpuIntIO);
}
#endif // USE_GYRO_EXTI

static bool mpu6000AccRead(accDev_t *acc)
{
    switch (acc->gyro->gyroModeSPI) {
    case GYRO_EXTI_INT:
    case GYRO_EXTI_NO_INT:
    {
        PifMpu60x0 *mpu = mpu6000Find(&acc->gyro->dev);
        if (!mpu) {
            return false;
        }

        acc->gyro->dev.txBuf[0] = MPU60X0_REG_ACCEL_XOUT_H | 0x80;

        // Waits for completion
        pifSpiDevice_Transfer(mpu->_p_spi, acc->gyro->dev.txBuf, &acc->gyro->dev.rxBuf[1], 7);

        // Fall through
        FALLTHROUGH;
    }

    case GYRO_EXTI_INT_DMA:
    {
        // If read was triggered in interrupt don't bother waiting. The worst that could happen is that we pick
        // up an old value.

        // This data was read from the gyro, which is the same SPI device as the acc
        uint16_t *accData = (uint16_t *)acc->gyro->dev.rxBuf;
        acc->ADCRaw[X] = __builtin_bswap16(accData[1]);
        acc->ADCRaw[Y] = __builtin_bswap16(accData[2]);
        acc->ADCRaw[Z] = __builtin_bswap16(accData[3]);
        break;
    }

    case GYRO_EXTI_INIT:
    default:
        break;
    }

    return true;
}

static bool mpu6000GyroRead(gyroDev_t *gyro)
{
    uint16_t *gyroData = (uint16_t *)gyro->dev.rxBuf;
    switch (gyro->gyroModeSPI) {
    case GYRO_EXTI_INIT:
    {
        // Initialise the tx buffer to all 0xff
        memset(gyro->dev.txBuf, 0xff, 16);
#ifdef USE_GYRO_EXTI
        // Check that minimum number of interrupts have been detected

        // We need some offset from the gyro interrupts to ensure sampling after the interrupt
        gyro->gyroDmaMaxDuration = 5;
        PifMpu60x0 *mpu = mpu6000Find(&gyro->dev);
        if (mpu && gyro->detectedEXTI > GYRO_EXTI_DETECT_THRESHOLD) {
            if (spiUseDMA(&gyro->dev)) {
                gyro->dev.txBuf[0] = MPU60X0_REG_ACCEL_XOUT_H | 0x80;
                pifSpiDevice_AttachEvtTransferDone(mpu->_p_spi, mpu6000TransferDone, gyro);
                gyro->gyroModeSPI = GYRO_EXTI_INT_DMA;
            } else {
                // Interrupts are present, but no DMA
                gyro->gyroModeSPI = GYRO_EXTI_INT;
            }
        } else
#endif
        {
            gyro->gyroModeSPI = GYRO_EXTI_NO_INT;
        }
        break;
    }

    case GYRO_EXTI_INT:
    case GYRO_EXTI_NO_INT:
    {
        PifMpu60x0 *mpu = mpu6000Find(&gyro->dev);
        if (!mpu) {
            return false;
        }

        gyro->dev.txBuf[0] = MPU60X0_REG_GYRO_XOUT_H | 0x80;

        // Waits for completion
        pifSpiDevice_Transfer(mpu->_p_spi, gyro->dev.txBuf, &gyro->dev.rxBuf[1], 7);

        gyro->gyroADCRaw[X] = __builtin_bswap16(gyroData[1]);
        gyro->gyroADCRaw[Y] = __builtin_bswap16(gyroData[2]);
        gyro->gyroADCRaw[Z] = __builtin_bswap16(gyroData[3]);
        break;
    }

    case GYRO_EXTI_INT_DMA:
    {
        // Acc and gyro data are not continuous (temperature is in between)

        // If read was triggered in interrupt don't bother waiting. The worst that could happen is that we pick
        // up an old value.
        gyro->gyroADCRaw[X] = __builtin_bswap16(gyroData[MPU6000_BURST_GYRO_INDEX]);
        gyro->gyroADCRaw[Y] = __builtin_bswap16(gyroData[MPU6000_BURST_GYRO_INDEX + 1]);
        gyro->gyroADCRaw[Z] = __builtin_bswap16(gyroData[MPU6000_BURST_GYRO_INDEX + 2]);
        break;
    }

    default:
        break;
    }

    return true;
}

static void mpu6000AccAndGyroInit(PifMpu60x0 *mpu, gyroDev_t *gyro)
{
    // Device was already reset by pifMpu60x0Spi_Init() so proceed with configuration

    // Clock Source PPL with Z axis gyro reference
    mpu6000RegisterWrite(mpu, MPU60X0_REG_PWR_MGMT_1, MPU60X0_CLKSEL_PLL_ZGYRO);

    // Disable Primary I2C Interface
    mpu6000RegisterWrite(mpu, MPU60X0_REG_USER_CTRL, MPU60X0_I2C_IF_DIS(1));

    mpu6000RegisterWrite(mpu, MPU60X0_REG_PWR_MGMT_2, 0x00);

    // Accel Sample Rate 1kHz
    // Gyroscope Output Rate =  1kHz when the DLPF is enabled
    mpu6000RegisterWrite(mpu, MPU60X0_REG_SMPLRT_DIV, gyro->mpuDividerDrops);

    // Gyro +/- 2000 DPS Full Scale
    pifMpu60x0_SetGyroConfig(mpu, MPU60X0_FS_SEL_2000DPS);
    delayMicroseconds(15);

    // Accel +/- 16 G Full Scale
    pifMpu60x0_SetAccelConfig(mpu, MPU60X0_AFS_SEL_16G);
    delayMicroseconds(15);

    mpu6000RegisterWrite(mpu, MPU60X0_REG_INT_PIN_CFG, MPU60X0_INT_RD_CLEAR(1));  // INT_ANYRD_2CLEAR

#ifdef USE_MPU_DATA_READY_SIGNAL
    mpu6000RegisterWrite(mpu, MPU60X0_REG_INT_ENABLE, MPU60X0_DATA_RDY_EN(1));
#endif
}

static void mpu6000SpiGyroInit(gyroDev_t *gyro)
{
    PifMpu60x0 *mpu = mpu6000Find(&gyro->dev);
    if (!mpu) {
        failureMode(FAILURE_GYRO_INIT_FAILED);
        return;
    }

#ifdef USE_GYRO_EXTI
    mpu6000IntExtiInit(gyro);
#endif

    mpu6000AccAndGyroInit(mpu, gyro);

    // Accel and Gyro DLPF Setting
    (mpu->_fn.write_byte)(mpu->_fn.p_device, MPU60X0_REG_CONFIG, mpuGyroDLPF(gyro));
    delayMicroseconds(1);

    spiSetClkDivisor(&gyro->dev, spiCalculateDivider(MPU6000_MAX_SPI_CLK_HZ));

    pifMpu60x0_ReadGyro(mpu, gyro->gyroADCRaw);

    if (((int8_t)gyro->gyroADCRaw[1]) == -1 && ((int8_t)gyro->gyroADCRaw[0]) == -1) {
        failureMode(FAILURE_GYRO_INIT_FAILED);
    }
}

static void mpu6000SpiAccInit(accDev_t *acc)
{
    acc->acc_1G = 512 * 4;
}

uint8_t mpu6000SpiDetect(const extDevice_t *dev)
{
    // pif_mpu60x0 waits on pif's 1 ms clock, which only advances once
    // pifLinker_Init() has succeeded.
    if (!pifLinker_IsReady()) {
        return MPU_NONE;
    }

    PifSpiPort *port = mpu6000PifPort(dev);
    if (!port) {
        return MPU_NONE;
    }

    // The reset that used to come first here, and the signal path reset after
    // it, are done by pifMpu60x0Spi_Init() in mpu6000SpiGyroDetect().
    const bool detected = pifMpu60x0Spi_Detect(port, (extDevice_t *)dev);
    delayMicroseconds(1); // Ensure CS high time is met which is violated on H7 without this delay

    return detected ? MPU_60x0_SPI : MPU_NONE;
}

bool mpu6000SpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != MPU_60x0_SPI) {
        return false;
    }

    acc->initFn = mpu6000SpiAccInit;
    acc->readFn = mpu6000AccRead;

    return true;
}

bool mpu6000SpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != MPU_60x0_SPI) {
        return false;
    }

    // Adds the chip to the PifSpiPort of its bus and resets it. Done here
    // rather than in initFn so that running out of PifSpiDevice slots fails
    // detection instead of leaving a gyro that cannot be read.
    PifSpiPort *port = mpu6000PifPort(&gyro->dev);
    PifMpu60x0 *mpu = mpu6000FindFree(&gyro->dev);
    if (!port || !mpu || !pifMpu60x0Spi_Init(mpu, PIF_ID_AUTO, port, &gyro->dev, &g_imu_sensor)) {
        return false;
    }

    gyro->initFn = mpu6000SpiGyroInit;
    gyro->readFn = mpu6000GyroRead;
    gyro->scale = GYRO_SCALE_2000DPS;
#ifdef USE_GYRO_EXTI
    gyro->gyroShortPeriod = clockMicrosToCycles(MPU6000_SHORT_THRESHOLD);
#endif
    return true;
}

#endif
