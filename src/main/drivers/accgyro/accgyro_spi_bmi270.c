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
#include <string.h>

#include "platform.h"

#ifdef USE_ACCGYRO_BMI270

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_spi_bmi270.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/nvic.h"
#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "sensors/gyro.h"

#include "sensor/pif_bmi270_spi.h"

// 10 MHz max SPI frequency
#define BMI270_MAX_SPI_CLK_HZ 10000000

#define BMI270_FIFO_FRAME_SIZE 6

// Need to see at least this many interrupts during initialisation to confirm EXTI connectivity
#define GYRO_EXTI_DETECT_THRESHOLD 1000

static PifBmi270 bmi270;

// Toggle the CS to switch the device into SPI mode.
// Device switches initializes as I2C and switches to SPI on a low to high CS transition
static void bmi270EnableSPI(const extDevice_t *dev)
{
    IOLo(dev->busType_u.spi.csnPin);
    delay(1);
    IOHi(dev->busType_u.spi.csnPin);
    delay(10);
}

uint8_t bmi270Detect(const extDevice_t *dev)
{
    bmi270EnableSPI(dev);

    if (pifBmi270Spi_Detect(dev->busType_u.spi.p_spi_port, (extDevice_t *)dev)) {
        return BMI_270_SPI;
    }

    return MPU_NONE;
}

static uint8_t getBmiOsrMode()
{
    switch(gyroConfig()->gyro_hardware_lpf) {
        case GYRO_HARDWARE_LPF_NORMAL:
            return BMI270_GC_GYR_BWP_OSR4;
        case GYRO_HARDWARE_LPF_OPTION_1:
            return BMI270_GC_GYR_BWP_OSR2;
        case GYRO_HARDWARE_LPF_OPTION_2:
            return BMI270_GC_GYR_BWP_NORM;
        case GYRO_HARDWARE_LPF_EXPERIMENTAL:
            return BMI270_GC_GYR_BWP_NORM;
    }
    return 0;
}

static void bmi270Config(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;

    // If running in hardware_lpf experimental mode then switch to FIFO-based,
    // 6.4KHz sampling, unfiltered data vs. the default 3.2KHz with hardware filtering
#ifdef USE_GYRO_DLPF_EXPERIMENTAL
    const bool fifoMode = (gyro->hardware_lpf == GYRO_HARDWARE_LPF_EXPERIMENTAL);
#else
    const bool fifoMode = false;
#endif

    pifBmi270Spi_Init(&bmi270, PIF_ID_AUTO, dev->busType_u.spi.p_spi_port, dev, &g_imu_sensor);

    // Toggle the chip into SPI mode
    bmi270EnableSPI(dev);

    pifBmi270_UploadConfig(&bmi270);

    // Configure the FIFO
    if (fifoMode) {
        (bmi270._fn.write_byte)(bmi270._p_spi, BMI270_REG_FIFO_CONFIG_0, 0);
        pif_Delay1ms(1);
        (bmi270._fn.write_byte)(bmi270._p_spi, BMI270_REG_FIFO_CONFIG_1, BMI270_FC1_FIFO_GYR_EN(1));
        pif_Delay1ms(1);
        (bmi270._fn.write_byte)(bmi270._p_spi, BMI270_REG_FIFO_DOWNS, 0);
        pif_Delay1ms(1);
        (bmi270._fn.write_byte)(bmi270._p_spi, BMI270_REG_FIFO_WTM_0, 0x06);
        pif_Delay1ms(1);
        (bmi270._fn.write_byte)(bmi270._p_spi, BMI270_REG_FIFO_WTM_1, 0x00);
        pif_Delay1ms(1);
    }

    // Configure the accelerometer
    (bmi270._fn.write_byte)(bmi270._p_spi, BMI270_REG_ACC_CONF, BMI270_AC_ACC_FILTER_PERF_HP | BMI270_AC_ACC_BWP_NORM_AVG4 | BMI270_AC_ACC_ODR_800);
	pif_Delay1ms(1);

    // Configure the accelerometer full-scale range
    (bmi270._fn.write_byte)(bmi270._p_spi, BMI270_REG_ACC_RANGE, BMI270_AR_ACC_RANGE_16G);
	pif_Delay1ms(1);

    // Configure the gyro
    (bmi270._fn.write_byte)(bmi270._p_spi, BMI270_REG_GYR_CONF, BMI270_GC_GYR_FILTER_PERF_HP | BMI270_GC_GYR_NOISE_PERF_HP | getBmiOsrMode() | BMI270_GC_GYR_ODR_3K2);
	pif_Delay1ms(1);

    // Configure the gyro full-range scale
    (bmi270._fn.write_byte)(bmi270._p_spi, BMI270_REG_GYR_RANGE, BMI270_GR_OIS_RANGE_2000);
	pif_Delay1ms(1);

    // Configure the gyro data ready interrupt
    if (fifoMode) {
        // Interrupt driven by FIFO watermark level
        (bmi270._fn.write_byte)(bmi270._p_spi, BMI270_REG_INT_MAP_DATA, BMI270_IMD_FWM_INT1(1));
    } else {
        // Interrupt driven by data ready
        (bmi270._fn.write_byte)(bmi270._p_spi, BMI270_REG_INT_MAP_DATA, BMI270_IMD_DRDY_INT1(1));
    }
	pif_Delay1ms(1);

    // Configure the behavior of the INT1 pin
    (bmi270._fn.write_byte)(bmi270._p_spi, BMI270_REG_INT1_IO_CTRL, BMI270_IIC_OUTPUT_EN(1) | BMI270_IIC_LVL_ACTIVE_HIGH);
	pif_Delay1ms(1);

    // Configure the device for  performance mode
    (bmi270._fn.write_byte)(bmi270._p_spi, BMI270_REG_PWR_CONF, BMI270_PC_FIFO_SELF_WAKE_UP(1));
	pif_Delay1ms(1);

    // Enable the gyro, accelerometer and temperature sensor - disable aux interface
    (bmi270._fn.write_byte)(bmi270._p_spi, BMI270_REG_PWR_CTRL, BMI270_PC_GYR_EN(1) | BMI270_PC_ACC_EN(1) |BMI270_PC_TEMP_EN(1));
	pif_Delay1ms(1);

    // Flush the FIFO
    if (fifoMode) {
        (bmi270._fn.write_byte)(bmi270._p_spi, BMI270_REG_CMD, BMI270_C_CMD_FIFO_FLUSH);
        pif_Delay1ms(1);
    }
}

extiCallbackRec_t bmi270IntCallbackRec;

/*
 * Gyro interrupt service routine
 */
#ifdef USE_GYRO_EXTI
// Called in ISR context
// Gyro read has just completed
busStatus_e bmi270Intcallback(uint32_t arg)
{
    gyroDev_t *gyro = (gyroDev_t *)arg;
    int32_t gyroDmaDuration = cmpTimeCycles(getCycleCounter(), gyro->gyroLastEXTI);

    if (gyroDmaDuration > gyro->gyroDmaMaxDuration) {
        gyro->gyroDmaMaxDuration = gyroDmaDuration;
    }

    gyro->dataReady = true;

    return BUS_READY;
}

void bmi270ExtiHandler(extiCallbackRec_t *cb)
{
    gyroDev_t *gyro = container_of(cb, gyroDev_t, exti);
    // Ideally we'd use a timer to capture such information, but unfortunately the port used for EXTI interrupt does
    // not have an associated timer
    uint32_t nowCycles = getCycleCounter();
    gyro->gyroSyncEXTI = gyro->gyroLastEXTI + gyro->gyroDmaMaxDuration;
    gyro->gyroLastEXTI = nowCycles;

    if (gyro->gyroModeSPI == GYRO_EXTI_INT_DMA) {
        gyro->dev.segments = gyro->segments;

        pifSpiDevice_Transfer(bmi270._p_spi, NULL, NULL, 0);
    }

    gyro->detectedEXTI++;

}

static void bmi270IntExtiInit(gyroDev_t *gyro)
{
    if (gyro->mpuIntExtiTag == IO_TAG_NONE) {
        return;
    }

    IO_t mpuIntIO = IOGetByTag(gyro->mpuIntExtiTag);

    IOInit(mpuIntIO, OWNER_GYRO_EXTI, 0);
    EXTIHandlerInit(&gyro->exti, bmi270ExtiHandler);
    EXTIConfig(mpuIntIO, &gyro->exti, NVIC_PRIO_MPU_INT_EXTI, IOCFG_IN_FLOATING, BETAFLIGHT_EXTI_TRIGGER_RISING);
    EXTIEnable(mpuIntIO);
}
#else
void bmi270ExtiHandler(extiCallbackRec_t *cb)
{
    gyroDev_t *gyro = container_of(cb, gyroDev_t, exti);
    gyro->dataReady = true;
}
#endif

static bool bmi270AccRead(accDev_t *acc)
{
    switch (acc->gyro->gyroModeSPI) {
    case GYRO_EXTI_INT:
    case GYRO_EXTI_NO_INT:
    {
        acc->gyro->dev.txBuf[0] = BMI270_REG_ACC_X_LSB | 0x80;

        pifSpiDevice_Transfer(bmi270._p_spi, acc->gyro->dev.txBuf, acc->gyro->dev.rxBuf, 8);

        // Fall through
        FALLTHROUGH;
    }

    case GYRO_EXTI_INT_DMA:
    {
        // If read was triggered in interrupt don't bother waiting. The worst that could happen is that we pick
        // up an old value.

        // This data was read from the gyro, which is the same SPI device as the acc
        uint16_t *accData = (uint16_t *)acc->gyro->dev.rxBuf;
        acc->ADCRaw[X] = accData[1];
        acc->ADCRaw[Y] = accData[2];
        acc->ADCRaw[Z] = accData[3];
        break;
    }

    case GYRO_EXTI_INIT:
    default:
        break;
    }

    return true;
}

static bool bmi270GyroReadRegister(gyroDev_t *gyro)
{
    uint16_t *gyroData = (uint16_t *)gyro->dev.rxBuf;
    switch (gyro->gyroModeSPI) {
    case GYRO_EXTI_INIT:
    {
        // Initialise the tx buffer to all 0x00
        memset(gyro->dev.txBuf, 0x00, 14);
#ifdef USE_GYRO_EXTI
        // Check that minimum number of interrupts have been detected

        // We need some offset from the gyro interrupts to ensure sampling after the interrupt
        gyro->gyroDmaMaxDuration = 5;
        // Using DMA for gyro access upsets the scheduler on the F4
        if (gyro->detectedEXTI > GYRO_EXTI_DETECT_THRESHOLD) {
            if (spiUseDMA(&gyro->dev)) {
                gyro->dev.callbackArg = (uint32_t)gyro;
                gyro->dev.txBuf[0] = BMI270_REG_ACC_X_LSB | 0x80;
                gyro->segments[0].len = 14;
                gyro->segments[0].callback = bmi270Intcallback;
                gyro->segments[0].u.buffers.txData = gyro->dev.txBuf;
                gyro->segments[0].u.buffers.rxData = gyro->dev.rxBuf;
                gyro->segments[0].negateCS = true;
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
        gyro->dev.txBuf[0] = BMI270_REG_GYR_X_LSB | 0x80;

        pifSpiDevice_Transfer(bmi270._p_spi, gyro->dev.txBuf, gyro->dev.rxBuf, 8);

        gyro->gyroADCRaw[X] = gyroData[1];
        gyro->gyroADCRaw[Y] = gyroData[2];
        gyro->gyroADCRaw[Z] = gyroData[3];

        break;
    }

    case GYRO_EXTI_INT_DMA:
    {
        // If read was triggered in interrupt don't bother waiting. The worst that could happen is that we pick
        // up an old value.
        gyro->gyroADCRaw[X] = gyroData[4];
        gyro->gyroADCRaw[Y] = gyroData[5];
        gyro->gyroADCRaw[Z] = gyroData[6];
        break;
    }

    default:
        break;
    }

    return true;
}

#ifdef USE_GYRO_DLPF_EXPERIMENTAL
static bool bmi270GyroReadFifo(gyroDev_t *gyro)
{
    enum {
        IDX_REG = 0,
        IDX_SKIP,
        IDX_FIFO_LENGTH_L,
        IDX_FIFO_LENGTH_H,
        IDX_GYRO_XOUT_L,
        IDX_GYRO_XOUT_H,
        IDX_GYRO_YOUT_L,
        IDX_GYRO_YOUT_H,
        IDX_GYRO_ZOUT_L,
        IDX_GYRO_ZOUT_H,
        BUFFER_SIZE,
    };

    bool dataRead = false;
    STATIC_DMA_DATA_AUTO uint8_t bmi270_tx_buf[BUFFER_SIZE] = {BMI270_REG_FIFO_LENGTH_LSB | 0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    STATIC_DMA_DATA_AUTO uint8_t bmi270_rx_buf[BUFFER_SIZE];

    // Burst read the FIFO length followed by the next 6 bytes containing the gyro axis data for
    // the first sample in the queue. It's possible for the FIFO to be empty so we need to check the
    // length before using the sample.
    pifSpiDevice_Transfer(bmi270._p_spi, (uint8_t *)bmi270_tx_buf, bmi270_rx_buf, BUFFER_SIZE);   // receive response

    int fifoLength = (uint16_t)((bmi270_rx_buf[IDX_FIFO_LENGTH_H] << 8) | bmi270_rx_buf[IDX_FIFO_LENGTH_L]);

    if (fifoLength >= BMI270_FIFO_FRAME_SIZE) {

        const int16_t gyroX = (int16_t)((bmi270_rx_buf[IDX_GYRO_XOUT_H] << 8) | bmi270_rx_buf[IDX_GYRO_XOUT_L]);
        const int16_t gyroY = (int16_t)((bmi270_rx_buf[IDX_GYRO_YOUT_H] << 8) | bmi270_rx_buf[IDX_GYRO_YOUT_L]);
        const int16_t gyroZ = (int16_t)((bmi270_rx_buf[IDX_GYRO_ZOUT_H] << 8) | bmi270_rx_buf[IDX_GYRO_ZOUT_L]);

        // If the FIFO data is invalid then the returned values will be 0x8000 (-32768) (pg. 43 of datasheet).
        // This shouldn't happen since we're only using the data if the FIFO length indicates
        // that data is available, but this safeguard is needed to prevent bad things in
        // case it does happen.
        if ((gyroX != INT16_MIN) || (gyroY != INT16_MIN) || (gyroZ != INT16_MIN)) {
            gyro->gyroADCRaw[X] = gyroX;
            gyro->gyroADCRaw[Y] = gyroY;
            gyro->gyroADCRaw[Z] = gyroZ;
            dataRead = true;
        }
        fifoLength -= BMI270_FIFO_FRAME_SIZE;
    }

    // If there are additional samples in the FIFO then we don't use those for now and simply
    // flush the FIFO. Under normal circumstances we only expect one sample in the FIFO since
    // the gyro loop is running at the native sample rate of 6.4KHz.
    // However the way the FIFO works in the sensor is that if a frame is partially read then
    // it remains in the queue instead of bein removed. So if we ever got into a state where there
    // was a partial frame or other unexpected data in the FIFO is may never get cleared and we
    // would end up in a lock state of always re-reading the same partial or invalid sample.
    if (fifoLength > 0) {
        // Partial or additional frames left - flush the FIFO
        (bmi270._fn.write_byte)(bmi270._p_spi, BMI270_REG_CMD, BMI270_C_CMD_FIFO_FLUSH);
    }

    return dataRead;
}
#endif

static bool bmi270GyroRead(gyroDev_t *gyro)
{
#ifdef USE_GYRO_DLPF_EXPERIMENTAL
    if (gyro->hardware_lpf == GYRO_HARDWARE_LPF_EXPERIMENTAL) {
        // running in 6.4KHz FIFO mode
        return bmi270GyroReadFifo(gyro);
    } else
#endif
    {
        // running in 3.2KHz register mode
        return bmi270GyroReadRegister(gyro);
    }
}

static void bmi270SpiGyroInit(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;

    bmi270Config(gyro);

#if defined(USE_GYRO_EXTI)
    bmi270IntExtiInit(gyro);
#endif

    spiSetClkDivisor(dev, spiCalculateDivider(BMI270_MAX_SPI_CLK_HZ));
}

static void bmi270SpiAccInit(accDev_t *acc)
{
    // sensor is configured during gyro init
    acc->acc_1G = 512 * 4;   // 16G sensor scale
}

bool bmi270SpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != BMI_270_SPI) {
        return false;
    }

    acc->initFn = bmi270SpiAccInit;
    acc->readFn = bmi270AccRead;

    return true;
}


bool bmi270SpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != BMI_270_SPI) {
        return false;
    }

    gyro->initFn = bmi270SpiGyroInit;
    gyro->readFn = bmi270GyroRead;
    gyro->scale = GYRO_SCALE_2000DPS;

    return true;
}

// Used to query the status register to determine what event caused the EXTI to fire.
// When in 3.2KHz mode the interrupt is mapped to the data ready state. However the data ready
// trigger will fire for both gyro and accelerometer. So it's necessary to check this register
// to determine which event caused the interrupt.
// When in 6.4KHz mode the interrupt is configured to be the FIFO watermark size of 6 bytes.
// Since in this mode we only put gyro data in the FIFO it's sufficient to check for the FIFO
// watermark reason as an idication of gyro data ready.
uint8_t bmi270InterruptStatus(gyroDev_t *gyro)
{
    uint8_t data[2] = { 0, 0 };

    (void)gyro;

    if ((bmi270._fn.read_bytes)(&bmi270._p_spi, BMI270_REG_INT_STATUS_1 | 0x80, data, 2)) {
        return data[1];
    } else {
        return 0;
    }
}
#endif // USE_ACCGYRO_BMI270
