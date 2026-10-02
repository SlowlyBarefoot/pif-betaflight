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

#if defined(USE_BARO) && (defined(USE_BARO_MS5611) || defined(USE_BARO_SPI_MS5611))

#include "build/build_config.h"

#include "barometer.h"
#include "barometer_ms5611.h"

#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/bus_i2c_pif.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/time.h"

#include "pif/pif_linker.h"

#include "sensor/pif_ms5611.h"

// 10 MHz max SPI frequency
#define MS5611_MAX_SPI_CLK_HZ 10000000

// MS5611, Standard address 0x77, which is MS5611_I2C_ADDR(1) of pif_ms5611
// (CSB low; GY-86 boards have it so)

#ifdef USE_BARO_SPI_MS5611
// PIF has no SPI transport for the MS5611, so on SPI the chip is still driven
// here, through TASK_BARO and the start/read/get functions of baroDev_t.

#define CMD_RESET               0x1E // ADC reset command
#define CMD_ADC_READ            0x00 // ADC read command
#define CMD_ADC_CONV            0x40 // ADC conversion command
#define CMD_ADC_D1              0x00 // ADC D1 conversion
#define CMD_ADC_D2              0x10 // ADC D2 conversion
#define CMD_ADC_256             0x00 // ADC OSR=256
#define CMD_ADC_512             0x02 // ADC OSR=512
#define CMD_ADC_1024            0x04 // ADC OSR=1024
#define CMD_ADC_2048            0x06 // ADC OSR=2048
#define CMD_ADC_4096            0x08 // ADC OSR=4096
#define CMD_PROM_RD             0xA0 // Prom read command
#define PROM_NB                 8

STATIC_UNIT_TESTED uint32_t ms5611_ut;  // static result of temperature measurement
STATIC_UNIT_TESTED uint32_t ms5611_up;  // static result of pressure measurement
STATIC_UNIT_TESTED uint16_t ms5611_c[PROM_NB];  // on-chip ROM
static uint8_t ms5611_osr = CMD_ADC_4096;
#define MS5611_DATA_FRAME_SIZE 3
static DMA_DATA_ZERO_INIT uint8_t sensor_data[MS5611_DATA_FRAME_SIZE];

static void ms5611BusInit(const extDevice_t *dev)
{
    IOHi(dev->busType_u.spi.csnPin); // Disable
    IOInit(dev->busType_u.spi.csnPin, OWNER_BARO_CS, 0);
    IOConfigGPIO(dev->busType_u.spi.csnPin, IOCFG_OUT_PP);
    spiSetClkDivisor(dev, spiCalculateDivider(MS5611_MAX_SPI_CLK_HZ));
}

static void ms5611BusDeinit(const extDevice_t *dev)
{
    spiPreinitByIO(dev->busType_u.spi.csnPin);
}

static void ms5611Reset(const extDevice_t *dev)
{
    busRawWriteRegister(dev, CMD_RESET, 1);

    delayMicroseconds(2800);
}

static uint16_t ms5611Prom(const extDevice_t *dev, int8_t coef_num)
{
    uint8_t rxbuf[2] = { 0, 0 };

    busRawReadRegisterBuffer(dev, CMD_PROM_RD + coef_num * 2, rxbuf, 2); // send PROM READ command

    return rxbuf[0] << 8 | rxbuf[1];
}

STATIC_UNIT_TESTED int8_t ms5611CRC(uint16_t *prom)
{
    int32_t i, j;
    uint32_t res = 0;
    uint8_t crc = prom[7] & 0xF;
    prom[7] &= 0xFF00;

    bool blankEeprom = true;

    for (i = 0; i < 16; i++) {
        if (prom[i >> 1]) {
            blankEeprom = false;
        }
        if (i & 1)
            res ^= ((prom[i >> 1]) & 0x00FF);
        else
            res ^= (prom[i >> 1] >> 8);
        for (j = 8; j > 0; j--) {
            if (res & 0x8000)
                res ^= 0x1800;
            res <<= 1;
        }
    }
    prom[7] |= crc;
    if (!blankEeprom && crc == ((res >> 12) & 0xF))
        return 0;

    return -1;
}

static void ms5611ReadAdc(const extDevice_t *dev)
{
    busRawReadRegisterBufferStart(dev, CMD_ADC_READ, sensor_data, MS5611_DATA_FRAME_SIZE); // read ADC
}

static void ms5611StartUT(baroDev_t *baro)
{
    busRawWriteRegisterStart(&baro->dev, CMD_ADC_CONV + CMD_ADC_D2 + ms5611_osr, 1); // D2 (temperature) conversion start!
}

static bool ms5611ReadUT(baroDev_t *baro)
{
    if (busBusy(&baro->dev, NULL)) {
        return false;
    }

    ms5611ReadAdc(&baro->dev);

    return true;
}

static bool ms5611GetUT(baroDev_t *baro)
{
    if (busBusy(&baro->dev, NULL)) {
        return false;
    }

    ms5611_ut = sensor_data[0] << 16 | sensor_data[1] << 8 | sensor_data[2];

    return true;
}

static void ms5611StartUP(baroDev_t *baro)
{
    busRawWriteRegisterStart(&baro->dev, CMD_ADC_CONV + CMD_ADC_D1 + ms5611_osr, 1); // D1 (pressure) conversion start!
}

static bool ms5611ReadUP(baroDev_t *baro)
{
    if (busBusy(&baro->dev, NULL)) {
        return false;
    }

    ms5611ReadAdc(&baro->dev);

    return true;
}

static bool ms5611GetUP(baroDev_t *baro)
{
    if (busBusy(&baro->dev, NULL)) {
        return false;
    }

    ms5611_up = sensor_data[0] << 16 | sensor_data[1] << 8 | sensor_data[2];

    return true;
}

STATIC_UNIT_TESTED void ms5611Calculate(int32_t *pressure, int32_t *temperature)
{
    uint32_t press;
    int64_t temp;
    int64_t delt;
    int64_t dT = (int64_t)ms5611_ut - ((uint64_t)ms5611_c[5] * 256);
    int64_t off = ((int64_t)ms5611_c[2] << 16) + (((int64_t)ms5611_c[4] * dT) >> 7);
    int64_t sens = ((int64_t)ms5611_c[1] << 15) + (((int64_t)ms5611_c[3] * dT) >> 8);
    temp = 2000 + ((dT * (int64_t)ms5611_c[6]) >> 23);

    if (temp < 2000) { // temperature lower than 20degC
        delt = temp - 2000;
        delt = 5 * delt * delt;
        off -= delt >> 1;
        sens -= delt >> 2;
        if (temp < -1500) { // temperature lower than -15degC
            delt = temp + 1500;
            delt = delt * delt;
            off -= 7 * delt;
            sens -= (11 * delt) >> 1;
        }
    temp -= ((dT * dT) >> 31);
    }
    press = ((((int64_t)ms5611_up * sens) >> 21) - off) >> 15;


    if (pressure)
        *pressure = press;
    if (temperature)
        *temperature = temp;
}

static bool ms5611SpiDetect(baroDev_t *baro)
{
    uint8_t sig;
    int i;

    extDevice_t *dev = &baro->dev;

    ms5611BusInit(dev);

    if (!busRawReadRegisterBuffer(dev, CMD_PROM_RD, &sig, 1) || sig == 0xFF) {
        goto fail;
    }

    ms5611Reset(dev);

    // read all coefficients
    for (i = 0; i < PROM_NB; i++)
        ms5611_c[i] = ms5611Prom(dev, i);

    // check crc, bail out if wrong - we are probably talking to BMP085 w/o XCLR line!
    if (ms5611CRC(ms5611_c) != 0) {
        goto fail;
    }

    busDeviceRegister(dev);

    // TODO prom + CRC
    baro->ut_delay = 10000;
    baro->up_delay = 10000;
    baro->start_ut = ms5611StartUT;
    baro->read_ut = ms5611ReadUT;
    baro->get_ut = ms5611GetUT;
    baro->start_up = ms5611StartUP;
    baro->read_up = ms5611ReadUP;
    baro->get_up = ms5611GetUP;
    baro->calculate = ms5611Calculate;

    return true;

fail:;
    ms5611BusDeinit(dev);

    return false;
}
#endif // USE_BARO_SPI_MS5611

#ifdef USE_BARO_MS5611
// On I2C the chip is driven by PIF's pif_ms5611. pifMs5611_Init() resets it
// and reads and checks the PROM coefficients. The samples are then read by a
// PIF task of pif_ms5611's own, with 4096 times oversampling as the native
// driver used, and handed to baro->evt_read, so TASK_BARO and the
// start/read/get functions of baroDev_t are not used for it.
static PifMs5611 ms5611;

// Read period of the PIF task: about 33 Hz. The native driver took about
// 34 ms a sample through TASK_BARO (10 ms for each conversion and between
// samples, plus the 1 ms steps around them), and the two 11 ms conversions of
// pif_ms5611 fit in this.
#define MS5611_READ_PERIOD_MS       30

static bool ms5611I2cDetect(baroDev_t *baro)
{
    extDevice_t *dev = &baro->dev;
    bool defaultAddressApplied = false;
    uint8_t sig;

    // pifMs5611_Init() waits 100 ms on pif's 1 ms clock after the reset, which
    // only advances once pifLinker_Init() has succeeded, and the samples only
    // reach Betaflight through evt_read.
    if (!pifLinker_IsReady() || !baro->evt_read) {
        return false;
    }

    PifI2cPort *port = i2cPifPort(dev->bus->busType_u.i2c.device);
    if (!port) {
        return false;
    }

    if (dev->busType_u.i2c.address == 0) {
        // Default address for MS5611
        dev->busType_u.i2c.address = MS5611_I2C_ADDR(1);
        defaultAddressApplied = true;
    }

    // Something has to answer the PROM read before the reset command goes out,
    // as the native driver checked, so another chip at the address is left alone.
    PifI2cDevice *probe = pifI2cPort_TemporaryDevice(port, dev->busType_u.i2c.address, NULL);
    if (!pifI2cDevice_ReadRegBytes(probe, MS5611_REG_READ_PROM, &sig, 1) || sig == 0xFF) {
        goto fail;
    }

    // Resets the chip and checks the PROM CRC, which fails on a BMP085 w/o XCLR line.
    if (ms5611._p_i2c) {
        pifMs5611_Clear(&ms5611);
    }
    if (!pifMs5611_Init(&ms5611, PIF_ID_AUTO, port, dev->busType_u.i2c.address, NULL)) {
        goto fail;
    }
    pifMs5611_SetOverSamplingRate(&ms5611, MS5611_OSR_4096);

    busDeviceRegister(dev);

    if (!pifMs5611_AttachTaskForReading(&ms5611, PIF_ID_AUTO, MS5611_READ_PERIOD_MS, baro->evt_read, TRUE)) {
        pifMs5611_Clear(&ms5611);
        goto fail;
    }

    return true;

fail:
    if (defaultAddressApplied) {
        dev->busType_u.i2c.address = 0;
    }

    return false;
}
#endif // USE_BARO_MS5611

bool ms5611Detect(baroDev_t *baro)
{
    delay(10); // No idea how long the chip takes to power-up, but let's make it 10ms

    switch (baro->dev.bus->busType) {
#ifdef USE_BARO_MS5611
    case BUS_TYPE_I2C:
        return ms5611I2cDetect(baro);
#endif
#ifdef USE_BARO_SPI_MS5611
    case BUS_TYPE_SPI:
        return ms5611SpiDetect(baro);
#endif
    default:
        return false;
    }
}
#endif
