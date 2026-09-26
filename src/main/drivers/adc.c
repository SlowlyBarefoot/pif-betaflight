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
#include "common/utils.h"

#ifdef USE_ADC

#include "build/build_config.h"
#include "build/debug.h"

#include "drivers/adc_impl.h"
#include "drivers/io.h"

#include "pg/adc.h"

#include "sensor/pif_adc.h"

#include "adc.h"

//#define DEBUG_ADC_CHANNELS

adcOperatingConfig_t adcOperatingConfig[ADC_CHANNEL_COUNT];

#if defined(STM32F7)
volatile FAST_DATA_ZERO_INIT uint16_t adcValues[ADC_CHANNEL_COUNT];
#else
volatile uint16_t adcValues[ADC_CHANNEL_COUNT];
#endif

// Every STM32 ADC is run at 12 bits.
#define ADC_RESOLUTION_BITS 12

#ifdef ADC_VOLTAGE_REFERENCE_MV
#define ADC_DEFAULT_VREF_MV ADC_VOLTAGE_REFERENCE_MV
#else
#define ADC_DEFAULT_VREF_MV 3300
#endif

STATIC_ASSERT(ADC_CHANNEL_COUNT <= PIF_ADC_MAX_CHANNELS, pifAdcChannelCount);

// Until adcPifInit() it has no channels, and reads as 0.
static PifAdc adcPif;

uint8_t adcChannelByTag(ioTag_t ioTag)
{
    for (uint8_t i = 0; i < ARRAYLEN(adcTagMap); i++) {
        if (ioTag == adcTagMap[i].tag)
            return adcTagMap[i].channel;
    }
    return 0;
}

ADCDevice adcDeviceByInstance(ADC_TypeDef *instance)
{
    if (instance == ADC1) {
        return ADCDEV_1;
    }

#if defined(ADC2)
    if (instance == ADC2) {
        return ADCDEV_2;
    }
#endif
#if defined(ADC3)
    if (instance == ADC3) {
        return ADCDEV_3;
    }
#endif
#if defined(ADC4)
    if (instance == ADC4) {
        return ADCDEV_4;
    }
#endif
#if defined(ADC5)
    if (instance == ADC5) {
        return ADCDEV_5;
    }
#endif

    return ADCINVALID;
}

uint16_t adcGetChannel(uint8_t channel)
{
    adcGetChannelValues();

#ifdef DEBUG_ADC_CHANNELS
    if (adcOperatingConfig[0].enabled) {
        debug[0] = adcValues[adcOperatingConfig[0].dmaIndex];
    }
    if (adcOperatingConfig[1].enabled) {
        debug[1] = adcValues[adcOperatingConfig[1].dmaIndex];
    }
    if (adcOperatingConfig[2].enabled) {
        debug[2] = adcValues[adcOperatingConfig[2].dmaIndex];
    }
    if (adcOperatingConfig[3].enabled) {
        debug[3] = adcValues[adcOperatingConfig[3].dmaIndex];
    }
#endif
    return adcValues[adcOperatingConfig[channel].dmaIndex];
}

static uint16_t adcPifRead(PifAdc *adc, uint8_t channel)
{
    UNUSED(adc);

    return adcValues[adcOperatingConfig[channel].dmaIndex];
}

void adcPifInit(void)
{
    pifAdc_Init(&adcPif, PIF_ID_AUTO, ADC_CHANNEL_COUNT, ADC_RESOLUTION_BITS, ADC_DEFAULT_VREF_MV, adcPifRead);
}

uint16_t adcGetMilliVolt(uint8_t channel)
{
    adcGetChannelValues();

    // Converted on demand, one channel at a time: the battery voltage, the
    // current and the RSSI are each read by a task of their own.
    pifAdc_SampleChannel(&adcPif, channel);
    return pifAdc_GetMilliVolt(&adcPif, channel);
}

void adcSetReferenceMv(uint16_t vrefMv)
{
    pifAdc_SetReference(&adcPif, vrefMv);
}

// Verify a pin designated by tag has connection to an ADC instance designated by device

bool adcVerifyPin(ioTag_t tag, ADCDevice device)
{
    if (!tag) {
        return false;
    }

    for (int map = 0 ; map < ADC_TAG_MAP_COUNT ; map++) {
#if defined(STM32F1)
        UNUSED(device);
        if ((adcTagMap[map].tag == tag)) {
            return true;
        }
#else
        if ((adcTagMap[map].tag == tag) && (adcTagMap[map].devices & (1 << device))) {
            return true;
        }
#endif
    }

    return false;
}

#ifdef USE_ADC_INTERNAL

int32_t adcVREFINTCAL;      // ADC value (12-bit) of band gap with Vref = VREFINTCAL_VREF
int32_t adcTSCAL1;
int32_t adcTSCAL2;

bool adcInternalSetupPif(PifAdc *adc, uint8_t vrefintChannel, uint8_t tempsensorChannel)
{
    return pifAdc_SetVrefint(adc, vrefintChannel, adcVREFINTCAL, VREFINT_CAL_VREF)
        && pifAdc_SetTemperature(adc, tempsensorChannel, adcTSCAL1, TEMPSENSOR_CAL1_TEMP * 10,
            adcTSCAL2, TEMPSENSOR_CAL2_TEMP * 10, TEMPSENSOR_CAL_VREFANALOG);
}
#endif // USE_ADC_INTERNAL

#else
uint16_t adcGetChannel(uint8_t channel)
{
    UNUSED(channel);
    return 0;
}

void adcPifInit(void)
{
}

uint16_t adcGetMilliVolt(uint8_t channel)
{
    UNUSED(channel);
    return 0;
}

void adcSetReferenceMv(uint16_t vrefMv)
{
    UNUSED(vrefMv);
}
#endif
