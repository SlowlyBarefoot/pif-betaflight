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

#if defined(USE_ADC_INTERNAL)

#include "build/debug.h"

#include "common/utils.h"

#include "drivers/adc.h"

#include "sensors/adcinternal.h"

#include "sensor/pif_adc.h"

// The internal reference and the temperature sensor, read through PIF's
// pif_adc: it filters them, works out the supply from the reference and the
// temperature from the factory calibration adcInternalSetupPif() gives it.
#define ADC_INTERNAL_VREFINT        0
#define ADC_INTERNAL_TEMPSENSOR     1
#define ADC_INTERNAL_CHANNEL_COUNT  2

// Low-pass over about 2^3 = 8 samples, as the moving average of 8 it replaces.
#define ADC_INTERNAL_FILTER_SHIFT   3

static PifAdc adcInternal;
static bool adcInternalReady;

static int16_t coreTemperature;
static uint16_t vrefMv;

uint16_t getVrefMv(void)
{
#ifdef ADC_VOLTAGE_REFERENCE_MV
    return ADC_VOLTAGE_REFERENCE_MV;
#else
    return vrefMv;
#endif
}

int16_t getCoreTemperatureCelsius(void)
{
    return coreTemperature;
}

static uint16_t adcInternalRead(PifAdc *adc, uint8_t channel)
{
    UNUSED(adc);

    return channel == ADC_INTERNAL_VREFINT ? adcInternalReadVrefint() : adcInternalReadTempsensor();
}

static void adcInternalStart(PifAdc *adc)
{
    UNUSED(adc);

    adcInternalStartConversion();
}

static void adcInternalUpdate(void)
{
    // pifAdc_Sample() starts the next conversion when it is done.
    pifAdc_Sample(&adcInternal);

    vrefMv = adcInternal._vref_mv;
    const int32_t decidegrees = pifAdc_GetValue(&adcInternal, ADC_INTERNAL_TEMPSENSOR);
    coreTemperature = (decidegrees + (decidegrees < 0 ? -5 : 5)) / 10;

    // The other channels are converted against the supply measured here.
    adcSetReferenceMv(getVrefMv());

    DEBUG_SET(DEBUG_ADC_INTERNAL, 0, coreTemperature);
    DEBUG_SET(DEBUG_ADC_INTERNAL, 1, pifAdc_GetRaw(&adcInternal, ADC_INTERNAL_VREFINT));
    DEBUG_SET(DEBUG_ADC_INTERNAL, 2, pifAdc_GetRaw(&adcInternal, ADC_INTERNAL_TEMPSENSOR));
    DEBUG_SET(DEBUG_ADC_INTERNAL, 3, vrefMv);
}

uint32_t adcInternalProcess(PifTask *p_task)
{
    UNUSED(p_task);

    if (!adcInternalReady || adcInternalIsBusy()) {
        return 0;
    }

    adcInternalUpdate();
    return 0;
}

void adcInternalInit(void)
{
    if (!pifAdc_Init(&adcInternal, PIF_ID_AUTO, ADC_INTERNAL_CHANNEL_COUNT, 12, 3300, adcInternalRead)
        || !adcInternalSetupPif(&adcInternal, ADC_INTERNAL_VREFINT, ADC_INTERNAL_TEMPSENSOR)
        || !pifAdc_SetFilter(&adcInternal, ADC_INTERNAL_FILTER_SHIFT)) {
        return;
    }

    // Starts the first conversion, and the filters start at the first sample,
    // so one finished conversion is all there is to wait for.
    pifAdc_AttachActStart(&adcInternal, adcInternalStart);
    while (adcInternalIsBusy()) {
        // empty
    }
    adcInternalUpdate();
    adcInternalReady = true;
}
#else
uint16_t getVrefMv(void)
{
#ifdef ADC_VOLTAGE_REFERENCE_MV
    return ADC_VOLTAGE_REFERENCE_MV;
#else
    return 3300;
#endif
}
#endif
