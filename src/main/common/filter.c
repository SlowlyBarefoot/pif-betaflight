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
#include <string.h>
#include <math.h>

#include "platform.h"

#include "common/filter.h"
#include "common/maths.h"
#include "common/utils.h"

// PT1, PT2 and PT3 low pass filters, by PIF's pif_pt_filter

static void ptFilterInit(PifPtFilter *filter, uint8_t order, float k)
{
    pifPtFilter_Init(filter, order, 0.0f, 1.0f);
    pifPtFilter_SetGain(filter, k);
}

// NULL filter

FAST_CODE float nullFilterApply(filter_t *filter, float input)
{
    UNUSED(filter);
    return input;
}


// PT1 Low Pass filter

float pt1FilterGain(float f_cut, float dT)
{
    return pifPtFilter_Gain(1, f_cut, dT);
}

void pt1FilterInit(pt1Filter_t *filter, float k)
{
    ptFilterInit(filter, 1, k);
}

void pt1FilterUpdateCutoff(pt1Filter_t *filter, float k)
{
    pifPtFilter_SetGain(filter, k);
}

FAST_CODE float pt1FilterApply(pt1Filter_t *filter, float input)
{
    return pifPtFilter_Apply(filter, input);
}

// PT2 Low Pass filter

float pt2FilterGain(float f_cut, float dT)
{
    return pifPtFilter_Gain(2, f_cut, dT);
}

void pt2FilterInit(pt2Filter_t *filter, float k)
{
    ptFilterInit(filter, 2, k);
}

void pt2FilterUpdateCutoff(pt2Filter_t *filter, float k)
{
    pifPtFilter_SetGain(filter, k);
}

FAST_CODE float pt2FilterApply(pt2Filter_t *filter, float input)
{
    return pifPtFilter_Apply(filter, input);
}

// PT3 Low Pass filter

float pt3FilterGain(float f_cut, float dT)
{
    return pifPtFilter_Gain(3, f_cut, dT);
}

void pt3FilterInit(pt3Filter_t *filter, float k)
{
    ptFilterInit(filter, 3, k);
}

void pt3FilterUpdateCutoff(pt3Filter_t *filter, float k)
{
    pifPtFilter_SetGain(filter, k);
}

FAST_CODE float pt3FilterApply(pt3Filter_t *filter, float input)
{
    return pifPtFilter_Apply(filter, input);
}


// Slew filter with limit

void slewFilterInit(slewFilter_t *filter, float slewLimit, float threshold)
{
    filter->state = 0.0f;
    filter->slewLimit = slewLimit;
    filter->threshold = threshold;
}

FAST_CODE float slewFilterApply(slewFilter_t *filter, float input)
{
    if (filter->state >= filter->threshold) {
        if (input >= filter->state - filter->slewLimit) {
            filter->state = input;
        }
    } else if (filter->state <= -filter->threshold) {
        if (input <= filter->state + filter->slewLimit) {
            filter->state = input;
        }
    } else {
        filter->state = input;
    }
    return filter->state;
}

// get notch filter Q given center frequency (f0) and lower cutoff frequency (f1)
float filterGetNotchQ(float centerFreq, float cutoffFreq)
{
    return pifBiquadFilter_NotchQ(centerFreq, cutoffFreq);
}

// Biquad filters, by PIF's pif_biquad_filter. refreshRate is the sample period in us.

static PifBiquadFilterType biquadPifType(biquadFilterType_e filterType)
{
    switch (filterType) {
    case FILTER_NOTCH:
        return BQFT_NOTCH;
    case FILTER_BPF:
        return BQFT_BANDPASS;
    case FILTER_LPF:
    default:
        return BQFT_LOWPASS;
    }
}

/* sets up a biquad filter as a 2nd order butterworth LPF */
void biquadFilterInitLPF(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate)
{
    biquadFilterInit(filter, filterFreq, refreshRate, PIF_BIQUAD_Q_BUTTERWORTH, FILTER_LPF, 1.0f);
}

// A frequency at or above Nyquist leaves the filter passing the input through.
void biquadFilterInit(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate, float Q, biquadFilterType_e filterType, float weight)
{
    pifBiquadFilter_Init(&filter->pif, biquadPifType(filterType), filterFreq, 1e6f / refreshRate, Q);
    filter->weight = weight;
}

// A frequency at or above Nyquist keeps the previous coefficients.
FAST_CODE void biquadFilterUpdate(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate, float Q, biquadFilterType_e filterType, float weight)
{
    pifBiquadFilter_Update(&filter->pif, biquadPifType(filterType), filterFreq, 1e6f / refreshRate, Q);
    filter->weight = weight;
}

FAST_CODE void biquadFilterUpdateLPF(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate)
{
    biquadFilterUpdate(filter, filterFreq, refreshRate, PIF_BIQUAD_Q_BUTTERWORTH, FILTER_LPF, 1.0f);
}

FAST_CODE float biquadFilterApplyDF1(biquadFilter_t *filter, float input)
{
    return pifBiquadFilter_Apply(&filter->pif, input);
}

/* Computes a biquadFilter_t filter in df1 and crossfades input with output */
FAST_CODE float biquadFilterApplyDF1Weighted(biquadFilter_t* filter, float input)
{
    const float result = pifBiquadFilter_Apply(&filter->pif, input);

    // crossfading of input and output to turn filter on/off gradually
    return filter->weight * result + (1 - filter->weight) * input;
}

// PIF runs every biquad in direct form I, which also copes with coefficient changes.
FAST_CODE float biquadFilterApply(biquadFilter_t *filter, float input)
{
    return pifBiquadFilter_Apply(&filter->pif, input);
}

// Moving average, by PIF's pif_moving_average. Until the window is full, the
// average is over the samples added so far.

void laggedMovingAverageInit(laggedMovingAverage_t *filter, uint16_t windowSize, float *buf)
{
    pifMovingAverage_Init(filter, buf, windowSize);
}

FAST_CODE float laggedMovingAverageUpdate(laggedMovingAverage_t *filter, float input)
{
    return pifMovingAverage_Apply(filter, input);
}

// Simple fixed-point lowpass filter based on integer math

int32_t simpleLPFilterUpdate(simpleLowpassFilter_t *filter, int32_t newVal)
{
    filter->fp = (filter->fp << filter->beta) - filter->fp;
    filter->fp += newVal << filter->fpShift;
    filter->fp >>= filter->beta;
    int32_t result = filter->fp >> filter->fpShift;
    return result;
}

void simpleLPFilterInit(simpleLowpassFilter_t *filter, int32_t beta, int32_t fpShift)
{
    filter->fp = 0;
    filter->beta = beta;
    filter->fpShift = fpShift;
}
