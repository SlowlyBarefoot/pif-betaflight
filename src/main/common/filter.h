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

#pragma once
#include <stdbool.h>

#include "filter/pif_biquad_filter.h"
#include "filter/pif_moving_average.h"
#include "filter/pif_pt_filter.h"

struct filter_s;
typedef struct filter_s filter_t;

// PT1, PT2 and PT3 are PIF's PifPtFilter of order 1, 2 and 3. Use
// pifPtFilter_Output() and pifPtFilter_Reset() to read or set the output.
typedef PifPtFilter pt1Filter_t;
typedef PifPtFilter pt2Filter_t;
typedef PifPtFilter pt3Filter_t;

typedef struct slewFilter_s {
    float state;
    float slewLimit;
    float threshold;
} slewFilter_t;

// A PIF biquad (direct form I) plus the weight that
// biquadFilterApplyDF1Weighted() crossfades the output with the input by.
typedef struct biquadFilter_s {
    PifBiquadFilter pif;
    float weight;
} biquadFilter_t;

typedef PifMovingAverage laggedMovingAverage_t;

typedef enum {
    FILTER_PT1 = 0,
    FILTER_BIQUAD,
    FILTER_PT2,
    FILTER_PT3,
} lowpassFilterType_e;

typedef enum {
    FILTER_LPF,    // 2nd order Butterworth section
    FILTER_NOTCH,
    FILTER_BPF,
} biquadFilterType_e;

typedef float (*filterApplyFnPtr)(filter_t *filter, float input);

float nullFilterApply(filter_t *filter, float input);

void biquadFilterInitLPF(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate);
void biquadFilterInit(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate, float Q, biquadFilterType_e filterType, float weight);
void biquadFilterUpdate(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate, float Q, biquadFilterType_e filterType, float weight);
void biquadFilterUpdateLPF(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate);

float biquadFilterApplyDF1(biquadFilter_t *filter, float input);
float biquadFilterApplyDF1Weighted(biquadFilter_t *filter, float input);
float biquadFilterApply(biquadFilter_t *filter, float input);
float filterGetNotchQ(float centerFreq, float cutoffFreq);

void laggedMovingAverageInit(laggedMovingAverage_t *filter, uint16_t windowSize, float *buf);
float laggedMovingAverageUpdate(laggedMovingAverage_t *filter, float input);

float pt1FilterGain(float f_cut, float dT);
void pt1FilterInit(pt1Filter_t *filter, float k);
void pt1FilterUpdateCutoff(pt1Filter_t *filter, float k);
float pt1FilterApply(pt1Filter_t *filter, float input);

float pt2FilterGain(float f_cut, float dT);
void pt2FilterInit(pt2Filter_t *filter, float k);
void pt2FilterUpdateCutoff(pt2Filter_t *filter, float k);
float pt2FilterApply(pt2Filter_t *filter, float input);

float pt3FilterGain(float f_cut, float dT);
void pt3FilterInit(pt3Filter_t *filter, float k);
void pt3FilterUpdateCutoff(pt3Filter_t *filter, float k);
float pt3FilterApply(pt3Filter_t *filter, float input);

void slewFilterInit(slewFilter_t *filter, float slewLimit, float threshold);
float slewFilterApply(slewFilter_t *filter, float input);

typedef struct simpleLowpassFilter_s {
    int32_t fp;
    int32_t beta;
    int32_t fpShift;
} simpleLowpassFilter_t;

int32_t simpleLPFilterUpdate(simpleLowpassFilter_t *filter, int32_t newVal);
void simpleLPFilterInit(simpleLowpassFilter_t *filter, int32_t beta, int32_t fpShift);
