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

#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"

#include "axis.h"
#include "maths.h"

#include "core/pif_math.h"

#if defined(FAST_MATH) || defined(VERY_FAST_MATH)

// The approximations are PIF's pif_math. Errors measured against libm:
// sin/cos 3e-7, atan2 3e-7 rad, acos 5e-7 rad, exp 3e-7 relative,
// log 1e-6 * max(1, |log(x)|).

float sin_approx(float x)
{
    return pifMath_SinApprox(x);
}

float cos_approx(float x)
{
    return pifMath_CosApprox(x);
}

float atan2_approx(float y, float x)
{
    return pifMath_Atan2Approx(y, x);
}

float acos_approx(float x)
{
    return pifMath_AcosApprox(x);
}

float exp_approx(float val)
{
    return pifMath_ExpApprox(val);
}

float log_approx(float val)
{
    return pifMath_LogApprox(val);
}

float pow_approx(float a, float b)
{
    return pifMath_PowApprox(a, b);
}
#endif

int gcd(int num, int denom)
{
    if (denom == 0) {
        return num;
    }

    return gcd(denom, num % denom);
}

int32_t applyDeadband(const int32_t value, const int32_t deadband)
{
    return pifMath_Deadband(value, deadband);
}

float fapplyDeadband(const float value, const float deadband)
{
    return pifMath_DeadbandF(value, deadband);
}

void devClear(stdev_t *dev)
{
    dev->m_n = 0;
}

void devPush(stdev_t *dev, float x)
{
    dev->m_n++;
    if (dev->m_n == 1) {
        dev->m_oldM = dev->m_newM = x;
        dev->m_oldS = 0.0f;
    } else {
        dev->m_newM = dev->m_oldM + (x - dev->m_oldM) / dev->m_n;
        dev->m_newS = dev->m_oldS + (x - dev->m_oldM) * (x - dev->m_newM);
        dev->m_oldM = dev->m_newM;
        dev->m_oldS = dev->m_newS;
    }
}

float devVariance(stdev_t *dev)
{
    return ((dev->m_n > 1) ? dev->m_newS / (dev->m_n - 1) : 0.0f);
}

float devStandardDeviation(stdev_t *dev)
{
    return sqrtf(devVariance(dev));
}

float degreesToRadians(int16_t degrees)
{
    return degrees * RAD;
}

int scaleRange(int x, int srcFrom, int srcTo, int destFrom, int destTo) {
    return pifMath_ScaleRange(x, srcFrom, srcTo, destFrom, destTo);
}

float scaleRangef(float x, float srcFrom, float srcTo, float destFrom, float destTo) {
    return pifMath_ScaleRangeF(x, srcFrom, srcTo, destFrom, destTo);
}

void buildRotationMatrix(fp_angles_t *delta, fp_rotationMatrix_t *rotation)
{
    float cosx, sinx, cosy, siny, cosz, sinz;
    float coszcosx, sinzcosx, coszsinx, sinzsinx;

    cosx = cos_approx(delta->angles.roll);
    sinx = sin_approx(delta->angles.roll);
    cosy = cos_approx(delta->angles.pitch);
    siny = sin_approx(delta->angles.pitch);
    cosz = cos_approx(delta->angles.yaw);
    sinz = sin_approx(delta->angles.yaw);

    coszcosx = cosz * cosx;
    sinzcosx = sinz * cosx;
    coszsinx = sinx * cosz;
    sinzsinx = sinx * sinz;

    rotation->m[0][X] = cosz * cosy;
    rotation->m[0][Y] = -cosy * sinz;
    rotation->m[0][Z] = siny;
    rotation->m[1][X] = sinzcosx + (coszsinx * siny);
    rotation->m[1][Y] = coszcosx - (sinzsinx * siny);
    rotation->m[1][Z] = -sinx * cosy;
    rotation->m[2][X] = (sinzsinx) - (coszcosx * siny);
    rotation->m[2][Y] = (coszsinx) + (sinzcosx * siny);
    rotation->m[2][Z] = cosy * cosx;
}

void applyMatrixRotation(float *v, fp_rotationMatrix_t *rotationMatrix)
{
    struct fp_vector *vDest = (struct fp_vector *)v;
    struct fp_vector vTmp = *vDest;

    vDest->X = (rotationMatrix->m[0][X] * vTmp.X + rotationMatrix->m[1][X] * vTmp.Y + rotationMatrix->m[2][X] * vTmp.Z);
    vDest->Y = (rotationMatrix->m[0][Y] * vTmp.X + rotationMatrix->m[1][Y] * vTmp.Y + rotationMatrix->m[2][Y] * vTmp.Z);
    vDest->Z = (rotationMatrix->m[0][Z] * vTmp.X + rotationMatrix->m[1][Z] * vTmp.Y + rotationMatrix->m[2][Z] * vTmp.Z);
}

// Median of 3 to 9 samples, by PIF's pif_math.
int32_t quickMedianFilter3(int32_t * v)
{
    return pifMath_MedianInt32(v, 3);
}

int32_t quickMedianFilter5(int32_t * v)
{
    return pifMath_MedianInt32(v, 5);
}

int32_t quickMedianFilter7(int32_t * v)
{
    return pifMath_MedianInt32(v, 7);
}

int32_t quickMedianFilter9(int32_t * v)
{
    return pifMath_MedianInt32(v, 9);
}

float quickMedianFilter3f(float * v)
{
    return pifMath_MedianFloat(v, 3);
}

float quickMedianFilter5f(float * v)
{
    return pifMath_MedianFloat(v, 5);
}

float quickMedianFilter7f(float * v)
{
    return pifMath_MedianFloat(v, 7);
}

float quickMedianFilter9f(float * v)
{
    return pifMath_MedianFloat(v, 9);
}

void arraySubInt32(int32_t *dest, int32_t *array1, int32_t *array2, int count)
{
    for (int i = 0; i < count; i++) {
        dest[i] = array1[i] - array2[i];
    }
}

int16_t qPercent(fix12_t q) {
    return (100 * q) >> 12;
}

int16_t qMultiply(fix12_t q, int16_t input) {
    return (input *  q) >> 12;
}

fix12_t  qConstruct(int16_t num, int16_t den) {
    return (num << 12) / den;
}
