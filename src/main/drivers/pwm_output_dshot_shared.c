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


#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_DSHOT

#include "build/debug.h"

#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/rcc.h"
#include "drivers/time.h"
#include "drivers/timer.h"
#if defined(STM32F4)
#include "stm32f4xx.h"
#elif defined(STM32F3)
#include "stm32f30x.h"
#endif

#include "pwm_output.h"
#include "drivers/dshot.h"
#include "drivers/dshot_dpwm.h"
#include "drivers/dshot_command.h"
#include "drivers/motor.h"

#include "pwm_output_dshot_shared.h"

FAST_DATA_ZERO_INIT uint8_t dmaMotorTimerCount = 0;
#ifdef STM32F7
FAST_DATA_ZERO_INIT motorDmaTimer_t dmaMotorTimers[MAX_DMA_TIMERS];
FAST_DATA_ZERO_INIT motorDmaOutput_t dmaMotors[MAX_SUPPORTED_MOTORS];
#else
motorDmaTimer_t dmaMotorTimers[MAX_DMA_TIMERS];
motorDmaOutput_t dmaMotors[MAX_SUPPORTED_MOTORS];
#endif

#ifdef USE_DSHOT_TELEMETRY
FAST_DATA_ZERO_INIT uint32_t inputStampUs;

FAST_DATA_ZERO_INIT dshotDMAHandlerCycleCounters_t dshotDMAHandlerCycleCounters;
#endif

motorDmaOutput_t *getMotorDmaOutput(uint8_t index)
{
    return &dmaMotors[index];
}

uint8_t getTimerIndex(TIM_TypeDef *timer)
{
    for (int i = 0; i < dmaMotorTimerCount; i++) {
        if (dmaMotorTimers[i].timer == timer) {
            return i;
        }
    }
    dmaMotorTimers[dmaMotorTimerCount++].timer = timer;
    return dmaMotorTimerCount - 1;
}


FAST_CODE void pwmWriteDshotInt(uint8_t index, uint16_t value)
{
    // Only noted here: the frames of every motor are built together in
    // pifDshot_Update(), where a queued command may take the place of this.
    pifDshot_SetThrottle(&dshotPif, index, value);
}

FAST_CODE void pwmWriteDshotFrames(PifDshot *pOwner, const uint16_t *pFrames, uint8_t count)
{
    UNUSED(pOwner);

    for (int index = 0; index < count; index++) {
        motorDmaOutput_t *const motor = &dmaMotors[index];

        if (!motor->configured) {
            continue;
        }

        const uint16_t packet = pFrames[index];
        uint8_t bufferSize;

#ifdef USE_DSHOT_DMAR
        if (useBurstDshot) {
            bufferSize = loadDmaBuffer(&motor->timer->dmaBurstBuffer[timerLookupChannelIndex(motor->timerHardware->channel)], 4, packet);
            motor->timer->dmaBurstLength = bufferSize * 4;
        } else
#endif
        {
            bufferSize = loadDmaBuffer(motor->dmaBuffer, 1, packet);
            motor->timer->timerDmaSources |= motor->timerDmaSource;
#ifdef USE_FULL_LL_DRIVER
            xLL_EX_DMA_SetDataLength(motor->dmaRef, bufferSize);
            xLL_EX_DMA_EnableResource(motor->dmaRef);
#else
            xDMA_SetCurrDataCounter(motor->dmaRef, bufferSize);
            xDMA_Cmd(motor->dmaRef, ENABLE);
#endif
        }
    }
}


#ifdef USE_DSHOT_TELEMETRY

void dshotEnableChannels(uint8_t motorCount);


#endif

#ifdef USE_DSHOT_TELEMETRY
FAST_CODE_NOINLINE bool pwmStartDshotMotorUpdate(void)
{
    if (!useDshotTelemetry) {
        return true;
    }
    const timeUs_t currentUs = micros();
    for (int i = 0; i < dshotPwmDevice.count; i++) {
        timeDelta_t usSinceInput = cmpTimeUs(currentUs, inputStampUs);
        if (usSinceInput >= 0 && usSinceInput < dmaMotors[i].dshotTelemetryDeadtimeUs) {
            return false;
        }
        if (dmaMotors[i].isInput) {
#ifdef USE_FULL_LL_DRIVER
            uint32_t edges = GCR_TELEMETRY_INPUT_LEN - xLL_EX_DMA_GetDataLength(dmaMotors[i].dmaRef);
#else
            uint32_t edges = GCR_TELEMETRY_INPUT_LEN - xDMA_GetCurrDataCounter(dmaMotors[i].dmaRef);
#endif

#ifdef USE_FULL_LL_DRIVER
            LL_EX_TIM_DisableIT(dmaMotors[i].timerHardware->tim, dmaMotors[i].timerDmaSource);
#else
            TIM_DMACmd(dmaMotors[i].timerHardware->tim, dmaMotors[i].timerDmaSource, DISABLE);
#endif

            if (edges > MIN_GCR_EDGES) {
                // The input capture timer counts 16 ticks per telemetry bit.
                const uint32_t gcr = pifDshot_EdgesToGcr(dmaMotors[i].dmaBuffer, edges, 16);
                if (!dshotTelemetryReceive(i, gcr) && i == 0) {
                    memcpy(dshotTelemetryState.inputBuffer, dmaMotors[i].dmaBuffer, sizeof(dshotTelemetryState.inputBuffer));
                }
            }
        }
        pwmDshotSetDirectionOutput(&dmaMotors[i]);
    }
    inputStampUs = 0;
    dshotEnableChannels(dshotPwmDevice.count);
    return true;
}

bool isDshotMotorTelemetryActive(uint8_t motorIndex)
{
    return dshotTelemetryState.motorState[motorIndex].telemetryActive;
}

bool isDshotTelemetryActive(void)
{
    const unsigned motorCount = motorDeviceCount();
    if (motorCount) {
        for (unsigned i = 0; i < motorCount; i++) {
            if (!isDshotMotorTelemetryActive(i)) {
                return false;
            }
        }
        return true;
    }
    return false;
}

#ifdef USE_DSHOT_TELEMETRY_STATS
int16_t getDshotTelemetryMotorInvalidPercent(uint8_t motorIndex)
{
    int16_t invalidPercent = 0;

    if (dshotTelemetryState.motorState[motorIndex].telemetryActive) {
        const uint32_t totalCount = dshotTelemetryQuality[motorIndex].packetCountSum;
        const uint32_t invalidCount = dshotTelemetryQuality[motorIndex].invalidCountSum;
        if (totalCount > 0) {
            invalidPercent = lrintf(invalidCount * 10000.0f / totalCount);
        }
    } else {
        invalidPercent = 10000;  // 100.00%
    }
    return invalidPercent;
}
#endif // USE_DSHOT_TELEMETRY_STATS
#endif // USE_DSHOT_TELEMETRY
#endif // USE_DSHOT
