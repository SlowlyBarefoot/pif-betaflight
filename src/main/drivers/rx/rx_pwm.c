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

#if defined(USE_PWM) || defined(USE_PPM)

#include "build/build_config.h"
#include "build/debug.h"

#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/pwm_output.h"
#include "drivers/timer.h"

#include "pg/rx_pwm.h"

#include "rx_pwm.h"

#include "pif/pif_linker.h"

#include "rc/pif_rc_ppm.h"

#define PPM_CAPTURE_COUNT 12

#if PPM_CAPTURE_COUNT > PWM_INPUT_PORT_COUNT
#define PWM_PORTS_OR_PPM_CAPTURE_COUNT PPM_CAPTURE_COUNT
#else
#define PWM_PORTS_OR_PPM_CAPTURE_COUNT PWM_INPUT_PORT_COUNT
#endif

// TODO - change to timer clocks ticks
#define INPUT_FILTER_TO_HELP_WITH_NOISE_FROM_OPENLRS_TELEMETRY_RX 0x03

static inputFilteringMode_e inputFilteringMode;

void pwmICConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t polarity);

typedef enum {
    INPUT_MODE_PPM,
    INPUT_MODE_PWM
} pwmInputMode_e;

typedef struct {
    pwmInputMode_e mode;
    uint8_t channel; // only used for pwm, ignored by ppm

    uint8_t state;
    uint8_t missedEvents;

    captureCompare_t rise;
    captureCompare_t fall;
    captureCompare_t capture;

    const timerHardware_t *timerHardware;
    timerCCHandlerRec_t edgeCb;
    timerOvrHandlerRec_t overflowCb;
} pwmInputPort_t;

static pwmInputPort_t pwmInputPorts[PWM_INPUT_PORT_COUNT];

static uint16_t captures[PWM_PORTS_OR_PPM_CAPTURE_COUNT];

#define PPM_TIMER_PERIOD 0x10000
#define PWM_TIMER_PERIOD 0x10000

static uint8_t ppmFrameCount = 0;
static uint8_t lastPPMFrameCount = 0;
static uint8_t ppmCountDivisor = 1;

#define PPM_IN_MIN_SYNC_PULSE_US    2700    // microseconds
#define PPM_IN_MIN_CHANNEL_PULSE_US 750     // microseconds
#define PPM_IN_MAX_CHANNEL_PULSE_US 2250    // microseconds
#define PPM_IN_MAX_NUM_CHANNELS     PWM_PORTS_OR_PPM_CAPTURE_COUNT

// The frame is decoded by pif_rc_ppm. This side only turns the 16-bit timer
// captures into the 32-bit microsecond timestamps pifRcPpm_sigTick() takes;
// PIF uses nothing but the difference between consecutive timestamps.
static PifRcPpm ppmRc;
static uint32_t ppmTimeUs;
static uint32_t ppmPreviousCapture;
static uint32_t ppmTimerPeriod = PPM_TIMER_PERIOD;
static uint8_t ppmOverflowCount;

bool isPPMDataBeingReceived(void)
{
    return (ppmFrameCount != lastPPMFrameCount);
}

void resetPPMDataReceivedState(void)
{
    lastPPMFrameCount = ppmFrameCount;
}

// Called from the timer ISR once pif_rc_ppm has the last channel of a frame.
// rxFrameCheck() picks the new frame up through isPPMDataBeingReceived().
static void ppmEvtReceive(PifRc *rc, uint16_t *channel, PifIssuerP issuer)
{
    UNUSED(issuer);

    int i;
    for (i = 0; i < rc->_channel_count && i < PPM_IN_MAX_NUM_CHANNELS; i++) {
        captures[i] = channel[i];
    }
    for (; i < PPM_IN_MAX_NUM_CHANNELS; i++) {
        captures[i] = PPM_RCVR_TIMEOUT;
    }
    ppmFrameCount++;
}

static bool ppmResetDevice(void)
{
    pifRcPpm_Clear(&ppmRc);
    if (!pifRcPpm_Init(&ppmRc, PIF_ID_AUTO, PPM_IN_MAX_NUM_CHANNELS, PPM_IN_MIN_SYNC_PULSE_US)) {
        return false;
    }
    pifRcPpm_SetValidRange(&ppmRc, PPM_IN_MIN_CHANNEL_PULSE_US, PPM_IN_MAX_CHANNEL_PULSE_US);
    pifRc_AttachEvtReceive(&ppmRc.parent, ppmEvtReceive, NULL);

    ppmTimeUs = 0;
    ppmPreviousCapture = 0;
    ppmTimerPeriod = PPM_TIMER_PERIOD;
    ppmOverflowCount = 0;
    return true;
}

static void ppmOverflowCallback(timerOvrHandlerRec_t* cbRec, captureCompare_t capture)
{
    UNUSED(cbRec);

    // capture is ARR, which differs from PPM_TIMER_PERIOD when the timer is
    // shared with the motor outputs.
    ppmTimerPeriod = (uint32_t)capture + 1;
    if (ppmOverflowCount < UINT8_MAX) {
        ppmOverflowCount++;
    }
}

static void ppmEdgeCallback(timerCCHandlerRec_t* cbRec, captureCompare_t capture)
{
    UNUSED(cbRec);

    uint32_t deltaTicks;

    // The update interrupt may be handled after this edge even when the
    // counter wrapped first, so the wrap is taken from the captures and the
    // overflow count only tells a gap of more than a whole period apart.
    if (ppmOverflowCount >= 2) {
        deltaTicks = UINT32_MAX / 2;
    } else if (capture >= ppmPreviousCapture) {
        deltaTicks = capture - ppmPreviousCapture;
    } else {
        deltaTicks = ppmTimerPeriod - ppmPreviousCapture + capture;
    }
    ppmPreviousCapture = capture;
    ppmOverflowCount = 0;

    // Divide value if Oneshot, Multishot or brushed motors are active and the timer is shared
    ppmTimeUs += deltaTicks / ppmCountDivisor;

    pifRcPpm_sigTick(&ppmRc, ppmTimeUs);
}

#define MAX_MISSED_PWM_EVENTS 10

bool isPWMDataBeingReceived(void)
{
    int channel;
    for (channel = 0; channel < PWM_PORTS_OR_PPM_CAPTURE_COUNT; channel++) {
        if (captures[channel] != PPM_RCVR_TIMEOUT) {
            return true;
        }
    }
    return false;
}

static void pwmOverflowCallback(timerOvrHandlerRec_t* cbRec, captureCompare_t capture)
{
    UNUSED(capture);
    pwmInputPort_t *pwmInputPort = container_of(cbRec, pwmInputPort_t, overflowCb);

    if (++pwmInputPort->missedEvents > MAX_MISSED_PWM_EVENTS) {
        captures[pwmInputPort->channel] = PPM_RCVR_TIMEOUT;
        pwmInputPort->missedEvents = 0;
    }
}

static void pwmEdgeCallback(timerCCHandlerRec_t *cbRec, captureCompare_t capture)
{
    pwmInputPort_t *pwmInputPort = container_of(cbRec, pwmInputPort_t, edgeCb);
    const timerHardware_t *timerHardwarePtr = pwmInputPort->timerHardware;

    if (pwmInputPort->state == 0) {
        pwmInputPort->rise = capture;
        pwmInputPort->state = 1;
#if defined(USE_HAL_DRIVER)
        pwmICConfig(timerHardwarePtr->tim, timerHardwarePtr->channel, TIM_ICPOLARITY_FALLING);
#else
        pwmICConfig(timerHardwarePtr->tim, timerHardwarePtr->channel, TIM_ICPolarity_Falling);
#endif
    } else {
        pwmInputPort->fall = capture;

        // compute and store capture
        pwmInputPort->capture = pwmInputPort->fall - pwmInputPort->rise;
        captures[pwmInputPort->channel] = pwmInputPort->capture;

        // switch state
        pwmInputPort->state = 0;
#if defined(USE_HAL_DRIVER)
        pwmICConfig(timerHardwarePtr->tim, timerHardwarePtr->channel, TIM_ICPOLARITY_RISING);
#else
        pwmICConfig(timerHardwarePtr->tim, timerHardwarePtr->channel, TIM_ICPolarity_Rising);
#endif
        pwmInputPort->missedEvents = 0;
    }
}

#ifdef USE_HAL_DRIVER

void pwmICConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t polarity)
{
    TIM_HandleTypeDef* Handle = timerFindTimerHandle(tim);
    if (Handle == NULL) return;

    TIM_IC_InitTypeDef TIM_ICInitStructure;

    TIM_ICInitStructure.ICPolarity = polarity;
    TIM_ICInitStructure.ICSelection = TIM_ICSELECTION_DIRECTTI;
    TIM_ICInitStructure.ICPrescaler = TIM_ICPSC_DIV1;

    if (inputFilteringMode == INPUT_FILTERING_ENABLED) {
        TIM_ICInitStructure.ICFilter = INPUT_FILTER_TO_HELP_WITH_NOISE_FROM_OPENLRS_TELEMETRY_RX;
    } else {
        TIM_ICInitStructure.ICFilter = 0x00;
    }

    HAL_TIM_IC_ConfigChannel(Handle, &TIM_ICInitStructure, channel);
    HAL_TIM_IC_Start_IT(Handle,channel);
}
#else
void pwmICConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t polarity)
{
    TIM_ICInitTypeDef TIM_ICInitStructure;

    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = channel;
    TIM_ICInitStructure.TIM_ICPolarity = polarity;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;

    if (inputFilteringMode == INPUT_FILTERING_ENABLED) {
        TIM_ICInitStructure.TIM_ICFilter = INPUT_FILTER_TO_HELP_WITH_NOISE_FROM_OPENLRS_TELEMETRY_RX;
    } else {
        TIM_ICInitStructure.TIM_ICFilter = 0x00;
    }

    TIM_ICInit(tim, &TIM_ICInitStructure);
}
#endif

void pwmRxInit(const pwmConfig_t *pwmConfig)
{
    inputFilteringMode = pwmConfig->inputFilteringMode;

    for (int channel = 0; channel < PWM_INPUT_PORT_COUNT; channel++) {

        pwmInputPort_t *port = &pwmInputPorts[channel];

        const timerHardware_t *timer = timerAllocate(pwmConfig->ioTags[channel], OWNER_PWMINPUT, RESOURCE_INDEX(channel));

        if (!timer) {
            /* TODO: maybe fail here if not enough channels? */
            continue;
        }

        port->state = 0;
        port->missedEvents = 0;
        port->channel = channel;
        port->mode = INPUT_MODE_PWM;
        port->timerHardware = timer;

        IO_t io = IOGetByTag(pwmConfig->ioTags[channel]);
        IOInit(io, OWNER_PWMINPUT, RESOURCE_INDEX(channel));
#ifdef STM32F1
        IOConfigGPIO(io, IOCFG_IPD);
#else
        IOConfigGPIOAF(io, IOCFG_AF_PP, timer->alternateFunction);
#endif
        timerConfigure(timer, (uint16_t)PWM_TIMER_PERIOD, PWM_TIMER_1MHZ);
        timerChCCHandlerInit(&port->edgeCb, pwmEdgeCallback);
        timerChOvrHandlerInit(&port->overflowCb, pwmOverflowCallback);
        timerChConfigCallbacks(timer, &port->edgeCb, &port->overflowCb);

#if defined(USE_HAL_DRIVER)
        pwmICConfig(timer->tim, timer->channel, TIM_ICPOLARITY_RISING);
#else
        pwmICConfig(timer->tim, timer->channel, TIM_ICPolarity_Rising);
#endif

    }
}

#define FIRST_PWM_PORT 0

#ifdef USE_PWM_OUTPUT
void ppmAvoidPWMTimerClash(TIM_TypeDef *pwmTimer)
{
    pwmOutputPort_t *motors = pwmGetMotors();
    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS; motorIndex++) {
        if (!motors[motorIndex].enabled || motors[motorIndex].channel.tim != pwmTimer) {
            continue;
        }

        ppmCountDivisor = timerClock(pwmTimer) / (pwmTimer->PSC + 1);
        return;
    }
}
#endif

void ppmRxInit(const ppmConfig_t *ppmConfig)
{
    if (!ppmResetDevice()) {
        return;
    }

    pwmInputPort_t *port = &pwmInputPorts[FIRST_PWM_PORT];

    const timerHardware_t *timer = timerAllocate(ppmConfig->ioTag, OWNER_PPMINPUT, 0);
    if (!timer) {
        /* TODO: fail here? */
        return;
    }

#ifdef USE_PWM_OUTPUT
    ppmAvoidPWMTimerClash(timer->tim);
#endif

    port->mode = INPUT_MODE_PPM;
    port->timerHardware = timer;

    IO_t io = IOGetByTag(ppmConfig->ioTag);
    IOInit(io, OWNER_PPMINPUT, 0);
#ifdef STM32F1
    IOConfigGPIO(io, IOCFG_IPD);
#else
    IOConfigGPIOAF(io, IOCFG_AF_PP, timer->alternateFunction);
#endif

    timerConfigure(timer, (uint16_t)PPM_TIMER_PERIOD, PWM_TIMER_1MHZ);
    timerChCCHandlerInit(&port->edgeCb, ppmEdgeCallback);
    timerChOvrHandlerInit(&port->overflowCb, ppmOverflowCallback);
    timerChConfigCallbacks(timer, &port->edgeCb, &port->overflowCb);

#if defined(USE_HAL_DRIVER)
    pwmICConfig(timer->tim, timer->channel, TIM_ICPOLARITY_RISING);
#else
    pwmICConfig(timer->tim, timer->channel, TIM_ICPolarity_Rising);
#endif
}

uint16_t ppmRead(uint8_t channel)
{
    return captures[channel];
}

uint16_t pwmRead(uint8_t channel)
{
    return captures[channel];
}
#endif
