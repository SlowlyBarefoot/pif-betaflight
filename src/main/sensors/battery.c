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

#include "stdbool.h"
#include "stdint.h"
#include "math.h"

#include "platform.h"

#include "build/debug.h"

#include "common/filter.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/config.h"
#include "config/feature.h"

#include "drivers/adc.h"

#include "fc/runtime_config.h"
#include "fc/rc_controls.h"

#include "flight/mixer.h"

#include "io/beeper.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "scheduler/scheduler.h"

#include "sensors/battery.h"

#include "sensor/pif_battery.h"

/**
 * Presence, cell count, voltage state and remaining charge come from PIF's pif_battery, fed with
 * the unfiltered meter voltage. The meters (sources, display and sag filters, mAh drawn), the
 * consumption state, LVC and the beeper stay here.
 *
 * terminology: meter vs sensors
 *
 * voltage and current sensors are used to collect data.
 * - e.g. voltage at an MCU ADC input pin, value from an ESC sensor.
 *   sensors require very specific configuration, such as resistor values.
 * voltage and current meters are used to process and expose data collected from sensors to the rest of the system.
 * - e.g. a meter exposes normalized, and often filtered, values from a sensor.
 *   meters require different or little configuration.
 *   meters also have different precision concerns, and may use different units to the sensors.
 *
 */

#define VBAT_STABLE_MAX_DELTA 20
#define VBAT_SETTLE_MS 500
#define LVC_AFFECT_TIME 10000000 //10 secs for the LVC to slowly kick in

// Battery monitoring stuff
static PifBattery pifBattery;
static bool pifBatteryUpdated;  // pifBattery has had a reading; until then the state is BATTERY_INIT
static lowVoltageCutoff_t lowVoltageCutoff;
//
static currentMeter_t currentMeter;
static voltageMeter_t voltageMeter;

static batteryState_e batteryState;
static batteryState_e voltageState;
static batteryState_e consumptionState;

#ifndef DEFAULT_CURRENT_METER_SOURCE
#ifdef USE_VIRTUAL_CURRENT_METER
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_VIRTUAL
#else
#ifdef USE_MSP_CURRENT_METER
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_MSP
#else
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_NONE
#endif
#endif
#endif

#ifndef DEFAULT_VOLTAGE_METER_SOURCE
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_NONE
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(batteryConfig_t, batteryConfig, PG_BATTERY_CONFIG, 3);

PG_RESET_TEMPLATE(batteryConfig_t, batteryConfig,
    // voltage
    .vbatmaxcellvoltage = VBAT_CELL_VOLTAGE_DEFAULT_MAX,
    .vbatmincellvoltage = VBAT_CELL_VOLTAGE_DEFAULT_MIN,
    .vbatwarningcellvoltage = 350,
    .vbatnotpresentcellvoltage = 300, //A cell below 3 will be ignored
    .voltageMeterSource = DEFAULT_VOLTAGE_METER_SOURCE,
    .lvcPercentage = 100, //Off by default at 100%

    // current
    .batteryCapacity = 0,
    .currentMeterSource = DEFAULT_CURRENT_METER_SOURCE,

    // cells
    .forceBatteryCellCount = 0, //0 will be ignored

    // warnings / alerts
    .useVBatAlerts = true,
    .useConsumptionAlerts = false,
    .consumptionWarningPercentage = 10,
    .vbathysteresis = 1, // 0.01V

    .vbatfullcellvoltage = 410,

    .vbatDisplayLpfPeriod = 30,
    .vbatSagLpfPeriod = 2,
    .ibatLpfPeriod = 10,
    .vbatDurationForWarning = 0,
    .vbatDurationForCritical = 0,
);

uint32_t batteryUpdateVoltage(PifTask *p_task)
{
    UNUSED(p_task);

    switch (batteryConfig()->voltageMeterSource) {
#ifdef USE_ESC_SENSOR
        case VOLTAGE_METER_ESC:
            if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
                voltageMeterESCRefresh();
                voltageMeterESCReadCombined(&voltageMeter);
            }
            break;
#endif
        case VOLTAGE_METER_ADC:
            voltageMeterADCRefresh();
            voltageMeterADCRead(VOLTAGE_SENSOR_ADC_VBAT, &voltageMeter);
            break;

        default:
        case VOLTAGE_METER_NONE:
            voltageMeterReset(&voltageMeter);
            break;
    }

    // Battery presence is only re-evaluated while disarmed: the battery *might* fall out in
    // flight, but if that happens the FC will likely be off too.
    pifBattery_HoldPresence(&pifBattery, ARMING_FLAG(ARMED));
    pifBattery_Update(&pifBattery);
    pifBatteryUpdated = true;

    DEBUG_SET(DEBUG_BATTERY, 0, voltageMeter.unfiltered);
    DEBUG_SET(DEBUG_BATTERY, 1, voltageMeter.displayFiltered);
    return 0;
}

static void updateBatteryBeeperAlert(void)
{
    switch (getBatteryState()) {
        case BATTERY_WARNING:
            beeper(BEEPER_BAT_LOW);

            break;
        case BATTERY_CRITICAL:
            beeper(BEEPER_BAT_CRIT_LOW);

            break;
        case BATTERY_OK:
        case BATTERY_NOT_PRESENT:
        case BATTERY_INIT:
            break;
    }
}

static bool isVoltageFromBat(uint16_t voltage)
{
    // We want to disable battery getting detected around USB voltage or 0V

    return (voltage >= batteryConfig()->vbatnotpresentcellvoltage  // Above ~0V
        && voltage <= batteryConfig()->vbatmaxcellvoltage)  // 1s max cell voltage check
        || voltage > batteryConfig()->vbatnotpresentcellvoltage * 2; // USB voltage - 2s or more check
}

// Pack voltage for pifBattery in mV. Voltages around USB power or 0 V read as no battery.
static int32_t readPifBatteryVoltage(PifBattery *p_owner)
{
    UNUSED(p_owner);
    return isVoltageFromBat(voltageMeter.unfiltered) ? voltageMeter.unfiltered * 10 : 0;
}

static void onPifBatteryState(PifBattery *p_owner, PifBatteryState oldState)
{
    if (oldState == BAS_INIT && p_owner->_state >= BAS_OK) {
        // Battery has just been connected and its cells counted
        lowVoltageCutoff.percentage = 100;
        lowVoltageCutoff.startTime = 0;
        if (batteryConfig()->forceBatteryCellCount == 0 && !ARMING_FLAG(ARMED)) {
            changePidProfileFromCellCount(p_owner->_cell_count);
        }
    }
}

// Thresholds and timing for pifBattery from the configuration, in mV and ms. PIF needs
// min < warning < full <= max, so out-of-order settings are pushed up to keep that.
static void batteryBuildPifConfig(PifBatteryConfig *config)
{
    const batteryConfig_t *bc = batteryConfig();

    config->cell_min_mv = MAX(bc->vbatmincellvoltage, 1) * 10;
    config->cell_warning_mv = MAX(bc->vbatwarningcellvoltage * 10, config->cell_min_mv + 10);
    config->cell_full_mv = MAX(bc->vbatfullcellvoltage * 10, config->cell_warning_mv + 10);
    config->cell_max_mv = MAX(bc->vbatmaxcellvoltage * 10, config->cell_full_mv);
    // vbat_hysteresis is for the pack in 0.01 V; PIF applies it per cell.
    config->hysteresis_mv = bc->vbathysteresis * 10;
    config->present_mv = MAX(bc->vbatnotpresentcellvoltage * 10, 1);
    config->settle_ms = VBAT_SETTLE_MS;
    config->settle_delta_mv = VBAT_STABLE_MAX_DELTA * 10;
    config->voltage_cutoff_hz = GET_BATTERY_LPF_FREQUENCY(bc->vbatDisplayLpfPeriod);
    config->current_cutoff_hz = 0.0f;   // the current meters filter, and mAh comes from them
    config->warning_delay_ms = bc->vbatDurationForWarning * 100;
    config->critical_delay_ms = bc->vbatDurationForCritical * 100;
}

static batteryState_e batteryStateFromPif(void)
{
    if (!pifBatteryUpdated) {
        return BATTERY_INIT;
    }
    switch (pifBattery._state) {
    case BAS_OK:
        return BATTERY_OK;
    case BAS_WARNING:
        return BATTERY_WARNING;
    case BAS_CRITICAL:
        return BATTERY_CRITICAL;
    case BAS_INIT:
        return BATTERY_INIT;
    case BAS_NOT_PRESENT:
    default:
        return BATTERY_NOT_PRESENT;
    }
}

static void batteryUpdateLVC(timeUs_t currentTimeUs)
{
    if (batteryConfig()->lvcPercentage < 100) {
        if (voltageState == BATTERY_CRITICAL && !lowVoltageCutoff.enabled) {
            lowVoltageCutoff.enabled = true;
            lowVoltageCutoff.startTime = currentTimeUs;
            lowVoltageCutoff.percentage = 100;
        }
        if (lowVoltageCutoff.enabled) {
            if (cmp32(currentTimeUs,lowVoltageCutoff.startTime) < LVC_AFFECT_TIME) {
                lowVoltageCutoff.percentage = 100 - (cmp32(currentTimeUs,lowVoltageCutoff.startTime) * (100 - batteryConfig()->lvcPercentage) / LVC_AFFECT_TIME);
            }
            else {
                lowVoltageCutoff.percentage = batteryConfig()->lvcPercentage;
            }
        }
    }

}

static void batteryUpdateConsumptionState(void)
{
    if (batteryConfig()->useConsumptionAlerts && batteryConfig()->batteryCapacity > 0 && getBatteryCellCount() > 0) {
        uint8_t batteryPercentageRemaining = calculateBatteryPercentageRemaining();

        if (batteryPercentageRemaining == 0) {
            consumptionState = BATTERY_CRITICAL;
        } else if (batteryPercentageRemaining <= batteryConfig()->consumptionWarningPercentage) {
            consumptionState = BATTERY_WARNING;
        } else {
            consumptionState = BATTERY_OK;
        }
    }
}

void batteryUpdateStates(timeUs_t currentTimeUs)
{
    const batteryState_e previousVoltageState = voltageState;

    voltageState = batteryStateFromPif();
    if (voltageState >= BATTERY_NOT_PRESENT) {
        consumptionState = voltageState;
    } else if (previousVoltageState >= BATTERY_NOT_PRESENT) {
        consumptionState = BATTERY_OK;
    }
    batteryUpdateConsumptionState();
    batteryUpdateLVC(currentTimeUs);
    batteryState = MAX(voltageState, consumptionState);
}

const lowVoltageCutoff_t *getLowVoltageCutoff(void)
{
    return &lowVoltageCutoff;
}

batteryState_e getBatteryState(void)
{
    return batteryState;
}

batteryState_e getVoltageState(void)
{
    return voltageState;
}

batteryState_e getConsumptionState(void)
{
    return consumptionState;
}

const char * const batteryStateStrings[] = {"OK", "WARNING", "CRITICAL", "NOT PRESENT", "INIT"};

const char * getBatteryStateString(void)
{
    return batteryStateStrings[getBatteryState()];
}

void batteryInit(void)
{
    //
    // presence
    //
    batteryState = BATTERY_INIT;

    PifBatteryConfig pifConfig;
    batteryBuildPifConfig(&pifConfig);
    if (!pifBattery_Init(&pifBattery, PIF_ID_AUTO, &pifConfig, readPifBatteryVoltage)) {
        pifBattery_Init(&pifBattery, PIF_ID_AUTO, &pif_battery_lipo, readPifBatteryVoltage);
    }
    pifBattery_SetCellCount(&pifBattery, batteryConfig()->forceBatteryCellCount);
    pifBattery_SetCapacity(&pifBattery, batteryConfig()->batteryCapacity);
    pifBattery.evt_state = onPifBatteryState;
    pifBatteryUpdated = false;

    //
    // voltage
    //
    voltageState = BATTERY_INIT;
    lowVoltageCutoff.enabled = false;
    lowVoltageCutoff.percentage = 100;
    lowVoltageCutoff.startTime = 0;

    voltageMeterReset(&voltageMeter);

    voltageMeterGenericInit();
    switch (batteryConfig()->voltageMeterSource) {
        case VOLTAGE_METER_ESC:
#ifdef USE_ESC_SENSOR
            voltageMeterESCInit();
#endif
            break;

        case VOLTAGE_METER_ADC:
            voltageMeterADCInit();
            break;

        default:
            break;
    }

    //
    // current
    //
    consumptionState = BATTERY_OK;
    currentMeterReset(&currentMeter);
    switch (batteryConfig()->currentMeterSource) {
        case CURRENT_METER_ADC:
            currentMeterADCInit();
            break;

        case CURRENT_METER_VIRTUAL:
#ifdef USE_VIRTUAL_CURRENT_METER
            currentMeterVirtualInit();
#endif
            break;

        case CURRENT_METER_ESC:
#ifdef ESC_SENSOR
            currentMeterESCInit();
#endif
            break;
        case CURRENT_METER_MSP:
#ifdef USE_MSP_CURRENT_METER
            currentMeterMSPInit();
#endif
            break;

        default:
            break;
    }
}

uint32_t batteryUpdateCurrentMeter(PifTask *p_task)
{
    const timeUs_t currentTimeUs = p_task->_last_execute_time;

    if (getBatteryCellCount() == 0) {
        currentMeterReset(&currentMeter);
        pifBattery_SetConsumedMah(&pifBattery, 0);
        return 0;
    }

    static uint32_t ibatLastServiced = 0;
    const int32_t lastUpdateAt = cmp32(currentTimeUs, ibatLastServiced);
    ibatLastServiced = currentTimeUs;

    switch (batteryConfig()->currentMeterSource) {
        case CURRENT_METER_ADC:
            currentMeterADCRefresh(lastUpdateAt);
            currentMeterADCRead(&currentMeter);
            break;

        case CURRENT_METER_VIRTUAL: {
#ifdef USE_VIRTUAL_CURRENT_METER
            throttleStatus_e throttleStatus = calculateThrottleStatus();
            bool throttleLowAndMotorStop = (throttleStatus == THROTTLE_LOW && featureIsEnabled(FEATURE_MOTOR_STOP));
            const int32_t throttleOffset = lrintf(mixerGetThrottle() * 1000);

            currentMeterVirtualRefresh(lastUpdateAt, ARMING_FLAG(ARMED), throttleLowAndMotorStop, throttleOffset);
            currentMeterVirtualRead(&currentMeter);
#endif
            break;
        }

        case CURRENT_METER_ESC:
#ifdef USE_ESC_SENSOR
            if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
                currentMeterESCRefresh(lastUpdateAt);
                currentMeterESCReadCombined(&currentMeter);
            }
#endif
            break;
        case CURRENT_METER_MSP:
#ifdef USE_MSP_CURRENT_METER
            currentMeterMSPRefresh(currentTimeUs);
            currentMeterMSPRead(&currentMeter);
#endif
            break;

        default:
        case CURRENT_METER_NONE:
            currentMeterReset(&currentMeter);
            break;
    }
    // The meters know the charge drawn best (the ESC and MSP ones are told it), so pifBattery takes
    // it from them for the capacity-based remaining charge.
    pifBattery_SetConsumedMah(&pifBattery, MAX(currentMeter.mAhDrawn, 0));
    return 0;
}

// From the mAh drawn if a capacity is set, otherwise from the cell voltage between
// vbat_min_cell_voltage (0 %) and vbat_full_cell_voltage (100 %).
uint8_t calculateBatteryPercentageRemaining(void)
{
    return pifBattery_GetRemainingPercent(&pifBattery);
}

void batteryUpdateAlarms(void)
{
    // use the state to trigger beeper alerts
    if (batteryConfig()->useVBatAlerts) {
        updateBatteryBeeperAlert();
    }
}

bool isBatteryVoltageConfigured(void)
{
    return batteryConfig()->voltageMeterSource != VOLTAGE_METER_NONE;
}

uint16_t getBatteryVoltage(void)
{
    return voltageMeter.displayFiltered;
}

uint16_t getLegacyBatteryVoltage(void)
{
    return (voltageMeter.displayFiltered + 5) / 10;
}

uint16_t getBatteryVoltageLatest(void)
{
    return voltageMeter.unfiltered;
}

uint8_t getBatteryCellCount(void)
{
    return pifBattery._cell_count;
}

uint16_t getBatteryAverageCellVoltage(void)
{
    const uint8_t batteryCellCount = getBatteryCellCount();
    return (batteryCellCount ? voltageMeter.displayFiltered / batteryCellCount : 0);
}

#if defined(USE_BATTERY_VOLTAGE_SAG_COMPENSATION)
uint16_t getBatterySagCellVoltage(void)
{
    const uint8_t batteryCellCount = getBatteryCellCount();
    return (batteryCellCount ? voltageMeter.sagFiltered / batteryCellCount : 0);
}
#endif

bool isAmperageConfigured(void)
{
    return batteryConfig()->currentMeterSource != CURRENT_METER_NONE;
}

int32_t getAmperage(void) {
    return currentMeter.amperage;
}

int32_t getAmperageLatest(void)
{
    return currentMeter.amperageLatest;
}

int32_t getMAhDrawn(void)
{
    return currentMeter.mAhDrawn;
}
