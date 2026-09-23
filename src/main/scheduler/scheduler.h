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

#include "common/time.h"
#include "config/config.h"
#include "pg/scheduler.h"

// A Betaflight task is a PifTask, and its task function is a PifEvtTaskLoop.
// Both types come from here. See src/main/pif/pif_linker.h on the MIN/MAX
// collision this used to cause and how both sides are guarded now.
#include "pif/pif_linker.h"

#define TASK_PERIOD_HZ(hz) (1000000 / (hz))
#define TASK_PERIOD_MS(ms) ((ms) * 1000)
#define TASK_PERIOD_US(us) (us)

#define TASK_STATS_MOVING_SUM_COUNT     8

#define LOAD_PERCENTAGE_ONE             100

// Some tasks have occasional peaks in execution time so normal moving average duration estimation doesn't work
// Decay the estimated max task duration by 1/(1 << TASK_EXEC_TIME_SHIFT) on every invocation
#define TASK_EXEC_TIME_SHIFT            7

typedef enum {
    // Kept for the task table. The scheduler no longer orders the ring by
    // priority; see maxSkipForTask() in scheduler.c for what it does decide.
    TASK_PRIORITY_REALTIME = -1, // Task will be run outside the scheduler logic
    TASK_PRIORITY_LOWEST = 1,
    TASK_PRIORITY_LOW = 2,
    TASK_PRIORITY_MEDIUM = 3,
    TASK_PRIORITY_MEDIUM_HIGH = 4,
    TASK_PRIORITY_HIGH = 5,
    TASK_PRIORITY_MAX = 255
} taskPriority_e;

typedef struct {
    timeUs_t     maxExecutionTimeUs;
    timeUs_t     totalExecutionTimeUs;
    timeUs_t     averageExecutionTimeUs;
    timeUs_t     averageDeltaTimeUs;
} cfCheckFuncInfo_t;

typedef struct {
    const char * taskName;
    const char * subTaskName;
    bool         isEnabled;
    int8_t       staticPriority;
    timeDelta_t  desiredPeriodUs;
    timeDelta_t  latestDeltaTimeUs;
    timeUs_t     maxExecutionTimeUs;
    timeUs_t     totalExecutionTimeUs;
    // PIF averages in whole microseconds, so the tenths these carry are always
    // zero. The names and the scale are kept because that is what the CLI and
    // the OSD task list format against.
    timeUs_t     averageExecutionTime10thUs;
    timeUs_t     averageDeltaTime10thUs;
#if defined(USE_LATE_TASK_STATISTICS)
    // Longest the dispatch of this task has ever trailed its release. It
    // replaces the late/run counts the old scheduler kept: PIF measures the
    // delay of every dispatch, so the worst case is had for free, while a
    // count of late runs would need a wrapper around every task to keep.
    timeUs_t     maxDelayUs;
#endif
} taskInfo_t;

typedef enum {
    /* Actual tasks */
    TASK_SYSTEM = 0,
    TASK_MAIN,
    TASK_GYRO,
    TASK_FILTER,
    TASK_PID,
    TASK_ACCEL,
    TASK_ATTITUDE,
    TASK_RX,
    TASK_SERIAL,
    TASK_DISPATCH,
    TASK_BATTERY_VOLTAGE,
    TASK_BATTERY_CURRENT,
    TASK_BATTERY_ALERTS,
#ifdef USE_BEEPER
    TASK_BEEPER,
#endif
#ifdef USE_GPS
    TASK_GPS,
#endif
#ifdef USE_MAG
    TASK_COMPASS,
#endif
#ifdef USE_BARO
    TASK_BARO,
#endif
#ifdef USE_RANGEFINDER
    TASK_RANGEFINDER,
#endif
#if defined(USE_BARO) || defined(USE_GPS)
    TASK_ALTITUDE,
#endif
#ifdef USE_DASHBOARD
    TASK_DASHBOARD,
#endif
#ifdef USE_TELEMETRY
    TASK_TELEMETRY,
#endif
#ifdef USE_LED_STRIP
    TASK_LEDSTRIP,
#endif
#ifdef USE_TRANSPONDER
    TASK_TRANSPONDER,
#endif
#ifdef USE_STACK_CHECK
    TASK_STACK_CHECK,
#endif
#ifdef USE_OSD
    TASK_OSD,
#endif
#ifdef USE_BST
    TASK_BST_MASTER_PROCESS,
#endif
#ifdef USE_ESC_SENSOR
    TASK_ESC_SENSOR,
#endif
#ifdef USE_CMS
    TASK_CMS,
#endif
#ifdef USE_VTX_CONTROL
    TASK_VTXCTRL,
#endif
#ifdef USE_CAMERA_CONTROL
    TASK_CAMCTRL,
#endif

#ifdef USE_RCDEVICE
    TASK_RCDEVICE,
#endif

#ifdef USE_ADC_INTERNAL
    TASK_ADC_INTERNAL,
#endif

#ifdef USE_PINIOBOX
    TASK_PINIOBOX,
#endif

#ifdef USE_CRSF_V3
    TASK_SPEED_NEGOTIATION,
#endif

    /* Count of real tasks */
    TASK_COUNT,

    /* Service task IDs */
    TASK_NONE = TASK_COUNT,
    TASK_SELF
} taskId_e;

typedef struct {
    // Configuration
    const char * taskName;
    const char * subTaskName;
    // Event driven tasks. Run from the idle pass of the scheduler; a check that
    // answers yes triggers the task. NULL for a task that only runs on its
    // period.
    bool (*checkFunc)(timeUs_t currentTimeUs, timeDelta_t currentDeltaTimeUs);
    // The task itself, called by PIF with its own PifTask. A non zero return
    // sets the period of the next release for TM_PERIOD, which is how a task
    // reschedules itself, and is ignored for the other modes.
    PifEvtTaskLoop taskFunc;
    PifTaskMode pifTaskMode;            // TM_PERIOD, TM_REALTIME or TM_EXTERNAL
    timeDelta_t desiredPeriodUs;        // target period of execution
    const int8_t staticPriority;        // how long the task may be held back by the realtime guard
} task_attribute_t;

typedef struct {
    // Task static data
    task_attribute_t *attribute;

    // The release, the timing and every statistic belong to this. NULL until
    // schedulerInit() has registered the task, and for a task this build has
    // compiled out.
    PifTask *p_task;

    // What the task last declared through schedulerSetNextStateTime(), kept so
    // that schedulerGetNextStateTime() hands the same number back. A state
    // machine uses the pair to carry its own per state estimate across runs;
    // the copy PIF gets is consumed by the dispatch it applies to.
    timeUs_t anticipatedExecutionTime;
} task_t;

void getCheckFuncInfo(cfCheckFuncInfo_t *checkFuncInfo);
void getTaskInfo(taskId_e taskId, taskInfo_t *taskInfo);
void rescheduleTask(taskId_e taskId, timeDelta_t newPeriodUs);
void setTaskEnabled(taskId_e taskId, bool newEnabledState);
timeDelta_t getTaskDeltaTimeUs(taskId_e taskId);
void schedulerIgnoreTaskExecTime();
bool schedulerGetIgnoreTaskExecTime();
void schedulerResetTaskStatistics(taskId_e taskId);
void schedulerResetTaskMaxExecutionTime(taskId_e taskId);
void schedulerResetCheckFunctionMaxExecutionTime(void);
void schedulerSetNextStateTime(timeDelta_t nextStateTime);
timeDelta_t schedulerGetNextStateTime();
void schedulerInit(void);
uint32_t taskSystemLoad(PifTask *p_task);
uint16_t getAverageSystemLoadPercent(void);

// Longest delay from the release of TASK_GYRO to its dispatch, and the number of
// those releases that were delayed by more than a whole gyro period. Both are
// measured by PIF across the realtime task and reported by the CLI.
uint32_t schedulerGetRealtimeMaxDelayUs(void);
uint32_t schedulerGetRealtimeMissCount(void);

// Clears both, along with the margin PIF keeps in front of the gyro release.
// For one off events only - a long blocking run that is over, or the start of a
// phase whose numbers should stand on their own. See the definition.
void schedulerResetRealtime(void);
