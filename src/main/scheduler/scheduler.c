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

#define SRC_MAIN_SCHEDULER_C_

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "drivers/accgyro/accgyro.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/time.h"
#include "common/utils.h"

#include "drivers/time.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/system.h"

#include "fc/core.h"
#include "fc/tasks.h"

#include "osd/osd.h"

#include "rx/rx.h"
#include "flight/failsafe.h"

#include "scheduler.h"

#include "sensors/gyro_init.h"

// DEBUG_SCHEDULER, timings for:
// 0 - Average time spent executing check function
// 1 - Time spent priortising
// 2 - time spent in scheduler

extern task_t tasks[];

static FAST_DATA_ZERO_INIT bool ignoreCurrentTaskExecRate;
static FAST_DATA_ZERO_INIT bool ignoreCurrentTaskExecTime;

int32_t schedLoopStartCycles;
static int32_t schedLoopStartMinCycles;
static int32_t schedLoopStartMaxCycles;
static uint32_t schedLoopStartDeltaDownCycles;
static uint32_t schedLoopStartDeltaUpCycles;

int32_t taskGuardCycles;
static int32_t taskGuardMinCycles;
static int32_t taskGuardMaxCycles;
static uint32_t taskGuardDeltaDownCycles;
static uint32_t taskGuardDeltaUpCycles;

FAST_DATA_ZERO_INIT uint16_t averageSystemLoadPercent = 0;

static FAST_DATA_ZERO_INIT bool gyroEnabled;

int32_t desiredPeriodCycles;
uint32_t lastTargetCycles;


void getTaskInfo(taskId_e taskId, taskInfo_t * taskInfo)
{
    task_t *task = getTask(taskId);
    PifTask *p_task = task->p_task;

    taskInfo->isEnabled = p_task != NULL;
    taskInfo->desiredPeriodUs = task->attribute->desiredPeriodUs;
    taskInfo->taskName = task->attribute->taskName;
    taskInfo->subTaskName = task->attribute->subTaskName;
    taskInfo->maxExecutionTimeUs = p_task->_max_execution_time;
    taskInfo->totalExecutionTimeUs = p_task->_total_execution_time * p_task->_unit;
    taskInfo->averageExecutionTimeUs = pifTask_GetAverageExecuteTime(p_task);
    taskInfo->averageDeltaTimeUs = pifTask_GetAverageDeltaTime(p_task);
    taskInfo->latestDeltaTimeUs = p_task->_delta_time;
}

void rescheduleTask(taskId_e taskId, timeDelta_t newPeriodUs)
{
    task_t *task;
    PifTaskMode mode = TM_NONE;
    uint16_t period;

    if (taskId < TASK_COUNT) {
        task = getTask(taskId);
    } else {
        return;
    }
    task->attribute->desiredPeriodUs = MAX(SCHEDULER_DELAY_LIMIT, newPeriodUs);  // Limit delay to 100us (10 kHz) to prevent scheduler clogging

    // Catch the case where the gyro loop is adjusted
    if (taskId == TASK_GYRO) {
        desiredPeriodCycles = (int32_t)clockMicrosToCycles((uint32_t)getTask(TASK_GYRO)->attribute->desiredPeriodUs);
    }

    switch (task->attribute->pifTaskMode) {
    case TM_PERIOD:
        if (task->attribute->desiredPeriodUs >= 5000) {
            period = task->attribute->desiredPeriodUs / 1000;
            mode = TM_PERIOD_MS;
        }
        else {
            period = task->attribute->desiredPeriodUs;
            mode = TM_PERIOD_US;
        }
        break;

    case TM_CHANGE:
        if (task->attribute->desiredPeriodUs >= 5000) {
            period = task->attribute->desiredPeriodUs / 1000;
            mode = TM_CHANGE_MS;
        }
        else {
            period = task->attribute->desiredPeriodUs;
            mode = TM_CHANGE_US;
        }
        break;

    default:
        break;
    }
    if (mode) {
        if (mode == task->p_task->_mode) {
            pifTask_ChangePeriod(task->p_task, period);
        }
        else {
            pifTask_ChangeMode(task->p_task, mode, period);
        }
    }
}

void setTaskEnabled(taskId_e taskId, bool enabled)
{
    PifTaskMode mode;
    uint16_t period;

    if (taskId < TASK_COUNT) {
        task_t *task = getTask(taskId);
        if (enabled) {
            switch (task->attribute->pifTaskMode) {
            case TM_ALWAYS:
                task->p_task = pifTaskManager_Add(task->attribute->pifTaskMode, 0, task->attribute->pifTaskFunc, task, true);
                break;

            case TM_PERIOD:
                if (task->attribute->desiredPeriodUs >= 5000) {
                    period = task->attribute->desiredPeriodUs / 1000;
                    mode = TM_PERIOD_MS;
                }
                else {
                    period = task->attribute->desiredPeriodUs;
                    mode = TM_PERIOD_US;
                }
                task->p_task = pifTaskManager_Add(mode, period, task->attribute->pifTaskFunc, task, true);
                break;

            case TM_CHANGE:
                if (task->attribute->desiredPeriodUs >= 5000) {
                    period = task->attribute->desiredPeriodUs / 1000;
                    mode = TM_CHANGE_MS;
                }
                else {
                    period = task->attribute->desiredPeriodUs;
                    mode = TM_CHANGE_US;
                }
                task->p_task = pifTaskManager_Add(mode, period, task->attribute->pifTaskFunc, task, true);
                break;

            default:
                task->p_task = pifTaskManager_Add(task->attribute->pifTaskMode, 0, task->attribute->pifTaskFunc, task, false);
                break;
            }
        } 
        else {
            pifTaskManager_Remove(task->p_task);
        }
    }
}

timeDelta_t getTaskDeltaTimeUs(taskId_e taskId)
{
    task_t *task = NULL;

    if (taskId < TASK_COUNT) {
        task = getTask(taskId);
        return task->p_task->_delta_time;
    } else {
        return 0;
    }
}

// Called by tasks executing what are known to be short states
void schedulerIgnoreTaskStateTime()
{
    ignoreCurrentTaskExecRate = true;
    ignoreCurrentTaskExecTime = true;
}

// Called by tasks with state machines to only count one state as determining rate
void schedulerIgnoreTaskExecRate()
{
    ignoreCurrentTaskExecRate = true;
}

// Called by tasks without state machines executing in what is known to be a shorter time than peak
void schedulerIgnoreTaskExecTime()
{
    ignoreCurrentTaskExecTime = true;
}

bool schedulerGetIgnoreTaskExecTime()
{
    return ignoreCurrentTaskExecTime;
}

void schedulerResetTaskStatistics(taskId_e taskId)
{
    task_t *task = getTask(taskId);

    if (taskId < TASK_COUNT) {
        task->anticipatedExecutionTime = 0;
        task->totalExecutionTimeUs = 0;
        task->maxExecutionTimeUs = 0;
    }
}

void schedulerResetTaskMaxExecutionTime(taskId_e taskId)
{
    if (taskId < TASK_COUNT) {
        task_t *task = getTask(taskId);
        task->maxExecutionTimeUs = 0;
    }
}

void schedulerInit(void)
{
    setTaskEnabled(TASK_SYSTEM, true);

    schedLoopStartMinCycles = clockMicrosToCycles(SCHED_START_LOOP_MIN_US);
    schedLoopStartMaxCycles = clockMicrosToCycles(SCHED_START_LOOP_MAX_US);
    schedLoopStartCycles = schedLoopStartMinCycles;
    schedLoopStartDeltaDownCycles = clockMicrosToCycles(1) / SCHED_START_LOOP_DOWN_STEP;
    schedLoopStartDeltaUpCycles = clockMicrosToCycles(1) / SCHED_START_LOOP_UP_STEP;

    taskGuardMinCycles = clockMicrosToCycles(TASK_GUARD_MARGIN_MIN_US);
    taskGuardMaxCycles = clockMicrosToCycles(TASK_GUARD_MARGIN_MAX_US);
    taskGuardCycles = taskGuardMinCycles;
    taskGuardDeltaDownCycles = clockMicrosToCycles(1) / TASK_GUARD_MARGIN_DOWN_STEP;
    taskGuardDeltaUpCycles = clockMicrosToCycles(1) / TASK_GUARD_MARGIN_UP_STEP;

    desiredPeriodCycles = (int32_t)clockMicrosToCycles((uint32_t)getTask(TASK_GYRO)->attribute->desiredPeriodUs);

    lastTargetCycles = getCycleCounter();

    for (taskId_e taskId = 0; taskId < TASK_COUNT; taskId++) {
        schedulerResetTaskStatistics(taskId);
    }
}

static timeDelta_t taskNextStateTime;

FAST_CODE void schedulerSetNextStateTime(timeDelta_t nextStateTime)
{
    taskNextStateTime = nextStateTime;
}

FAST_CODE timeDelta_t schedulerGetNextStateTime()
{
    return ((task_t*)(pifTaskManager_CurrentTask()->_p_client))->anticipatedExecutionTime >> TASK_EXEC_TIME_SHIFT;
}


uint16_t taskSystem(PifTask *p_task)
{
    uint32_t nowCycles;
    uint32_t nextTargetCycles = 0;
    int32_t schedLoopRemainingCycles;

    UNUSED(p_task);

    if (gyroEnabled) {
        // Realtime gyro/filtering/PID tasks get complete priority
        nowCycles = getCycleCounter();
        nextTargetCycles = lastTargetCycles + desiredPeriodCycles;
        schedLoopRemainingCycles = cmpTimeCycles(nextTargetCycles, nowCycles);

        if (schedLoopRemainingCycles < -desiredPeriodCycles) {
            /* A task has so grossly overrun that at entire gyro cycle has been skipped
             * This is most likely to occur when connected to the configurator via USB as the serial
             * task is non-deterministic
             * Recover as best we can, advancing scheduling by a whole number of cycles
             */
            nextTargetCycles += desiredPeriodCycles * (1 + (schedLoopRemainingCycles / -desiredPeriodCycles));
            schedLoopRemainingCycles = cmpTimeCycles(nextTargetCycles, nowCycles);
        }

        // Tune out the time lost between completing the last task execution and re-entering the scheduler
        if ((schedLoopRemainingCycles < schedLoopStartMinCycles) &&
            (schedLoopStartCycles < schedLoopStartMaxCycles)) {
            schedLoopStartCycles += schedLoopStartDeltaUpCycles;
        }

        // Once close to the timing boundary, poll for it's arrival
        if (schedLoopRemainingCycles < schedLoopStartCycles) {
            if (schedLoopStartCycles > schedLoopStartMinCycles) {
                schedLoopStartCycles -= schedLoopStartDeltaDownCycles;
            }
#if !defined(UNIT_TEST)
            while (schedLoopRemainingCycles > 0) {
                nowCycles = getCycleCounter();
                schedLoopRemainingCycles = cmpTimeCycles(nextTargetCycles, nowCycles);
            }
            DEBUG_SET(DEBUG_SCHEDULER_DETERMINISM, 0, clockCyclesTo10thMicros(cmpTimeCycles(nowCycles, lastTargetCycles)));
#endif
            pifTask_SetTrigger(getTask(TASK_GYRO)->p_task);

            lastTargetCycles = nextTargetCycles;
       }
    }

    if (rxUpdateCheck(pif_timer1us, cmpTimeUs(pif_timer1us, p_task->_last_execute_time))) {
        pifTask_SetTrigger(getTask(TASK_RX)->p_task);
    }
#ifdef USE_OSD
   if (osdUpdateCheck(pif_timer1us, cmpTimeUs(pif_timer1us, p_task->_last_execute_time))) {
        pifTask_SetTrigger(getTask(TASK_OSD)->p_task);
    }
#endif    
    return 0;
}

void schedulerEnableGyro(void)
{
    gyroEnabled = true;
}

uint16_t getAverageSystemLoadPercent(void)
{
    return pif_performance._use_rate;
}

float schedulerGetCycleTimeMultiplier(void)
{
    return (float)clockMicrosToCycles(getTask(TASK_GYRO)->attribute->desiredPeriodUs) / desiredPeriodCycles;
}
