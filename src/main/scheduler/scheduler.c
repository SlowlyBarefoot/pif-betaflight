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

/*
 * scheduler.c - Betaflight's scheduler, on PIF's task manager.
 *
 * The queue, the dynamic priorities and the cycle counter of the scheduler this
 * replaces are gone; PIF decides when a task runs. There is no dispatch wrapper
 * left either:
 * a Betaflight task function is a PifEvtTaskLoop and PIF calls it directly, so
 * the release, the timing and every statistic have exactly one owner. What is
 * left in this file is registration, the scheduler.h API the rest of the
 * firmware still calls, and the check functions of the event driven tasks.
 *
 * The mapping, taken from task_attribute_t::pifTaskMode in fc/tasks.c:
 *
 *   TASK_GYRO           TM_REALTIME, period gyro.sampleLooptime
 *   TASK_FILTER         TM_EXTERNAL, released by taskGyroSample()
 *   TASK_PID            TM_EXTERNAL, released by taskGyroSample(), or by
 *                       taskFiltering() when both fall in the same gyro cycle
 *   everything else     TM_PERIOD, period task_attribute_t::desiredPeriodUs
 *
 * PIF releases the realtime task ahead of the whole ring and refuses to start
 * any other task whose run would not finish before that release. The length it
 * judges by is the measured maximum over a moving window, or what the task
 * declared through schedulerSetNextStateTime() for its next run. That is what
 * the old scheduler did by hand with anticipatedExecutionTime, taskGuardCycles
 * and the busy wait on the cycle counter, so none of it is here.
 *
 * What the old scheduler did and this does not:
 *
 * - The gyro EXTI lock. scheduler() measured the real gyro interrupt rate and
 *   slid its target time to take out the skew. PIF releases TM_REALTIME on the
 *   micros() grid instead. To get the lock back, the PIF way is a
 *   pifTask_SetTrigger() from the gyro EXTI handler and a period of zero.
 * - Static priority ordering. PIF's ring is round robin. The priorities are
 *   folded into PifTask::max_skip below, which is how long a task may be held
 *   back by the realtime guard, so a high priority task still gets through
 *   sooner.
 * - schedulerIgnoreTaskExecRate(), which is gone along with its call sites. It
 *   kept one state of a state machine from defining the reported rate. PIF
 *   writes _delta_time when it decides to release a task, not when it
 *   dispatches it, so there is no point at which a run can be left out of the
 *   measurement; a state machine now reports its dispatch rate rather than its
 *   logical cycle rate. Bringing it back means a PifTask that can skip a delta
 *   sample, the way pifTask_IgnoreBlockTime() skips a block time sample.
 *
 * One consequence to keep in mind: a PIF trigger is not subject to the
 * realtime guard, so a triggered dispatch happens without asking whether the
 * run fits before the next gyro release. TASK_FILTER and TASK_PID want exactly
 * that. TASK_RX and TASK_OSD get it too, because their check functions release
 * them the same way, so those two can start late in a gyro period where the
 * old scheduler would have held them back. They are the two tasks Betaflight
 * already lets break determinism on purpose - that is what rx_relax_determinism
 * and osd_relax_determinism are - but here it is every release rather than a
 * last resort.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/time.h"
#include "common/utils.h"

#include "drivers/time.h"

#include "fc/tasks.h"

#include "scheduler.h"


extern task_t tasks[];

// The tasks that carry a checkFunc, gathered from the attribute table at init
// so the idle callback does not walk all of TASK_COUNT on every pass. Only
// TASK_RX and TASK_OSD have one.
static FAST_DATA_ZERO_INIT taskId_e checkTaskIds[TASK_COUNT];
static FAST_DATA_ZERO_INIT int checkTaskCount;

// Which dispatch called schedulerIgnoreTaskExecTime(). Keyed by the task and
// by the timestamp PIF stamped that dispatch with, so the flag clears itself
// when the next run starts - there is no wrapper left to clear it in, and a
// plain bool would leak into the run after the one that set it.
static FAST_DATA_ZERO_INIT PifTask *ignoreExecTimeTask;
static FAST_DATA_ZERO_INIT uint32_t ignoreExecTimeAt;

static FAST_DATA_ZERO_INIT uint16_t averageSystemLoadPercent;

timeUs_t checkFuncMaxExecutionTimeUs;
timeUs_t checkFuncTotalExecutionTimeUs;
timeUs_t checkFuncMovingSumExecutionTimeUs;
timeUs_t checkFuncMovingSumDeltaTimeUs;


// The realtime guard holds a task back while its run would not finish before
// the next gyro release, and max_skip is how many passes in a row it may do
// that before letting the task through anyway. The old scheduler used
// staticPriority to pick what ran first; here it picks what waits least.
static uint16_t maxSkipForTask(taskId_e taskId, int8_t staticPriority)
{
    // rx_relax_determinism and osd_relax_determinism say how many refused
    // attempts these two may collect before they are scheduled regardless of
    // the determinism it costs. That is what PifTask::max_skip counts, so the
    // settings keep their meaning.
    if ((taskId == TASK_RX) && schedulerConfig()->rxRelaxDeterminism) {
        return schedulerConfig()->rxRelaxDeterminism;
    }
#ifdef USE_OSD
    if ((taskId == TASK_OSD) && schedulerConfig()->osdRelaxDeterminism) {
        return schedulerConfig()->osdRelaxDeterminism;
    }
#endif

    if (staticPriority >= TASK_PRIORITY_HIGH) {
        return 2;
    }
    if (staticPriority <= TASK_PRIORITY_LOWEST) {
        return 10;
    }
    return 12 - 2 * staticPriority;
}

// Resolves TASK_SELF through PIF rather than through a global of our own, and
// rejects anything that is not a real task. Returns NULL when there is nothing
// to act on, which every caller below tests for.
static task_t *resolveTask(taskId_e taskId)
{
    if (taskId == TASK_SELF) {
        PifTask *p_task = pifTaskManager_CurrentTask();
        return p_task ? (task_t *)p_task->_p_client : NULL;
    }
    return (taskId < TASK_COUNT) ? getTask(taskId) : NULL;
}

// TM_EXTERNAL has no period of its own; for the rest it is what the task table
// asked for, as rescheduleTask() may since have changed it.
static uint32_t taskPeriodUs(const task_t *task)
{
    if (task->attribute->pifTaskMode == TM_EXTERNAL) return 0;

    return (uint32_t)task->attribute->desiredPeriodUs;
}

// pifTask_GetAverage*() reports PIF_TASK_AVERAGE_NONE until it has enough
// samples. The CLI wants a number it can divide, and zero is what it showed
// before the first samples arrived anyway.
static uint32_t averageOrZero(uint32_t average)
{
    return (average == PIF_TASK_AVERAGE_NONE) ? 0 : average;
}

// PIF's idle callback: it runs only on a pass that dispatched no task, and
// only while its measured run still fits before the next gyro release. That is
// where the check functions of the event driven tasks belong - the old
// scheduler guarded them with a margin of its own for the same reason, and
// running them here also keeps them from ever landing between TASK_GYRO and
// TASK_FILTER.
static FAST_CODE void schedulerIdle(void)
{
    const timeUs_t currentTimeUs = micros();

    for (int i = 0; i < checkTaskCount; i++) {
        task_t *task = getTask(checkTaskIds[i]);
        PifTask *p_task = task->p_task;

        // Disabled, or already signalled and waiting for its turn. __trigger is
        // private to PIF and there is no accessor for it; it is only read here,
        // and it holds exactly the "signalled but not yet run" state that the
        // old scheduler kept in dynamicPriority.
        if (!p_task || p_task->__trigger) {
            continue;
        }

        if (task->attribute->checkFunc(currentTimeUs, cmpTimeUs(currentTimeUs, p_task->_last_execute_time))) {
            const timeUs_t checkFuncExecutionTimeUs = cmpTimeUs(micros(), currentTimeUs);
            checkFuncMovingSumExecutionTimeUs += checkFuncExecutionTimeUs - checkFuncMovingSumExecutionTimeUs / TASK_STATS_MOVING_SUM_COUNT;
            checkFuncMovingSumDeltaTimeUs += p_task->_delta_time - checkFuncMovingSumDeltaTimeUs / TASK_STATS_MOVING_SUM_COUNT;
            checkFuncTotalExecutionTimeUs += checkFuncExecutionTimeUs;   // time consumed by scheduler + task
            if (checkFuncExecutionTimeUs > checkFuncMaxExecutionTimeUs) {
                checkFuncMaxExecutionTimeUs = checkFuncExecutionTimeUs;
            }
            pifTask_SetTrigger(p_task, 0);
        }
    }
}

uint32_t taskSystemLoad(PifTask *p_task)
{
    UNUSED(p_task);

    // PIF closes its own measurement window once a second and reports the share
    // of it spent in tasks and in the timer and idle callbacks.
    averageSystemLoadPercent = pif_performance._task_load;

#if defined(SIMULATOR_BUILD)
    averageSystemLoadPercent = 0;
#endif

    // 0 - % CPU busy
    // 1 - gyro releases that slipped a whole period since boot
    // 2 - longest delay from a gyro release to its dispatch, in us
    DEBUG_SET(DEBUG_TIMING_ACCURACY, 0, averageSystemLoadPercent);
    DEBUG_SET(DEBUG_TIMING_ACCURACY, 1, pif_performance._miss_count);
    DEBUG_SET(DEBUG_TIMING_ACCURACY, 2, pif_performance._max_delay);

    // Shorter than the moving average of the samples that do the reporting above
    schedulerIgnoreTaskExecTime();
    return 0;
}

void getCheckFuncInfo(cfCheckFuncInfo_t *checkFuncInfo)
{
    checkFuncInfo->maxExecutionTimeUs = checkFuncMaxExecutionTimeUs;
    checkFuncInfo->totalExecutionTimeUs = checkFuncTotalExecutionTimeUs;
    checkFuncInfo->averageExecutionTimeUs = checkFuncMovingSumExecutionTimeUs / TASK_STATS_MOVING_SUM_COUNT;
    checkFuncInfo->averageDeltaTimeUs = checkFuncMovingSumDeltaTimeUs / TASK_STATS_MOVING_SUM_COUNT;
}

void getTaskInfo(taskId_e taskId, taskInfo_t *taskInfo)
{
    memset(taskInfo, 0, sizeof(*taskInfo));

    const task_t *task = resolveTask(taskId);

    if (!task) return;

    taskInfo->taskName = task->attribute->taskName;
    taskInfo->subTaskName = task->attribute->subTaskName;
    taskInfo->desiredPeriodUs = task->attribute->desiredPeriodUs;
    taskInfo->staticPriority = task->attribute->staticPriority;

    // A disabled task is not registered with PIF at all, so it has no
    // statistics to report and everything past here stays zero.
    PifTask *p_task = task->p_task;
    taskInfo->isEnabled = (p_task != NULL);
    if (!p_task) return;

    taskInfo->maxExecutionTimeUs = p_task->_max_execution_time;
    taskInfo->totalExecutionTimeUs = p_task->_total_execution_time;
    taskInfo->latestDeltaTimeUs = (timeDelta_t)p_task->_delta_time;
    taskInfo->averageExecutionTime10thUs = averageOrZero(pifTask_GetAverageExecuteTime(p_task)) * 10;
    taskInfo->averageDeltaTime10thUs = averageOrZero(pifTask_GetAverageDeltaTime(p_task)) * 10;
#if defined(USE_LATE_TASK_STATISTICS)
    taskInfo->maxDelayUs = p_task->_max_delay;
#endif
}

void rescheduleTask(taskId_e taskId, timeDelta_t newPeriodUs)
{
    task_t *task = resolveTask(taskId);

    if (!task) return;

    // Limit delay to 100us (10 kHz) to prevent scheduler clogging
    if (newPeriodUs < SCHEDULER_DELAY_LIMIT) {
        newPeriodUs = SCHEDULER_DELAY_LIMIT;
    }
    task->attribute->desiredPeriodUs = newPeriodUs;

    // TASK_FILTER and TASK_PID are TM_EXTERNAL: they have no period to change,
    // they run when TASK_GYRO releases them. desiredPeriodUs above is still
    // what the CLI reports them against.
    if (task->p_task && (task->attribute->pifTaskMode != TM_EXTERNAL)) {
        pifTask_ChangePeriod(task->p_task, (uint32_t)newPeriodUs);
    }
}

void setTaskEnabled(taskId_e taskId, bool enabled)
{
    task_t *task = resolveTask(taskId);

    if (!task) return;

    if (enabled) {
        // Nothing to run, or running already. Re-adding would take a second
        // PifTask slot and leave the first one orphaned in the ring.
        if (!task->attribute->taskFunc || task->p_task) return;

        const taskId_e id = (taskId_e)(task - tasks);

        task->p_task = pifTaskManager_Add(PIF_ID_USER(id), task->attribute->pifTaskMode,
                taskPeriodUs(task), task->attribute->taskFunc, task, TRUE);
        if (!task->p_task) {
            // Out of PifTask slots, or PIF never came up. Raise PIF_TASK_SIZE in
            // pif_linker.h if it is the former; either way the task simply never
            // runs and the CLI reports it as disabled.
            return;
        }

        task->p_task->name = task->attribute->taskName;
        task->p_task->max_skip = maxSkipForTask(id, task->attribute->staticPriority);
    } else if (task->p_task) {
        // PIF has no pause that covers every mode: a trigger releases a task
        // whether or not it is paused, and TASK_FILTER, TASK_PID, TASK_RX and
        // TASK_OSD are all released by trigger. Taking the task off the ring is
        // the only disable that holds for all of them, and it shortens the ring
        // for the tasks that are left. The cost is that a task switched off and
        // on again starts its statistics from zero.
        pifTaskManager_Remove(task->p_task);
        task->p_task = NULL;
    }
}

timeDelta_t getTaskDeltaTimeUs(taskId_e taskId)
{
    const task_t *task = resolveTask(taskId);

    return (task && task->p_task) ? (timeDelta_t)task->p_task->_delta_time : 0;
}

// Called by a task whose current run is not representative: a state machine in
// one of its short states, or a run that bailed out early. It absorbed
// schedulerIgnoreTaskStateTime(), which said the same thing once the rate half
// of that name went with schedulerIgnoreTaskExecRate().
void schedulerIgnoreTaskExecTime()
{
    PifTask *p_task = pifTaskManager_CurrentTask();

    if (!p_task) return;

    // Keep this run out of the block time, which is what decides whether the
    // task may start before the next gyro release. The execution time
    // statistics still take it: PIF has no way to leave a run out of those, and
    // a peak the CLI hides is a peak nobody fixes.
    pifTask_IgnoreBlockTime(p_task);

    ignoreExecTimeTask = p_task;
    ignoreExecTimeAt = p_task->_last_execute_time;
}

bool schedulerGetIgnoreTaskExecTime()
{
    PifTask *p_task = pifTaskManager_CurrentTask();

    return p_task && (ignoreExecTimeTask == p_task) && (ignoreExecTimeAt == p_task->_last_execute_time);
}

void schedulerResetTaskStatistics(taskId_e taskId)
{
    task_t *task = resolveTask(taskId);

    if (!task) return;

    task->anticipatedExecutionTime = 0;
    if (task->p_task) {
        pifTask_ResetStatistics(task->p_task);
    }
}

void schedulerResetTaskMaxExecutionTime(taskId_e taskId)
{
    const task_t *task = resolveTask(taskId);

    if (!task || !task->p_task) return;

    pifTask_ResetMaxExecutionTime(task->p_task);
    // The moving window forgets an outlier on its own, but until it does, a
    // maximum left over from a one off long run keeps the task from being
    // started before a gyro release. Clearing the max asked for here covers
    // that too.
    pifTask_ResetMaxBlockTime(task->p_task);
}

void schedulerResetCheckFunctionMaxExecutionTime(void)
{
    checkFuncMaxExecutionTimeUs = 0;
}

void schedulerInit(void)
{
    checkTaskCount = 0;
    ignoreExecTimeTask = NULL;

    checkFuncMaxExecutionTimeUs = 0;
    checkFuncTotalExecutionTimeUs = 0;
    checkFuncMovingSumExecutionTimeUs = 0;
    checkFuncMovingSumDeltaTimeUs = 0;

    // Nothing below may touch PIF if it never came up: pifTaskManager_Add()
    // would write to an object array that was never allocated, and
    // pifTaskManager_SetIdle() would call a clock callback that is still NULL.
    // Every task then stays unregistered, is reported as disabled, and never
    // runs.
    if (!pifLinker_IsReady()) return;

    for (taskId_e taskId = 0; taskId < TASK_COUNT; taskId++) {
        task_t *task = getTask(taskId);

        task->anticipatedExecutionTime = 0;
        if (task->attribute->checkFunc) {
            checkTaskIds[checkTaskCount++] = taskId;
        }
    }

    // The check functions of the event driven tasks, run on a pass that had
    // nothing else to do.
    pifTaskManager_SetIdle(schedulerIdle, 0);

    setTaskEnabled(TASK_SYSTEM, true);
}

FAST_CODE void schedulerSetNextStateTime(timeDelta_t nextStateTime)
{
    PifTask *p_task = pifTaskManager_CurrentTask();

    if (!p_task) return;

    // PIF judges the next run by this instead of by the longest run it has
    // measured, which is what a state machine whose states differ by an order
    // of magnitude needs: without it the shortest state is refused whenever
    // there is no room for the longest one.
    pifTask_SetNextBlockTime(p_task, (nextStateTime > 0) ? (uint32_t)nextStateTime : 0);

    // PIF consumes the declaration at the next dispatch, so the caller could
    // not read it back. Keeping a copy is what lets a state machine carry its
    // own per state estimate from one run to the next.
    ((task_t *)p_task->_p_client)->anticipatedExecutionTime = (timeUs_t)nextStateTime;
}

FAST_CODE timeDelta_t schedulerGetNextStateTime()
{
    PifTask *p_task = pifTaskManager_CurrentTask();

    return p_task ? (timeDelta_t)((task_t *)p_task->_p_client)->anticipatedExecutionTime : 0;
}

uint16_t getAverageSystemLoadPercent(void)
{
    return averageSystemLoadPercent;
}

uint32_t schedulerGetRealtimeMaxDelayUs(void)
{
    return pif_performance._max_delay;
}

uint32_t schedulerGetRealtimeMissCount(void)
{
    return pif_performance._miss_count;
}

void schedulerResetRealtime(void)
{
    // This clears more than the two numbers above: PIF keeps a margin that a
    // run has to leave free before the gyro release, and it grows on every late
    // release but only shrinks one step per release that was on time. A single
    // long run therefore pins it at its maximum and keeps costing throughput
    // long after the run that caused it. Resetting it here is the point of the
    // call, and it is why this belongs on one off events rather than anywhere
    // that repeats: the margin has to relearn its level from the releases that
    // follow.
    pifTaskManager_ResetRealtime();
}
