/*
 * pif_conf.h - PIF build configuration for Betaflight.
 *
 * PIF (core/pif.h) includes this file by the fixed name "pif_conf.h", so the
 * name cannot be changed. Derived from pif/include/pif_conf_temp.h; only the
 * options this port actually needs are enabled, the rest are left commented
 * with the template's defaults so the next step can turn them on in place.
 */

#ifndef PIF_CONF_H
#define PIF_CONF_H


// -------- pif Configuration --------------------

// Adds __max_loop_time1us accounting and the 1 minute performance state.
// Off for now: it costs time in the scheduler loop and there is no scheduler
// loop yet.
//#define PIF_DEBUG

#define PIF_INLINE							inline

// pif.h already falls back to __attribute__ ((weak)) for GCC, which is the
// only compiler this target is built with.
//#define PIF_WEAK							__attribute__ ((weak))


// -------- pifCollectSignal ---------------------

//#define PIF_COLLECT_SIGNAL


// -------- pifLog -------------------------------

// No log for now. pifLog needs a PifUart, which means the communication
// module and a serial port taken away from the flight controller. Betaflight
// prints through its own CLI, so the PIF log is only worth wiring up when
// pifTaskManager_Print() output is actually wanted.
// Leaving this on also keeps pif_log.c out of the build.
#define PIF_NO_LOG
//#define PIF_LOG_COMMAND

//#define PIF_LOG_LINE_SIZE					80


// -------- pifTask ------------------------------

//#define PIF_TASK_STACK_SIZE				5

// Identifiers for resources that must not be preempted by a yield. Betaflight
// shares I2C and SPI buses between tasks, so these will be needed once tasks
// are moved onto PIF.
#define DISALLOW_YIELD_ID_NONE				0
#define DISALLOW_YIELD_ID_I2C				1
#define DISALLOW_YIELD_ID_SPI				2

// Per task execution time / delta time / trigger delay statistics. This is
// what the project is ultimately after, but it enlarges PifTask and no task
// is registered yet, so it stays off until tasks are ported.
//#define PIF_USE_TASK_STATISTICS

// Moving window of the longest run without yielding, used by TM_REALTIME to
// skip a task that would not finish before the next release.
//#define PIF_USE_BLOCK_TIME

//#define PIF_TASK_GUARD_MIN_US				2
//#define PIF_TASK_GUARD_MAX_US				100

//#define PIF_TASK_MAX_SKIP					10


// -------- pifTimer -----------------------------

//#define PIF_PWM_MAX_DUTY					1000


#endif  // PIF_CONF_H
