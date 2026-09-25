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

// Adds __max_loop_time1us accounting and the 1 minute performance state, plus
// the pif_act_task_signal hook around every dispatch. Off: it costs time in
// the loop that now runs the whole flight controller, and nothing reads what
// it produces.
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
// shares I2C and SPI buses between tasks, so a task that holds one has to name
// it here before scheduler.c can let any task yield. Nothing yields yet.
#define DISALLOW_YIELD_ID_NONE				0
#define DISALLOW_YIELD_ID_I2C				1
#define DISALLOW_YIELD_ID_SPI				2

// Per task execution time / delta time / trigger delay statistics. Every
// Betaflight task is a PifTask now, so this is what "tasks" in the CLI is
// ultimately meant to report, and pifTaskManager_Print() needs it.
#define PIF_USE_TASK_STATISTICS

// Moving window of the longest run without yielding, used by TM_REALTIME to
// skip a task that would not finish before the next release. Implied by
// PIF_USE_TASK_STATISTICS above, and stated here because the gyro loop depends
// on it: without it _fitsInSlack() lets every task start and TASK_GYRO loses
// the guarantee it was made TM_REALTIME for.
#define PIF_USE_BLOCK_TIME

//#define PIF_TASK_GUARD_MIN_US				2
//#define PIF_TASK_GUARD_MAX_US				100

//#define PIF_TASK_MAX_SKIP					10


// -------- pifGps -------------------------------

// UBX payload bytes a received packet may carry. The largest one io/gps.c
// reads is NAV-SAT, 8 + 12 bytes per satellite, and a M9N reports up to 42.
// The PifGpsUblox holds one packet of this size.
#define PIF_GPS_UBLOX_RX_PAYLOAD_SIZE		(8 + 12 * 42)

// Transmit ring buffer of the PifGpsUblox, from the PIF heap. The largest
// message io/gps.c sends is CFG-GNSS with 7 config blocks: 60 bytes of
// payload, 8 of framing and 4 of queue header.
#define PIF_GPS_UBLOX_TX_SIZE				128


// -------- pifMsp -------------------------------

// No buffers from the PIF heap: msp/msp_serial.c gives every PifMspV2 its
// own receive buffer (mspPort_t.inBuf) and answer buffer (in CCM) when the
// port is allocated, since the answer buffer has to hold a 4 KB dataflash
// read frame.
#define PIF_MSP_RX_PACKET_SIZE				0
#define PIF_MSP_TX_ANSWER_SIZE				0

// No receive timeout, as before: a PifTimer per MSP port for something
// Betaflight never did. A broken frame still ends at its checksum.
#define PIF_MSP_RECEIVE_TIMEOUT				0


// -------- pifTimer -----------------------------

//#define PIF_PWM_MAX_DUTY					1000


#endif  // PIF_CONF_H
