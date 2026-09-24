/*
 * pif_linker.h - the seam between Betaflight and PIF.
 *
 * Every Betaflight task is a PifTask and its task function is a PifEvtTaskLoop,
 * so PifTask has to be a type that scheduler.h can name and that the task
 * functions can take. This header is where PIF enters the Betaflight build; a
 * source that defines a task function includes it, and scheduler.h includes it
 * for the two types in task_attribute_t.
 *
 * On the macro collision that used to keep PIF out of here: core/pif.h defines
 * MIN, MAX and ABS, and so does common/maths.h. Both sides are now guarded -
 * pif.h leaves them alone when they already exist, and maths.h undefines before
 * defining - so whichever order a translation unit includes them in, the
 * Betaflight versions win wherever maths.h is in scope. They are the ones that
 * evaluate their arguments once. A source that uses MIN or MAX must therefore
 * include common/maths.h, which is what Betaflight already required of it.
 */

#ifndef PIF_LINKER_H
#define PIF_LINKER_H

#include <stdbool.h>
#include <stdint.h>

#include "core/pif.h"
#include "core/pif_task_manager.h"
#include "sensor/pif_imu_sensor.h"

// Number of PifTask slots the task manager is created with. Every Betaflight
// task takes one (scheduler.c registers the whole table), TASK_COUNT in
// scheduler.h reaches 34 with every feature compiled in, and PIF modules bring
// tasks of their own, so this leaves room above that. A task that finds no slot
// is simply not registered - pifTaskManager_Add() returns NULL - and shows up
// as disabled in the CLI, so the headroom matters more than the bytes.
#define PIF_TASK_SIZE			40

// Number of timer process slots (pifTaskManager_AddTimer). Each
// PifTimerManager takes one, and there is one so far: the 1 ms manager.
#define PIF_TASK_TIMER_SIZE		4

// Number of PifTimer slots in the 1 ms timer manager.
#define PIF_TIMER_1MS_SIZE		8

// PIF allocates its object arrays with calloc() at init. The F405 linker
// script reserves no heap (_Min_Heap_Size = 0 in src/link/stm32_flash_f4_split.ld),
// so pif_linker.c supplies _sbrk() over a static buffer of this size instead
// of growing the heap into the stack. Allocation happens only during
// pifLinker_Init(), so running out shows up as pifLinker_Init() returning
// false rather than as a late failure.
//
// What has to fit, with pifObjArray adding 8 bytes of node to every element:
//
//   PIF_TASK_SIZE       x (8 + sizeof(PifTask))
//   PIF_TASK_TIMER_SIZE x (8 + sizeof(PifTaskTimer))
//   PIF_TIMER_1MS_SIZE  x (8 + sizeof(PifTimer)) + 4 bytes per removal slot
//   I2C_PIF_DEVICE_COUNT x (8 + sizeof(PifI2cDevice)) per I2C bus a PIF
//                       driver uses (drivers/bus_i2c_pif.c, allocated on
//                       first use rather than in pifLinker_Init())
//   SPI_PIF_DEVICE_COUNT x (8 + sizeof(PifSpiDevice)) per SPI bus a PIF
//                       driver uses (drivers/bus_spi_pif.c, likewise)
//   12 x sizeof(uint16_t) for the PPM channel buffer when the receiver is
//                       PPM (drivers/rx/rx_pwm.c, allocated in ppmRxInit())
//
// sizeof(PifTask) is 144 bytes with PIF_USE_TASK_STATISTICS and
// PIF_USE_BLOCK_TIME on, which pif_conf.h does turn on, so the tasks alone take
// 40 x 152 = 6080 bytes. The timer and PifTimer slots bring the total to around
// 6.7 KB and leave about 1.5 KB spare. Anything added to PifTask costs 152
// bytes here per 40 slots, so check this figure when PIF grows a field.
#define PIF_HEAP_SIZE			8192

// The one PifImuSensor of the board. A PIF gyro, accelerometer or magnetometer
// driver registers its read function and gain on it when it is initialised,
// and all three share it, so every PIF sensor driver has to be handed this
// instance rather than one of its own. pifLinker_Init() clears it; the PIF
// alignment is left unset because Betaflight aligns the samples itself.
extern PifImuSensor g_imu_sensor;

// Initialises pif, the task manager and the 1 ms timer manager, in that order.
// Must run after systemInit(), because pif is handed micros() as its 1 us
// clock, and before the first pifLinker_sigTimer1ms().
//
// Returns 0 (E_SUCCESS) when all three came up, otherwise the PifError of the
// step that failed. 3 is E_OUT_OF_HEAP, which means PIF_HEAP_SIZE is too small
// for the slot counts above. The tick and the loop below stay dormant unless
// this returned 0.
uint8_t pifLinker_Init(void);

// Whether pifLinker_Init() has succeeded, so PIF may be called. Callers that
// only ever run after a successful init do not need it; scheduler.c does,
// because schedulerInit() would otherwise register tasks on a task manager
// that was never created and hand PIF a clock callback it has not got.
bool pifLinker_IsReady(void);

// 1 ms tick. Called from SysTick_Handler(); advances the pif clock and the
// 1 ms timer manager. Does nothing until pifLinker_Init() has succeeded.
void pifLinker_sigTimer1ms(void);

// One pass of the PIF scheduler, called from run() in main.c. Since
// scheduler.c put every Betaflight task on the task manager, this is the
// scheduler: it dispatches at most one task, runs the check functions of the
// event driven tasks when it dispatched none, and closes the CPU load window
// once a second. Does nothing until pifLinker_Init() has succeeded.
void pifLinker_Loop(void);

#endif  // PIF_LINKER_H
