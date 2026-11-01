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
#include "core/pif_timer_manager.h"
#include "sensor/pif_imu_sensor.h"

// Number of PifTask slots the task manager is created with. Every Betaflight
// task takes one (scheduler.c registers the whole table), TASK_COUNT in
// scheduler.h reaches 33 with every feature compiled in, and PIF modules bring
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
// pif_Init(), so running out shows up as pif_Init() returning FALSE rather
// than as a late failure.
//
// What has to fit, with pifObjArray adding 8 bytes of node to every element:
//
//   PIF_TASK_SIZE       x (8 + sizeof(PifTask))
//   PIF_TASK_TIMER_SIZE x (8 + sizeof(PifTaskTimer))
//   PIF_TIMER_1MS_SIZE  x (8 + sizeof(PifTimer)) + 4 bytes per removal slot
//   I2C_PIF_DEVICE_COUNT x (8 + sizeof(PifI2cDevice)) per I2C bus a PIF
//                       driver uses (drivers/bus_i2c_pif.c, allocated on
//                       first use rather than in pif_Init())
//   SPI_PIF_DEVICE_COUNT x (8 + sizeof(PifSpiDevice)) per SPI bus a PIF
//                       driver uses (drivers/bus_spi_pif.c, likewise)
//   12 x sizeof(uint16_t) for the PPM channel buffer when the receiver is
//                       PPM (drivers/rx/rx_pwm.c, allocated in ppmRxInit())
//   2 x sizeof(PifRingBuffer) per hardware UART, for the PifUart that holds
//                       its RX and TX buffers (drivers/serial_uart.c, allocated
//                       on the first open of the UART and kept; the buffers
//                       themselves are the static uartNRxBuffer/TxBuffer).
//                       sizeof(PifRingBuffer) is 28 bytes, about 80 bytes per
//                       UART with the allocator overhead, so up to 480 bytes
//                       with all six F4 UARTs in use. No tasks.
//   PIF_GPS_UBLOX_TX_SIZE (128) bytes for the transmit ring buffer of the
//                       u-blox GPS driver when the GPS provider is UBLOX
//                       (io/gps.c, allocated in gpsInit()). Its PifUart has
//                       no buffers but takes 1 of the PIF_TASK_SIZE task
//                       slots for its TX task.
//   Nothing for the PifMspV2 of each MSP port (msp/msp_serial.c): its receive
//                       and answer buffers are static, it has no PifUart and
//                       no task, and PIF_MSP_RECEIVE_TIMEOUT is 0, so no
//                       PifTimer either.
//   Nothing for the PifDshot of the motor output (drivers/dshot.c), the two
//                       PifAdc of the ADC (drivers/adc.c, sensors/adcinternal.c)
//                       or the PifFlash of a config save (config/config_streamer.c):
//                       they are static or on the stack, keep their motors,
//                       channels and command queue in fixed arrays, and are
//                       driven from existing tasks, so they take no heap, no
//                       task and no timer.
//
// sizeof(PifTask) is 156 bytes with PIF_USE_TASK_STATISTICS, PIF_USE_BLOCK_TIME
// and PIF_USE_TASK_REALTIME on, which pif_conf.h does turn on, so the tasks
// alone take 40 x 164 = 6560 bytes. The task timer slots (4 x (8 + 36)) and
// the PifTimer slots (8 x (8 + 24) + 8 x 4) bring the total to about 7 KB,
// and the 1 KB on top of 8 KB is for the UART ring buffers, which leaves about
// 1 KB spare for the I2C/SPI devices and the rest. Anything added to PifTask
// costs 40 bytes here per 4 bytes of field, so check this figure when PIF
// grows a field.
#define PIF_HEAP_SIZE			(8192 + 1024)

// The one PifImuSensor of the board. A PIF gyro, accelerometer or magnetometer
// driver registers its read function and gain on it when it is initialised,
// and all three share it, so every PIF sensor driver has to be handed this
// instance rather than one of its own. pifLinker_Setup() clears it; the PIF
// alignment is left unset because Betaflight aligns the samples itself.
extern PifImuSensor g_imu_sensor;

// The 1 ms timer manager. Nothing registers a PifTimer on it yet; it exists so
// that the tick path is in place and can be measured.
extern PifTimerManager g_timer1ms;

// PIF's 1 us clock, micros() narrowed to 32 bits. Handed to pif_Init(), so it
// has to be running by then: systemInit() sets usTicks and starts SysTick.
uint32_t pifActTimer1us(void);

// The port setup handed to pif_Init(). Clears g_imu_sensor, then creates the
// task manager and the 1 ms timer manager, in that order: the timer manager
// takes one of the task manager's timer slots. Runs at PST_TIMER, so it may
// wait with pif_Delay1ms(). On FALSE, pif_error holds the cause, E_OUT_OF_HEAP
// meaning PIF_HEAP_SIZE is too small for the slot counts above, and pif_Init()
// undoes the task manager through pif_Exit().
BOOL pifLinker_Setup(void);

// Whether pif_Init() has succeeded, so PIF may be called. pif_state starts at
// PST_NONE and only pif_Init() raises it to PST_RUNNING, after every setup has
// succeeded; a failed setup drops it back through pif_Exit(). Callers that
// only ever run after a successful init do not need it; scheduler.c does,
// because schedulerInit() would otherwise register tasks on a task manager
// that was never created and hand PIF a clock callback it has not got.
bool pifLinker_IsReady(void);

#endif  // PIF_LINKER_H
