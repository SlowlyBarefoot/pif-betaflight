/*
 * pif_linker.h - the seam between Betaflight and PIF.
 *
 * Deliberately free of PIF headers. core/pif.h defines MIN, MAX, ABS, BOOL,
 * TRUE and FALSE, and common/maths.h defines MIN, MAX and ABS differently, so
 * a translation unit that pulls in both gets a redefinition and silently
 * changes which one it uses. Betaflight sources include this file; only
 * pif_linker.c includes PIF itself.
 */

#ifndef PIF_LINKER_H
#define PIF_LINKER_H

#include <stdint.h>

// Number of PifTask slots the task manager is created with. TASK_COUNT in
// scheduler.h reaches 34 with every feature compiled in, and PIF modules
// bring tasks of their own, so this leaves room above that. A task that finds
// no slot is simply not registered - pifTaskManager_Add() returns NULL and
// says nothing - so the headroom matters more than the few bytes it costs.
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
//
// sizeof(PifTask) is 76 bytes as configured today and grows to 140 once
// PIF_USE_TASK_STATISTICS and PIF_USE_BLOCK_TIME are turned on in pif_conf.h,
// which is the case this is sized for: about 6.4 KB at 40 tasks. Enabling
// those options must not need a second edit here.
#define PIF_HEAP_SIZE			8192

// Initialises pif, the task manager and the 1 ms timer manager, in that order.
// Must run after systemInit(), because pif is handed micros() as its 1 us
// clock, and before the first pifLinker_sigTimer1ms().
//
// Returns 0 (E_SUCCESS) when all three came up, otherwise the PifError of the
// step that failed. 3 is E_OUT_OF_HEAP, which means PIF_HEAP_SIZE is too small
// for the slot counts above. The tick and the loop below stay dormant unless
// this returned 0.
uint8_t pifLinker_Init(void);

// 1 ms tick. Called from SysTick_Handler(); advances the pif clock and the
// 1 ms timer manager. Does nothing until pifLinker_Init() has succeeded.
void pifLinker_sigTimer1ms(void);

// One pass of the PIF scheduler. Called from run() in main.c, next to
// Betaflight's scheduler(), so PIF turns over at the same rate. This is what
// dispatches the work the 1 ms tick only marked as due, and what closes the
// CPU load window. Does nothing until pifLinker_Init() has succeeded.
void pifLinker_Loop(void);

#endif  // PIF_LINKER_H
