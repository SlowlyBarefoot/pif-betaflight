/*
 * pif_linker.c - the seam between Betaflight and PIF.
 *
 * Holds everything PIF needs from the platform: the 1 us clock, the heap, the
 * 1 ms tick and the loop. What Betaflight needs from PIF is the task manager,
 * and scheduler/scheduler.c is where that is put to work.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/time.h"

#include "pif/pif_linker.h"

#include "core/pif_timer_manager.h"


// The 1 ms timer manager. Nothing registers a PifTimer on it yet; it exists so
// that the tick path is in place and can be measured.
static PifTimerManager s_timer1ms;

// Not a PifError, so it can never be confused with one that really happened.
#define PIF_LINKER_DOWN			0xFF

// Whether PIF may be used, which the tick and the loop test. E_SUCCESS is
// written in one place only, after every init step has succeeded, so no
// failure path can leave it reading as up - that is why nothing writes it on
// the way out of a failed init. SysTick is already running by the time
// pifLinker_Init() is called, so the tick really can arrive before PIF exists
// and the starting value has to cover that window too.
static volatile uint8_t s_init_error = PIF_LINKER_DOWN;


/*
 * Heap for PIF.
 *
 * PIF builds its object arrays with calloc(). The F405 linker script reserves
 * no heap at all (_Min_Heap_Size = 0), and the _sbrk() in libnosys hands out
 * memory from _end upwards with no bound, which on this layout means growing
 * straight into the stack in CCM. Replacing _sbrk() with a fixed buffer keeps
 * PIF's memory in .bss where the linker reports it, and turns exhaustion into
 * an E_OUT_OF_HEAP from pifLinker_Init() instead of a corrupted stack.
 *
 * Nothing else in an F4 build calls malloc(), so this is PIF's heap alone.
 *
 * SITL is a hosted build with a real allocator behind it, and replacing its
 * sbrk() would take the whole process down with PIF's buffer.
 */
#ifndef SIMULATOR_BUILD

static uint8_t s_heap[PIF_HEAP_SIZE] __attribute__((aligned(8)));
static uint8_t *s_heap_end = s_heap;

void *_sbrk(int incr) __attribute__((used, externally_visible));

void *_sbrk(int incr)
{
    uint8_t *prev = s_heap_end;

    if (incr > 0) {
        if ((size_t)incr > (size_t)(s_heap + PIF_HEAP_SIZE - s_heap_end)) {
            return (void *)-1;
        }
    } else if (incr < 0) {
        if ((size_t)-incr > (size_t)(s_heap_end - s_heap)) {
            return (void *)-1;
        }
    }

    s_heap_end += incr;
    return prev;
}

#endif  // SIMULATOR_BUILD


// micros() is timeUs_t, which is uint64_t when USE_64BIT_TIME is set, while
// PIF's 1 us clock is uint32_t. Narrow it here rather than relying on the
// function pointer types matching.
static uint32_t pifActTimer1us(void)
{
    return (uint32_t)micros();
}

uint8_t pifLinker_Init(void)
{
    // A retry after a failure has to start dormant again, and the first call
    // finds it dormant already.
    s_init_error = PIF_LINKER_DOWN;
    pif_error = E_SUCCESS;

    // pif_Init() stores the clock callback; pifTaskManager_Init() below reads
    // it straight away to stamp the CPU load window, so micros() has to be
    // running by now. systemInit() has set usTicks and started SysTick.
    if (!pif_Init(pifActTimer1us)) {
        return (uint8_t)pif_error;
    }
    // The task manager has to exist before any timer manager:
    // pifTimerManager_Init() takes one of its timer process slots.
    if (!pifTaskManager_Init(PIF_TASK_SIZE, PIF_TASK_TIMER_SIZE)) {
        return (uint8_t)pif_error;
    }

    if (!pifTimerManager_Init(&s_timer1ms, PIF_ID_AUTO, 1000, PIF_TIMER_1MS_SIZE)) {
        return (uint8_t)pif_error;
    }

    s_init_error = (uint8_t)E_SUCCESS;
    return s_init_error;
}

bool pifLinker_IsReady(void)
{
    return s_init_error == (uint8_t)E_SUCCESS;
}

void pifLinker_sigTimer1ms(void)
{
    if (s_init_error != (uint8_t)E_SUCCESS) {
        return;
    }

    pif_sigTimer1ms();
    pifTimerManager_sigTick(&s_timer1ms);
}

void pifLinker_Loop(void)
{
    if (s_init_error != (uint8_t)E_SUCCESS) {
        return;
    }

    pifTaskManager_Loop();
}
