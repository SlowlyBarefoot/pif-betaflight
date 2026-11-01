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


PifImuSensor g_imu_sensor;

PifTimerManager g_timer1ms;

/*
 * Heap for PIF.
 *
 * PIF builds its object arrays with calloc(). The F405 linker script reserves
 * no heap at all (_Min_Heap_Size = 0), and the _sbrk() in libnosys hands out
 * memory from _end upwards with no bound, which on this layout means growing
 * straight into the stack in CCM. Replacing _sbrk() with a fixed buffer keeps
 * PIF's memory in .bss where the linker reports it, and turns exhaustion into
 * an E_OUT_OF_HEAP from pif_Init() instead of a corrupted stack.
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
uint32_t pifActTimer1us(void)
{
    return (uint32_t)micros();
}

BOOL pifLinker_Setup(void)
{
    // Before the sensors are detected, which is where the PIF drivers attach
    // themselves to it.
    pifImuSensor_Init(&g_imu_sensor);

    // pifTaskManager_Init() reads the clock callback straight away to stamp
    // the CPU load window; pif_Init() has stored it before calling this.
    if (!pifTaskManager_Init(PIF_TASK_SIZE, PIF_TASK_TIMER_SIZE)) {
        return FALSE;
    }

    // Keep this the last step. pif_Exit() clears only the task manager, which
    // takes the timer manager's task with it but leaves g_timer1ms holding its
    // timers, so a step that could fail after this one would have to call
    // pifTimerManager_Clear(&g_timer1ms) itself before returning FALSE.
    return pifTimerManager_Init(&g_timer1ms, PIF_ID_AUTO, 1000, PIF_TIMER_1MS_SIZE);
}

bool pifLinker_IsReady(void)
{
    return pif_state == PST_RUNNING;
}
