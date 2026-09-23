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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "fc/init.h"

#include "pif/pif_linker.h"

void run(void);

int main(void)
{
    init();

    run();

    return 0;
}

void FAST_CODE run(void)
{
    while (true) {
        // The scheduler. Every Betaflight task is a PifTask (see
        // src/main/scheduler/scheduler.c), so one pass of the PIF task manager
        // dispatches at most one of them, runs the check functions of the
        // event driven tasks when it dispatched none, and closes the CPU load
        // window once a second.
        pifLinker_Loop();
#ifdef SIMULATOR_BUILD
        delayMicroseconds_real(50); // max rate 20kHz
#endif
    }
}
