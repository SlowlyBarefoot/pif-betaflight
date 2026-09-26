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

#pragma once

#include <stdint.h>
#include <stdbool.h>

// Streams data out to the EEPROM, padding to the write size as
// needed, and updating the checksum as it goes.

#include "storage/pif_flash.h"

// The program unit of the storage: what a single program operation writes.
#ifdef CONFIG_IN_EXTERNAL_FLASH
#define CONFIG_STREAMER_BUFFER_SIZE 8 // Must not be greater than the smallest flash page size of all compiled-in flash devices.
#elif defined(STM32H743xx) || defined(STM32H750xx) || defined(STM32H723xx) || defined(STM32H725xx)
#define CONFIG_STREAMER_BUFFER_SIZE 32  // Flash word = 256-bits
#elif defined(STM32H7A3xx) || defined(STM32H7A3xxQ)
#define CONFIG_STREAMER_BUFFER_SIZE 16  // Flash word = 128-bits
#elif defined(STM32G4)
#define CONFIG_STREAMER_BUFFER_SIZE 8   // Flash word = 64-bits
#else
#define CONFIG_STREAMER_BUFFER_SIZE 4
#endif

typedef struct config_streamer_s {
    PifFlash flash;
    int err;
} config_streamer_t;

void config_streamer_init(config_streamer_t *c);

void config_streamer_start(config_streamer_t *c, uintptr_t base, int size);
int config_streamer_write(config_streamer_t *c, const uint8_t *p, uint32_t size);
int config_streamer_flush(config_streamer_t *c);

int config_streamer_finish(config_streamer_t *c);
int config_streamer_status(config_streamer_t *c);
