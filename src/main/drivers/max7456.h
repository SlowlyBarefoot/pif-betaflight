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

#include "drivers/display.h"

#include "osd/pif_max7456.h"

/** PAL or NTSC, value is number of chars total */
#define VIDEO_BUFFER_CHARS_NTSC   390
#define VIDEO_BUFFER_CHARS_PAL    480
#define VIDEO_LINES_NTSC          13
#define VIDEO_LINES_PAL           16

typedef enum {
    // IO defined and MAX7456 was detected
    MAX7456_INIT_OK = 0,
    // IO defined, but MAX7456 could not be detected (maybe not yet
    // powered on)
    MAX7456_INIT_NOT_FOUND = -1,
    // No MAX7456 IO defined, which means either the we don't have it or
    // it's not properly configured
    MAX7456_INIT_NOT_CONFIGURED = -2,
} max7456InitStatus_e;

extern PifMax7456 max7456;
extern uint16_t maxScreenSize;
struct vcdProfile_s;
void    max7456HardwareReset(void);
struct max7456Config_s;
void    max7456PreInit(const struct max7456Config_s *max7456Config);
max7456InitStatus_e max7456Init(const struct max7456Config_s *max7456Config, const struct vcdProfile_s *vcdProfile, bool cpuOverclock);
bool    max7456ReInitIfRequired(bool forceStallCheck);
bool     max7456DrawScreen(void);
uint8_t max7456GetRowsCount(void);
void    max7456Write(uint8_t x, uint8_t y, const char *buff);
void    max7456WriteChar(uint8_t x, uint8_t y, uint8_t c);
void    max7456ClearScreen(void);
void    max7456RefreshAll(void);
bool    max7456DmaInProgress(void);
bool    max7456BuffersSynced(void);
bool    max7456LayerSupported(displayPortLayer_e layer);
bool    max7456LayerSelect(displayPortLayer_e layer);
bool    max7456LayerCopy(displayPortLayer_e destLayer, displayPortLayer_e sourceLayer);
void max7456ClearShadowBuffer(void);
void    max7456SetBackgroundType(displayPortBackground_e backgroundType);
