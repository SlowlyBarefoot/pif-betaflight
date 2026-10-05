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

#include "core/pif_stream_buffer.h"

// Buffer-based serializer/deserializer on PIF's PifStreamBuffer. A write or
// read past the end does nothing and sets _overflow (a read returns 0).
// The data pointer comes first, so sbuf_t* is still equivalent to uint8_t **.

typedef PifStreamBuffer sbuf_t;

// Initializer for an sbuf_t over [start, stop), for use in aggregate initializers.
#define SBUF_INIT(start, stop) { ._p_ptr = (start), ._p_end = (stop), ._overflow = FALSE }

sbuf_t *sbufInit(sbuf_t *sbuf, uint8_t *ptr, uint8_t *end);

void sbufWriteU8(sbuf_t *dst, uint8_t val);
void sbufWriteU16(sbuf_t *dst, uint16_t val);
void sbufWriteU32(sbuf_t *dst, uint32_t val);
void sbufWriteU16BigEndian(sbuf_t *dst, uint16_t val);
void sbufWriteU32BigEndian(sbuf_t *dst, uint32_t val);
void sbufFill(sbuf_t *dst, uint8_t data, int len);
void sbufWriteData(sbuf_t *dst, const void *data, int len);
void sbufWriteString(sbuf_t *dst, const char *string);
void sbufWriteStringWithZeroTerminator(sbuf_t *dst, const char *string);

uint8_t sbufReadU8(sbuf_t *src);
uint16_t sbufReadU16(sbuf_t *src);
uint32_t sbufReadU32(sbuf_t *src);
void sbufReadData(sbuf_t *dst, void *data, int len);

int sbufBytesRemaining(sbuf_t *buf);
uint8_t* sbufPtr(sbuf_t *buf);
const uint8_t* sbufConstPtr(const sbuf_t *buf);
void sbufAdvance(sbuf_t *buf, int size);

void sbufSwitchToReader(sbuf_t *buf, uint8_t * base);
