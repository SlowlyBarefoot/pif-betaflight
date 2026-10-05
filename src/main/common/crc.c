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

#include <stdint.h>

#include "common/crc.h"

#include "platform.h"

#include "streambuf.h"

#include "core/pif.h"

// The CRCs are PIF's pifCrc16_*, pifCrc8_* and pifCheckXor_*. Those take
// 16-bit lengths, so longer blocks are fed in pieces.

#define CRC_CHUNK_MAX   0xFFFF

uint16_t crc16_ccitt(uint16_t crc, unsigned char a)
{
    return pifCrc16_Add(crc, a);
}

uint16_t crc16_ccitt_update(uint16_t crc, const void *data, uint32_t length)
{
    const uint8_t *p = (const uint8_t *)data;

    while (length) {
        const uint16_t chunk = length > CRC_CHUNK_MAX ? CRC_CHUNK_MAX : length;
        crc = pifCrc16_Update(crc, p, chunk);
        p += chunk;
        length -= chunk;
    }
    return crc;
}

void crc16_ccitt_sbuf_append(sbuf_t *dst, uint8_t *start)
{
    const uint16_t crc = crc16_ccitt_update(0, start, sbufPtr(dst) - start);
    sbufWriteU16(dst, crc);
}

uint8_t crc8_calc(uint8_t crc, unsigned char a, uint8_t poly)
{
    return pifCrc8_Add(crc, a, poly);
}

uint8_t crc8_update(uint8_t crc, const void *data, uint32_t length, uint8_t poly)
{
    const uint8_t *p = (const uint8_t *)data;

    while (length) {
        const uint16_t chunk = length > CRC_CHUNK_MAX ? CRC_CHUNK_MAX : length;
        crc = pifCrc8_Update(crc, p, chunk, poly);
        p += chunk;
        length -= chunk;
    }
    return crc;
}

void crc8_sbuf_append(sbuf_t *dst, uint8_t *start, uint8_t poly)
{
    const uint8_t crc = crc8_update(0, start, sbufPtr(dst) - start, poly);
    sbufWriteU8(dst, crc);
}

uint8_t crc8_xor_update(uint8_t crc, const void *data, uint32_t length)
{
    const uint8_t *p = (const uint8_t *)data;

    while (length) {
        const uint16_t chunk = length > CRC_CHUNK_MAX ? CRC_CHUNK_MAX : length;
        crc = pifCheckXor_Update(crc, p, chunk);
        p += chunk;
        length -= chunk;
    }
    return crc;
}

void crc8_xor_sbuf_append(sbuf_t *dst, uint8_t *start)
{
    const uint8_t crc = crc8_xor_update(0, start, sbufPtr(dst) - start);
    sbufWriteU8(dst, crc);
}

// Fowler–Noll–Vo hash function; see https://en.wikipedia.org/wiki/Fowler–Noll–Vo_hash_function
uint32_t fnv_update(uint32_t hash, const void *data, uint32_t length)
{
    const uint8_t *p = (const uint8_t *)data;
    const uint8_t *pend = p + length;

    for (; p != pend; p++) {
        hash *= FNV_PRIME;
        hash ^= *p;
    }

    return hash;
}

