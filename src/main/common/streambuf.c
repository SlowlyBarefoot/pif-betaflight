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

#include <string.h>
#include <stdint.h>

#include "platform.h"

#include "streambuf.h"

sbuf_t *sbufInit(sbuf_t *sbuf, uint8_t *ptr, uint8_t *end)
{
    const int size = end > ptr ? end - ptr : 0;

    pifStreamBuffer_Init(sbuf, ptr, size > UINT16_MAX ? UINT16_MAX : size);
    return sbuf;
}

void sbufWriteU8(sbuf_t *dst, uint8_t val)
{
    pifStreamBuffer_WriteU8(dst, val);
}

void sbufWriteU16(sbuf_t *dst, uint16_t val)
{
    pifStreamBuffer_WriteU16(dst, val);
}

void sbufWriteU32(sbuf_t *dst, uint32_t val)
{
    pifStreamBuffer_WriteU32(dst, val);
}

void sbufWriteU16BigEndian(sbuf_t *dst, uint16_t val)
{
    pifStreamBuffer_WriteU16Be(dst, val);
}

void sbufWriteU32BigEndian(sbuf_t *dst, uint32_t val)
{
    pifStreamBuffer_WriteU32Be(dst, val);
}

void sbufFill(sbuf_t *dst, uint8_t data, int len)
{
    if (len > 0) {
        pifStreamBuffer_Fill(dst, data, len);
    }
}

void sbufWriteData(sbuf_t *dst, const void *data, int len)
{
    if (len > 0) {
        pifStreamBuffer_WriteData(dst, data, len);
    }
}

void sbufWriteString(sbuf_t *dst, const char *string)
{
    pifStreamBuffer_WriteString(dst, string, FALSE);
}

void sbufWriteStringWithZeroTerminator(sbuf_t *dst, const char *string)
{
    pifStreamBuffer_WriteString(dst, string, TRUE);
}

uint8_t sbufReadU8(sbuf_t *src)
{
    return pifStreamBuffer_ReadU8(src);
}

uint16_t sbufReadU16(sbuf_t *src)
{
    return pifStreamBuffer_ReadU16(src);
}

uint32_t sbufReadU32(sbuf_t *src)
{
    return pifStreamBuffer_ReadU32(src);
}

// Unlike the original, this advances past the data read, which is what every caller expects.
void sbufReadData(sbuf_t *src, void *data, int len)
{
    if (len > 0) {
        pifStreamBuffer_ReadData(src, data, len);
    }
}

// reader - return bytes remaining in buffer
// writer - return available space
int sbufBytesRemaining(sbuf_t *buf)
{
    return pifStreamBuffer_Remaining(buf);
}

uint8_t* sbufPtr(sbuf_t *buf)
{
    return buf->_p_ptr;
}

const uint8_t* sbufConstPtr(const sbuf_t *buf)
{
    return buf->_p_ptr;
}

// advance buffer pointer
// reader - skip data
// writer - commit written data
void sbufAdvance(sbuf_t *buf, int size)
{
    if (size > 0) {
        pifStreamBuffer_Advance(buf, size);
    }
}

// modifies streambuf so that written data are prepared for reading
void sbufSwitchToReader(sbuf_t *buf, uint8_t *base)
{
    pifStreamBuffer_SwitchToReader(buf, base);
}
