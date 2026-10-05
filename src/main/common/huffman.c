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

#ifdef USE_HUFFMAN

#include "huffman.h"

// Returns the number of bytes written, or -1 if the output buffer is too small.
int huffmanEncodeBuf(uint8_t *outBuf, int outBufLen, const uint8_t *inBuf, int inLen, const huffmanTable_t *huffmanTable)
{
    PifHuffmanEncoder encoder;

    pifHuffman_InitEncoder(&encoder, huffmanTable, outBuf, outBufLen > UINT16_MAX ? UINT16_MAX : outBufLen);
    if (inLen > UINT16_MAX || !pifHuffman_Encode(&encoder, inBuf, inLen)) {
        return -1;
    }
    return encoder._bytes;
}

#endif
