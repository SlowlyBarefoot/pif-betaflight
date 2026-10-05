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

#include "codec/pif_huffman.h"

// The table is in PIF's PifHuffmanCode format; encode with PIF's
// pifHuffman_InitEncoder() and pifHuffman_Encode(), or huffmanEncodeBuf().
#define HUFFMAN_TABLE_SIZE PIF_HUFFMAN_SYMBOLS
typedef PifHuffmanCode huffmanTable_t;

extern const huffmanTable_t huffmanTable[HUFFMAN_TABLE_SIZE];

struct huffmanInfo_s {
    uint16_t uncompressedByteCount;
};

#define HUFFMAN_INFO_SIZE sizeof(struct huffmanInfo_s)

int huffmanEncodeBuf(uint8_t *outBuf, int outBufLen, const uint8_t *inBuf, int inLen, const huffmanTable_t *huffmanTable);
