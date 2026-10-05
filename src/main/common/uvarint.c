/*
 * This file is part of Cleanflight, Betaflight and INAV.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 *
 * @author Alberto Garcia Hierro <alberto@garciahierro.com>
 */

#include "platform.h"

#include "common/uvarint.h"

#include "codec/pif_encoding.h"

// Base-128 varints by PIF's pif_encoding.

int uvarintEncode(uint32_t val, uint8_t *ptr, size_t size)
{
    const uint8_t written = pifEncoding_UvarintEncode(val, ptr, size > UINT16_MAX ? UINT16_MAX : size);
    return written ? written : -1;
}

// Returns the bytes consumed, -1 if the data ends before the varint does, or
// -2 if the varint does not fit in 32 bits.
int uvarintDecode(uint32_t *val, const uint8_t *ptr, size_t size)
{
    const uint16_t available = size > UINT16_MAX ? UINT16_MAX : size;
    const uint8_t consumed = pifEncoding_UvarintDecode(val, ptr, available);

    if (consumed) {
        return consumed;
    }
    return available < PIF_UVARINT32_MAX_SIZE ? -1 : -2;
}
