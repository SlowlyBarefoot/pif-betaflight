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

#if defined(USE_DSHOT) && defined(USE_DSHOT_TELEMETRY)

// Returns the 21-bit GCR frame of the answer on pin bit of the port samples,
// PIF_DSHOT_GCR_NONE if there is none or PIF_DSHOT_GCR_INVALID.
uint32_t decode_bb_bitband( uint16_t buffer[], uint32_t count, uint32_t bit);

#endif
