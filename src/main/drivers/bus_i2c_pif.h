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

/*
 * bus_i2c_pif.h - a PifI2cPort for each Betaflight I2C bus.
 *
 * PIF sensor drivers talk to their chip through a PifI2cPort. This gives each
 * I2CDevice one on top of drivers/bus_i2c.h, so a PIF driver shares the bus
 * with the Betaflight drivers on it and works on every I2C implementation.
 * Transfers start with i2cReadBuffer()/i2cWriteBuffer() and are polled to the
 * end through act_check, which lets pifI2cDevice_StartRead() run without
 * blocking. A transfer that times out is handed to i2cRecover() through
 * act_recover.
 */

#pragma once

#include "drivers/bus_i2c.h"

#include "communication/pif_i2c.h"

// Number of PifI2cDevice slots on each port.
#define I2C_PIF_DEVICE_COUNT    4

// Returns the PifI2cPort of an I2C bus, initialising it on first use, or NULL
// if the bus is invalid or the port could not be allocated. The bus itself
// must already have been brought up with i2cInit().
PifI2cPort *i2cPifPort(I2CDevice device);
