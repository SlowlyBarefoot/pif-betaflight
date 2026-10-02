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

#include "platform.h"

#ifdef USE_TARGET_CONFIG

#include "io/serial.h"

#include "config_helper.h"

static targetSerialPortFunction_t targetSerialPortFunction[] = {
    { SERIAL_PORT_USART1, FUNCTION_RX_SERIAL },
    { SERIAL_PORT_USART2, FUNCTION_MSP },       // ST-LINK virtual COM port
    { SERIAL_PORT_USART6, FUNCTION_GPS },
    { SERIAL_PORT_SOFTSERIAL1, FUNCTION_MSP },  // Bluetooth
};

void targetConfiguration(void)
{
    targetSerialPortFunctionConfig(targetSerialPortFunction, ARRAYLEN(targetSerialPortFunction));

    // Soft serial is only good up to 19200 baud; set the HC-05 to match
    // (AT+UART=19200,0,0).
    const int softSerialIndex = findSerialPortIndexByIdentifier(SERIAL_PORT_SOFTSERIAL1);
    if (softSerialIndex >= 0) {
        serialConfigMutable()->portConfigs[softSerialIndex].msp_baudrateIndex = BAUD_19200;
    }
}
#endif
