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

#define USE_TARGET_CONFIG

#define TARGET_BOARD_IDENTIFIER "N411" // STM32 Nucleo F411RE
#define USBD_PRODUCT_STRING     "NucleoF411RE"

// PA5 is also SPI1_SCK, which only the nRF24 RX_SPI receiver would use.
#define LED0_PIN                PA5  // D13, onboard LED LD2
#define LED1_PIN                PB5  // D4
#define LED2_PIN                PA4  // A2

// Active buzzer on PC9 (CN10 pin 1), driven push-pull and high to sound, as
// a buzzer module or an NPN transistor stage wants. The F411RE has no PD12.
#define USE_BEEPER
#define BEEPER_PIN              PC9
#define BEEPER_INVERTED

#define USE_EXTI

#define USE_SPI
//#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2

#define SPI2_NSS_PIN PB12
#define SPI2_SCK_PIN PB13
#define SPI2_MISO_PIN PB14
#define SPI2_MOSI_PIN PB15

// GY-86 on I2C1 (PB8 SCL / PB9 SDA, the D15 / D14 pins of the Arduino header):
// MPU6050 at 0x68, MS5611 at 0x77, and HMC5883L at 0x1E behind the MPU6050's
// auxiliary bus, which the MPU6050 driver opens with I2C_BYPASS_EN.
// I2C gyros are only set up on single gyro builds (pg/gyrodev.c), and
// common_pre.h turns multi gyro on for every target with this much flash.
#undef USE_MULTI_GYRO

#define USE_GYRO
#define USE_FAKE_GYRO
#define USE_GYRO_MPU6050

#define USE_ACC
#define USE_FAKE_ACC
#define USE_ACC_MPU6050

#define USE_EXTI

// GY-86 INT on PB4 (D5), the data ready interrupt of the MPU6050. Off: for an
// I2C gyro it only sets gyroDev_t.dataReady, which nothing reads, so it would
// cost an interrupt per sample for nothing. PC13 is not used: it is the user
// button B1, whose RC debounce filter would slow the 50us INT pulse, and it is
// left out of TARGET_IO_PORTC.
//#define USE_GYRO_EXTI
//#define GYRO_1_EXTI_PIN         PB4
//#define USE_MPU_DATA_READY_SIGNAL
//#define ENSURE_MPU_DATA_READY_IS_LOW

#define USE_BARO
#define USE_FAKE_BARO
//#define USE_BARO_BMP085
//#define USE_BARO_BMP280
#define USE_BARO_MS5611

//#define USE_MAX7456
//#define MAX7456_SPI_INSTANCE    SPI2
//#define MAX7456_SPI_CS_PIN      SPI2_NSS_PIN

#define USE_CMS

//#define USE_SDCARD
//#define SDCARD_SPI_INSTANCE     SPI2
//#define SDCARD_SPI_CS_PIN       PB12
//// Note, this is the same DMA channel as UART1_RX. Luckily we don't use DMA for USART Rx.
//#define SDCARD_DMA_CHANNEL_TX               DMA1_Channel5
// Performance logging for SD card operations:
// #define AFATFS_USE_INTROSPECTIVE_LOGGING

#define USE_MAG
#define USE_FAKE_MAG
//#define USE_MAG_AK8963
//#define USE_MAG_AK8975
#define USE_MAG_HMC5883

#define USE_RX_SPI
#define RX_SPI_INSTANCE         SPI1
// Nordic Semiconductor uses 'CSN', STM uses 'NSS'
#define RX_CE_PIN               PC7 // D9
#define RX_NSS_PIN              PB6 // D10
// NUCLEO has NSS on PB6, rather than the standard PA4

#define SPI1_NSS_PIN            RX_NSS_PIN
#define SPI1_SCK_PIN            PA5 // D13
#define SPI1_MISO_PIN           PA6 // D12
#define SPI1_MOSI_PIN           PA7 // D11

#define USE_RX_NRF24
#define USE_RX_CX10
#define USE_RX_H8_3D
#define USE_RX_INAV
#define USE_RX_SYMA
#define USE_RX_V202
#define RX_SPI_DEFAULT_PROTOCOL RX_SPI_NRF24_H8_3D

// No USB VCP: the Nucleo has no USB connector on PA11 / PA12, which carry
// USART6 instead. The ST-LINK virtual COM port is USART2 (PA2 / PA3).
//#define USE_VCP

#define USE_UART1
#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

#define USE_UART2
#define UART2_TX_PIN            PA2
#define UART2_RX_PIN            PA3

// The F411 has no USART3, UART4 or UART5. USART6 goes on PA11 / PA12 (CN10
// pins 14 / 12), as its other pins PC6 / PC7 are motors.
#define USE_UART6
#define UART6_TX_PIN            PA11
#define UART6_RX_PIN            PA12

// Bluetooth serial module (HC-05) for MSP on SOFTSERIAL1, at 19200 baud
// (config.c). Only RX needs a timer channel (PA1 is TIM2 CH2 in target.c);
// TX is a plain GPIO clocked by the same timer. USART2 cannot take it: its
// PA2 / PA3 are wired to the ST-LINK virtual COM port.
#define USE_SOFTSERIAL1
#define SOFTSERIAL1_RX_PIN      PA1  // A1
#define SOFTSERIAL1_TX_PIN      PA15 // CN7 pin 17
#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       5 // USART1, USART2, USART6, SOFTSERIAL1, SOFTSERIAL2

// USART1 the serial RC receiver (iBUS), USART2 MSP, USART6 GPS and
// SOFTSERIAL1 MSP over Bluetooth (config.c).
// A PPM receiver can be tried on PA0 (A0, TIM5 CH1) by setting the RX_PPM
// feature in place of RX_SERIAL.
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_IBUS
#define DEFAULT_FEATURES        (FEATURE_GPS | FEATURE_SOFTSERIAL)

#define USE_ESCSERIAL
//#define ESCSERIAL_TIMER_TX_PIN  PB8  // (HARDARE=0,PPM), now I2C1 SCL

#define USE_I2C

#define USE_I2C_DEVICE_1
#define I2C1_SCL                PB8  // D15
#define I2C1_SDA                PB9  // D14
// 400kHz rather than the 800kHz default: the HMC5883L, MS5611 and MPU6050 of
// the GY-86 are all specified up to 400kHz, and the board hangs on jumper wires.
#define I2C1_CLOCKSPEED         400

#define USE_I2C_DEVICE_2
// EEPROM on I2C2 (PB10 SCL / PB3 SDA, the D6 / D3 pins of the Arduino header;
// PB3 is I2C2_SDA on AF9, which only the F401 / F411 have).
#define I2C2_SCL                PB10 // D6
#define I2C2_SDA                PB3  // D3
#define I2C2_CLOCKSPEED         400

#define USE_I2C_DEVICE_3
#define I2C3_SCL                NONE // PA8
#define I2C3_SDA                NONE // PC9
#define I2C_DEVICE              (I2CDEV_1)

#define USE_ADC
#define ADC_INSTANCE            ADC1
#define ADC1_DMA_OPT            1  // DMA 2 Stream 4 Channel 0 (compat default)
// The F411 has ADC1 only.
#define VBAT_ADC_PIN            PC0
#define CURRENT_METER_ADC_PIN   PC1
#define RSSI_ADC_PIN            PC2
#define EXTERNAL1_ADC_PIN       PC3

// The battery voltage is read on PC0 (A5) through a 10k / 1k divider for the
// default vbat_scale of 110. The current sensor on PC1 (A4) is left off, as
// a floating pin would read noise; set current_meter = ADC once one is wired.
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC

#define USE_SONAR
#define SONAR_TRIGGER_PIN       PB0
#define SONAR_ECHO_PIN          PB1

#define MAX_SUPPORTED_MOTORS    12

#define TARGET_IO_PORTA (0xffff & ~(BIT(14)|BIT(13)))
#define TARGET_IO_PORTB (0xffff & ~(BIT(2)))
#define TARGET_IO_PORTC (0xffff & ~(BIT(15)|BIT(14)|BIT(13)))
#define TARGET_IO_PORTD BIT(2)

#define USABLE_TIMER_CHANNEL_COUNT 8
#define USED_TIMERS             (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(5))
