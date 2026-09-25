###############################################################################
# PIF (Platform-independent framework)
#
# PIF is developed in its own repository alongside this fork, so it is not
# vendored into lib/. The default layout is
#
#     <parent>/pif             <- PIF
#     <parent>/pif-betaflight  <- this repository
#
# Set PIF_DIR in make/local.mk (gitignored) if PIF lives somewhere else:
#
#     PIF_DIR := /path/to/pif
#
# Only the sources needed so far are built: the core, pif_i2c, pif_imu_sensor
# and pif_qmc5883 for the QMC5883L compass driver, pif_spi and pif_dps310*
# for the DPS310 barometer driver, pif_max7456 for the MAX7456 OSD driver,
# pif_rc and pif_rc_ppm for the PPM receiver input, pif_ring_buffer and
# pif_uart for the hardware UART driver, pif_rc_ibus with them for the iBUS
# receiver and telemetry, pif_buzzer for the beeper, and pif_bmi270 and
# pif_bmi270_spi for the BMI270 gyro and accelerometer, pif_gps and
# pif_gps_ublox for the NMEA and UBX GPS parsers, and pif_msp and pif_msp_v2
# for the MSP serial ports. pif_log.c is left out
# because PIF_NO_LOG is set in src/main/pif/pif_conf.h.
###############################################################################

PIF_DIR ?= $(ROOT)/../pif

ifeq ($(wildcard $(PIF_DIR)/include/core/pif.h),)
$(error PIF sources not found under PIF_DIR=$(PIF_DIR). Clone PIF next to this repository or set PIF_DIR in make/local.mk)
endif

# $(PIF_DIR)/include resolves "core/pif.h"; src/main/pif holds pif_conf.h,
# which core/pif.h includes by that bare name.
INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(PIF_DIR)/include \
                   $(ROOT)/src/main/pif

VPATH           := $(VPATH):$(PIF_DIR)/source/core:$(PIF_DIR)/source/communication:$(PIF_DIR)/source/sensor:$(PIF_DIR)/source/osd:$(PIF_DIR)/source/rc:$(PIF_DIR)/source/sound:$(PIF_DIR)/source/gps:$(PIF_DIR)/source/protocol:$(ROOT)/src/main/pif

PIF_SRC = \
            pif.c \
            pif_obj_array.c \
            pif_ring_buffer.c \
            pif_task.c \
            pif_task_manager.c \
            pif_timer.c \
            pif_timer_manager.c \
            pif_i2c.c \
            pif_uart.c \
            pif_imu_sensor.c \
            pif_qmc5883.c \
            pif_spi.c \
            pif_dps310.c \
            pif_dps310_i2c.c \
            pif_dps310_spi.c \
            pif_bmi270.c \
            pif_bmi270_spi.c \
            pif_max7456.c \
            pif_rc.c \
            pif_rc_ibus.c \
            pif_rc_ppm.c \
            pif_buzzer.c \
            pif_gps.c \
            pif_gps_ublox.c \
            pif_msp.c \
            pif_msp_v2.c \
            pif_linker.c

SRC += $(PIF_SRC)
