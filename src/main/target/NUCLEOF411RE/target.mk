F411_TARGETS  += $(TARGET)
FEATURES    =

TARGET_SRC = \
            drivers/accgyro/accgyro_mpu6050.c \
            drivers/accgyro/accgyro_fake.c \
            drivers/barometer/barometer_ms5611.c \
            drivers/barometer/barometer_fake.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/compass/compass_fake.c \
            drivers/rx/rx_nrf24l01.c \
            rx/nrf24_cx10.c \
            rx/nrf24_inav.c \
            rx/nrf24_h8_3d.c \
            rx/nrf24_syma.c \
            rx/nrf24_v202.c
