#pragma once

#include "core/pif_task.h"
#include "core/pif_timer_manager.h"
#include "sensor/pif_imu_sensor.h"


#define TASK_SIZE				40
#define TIMER_1MS_SIZE			3

#define DISALLOW_YIELD_ID_I2C   1
#define DISALLOW_YIELD_ID_SPI   2

#define PIF_ID_LOG_RX_TASK      PIF_ID_USER(0)
#define PIF_ID_LOG_TX_TASK      PIF_ID_USER(1)

#define PIF_ID_UART(N)          (PIF_ID_USER(0x10) + (N))
#define PIF_ID_UART_2_IDX(N)    ((N) - PIF_ID_USER(0x10))

#define PIF_ID_I2C(N)           (PIF_ID_USER(0x20) + (N))
#define PIF_ID_I2C_2_IDX(N)     ((N) - PIF_ID_USER(0x20))

#define PIF_ID_SPI(N)           (PIF_ID_USER(0x30) + (N))
#define PIF_ID_SPI_2_IDX(N)     ((N) - PIF_ID_USER(0x30))

#define USE_LOG

#ifdef USE_LOG
    #undef USE_RCDEVICE
#endif

extern PifImuSensor g_imu_sensor;
extern PifTimerManager g_timer_1ms;


extern BOOL logInit();
