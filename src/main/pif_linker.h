#pragma once

#include "core/pif_task.h"
#include "core/pif_timer.h"
#include "sensor/pif_imu_sensor.h"


#define TASK_SIZE				35
#define TIMER_1MS_SIZE			3

#define DISALLOW_YIELD_ID_I2C   1
#define DISALLOW_YIELD_ID_SPI   2

#define PIF_ID_UART(N)              (0x100 + (N))
#define PIF_ID_UART_2_IDX(N)        ((N) - 0x100)


extern PifImuSensor g_imu_sensor;
extern PifTimerManager g_timer_1ms;
