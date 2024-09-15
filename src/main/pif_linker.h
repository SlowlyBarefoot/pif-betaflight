#pragma once

#include "core/pif_task.h"
#include "core/pif_timer.h"
#include "sensor/pif_imu_sensor.h"


#define TASK_SIZE				35
#define TIMER_1MS_SIZE			3


extern PifImuSensor g_imu_sensor;
extern PifTimerManager g_timer_1ms;
