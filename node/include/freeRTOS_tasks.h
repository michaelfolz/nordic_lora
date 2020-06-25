#ifndef _FREERTOS_TASKS_H_INCLUDED
#define _FREERTOS_TASKS_H_INCLUDED

// Nordic & External Libraries Includes 
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "bsp.h"
#include "nordic_common.h"
#include "nrf_drv_clock.h"
#include "sdk_errors.h"
#include "app_error.h"

// task includes 
#include "task_led.h"
#include "task_ble.h"
#include "task_debug.h"
#include "task_imu.h"
#include "task_control.h"
#include "task_lora.h"
#include "task_usbd.h"

int32_t freeRTOS_StartTasks(void);

#endif /* _FREERTOS_TASKS_H_*/
