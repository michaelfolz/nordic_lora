#ifndef _TASK_CONTROL_H_INCLUDED
#define _TASK_CONTROL_H_INCLUDED

// Nordic & External Libraries Includes 
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "bsp.h"
#include "nordic_common.h"
#include "nrf_drv_clock.h"
#include "sdk_errors.h"
#include "app_error.h"

/**
 * main control task, responsible for controlling all IO
 * @param pvParameter 
 */
void control_task_function(void * pvParameter);


#endif /* _TASK_CONTROL_H_*/
