#ifndef _TASK_DEBUG_H_INCLUDED
#define _TASK_DEBUG_H_INCLUDED

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
 * basic debug function, used to printout memory/task usage with freeRTOS 
 * @param pvParameter 
 */
void debug_task_function(void * pvParameter);


#endif /* _TASK_DEBUG_H_*/
