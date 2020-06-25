#ifndef _TASK_USBD_H_INCLUDED
#define _TASK_USBD_H_INCLUDED

extern "C" {
	// Nordic & External Libraries Includes 
	#include "FreeRTOS.h"
	#include "task.h"
	#include "timers.h"
	#include "bsp.h"
	#include "nordic_common.h"
	#include "nrf_drv_clock.h"
	#include "sdk_errors.h"
	#include "app_error.h"

	// nrf log 
	#include "nrf_log.h"
	#include "nrf_log_ctrl.h"
	#include "nrf_log_default_backends.h"
}


void usbd_task_function(void * pvParameter);


#endif /* _TASK_USBD_H_*/
