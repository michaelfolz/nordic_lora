#ifndef _TASK_LED_H_INCLUDED
#define _TASK_LED_H_INCLUDED

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

	#include "sw_spi.h"
	#include "gpioe.h"
}

/**
 * basic led toggle function, used for testing 
 * @param pvParameter 
 */
void led_toggle_task_function(void * pvParameter);


#endif /* _TASK_LED_H_*/
