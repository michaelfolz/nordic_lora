#ifndef _TASK_LORA_H_INCLUDED
#define _TASK_LORA_H_INCLUDED




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

	#include "nordic_common.h"
	#include "nrf.h"
	#include "app_error.h"
	#include "nrf.h"
	#include "nrf_delay.h"
	#include "nrf_wdt.h"
	#include "nrf_gpio.h"
	#include "app_timer.h"

	// nrf log 
	#include "nrf_log.h"
	#include "nrf_log_ctrl.h"
	#include "nrf_log_default_backends.h"


	#include "nrf_drv_gpiote.h"
	#include "app_error.h"
	#include "boards.h"

	#include "spi.h"
	#include "gpioe.h"

	#include "sw_spi.h"
}


void lora_task_function(void * pvParameter);

#endif /* _TASK_LORA_H_*/
