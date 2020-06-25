#ifndef _TASK_IMU_H_INCLUDED
#define _TASK_IMU_H_INCLUDED




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

	#include "i2c.h"
}

/**
 * imu function, responsible for setting up the imu device, collecting data and sending it out to the control task 
 * @param pvParameter 
 */
void imu_task_function(void * pvParameter);

#endif /* _TASK_IMU_H_*/
