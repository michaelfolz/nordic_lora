#ifndef _TASK_BLE_H_INCLUDED
#define _TASK_BLE_H_INCLUDED



#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

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
	#include "ble.h"
	#include "ble_hci.h"
	#include "ble_srv_common.h"
	#include "ble_advdata.h"
	#include "ble_conn_params.h"
	#include "app_timer.h"
	#include "app_button.h"
	#include "ble_lbs.h"
	#include "bsp.h"
	#include "ble_gap.h"

	#include "nrf.h"
	#include "nrf_delay.h"
	#include "nrf_wdt.h"
	#include "nrf_gpio.h"
	#include "app_timer.h"
	#include "nrf_pwr_mgmt.h"


	// nrf log 
	#include "nrf_log.h"
	#include "nrf_log_ctrl.h"
	#include "nrf_log_default_backends.h"

	#include "nrf_ble.h"
}

/**
 * ble task responsible for handelling all ble communications in/out 
 * @param pvParameter 
 */
void ble_task_function(void * pvParameter);


#endif /* _TASK_BLE_H_*/
