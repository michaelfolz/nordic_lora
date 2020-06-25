#ifndef _GPIOE_H_INCLUDED
#define _GPIOE_H_INCLUDED

#include <stdbool.h>
#include <stdint.h>

#include "bsp.h"
#include "sdk_errors.h"
#include "nrf_gpio.h"
#include "app_timer.h"

// nrf log 
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#include "nrf_drv_gpiote.h"
#include "app_error.h"
#include "boards.h"

#include "sw_spi.h"

#ifdef __cplusplus
extern "C"
{
#endif

	void error_panic(char *data, unsigned char length);

#ifdef __cplusplus
}
#endif


#endif /* _GPIOE */
