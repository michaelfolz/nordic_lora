#ifndef _SPI_H_
#define _SPI_H_

#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"

#include "bsp.h"
#include "app_util_platform.h"
#include "app_timer.h"
#include "nrf_drv_clock.h"

// gpio drivers
#include "nrf_drv_gpiote.h"
#include "nrf_gpiote.h"
#include "nrf_gpio.h"

#include "nrf_drv_spi.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"

#ifdef __cplusplus
extern "C"
{
#endif
	ret_code_t spi_init(bool interruptEnable);
	ret_code_t spi_transfer(uint8_t const * p_tx_buffer, uint8_t tx_buffer_length, uint8_t * p_rx_buffer, uint8_t rx_buffer_length);

#ifdef __cplusplus
}
#endif



#endif /* _SPI_H_*/
