#ifndef _SW_SPI_H_INCLUDED
#define _SW_SPI_H_INCLUDED

#include <stdbool.h>
#include <stdint.h>

#include "nrf_gpio.h"
#include "nrf_delay.h"


#ifdef __cplusplus
extern "C"
{
#endif

	void sw_spi_send_initalize(uint8_t clock, uint8_t dio, uint8_t cs);
	void sw_spi_send_packet(char *data, uint8_t length);
	void sw_spi_send_char_msb(char data);

#ifdef __cplusplus
}
#endif

#endif /* _SW_SPI_H_*/
