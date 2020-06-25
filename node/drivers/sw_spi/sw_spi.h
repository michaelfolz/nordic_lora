#ifndef _SW_SPI_H_INCLUDED
#define _SW_SPI_H_INCLUDED

#include <stdbool.h>
#include <stdint.h>

#include "nrf_gpio.h"
#include "nrf_delay.h"

#define SW_SPI_PIN_SET(PIN_NBR) (nrf_gpio_pin_set(PIN_NBR))
#define SW_SPI_PIN_CLEAR(PIN_NBR) (nrf_gpio_pin_clear(PIN_NBR))

void sw_spi_send_initalize(uint8_t clock, uint8_t dio, uint8_t cs);
void sw_spi_send_packet(char *data, unsigned char length);
void sw_spi_send_char(char data);

#endif /* _SW_SPI_H_*/
