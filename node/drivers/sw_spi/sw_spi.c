#include "sw_spi.h"

static uint8_t gSPI_CLOCK; 
static uint8_t gSPI_DIO; 
static uint8_t gSPI_CS; 

void sw_spi_send_initalize(uint8_t clock, uint8_t dio, uint8_t cs)
{
    gSPI_CLOCK = clock; 
    gSPI_DIO = dio;
    gSPI_CS = cs; 

	nrf_gpio_cfg_output(gSPI_CLOCK);
	nrf_gpio_cfg_output(gSPI_DIO);
	nrf_gpio_cfg_output(gSPI_CS);

	return; 
}

void sw_spi_send_packet(char *data, uint8_t length)
{
	uint8_t i =0; 
	for(i =0; i < length; i++)
	{
		sw_spi_send_char_msb(*data++);
	}

	return; 
}



void sw_spi_send_char_msb(char data)
{
	// select device (active low)
	nrf_gpio_pin_clear(gSPI_CS);

	// send bits 7..0
	for (uint8_t i = 0; i < 8; i++)
	{
		// consider leftmost bit
		// set line high if bit is 1, low if bit is 0
		
		(data & 0x80) ? nrf_gpio_pin_set(gSPI_DIO) : nrf_gpio_pin_clear(gSPI_DIO);

		// pulse clock to indicate that bit value should be read
		nrf_gpio_pin_clear(gSPI_CLOCK);
		nrf_delay_us(1);
		// shift byte left so next bit will be leftmost
		data <<= 1;
		nrf_gpio_pin_set(gSPI_CLOCK);
		nrf_delay_us(1);
	}

	// deselect device
	nrf_gpio_pin_clear(gSPI_CLOCK);
	nrf_gpio_pin_set(gSPI_CS);
	return;
}
