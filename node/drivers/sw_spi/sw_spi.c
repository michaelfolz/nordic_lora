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

void sw_spi_send_packet(char *data, unsigned char length)
{
	unsigned char count =0; 
	for(count =0; count < length; count++)
	{
		sw_spi_send_char(*data++);
	}

	return; 
}


// transmit byte serially, MSB first
void sw_spi_send_char(char data)
{
	 uint8_t i;

		// select device (active low)
	 SW_SPI_PIN_CLEAR(gSPI_CS);

	 // send bits 7..0
	 for (i = 0; i < 8; i++)
    {
        // consider leftmost bit
        // set line high if bit is 1, low if bit is 0
        if (data & 0x80)
        	 SW_SPI_PIN_SET(gSPI_DIO);
        else
        	 SW_SPI_PIN_CLEAR(gSPI_DIO);

        // pulse clock to indicate that bit value should be read
        SW_SPI_PIN_CLEAR(gSPI_CLOCK);
        nrf_delay_us(1);
        // shift byte left so next bit will be leftmost
        data <<= 1;
        SW_SPI_PIN_SET(gSPI_CLOCK);
        nrf_delay_us(1);
	 }

	 // deselect device
	 SW_SPI_PIN_CLEAR(gSPI_CLOCK);
	 SW_SPI_PIN_SET(gSPI_CS);
}
