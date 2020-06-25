#include "gpioe.h"

#define PIN_IN  NRF_GPIO_PIN_MAP(1,3)
#define PIN_IN2 NRF_GPIO_PIN_MAP(1,4)

#include "sw_spi.h"

__attribute__((weak)) int16_t lora_tx_interrupt(void)
{
    return -1;
}


__attribute__((weak)) int16_t lora_rx_interrupt(void)
{
   return -1;
}


void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	sw_spi_send_char('x');
	lora_tx_interrupt();
    return;
}


void in_pin_handler2(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{

	sw_spi_send_char('y');
	lora_rx_interrupt();
    return;
}


/**
 * @brief Function for configuring: PIN_IN pin for input, PIN_OUT pin for output,
 * and configures GPIOTE to give an interrupt on pin change.
 */
void gpio_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(PIN_IN, &in_config, in_pin_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(PIN_IN, true);

    err_code = nrf_drv_gpiote_in_init(PIN_IN2, &in_config, in_pin_handler2);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(PIN_IN2, true);
}