#include "spi.h"


#define SPI_INSTANCE  1 /**< SPI instance index. */
#define SPI_SCK_PIN                                 NRF_GPIO_PIN_MAP(1,15)
#define SPI_MISO_PIN                                NRF_GPIO_PIN_MAP(1,14)
#define SPI_MOSI_PIN                                NRF_GPIO_PIN_MAP(1,13)
#define SPI_SS_PIN                                  NRF_GPIO_PIN_MAP(1,10)
    
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */


ret_code_t spi_init(bool interruptEnable)
{
    ret_code_t error = NRF_SUCCESS;

    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = SPI_SS_PIN;
    spi_config.miso_pin = SPI_MISO_PIN;
    spi_config.mosi_pin = SPI_MOSI_PIN;
    spi_config.sck_pin  = SPI_SCK_PIN;
    // set to freq 2MHZ
    spi_config.frequency = NRF_DRV_SPI_FREQ_1M;
    spi_config.bit_order  = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,    
 
    error = (nrf_drv_spi_init(&spi, &spi_config, NULL, NULL));
    if(error != NRF_SUCCESS)
    {
        //
        return error; 
    }

    return error; 
}

ret_code_t spi_transfer(uint8_t const * p_tx_buffer, uint8_t tx_buffer_length, uint8_t * p_rx_buffer, uint8_t rx_buffer_length)
{
    ret_code_t error = NRF_SUCCESS;

    error = nrf_drv_spi_transfer(&spi, p_tx_buffer, tx_buffer_length, p_rx_buffer, rx_buffer_length);
    if(error != NRF_SUCCESS)
    {
        //
        return error; 
    }

    return error; 
}


