
#include "i2c.h"

/* TWI instance ID. */
#define TWI_CONTROLLER_1_INSTANCE_ID     0


/* TWI instance. */
static const nrf_drv_twi_t gTWI_Instance = NRF_DRV_TWI_INSTANCE(TWI_CONTROLLER_1_INSTANCE_ID);

// global intance config for controller 1 
const nrf_drv_twi_config_t gTWI_Instance_1_Config =
{
   .scl                = 13,
   .sda                = 15,
   .frequency          = TWI_DEFAULT_CONFIG_FREQUENCY,
   .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
   .clear_bus_init     = true, //TWI_DEFAULT_CONFIG_CLR_BUS_INIT,
   .hold_bus_uninit    = TWI_DEFAULT_CONFIG_HOLD_BUS_UNINIT
};


/**
 * @brief Initialize the master TWI.
 *
 * Function used to initialize the master TWI interface that would communicate with simulated EEPROM.
 *
 * @return NRF_SUCCESS or the reason of failure.
 */
static ret_code_t twi_master_init(nrf_drv_twi_t instance, nrf_drv_twi_config_t config )
{
    ret_code_t ret;

    ret = nrf_drv_twi_init(&instance, &config, NULL, NULL);

    if (NRF_SUCCESS == ret)
    {
        nrf_drv_twi_enable(&instance);
    }

    return ret;
}


/**
 * external facing function responsible for setting up both TWI connections. 
 * @return  error state. 
 */
ret_code_t twi_init(void)
{
    ret_code_t ret;

    ret = twi_master_init(gTWI_Instance, gTWI_Instance_1_Config);
    if (NRF_SUCCESS != ret)
    {
        // do something
    }


    return ret;
}

/**
 *  Function used to send out a read request for a given length of data
 * @param  addr       desired i2c address
 * @param  p_data     pointer to the data
 * @param  length     length of the read
 * @return            error state
 */
ret_code_t twi_rx(uint8_t addr, uint8_t *p_data, uint8_t length)
{
    ret_code_t ret;

    ret = nrf_drv_twi_rx(&gTWI_Instance, addr, p_data, length);
    if (NRF_SUCCESS != ret)
    {
        // do something
    }

    return ret;
}

/**
 *  Function used to send out a write request for a given length of data
 * @param  addr       desired i2c address
 * @param  p_data     pointer to the data
 * @param  length     length of the read
 * @return            error state
 */
ret_code_t twi_tx(uint8_t addr, uint8_t *p_data, uint8_t length)
{
    ret_code_t ret;

    ret = nrf_drv_twi_tx(&gTWI_Instance, addr, p_data, length, false );
    if (NRF_SUCCESS != ret)
    {
        // do something
    }

    return ret;
}
