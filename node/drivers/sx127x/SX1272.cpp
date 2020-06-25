
#include "SX1272.h"
#include "sw_spi.h"
                                    // power, chan, mode, cw, current
SX127X_LoRA_Settings LoRA_Default = { SX127X_POWER_MAXIMUM, CH_10_868 , 4, 0x12, 0x1B};


#define HIGH 1
#define LOW  0
#define OUTPUT 1

const uint8_t SX127X_SpreadFactor[SX127X_MODES] =
{
    0x00,          // not a valid SF
    (SF_12),       // SF = 12
    (SF_12),       // SF = 12
    (SF_10),       // SF = 10
    (SF_12),       // SF = 12
    (SF_10),       // SF = 10
    (SF_11),       // SF = 11
    (SF_9),        // SF = 9
    (SF_9),        // SF = 9
    (SF_8),        // SF = 8
    (SF_7)         // SF = 7
};

const uint8_t SX127X_Bandwidth[SX127X_MODES] = 
{
    0x00,                // not a valid bw
    (BW_125),            // BW = 125 KHz           
    (BW_250),            // BW = 250 KHz           
    (BW_125),            // BW = 125 KHz           
    (BW_500),            // BW = 500 KHz           
    (BW_250),            // BW = 250 KHz           
    (BW_500),            // BW = 500 KHz           
    (BW_250),            // BW = 250 KHz           
    (BW_500),            // BW = 500 KHz           
    (BW_500),            // BW = 500 KHz           
    (BW_500)             // BW = 500 KHz           
};




uint8_t sx127x_print_error(const char *message, uint8_t length)
{ 
    // this function must be overridden by the application software. 
    
    sw_spi_send_packet((char*)message, length);
    return 1;
}

uint8_t spi_txrx_byte(uint8_t data)
{ 
    const uint8_t tx_buffer[2] = {data, 0x00};
    uint8_t rx_buffer[2];
    ret_code_t error; 

    error = spi_transfer(tx_buffer, 1, rx_buffer, 1);

    // this function must be overridden by the application software. 
    return rx_buffer[0];
}

uint8_t gpio_write(uint8_t pin, uint8_t data)
{ 
    ( data == HIGH) ?  nrf_gpio_pin_set(pin) :  nrf_gpio_pin_clear(pin);
    // this function must be overridden by the application software. 
    return 1;
}

uint8_t gpio_mode(uint8_t pin, uint8_t mode)
{ 

    nrf_gpio_cfg_output(pin);
    // this function must be overridden by the application software. 
    return 1;
}



uint8_t delay_ms(uint32_t delayms)
{ 
   nrf_delay_ms(delayms);
   return 1;
}


SX127X::~SX127X()
{
    return; 
}

SX127X::SX127X()
{
    // Initialize class variables
    _bandwidth = BW_125;
    _codingRate = CR_5;
    _spreadingFactor = SF_7;
    _channel = CH_12_900;
    _power = 15;
    _packetNumber = 0;
    _syncWord=0x12;
    return; 
};


#define SX127X_SS                                  NRF_GPIO_PIN_MAP(1,12)  
#define SX127X_RST                                  NRF_GPIO_PIN_MAP(1,6)  

#define SX127X_ERROR_CHIP_UNSUPPORTED           0x10
#define SX127X_ERROR_UNABLE_TO_SET_LORA_STATE   0x11

/*
 Function: Sets the module ON.
 Returns: uint8_t setLORA state
*/
uint8_t SX127X::ON()
{
    uint8_t error = 0;
    uint8_t version; 

    // Powering the module
    gpio_mode(SX127X_SS,OUTPUT);
    gpio_write(SX127X_SS,HIGH);
    delay_ms(10);
    
    gpio_mode(SX127X_RST,OUTPUT);

    // request device reset 
    gpio_write(SX127X_RST,HIGH);
    delay_ms(10);
    gpio_write(SX127X_RST,LOW);
    delay_ms(10);
        gpio_write(SX127X_RST,HIGH);
    delay_ms(10);
    // Read Version 
    version = readRegister(REG_VERSION);
    switch(version)
    {
        case SX1272_REGISTER_VERSION_CODE:
            _board = SX1272Chip;  
            break;

        case SX1276_REGISTER_VERSION_CODE:
            _board = SX1276Chip;  
            break;
        default:
            error = SX127X_ERROR_CHIP_UNSUPPORTED; 
            break; 
    }

    // if the chipset isnt supported return and print errors
    if(error != 0)
    {
        sx127x_print_error("Unsuppoted LoRA chipset", 23);
        return error; 
    }

    // set LoRa mode
    error = setLORA();
    if(error != 0)
    {
        sx127x_print_error("Unable to setup LoRA status", 27);
        return error; 
    }

    // verify the current has been set 
    error = setMaxCurrent(LoRA_Default.maxCurrent);
    if(error != 0)
    {
        sx127x_print_error("Unable to set max current", 27);
        return error; 
    }

    // verify syncword has been set 
    error = setSyncWord(LoRA_Default.syncWord);
    if(error != 0)
    {
        sx127x_print_error("Unable to set syncword", 22);
        return error; 
    }

    // set the lora mode 
    error = setMode(LoRA_Default.mode);
    if(error != 0)
    {
        sx127x_print_error("Unable to set LoRA mode", 23);
        return error; 
    }
 
    // set the channel 
    error = setChannel(LoRA_Default.channelSet);
    if(error != 0)
    {
        sx127x_print_error("Unable to set Channel", 21);
        return error; 
    }

    // set power     
    error = setPower(LoRA_Default.power);
    if(error != 0)
    {
        sx127x_print_error("Unable to set power", 19);
        return error; 
    }
 
    
    return error;
}

/*
 Function: Sets the module OFF.
 Returns: Nothing
*/
void SX127X::OFF()
{
    // Powering the module
    gpio_mode(SX127X_SS,OUTPUT);
    gpio_write(SX127X_SS,LOW);
    return;
}

/*
 Function: Reads the indicated register.
 Returns: The content of the register
 Parameters:
   address: address register to read from
*/
uint8_t SX127X::readRegister(uint8_t address)
{
    uint8_t value = 0x00;

    gpio_write(SX127X_SS,LOW);
    bitClear(address, 7);       // Bit 7 cleared to write in registers
    spi_txrx_byte(address);
    value = spi_txrx_byte(0x00);
    gpio_write(SX127X_SS,HIGH);

    return value;
}

/*
 Function: Writes on the indicated register.
 Returns: Nothing
 Parameters:
   address: address register to write in
   data : value to write in the register
*/
void SX127X::writeRegister(uint8_t address, uint8_t data)
{

    gpio_write(SX127X_SS,LOW);
    bitSet(address, 7);         // Bit 7 set to read from registers
    spi_txrx_byte(address);
    spi_txrx_byte(data);
    gpio_write(SX127X_SS,HIGH);
    return; 
}

/**
 * Function: writes & read to the indicated register return 0 if equal
 * @param  address  address
 * @param  data     byte to write
 * @return          0 if equal
 */
int8_t SX127X::writeReadRegister(uint8_t address, uint8_t data)
{
    int8_t error =0; 
    uint8_t dataRead = 0;
    writeRegister(address, data);

    delay_ms(10);

    dataRead = readRegister(address);

    if(data != dataRead)
        return -1; 
    
    return error; 
}


/**
 * Function: Clears the interruption flags
 * @return Nothing
 */
int8_t SX127X::clearFlags()
{
    int8_t error =0;
    uint8_t st0;

    st0 = readRegister(REG_OP_MODE);        // Save the previous status
    
    // Stdby mode to write in registers
    error = writeReadRegister(REG_OP_MODE, LORA_STANDBY_MODE);  
    if(error !=0)
        return error;

    // LoRa mode flags register
    error = writeReadRegister(REG_IRQ_FLAGS, 0xFF); 
    if(error !=0)
        return error;

    // Getting back to previous status
    error = writeReadRegister(REG_OP_MODE, st0);   
    if(error !=0)
        return error;

    return error; 
}


/**
 * Function: requests SX127X module to be setup in lora mode 
 * @return error state
 */
int8_t SX127X::setLORA()
{
    uint8_t retry=0;
    uint8_t error = 0;
    uint8_t st0;

    do {
        writeRegister(REG_OP_MODE, FSK_SLEEP_MODE);    // Sleep mode (mandatory to set LoRa mode)
        writeRegister(REG_OP_MODE, LORA_SLEEP_MODE);    // LoRa sleep mode
        writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);
        delay_ms(200);
        // read the operation register, LoRa should exit standby mode 
        st0 = readRegister(REG_OP_MODE);
        retry++; 

        if(retry > 10)
        {
            error = SX127X_ERROR_UNABLE_TO_SET_LORA_STATE;
            break;
        }
    } while (st0!=LORA_STANDBY_MODE); 
        
    return error;
}



/**
 * Function: sets up the communications based on a desired LoRA mode
 * @param  mode  desired mode
 * @return       0 if no issue
 */
int8_t SX127X::setMode(uint8_t mode)
{
    int8_t error = 0;
    uint8_t st0;
    uint8_t spreadingFactor =0, bandwidth =0; 

    st0 = readRegister(REG_OP_MODE);        // Save the previous status
    writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);  // LoRa standby mode

    spreadingFactor = SX127X_SpreadFactor[mode];
    bandwidth = SX127X_Bandwidth[mode];
    
    // always set the coding rate to 5
    error = setCR(CR_5);    
    if(error != 0)
    {
        return error;
    }

    // set the spreading factor
    error = setSF(spreadingFactor);      
    if(error != 0)
    {
        return error;
    }

    // Set the bandwidth 
    error = setBW(bandwidth);      
    if(error != 0)
    {
        return error;
    }

    writeRegister(REG_OP_MODE, st0);    // Getting back to previous status
    delay_ms(10);
    return error;
}


/**
 *  Function: Checks if SF is a valid value.
 * @param  spr Spread factor
 * @return     true if aviailbe
 */
bool SX127X::isSF(uint8_t spr)
{

    // Checking available values for _spreadingFactor
    switch(spr)
    {
        case SF_7:
        case SF_8:
        case SF_9:
        case SF_10:
        case SF_11:
        case SF_12:
            return true;
            break;

        default:
            return false;
    }
    
    return false; 
}

/**
 * Function: Gets the SF within the module is configured.
 * @return    0 if no error
 */
int8_t  SX127X::getSF()
{
    int8_t error = 0;
    uint8_t config2;

    // take out bits 7-4 from REG_MODEM_CONFIG2 indicates _spreadingFactor
    config2 = (readRegister(REG_MODEM_CONFIG2)) >> 4;
    _spreadingFactor = config2;

    if(!isSF(_spreadingFactor) )
    {
        return -1;
    }
 
    return error;
}



/**
 * Function: Sets the indicated SF in the module.
 * @param  spr [description]
 * @return     0 if no error 
 */
int8_t SX127X::setSF(uint8_t spr)
{
    uint8_t st0;
    int8_t error = 0;
    uint8_t config1;
    uint8_t config2;

    st0 = readRegister(REG_OP_MODE);    // Save the previous status
    writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);  // LoRa standby mode

    config2 = (readRegister(REG_MODEM_CONFIG2));    // Save config2 to modify SF value (bits 7-4)

    switch(spr)
    {
        case SF_7:  
        {   
            config2 = config2 & 0x7F;  // clears bits 7 from REG_MODEM_CONFIG2
            config2 = config2 | 0x70;  // sets bits 6, 5 & 4
            break;
        }
        case SF_8:  
        {
            config2 = config2 & 0x8F;  // clears bits 6, 5 & 4 from REG_MODEM_CONFIG2
            config2 = config2 | 0x80;  // sets bit 7 from REG_MODEM_CONFIG2
            break;
        }
        case SF_9:
        {
            config2 = config2 & 0x9F;  // clears bits 6, 5 & 4 from REG_MODEM_CONFIG2
            config2 = config2 | 0x90;  // sets bits 7 & 4 from REG_MODEM_CONFIG2
            break;
        }
        case SF_10: 
        {   config2 = config2 & 0xAF;  // clears bits 6 & 4 from REG_MODEM_CONFIG2
            config2 = config2 | 0xA0;  // sets bits 7 & 5 from REG_MODEM_CONFIG2
            break;
        }
        case SF_11: 
        {   config2 = config2 & 0xBF;  // clears bit 6 from REG_MODEM_CONFIG2
            config2 = config2 | 0xB0;  // sets bits 7, 5 & 4 from REG_MODEM_CONFIG2
            break;
        }
        case SF_12: 
        {   config2 = config2 & 0xCF;  // clears bits 5 & 4 from REG_MODEM_CONFIG2
            config2 = config2 | 0xC0;  // sets bits 7 & 6 from REG_MODEM_CONFIG2
            break;
        }
    }
  
    // LoRa detection Optimize: 0x03 --> SF7 to SF12
    writeRegister(REG_DETECT_OPTIMIZE, 0x03);

    // LoRa detection threshold: 0x0A --> SF7 to SF12
    writeRegister(REG_DETECTION_THRESHOLD, 0x0A);
    
    if (_board==SX1272Chip) 
    {   
        config2 = config2 | 0x04;
        writeRegister(REG_MODEM_CONFIG1, config1);      // Update config1
    }
    else {
        // set the AgcAutoOn in bit 2 of REG_MODEM_CONFIG3
        uint8_t config3 = (readRegister(REG_MODEM_CONFIG3));
        config3=config3 | 0x04;
        writeRegister(REG_MODEM_CONFIG3, config3);
    }

    // here we write the new SF
    writeRegister(REG_MODEM_CONFIG2, config2);      // Update config2
    delay_ms(10);

    writeRegister(REG_OP_MODE, st0);    // Getting back to previous status
    delay_ms(10);

    // check if valid 
    if(!isSF(spr) )
    { 
    // Checking available value for _spreadingFactor
        return -1; 
    }
    _spreadingFactor = spr;
    
    return error;
}

/**
 * Function:   Checks if BW is a valid value.
 * @param  band desired band
 * @return      true if valid
 */
bool SX127X::isBW(uint16_t band)
{

    if (_board==SX1272Chip) {
        switch(band)
        {
            case BW_125:
            case BW_250:
            case BW_500:
                return true;
                break;

            default:
                return false;
        }
    }
    else {
        switch(band)
        {
            case BW_7_8:
            case BW_10_4:
            case BW_15_6:
            case BW_20_8:
            case BW_31_25:
            case BW_41_7:
            case BW_62_5:
            case BW_125:
            case BW_250:
            case BW_500:
                return true;
                break;

            default:
                return false;
        }
    }
    return false; 
}

/**
 * Function: Reads the current BW and returns 0 if read matches the previously set value
 * @return [description]
 */
int8_t  SX127X::getBW()
{
    uint8_t error = 0;
    uint8_t config1;

    if (_board==SX1272Chip) 
    {
        // take out bits 7-6 from REG_MODEM_CONFIG1 indicates _bandwidth
        config1 = (readRegister(REG_MODEM_CONFIG1)) >> 6;
    }
    else {
        // take out bits 7-4 from REG_MODEM_CONFIG1 indicates _bandwidth
        config1 = (readRegister(REG_MODEM_CONFIG1)) >> 4;
    }

    _bandwidth = config1;

    if(! ((config1 == _bandwidth) && isBW(_bandwidth)) )
    {
        error = -1;
    }

    return error;
}

/**
 * Function      responsible for writing the desired bandwidth to the resisters
 * @param  band  desired band
 * @return       0 if no error 
 */
int8_t  SX127X::setBW(uint16_t band)
{
    uint8_t st0;
    int8_t error = 0;
    uint8_t config1;

    if(!isBW(band))
    {
        return -1;
    }

    st0 = readRegister(REG_OP_MODE);    // Save the previous status

    writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);  // LoRa standby mode
    config1 = (readRegister(REG_MODEM_CONFIG1));    // Save config1 to modify only the BW

    // for SX1276
    if (_board==SX1272Chip) 
    {
        switch(band)
        {
            case BW_125:  
                config1 = config1 & 0x3F;    // clears bits 7 & 6 from REG_MODEM_CONFIG1
                getSF();
                if( _spreadingFactor == 11 )
                { // LowDataRateOptimize (Mandatory with BW_125 if SF_11)
                    config1 = config1 | 0x01;
                }
                if( _spreadingFactor == 12 )
                { // LowDataRateOptimize (Mandatory with BW_125 if SF_12)
                    config1 = config1 | 0x01;
                }
                break;
            case BW_250:  
                config1 = config1 & 0x7F;    // clears bit 7 from REG_MODEM_CONFIG1
                config1 = config1 | 0x40;  // sets bit 6 from REG_MODEM_CONFIG1
                break;
            case BW_500: 
                config1 = config1 & 0xBF;    //clears bit 6 from REG_MODEM_CONFIG1
                config1 = config1 | 0x80;  //sets bit 7 from REG_MODEM_CONFIG1
                break;
        }
    }
    else if(_board==SX1276Chip)  
    {
        // SX1276
        config1 = config1 & 0x0F;  // clears bits 7 - 4 from REG_MODEM_CONFIG1
        switch(band)
        {
            case BW_125:
                // 0111
                config1 = config1 | 0x70;
                getSF();
                if( _spreadingFactor == 11 || _spreadingFactor == 12)
                { // LowDataRateOptimize (Mandatory with BW_125 if SF_11 or SF_12)
                    uint8_t config3=readRegister(REG_MODEM_CONFIG3);
                    config3 = config3 | 0x04;
                    writeRegister(REG_MODEM_CONFIG3,config3);
                }
                break;
            case BW_250:
                // 1000
                config1 = config1 | 0x80;
                break;
            case BW_500:
                // 1001
                config1 = config1 | 0x90;
                break;
        }
    }
    else 
    {
        return -1; 
    }

    error = writeReadRegister(REG_MODEM_CONFIG1,config1);       // Update config1
    if(error ==0)
    {
        _bandwidth = band;
    }
    writeRegister(REG_OP_MODE, st0);    // Getting back to previous status
   
    return error;
}

/**
 *  Function: Checks if CR is a valid value.
 * @param  cod CR
 * @return     1 if valid
 */
bool SX127X::isCR(uint8_t cod)
{

    // Checking available values for _codingRate
    switch(cod)
    {
    case CR_5:
    case CR_6:
    case CR_7:
    case CR_8:
        return true;
        break;

    default:
        return false;
    }
    return false; 
}

/**
 *  Function: Indicates the CR within the module is configured.
 * @return 0 if no error 
 */
int8_t  SX127X::getCR()
{
    int8_t error = 0;
    uint8_t config1;
      
    if (_board==SX1272Chip) {
        // take out bits 7-3 from REG_MODEM_CONFIG1 indicates _bandwidth & _codingRate
        config1 = ((readRegister(REG_MODEM_CONFIG1)) >> 3);
    }
    else {
        // take out bits 7-1 from REG_MODEM_CONFIG1 indicates _bandwidth & _codingRate
        config1 = (readRegister(REG_MODEM_CONFIG1)) >> 1;
    }

    _codingRate = config1 & 0x07;

    if(!isCR(_codingRate) )
    {
        error = -1;
    }

    return error;
}

/**
 * Function:    Sets the indicated CR in the module.
 * @param  cod  current CR
 * @return      0 if error occurs 
 */
int8_t  SX127X::setCR(uint8_t cod)
{
    int8_t error = 0;
    uint8_t st0;
    uint8_t config1;

    st0 = readRegister(REG_OP_MODE);        // Save the previous status

    // Set Standby mode to write in registers
    writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);      

    config1 = readRegister(REG_MODEM_CONFIG1);  // Save config1 to modify only the CR

    
    if (_board==SX1272Chip)
    {
        switch(cod)
        {
            case CR_5: 
                // clear bits 5 & 4 from REG_MODEM_CONFIG1
                config1 = (config1 | 0x08) & 0xCF;  // sets bit 3 from REG_MODEM_CONFIG1
                break;
            case CR_6: 
                // clears bits 5 & 3 from REG_MODEM_CONFIG1
                config1 = (config1 | 0x10) & 0xD7;  // sets bit 4 from REG_MODEM_CONFIG1
                break;
            case CR_7: 
                 // clears bit 5 from REG_MODEM_CONFIG1
                config1 = (config1 | 0x18) & 0xDF;  // sets bits 4 & 3 from REG_MODEM_CONFIG1
                break;
            case CR_8: 
                // clears bits 4 & 3 from REG_MODEM_CONFIG1
                config1 = (config1 | 0x20) & 0xE7;  // sets bit 5 from REG_MODEM_CONFIG1
                break;
        }
    }
    else if(_board==SX1276Chip) 
    {
        // SX1276
        config1 = config1 & 0xF1;  // clears bits 3 - 1 from REG_MODEM_CONFIG1
        switch(cod)
        {
            case CR_5:
                config1 = config1 | 0x02;
                break;
            case CR_6:
                config1 = config1 | 0x04;
                break;
            case CR_7:
                config1 = config1 | 0x06;
                break;
            case CR_8:
                config1 = config1 | 0x08;
                break;
        }
    }
    else 
    {
        return -1; 
    }

    if( writeReadRegister(REG_MODEM_CONFIG1, config1) != 0)       // Update config1
    {
        return -1;
    }

    writeRegister(REG_OP_MODE,st0); // Getting back to previous status

    return error;
}

/**
 * Function:  responsible for setting up the frequency channel and writing to the registers
 * @param     ch desired channel 
 * @return    0 if no error occurs
 */
int8_t SX127X::setChannel(uint32_t ch)
{
    int8_t error = 2;
    uint8_t freq1, freq2, freq3;
  
    // LoRa Stdby mode in order to write in registers
    writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);

    freq3 = ((ch >> 16) & 0x0FF);       // frequency channel MSB
    freq2 = ((ch >> 8) & 0x0FF);        // frequency channel MIB
    freq1 = (ch & 0xFF);                // frequency channel LSB

    error = writeReadRegister(REG_FRF_MSB, freq3);
    if(error != 0)
    {
        return -1; 
    }

    error = writeReadRegister(REG_FRF_MID, freq2);
    if(error != 0)
    {
        return -1; 
    }

    error = writeReadRegister(REG_FRF_LSB, freq1);
    if(error != 0)
    {
        return -1; 
    }

    return error;
}

/**
 * Function: Responsible for pulling the current power from the register and returns it
 * @return current power 
 */
uint8_t SX127X::getPower()
{
    return readRegister(REG_PA_CONFIG);
}

/**
 * Function: responsible for setting the output power for the antenna
 * @param  p desired power rate
 * @return   0 if error occurs 
 */
int8_t SX127X::setPower(char p)
{
    uint8_t st0;
    int8_t error = 0;
    uint8_t value = 0x00;

    uint8_t RegPaDacReg = (_board==SX1272Chip)?0x5A:0x4D;

    st0 = readRegister(REG_OP_MODE);      // Save the previous status
    
    writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);
 

    value = _power;

    if (p==SX127X_POWER_MAXIMUM)
     {
        // we set the PA_BOOST pin
        value = value | 0x80;
        // and then set the high output power config with register REG_PA_DAC
        writeRegister(RegPaDacReg, 0x87);
    }
    else {
        // disable high power output in all other cases
        writeRegister(RegPaDacReg, 0x84);
    }
    
    if (_board==SX1272Chip) {
        writeRegister(REG_PA_CONFIG, value);    // Setting output power value
    }
    else {
        // set MaxPower to 7 -> Pmax=10.8+0.6*MaxPower [dBm] = 15
        value = value | 0x70;
        // then Pout = Pmax-(15-_power[3:0]) if  PaSelect=0 (RFO pin for +13dBm)
        writeRegister(REG_PA_CONFIG, value);
    }

    _power=value;

    writeRegister(REG_OP_MODE, st0);    // Getting back to previous status

    return error;
}

/**
 * Function: Gets the preamble length from the module.
 * @return preamble length 
 */
uint16_t SX127X::getPreambleLength()
{
    int8_t state = 0;
    uint16_t preamblelength = 0;
    uint8_t p_length;

    p_length = readRegister(REG_PREAMBLE_MSB_LORA);
    // Saving MSB preamble length in LoRa mode
    preamblelength = (p_length << 8) & 0xFFFF;

    p_length = readRegister(REG_PREAMBLE_LSB_LORA);
    // Saving LSB preamble length in LoRa mode
    preamblelength += (p_length & 0xFFFF);

    return preamblelength;
}

/**
 *  Function: Sets the preamble length in the module
 * @param  l desired preamble length 
 * @return   error state
 */
int8_t SX127X::setPreambleLength(uint16_t l)
{
    int8_t error = 0;
    uint8_t p_length;

    writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);    // Set Standby mode to write in registers
    p_length = ((l >> 8) & 0x0FF);
    // Storing MSB preamble length in LoRa mode
    writeRegister(REG_PREAMBLE_MSB_LORA, p_length);
    p_length = (l & 0x0FF);
    // Storing LSB preamble length in LoRa mode
    writeRegister(REG_PREAMBLE_LSB_LORA, p_length);

    return error;
}

/**
 * Function: returns the stored node address 
 * @return node address
 */
uint8_t SX127X::getNodeAddress()
{
    uint8_t st0; 
    st0 = readRegister(REG_OP_MODE);    // Save the previous status
    // Allowing access to FSK registers while in LoRa standby mode
    writeRegister(REG_OP_MODE, LORA_STANDBY_FSK_REGS_MODE);
        // Saving node address
    _nodeAddress = readRegister(REG_NODE_ADRS);

    writeRegister(REG_OP_MODE, st0); 

    return _nodeAddress;
}

/**
 * Function: responsible for setting up the node address
 * @param  addr desired node addr
 * @return      0 if error occurs
 */
int8_t SX127X::setNodeAddress(uint8_t addr)
{
    uint8_t error = 0;
    uint8_t st0;
    uint8_t value;
    // Saving node address
    _nodeAddress = addr;
    st0 = readRegister(REG_OP_MODE);      // Save the previous status

    writeRegister(REG_OP_MODE, LORA_STANDBY_FSK_REGS_MODE);

    // Storing node and broadcast address
    writeRegister(REG_NODE_ADRS, addr);
    writeRegister(REG_BROADCAST_ADRS, BROADCAST_0);

    value = readRegister(REG_NODE_ADRS);
    writeRegister(REG_OP_MODE, st0);        // Getting back to previous status

    if( value != _nodeAddress )
    {
        return -1;
    }

    return error;
}

/**
 * Function: returns the SNR value 
 * @return SNR value
 */
int8_t SX127X::getSNR()
{   
    return  readRegister(REG_PKT_SNR_VALUE) >> 2;
}

/**
 * Function: Returns a uint16_t representing the the rssi value of the most recent recieved packet
 * @return RSSI value 
 */
int16_t SX127X::getRSSIpacket()
{  
    int8_t SNR = 0;
    int16_t RSSIpacket =0;
    RSSIpacket = readRegister(REG_PKT_RSSI_VALUE);
   
    SNR = getSNR();

    if( SNR < 0 )
    {
        RSSIpacket = -(OFFSET_RSSI+(_board==SX1276Chip?20:0)) + (double)RSSIpacket + (double)SNR*0.25;
    }
    else
    {
        RSSIpacket = -(OFFSET_RSSI+(_board==SX1276Chip?20:0)) + (double)RSSIpacket;
    }

    return RSSIpacket;
}

/**
 * Function: sets the maximum current the SX127X device has access 
 * @param  rate current rate 
 * @return      error state 0 if no error 
 */
int8_t SX127X::setMaxCurrent(uint8_t rate)
{
    int8_t error = 0;
    uint8_t st0;

    // Maximum rate value = 0x1B, because maximum current supply = 240 mA
    if (rate > 0x1B)
    {
        rate = 0x1B; // set the requested rate to 240mA 
    }

    // Enable Over Current Protection
    rate |= 0x20;

    st0 = readRegister(REG_OP_MODE);    // Save the previous status
    
    writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);  // Set LoRa Standby mode to write in registers
    writeRegister(REG_OCP, rate);       // Modifying maximum current supply

    writeRegister(REG_OP_MODE, st0);        // Getting back to previous status
   
    return error;
}

/**
 * Function: Places SX127X into RX mode 
 * @return error state 0 if no error 
 */
uint8_t SX127X::receive()
{
    uint8_t error = 0;

    // Initializing _packet_received struct
    memset( &_packet_received, 0x00, sizeof(_packet_received) );

    writeRegister(REG_PA_RAMP, 0x08);

    writeRegister(REG_LNA, LNA_MAX_GAIN);
    writeRegister(REG_FIFO_ADDR_PTR, 0x00);  // Setting address pointer in FIFO data buffer

    if (_spreadingFactor == SF_10 || _spreadingFactor == SF_11 || _spreadingFactor == SF_12) {
        writeRegister(REG_SYMB_TIMEOUT_LSB,0x05);
    } else {
        writeRegister(REG_SYMB_TIMEOUT_LSB,0x08);
    }

    writeRegister(REG_FIFO_RX_BYTE_ADDR, 0x00); // Setting current value of reception buffer pointer

    _packet_sent.length =MAX_LENGTH;

    //set packet length 
    writeRegister(REG_PAYLOAD_LENGTH_LORA, _packet_sent.length);
    // Place LORA mode - Rx
    writeRegister(REG_OP_MODE, LORA_RX_MODE);     

    return error;
}

/**
 * Function: Should only be called when DIO_0 is asserted, pulls recieved packet from the registers stores it in global 
 * @return      error state 0 if no error 
 */
int8_t SX127X::getPacket(void)
{
    uint8_t error = 0;
    uint8_t value = 0x00;

    value = readRegister(REG_IRQ_FLAGS);
    if((value && REG_IRQ_RXDONE_FLAG)  && (value, REG_IRQ_VALID_HEADER_FLAG))
    {
        writeRegister(REG_FIFO_ADDR_PTR, 0x00);     // Setting address pointer in FIFO data buffer

        _packet_received.dst = readRegister(REG_FIFO);   // Storing first byte of the received packet
        _packet_received.type = readRegister(REG_FIFO);      // Reading second byte of the received packet
        _packet_received.src = readRegister(REG_FIFO);       // Reading second byte of the received packet
        _packet_received.packnum = readRegister(REG_FIFO);   // Reading third byte of the received packet
        _packet_received.length = readRegister(REG_RX_NB_BYTES);
       
        for(unsigned int i = 0; i < _packet_received.length; i++)
        {
            _packet_received.data[i] = readRegister(REG_FIFO); // Storing payload
        }

        _packet_received.length -= OFFSET_PAYLOADLENGTH;
        writeRegister(REG_FIFO_ADDR_PTR, 0x00);  // Setting address pointer in FIFO data buffer
        // clear the flags, this pulls DIO_0 low 
        clearFlags();   
    }
  
    return error;
}

/**
 * Function: Checks the IRQ register, returns 0 is the TX_DONE_Flag has been set. 
 *         Typically read after DIO_5 is de-asserted. 
 * @return error state
 */
int8_t SX127X::checkTransmissionStatus(void)
{
    uint8_t error = 0;
    uint8_t value = 0x00;
    
    value = readRegister(REG_IRQ_FLAGS);

    if(!(value && REG_IRQ_TX_DONE_FLAG)) 
        error = -1;

    clearFlags();

    return error;
}

/**
 * Function: sets up the registers and places the lora module into TX mode
 * @param  dest    destination address
 * @param  payload pointer to payload
 * @param  length  length of payload
 * @return         returns 0 if no errors 
 */
int8_t SX127X::sendPacket(uint8_t dest, uint8_t *payload, uint8_t length)
{
    uint8_t error = 0;
    uint8_t st0;
    
    // limit the length to maximum payload
    if(length > MAX_PAYLOAD)
    {
        length = MAX_PAYLOAD; 
    }

    st0 = readRegister(REG_OP_MODE);    // Save the previous status
    clearFlags();   // clear the flags 

    writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);  // set lora module into standby mode
    // Sending new packet
    _packet_sent.dst = dest;  // Setting destination in packet structure
    _packet_sent.src = _nodeAddress; // Setting source in packet structure
    _packet_sent.packnum = _packetNumber;    // Setting packet number in packet structure
    _packet_sent.length = length + OFFSET_PAYLOADLENGTH;

    // write out packet length 
    writeRegister(REG_PAYLOAD_LENGTH_LORA, _packet_sent.length);

    _packet_sent.type = (PKT_TYPE_DATA | PKT_FLAG_DATA_WAPPKEY);

    // Setting address pointer in FIFO data buffer
    writeRegister(REG_FIFO_ADDR_PTR, 0x80);  

    // Writing packet to send in FIFO
    writeRegister(REG_FIFO, _packet_sent.dst);       // Writing the destination in FIFO
    writeRegister(REG_FIFO, _packet_sent.type);      // Writing the packet type in FIFO
    writeRegister(REG_FIFO, _packet_sent.src);       // Writing the source in FIFO
    writeRegister(REG_FIFO, _packet_sent.packnum);   // Writing the packet number in FIFO
   
    for(unsigned int i = 0; i < length; i++)
    {
        // Writing the payload in FIFO
        writeRegister(REG_FIFO, payload[i]); 
    }

    // Getting back to previous status
    writeRegister(REG_OP_MODE, st0);
    // Setup LORA mode - Tx
    writeRegister(REG_OP_MODE, LORA_TX_MODE);

    return error;
}


/**
 * Function:  writes the desired syncword to the register
 * @param     sw the desired sync word 
 * @return    0 if no error occurs
 */
int8_t  SX127X::setTX(void)
{
    uint8_t error = 0;
    uint8_t st0;
    
    st0 = readRegister(REG_OP_MODE);    // Save the previous status
    clearFlags();   // clear the flags 

    // Getting back to previous status
    writeRegister(REG_OP_MODE, st0);
    // Setup LORA mode - Tx
    writeRegister(REG_OP_MODE, LORA_TX_MODE);
    delay_ms(100);
    return error;
}


/**
 * Function:  writes the desired syncword to the register
 * @param     sw the desired sync word 
 * @return    0 if no error occurs
 */
int8_t  SX127X::setSyncWord(uint8_t sw)
{
    int8_t error =0; 
    uint8_t st0;

    st0 = readRegister(REG_OP_MODE);        // Save the previous status
    writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);      // Set Standby mode to write in registers

    error = writeReadRegister(REG_SYNC_WORD, sw);
    if(error != 0)
    {
        return -1; 
    }

    _syncWord = sw;  
    writeRegister(REG_OP_MODE,st0); // Getting back to previous status
    delay_ms(100);
    return error;
}

/**
 * Function:  places the lora module into sleep mode
 * @return    0 if successful 
 */
int8_t SX127X::setSleepMode() 
{
    int8_t error = 0;
    uint8_t value;

    // Request module is placed in sleep mode
    writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);
    writeRegister(REG_OP_MODE, LORA_SLEEP_MODE);    
    
    value = readRegister(REG_OP_MODE);

    (value == LORA_SLEEP_MODE) ? ( error = 0) : (error = -1); 

    return error;
}
