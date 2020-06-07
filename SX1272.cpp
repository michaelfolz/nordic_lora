

#include "SX1272.h"
#include <SPI.h>

#define SX1272_REGISTER_VERSION_CODE 0x22
#define SX1276_REGISTER_VERSION_CODE 0x12


SX1272::SX1272()
{
    // Initialize class variables
    _bandwidth = BW_125;
    _codingRate = CR_5;
    _spreadingFactor = SF_7;
    _channel = CH_12_900;
    _header = HEADER_ON;
    _CRC = CRC_OFF;
    _modem = FSK;
    _power = 15;
    _packetNumber = 0;
    _reception = CORRECT_PACKET;
    _retries = 0;
    // added by C. Pham
    _defaultSyncWord=0x12;
    _rawFormat=false;
    _extendedIFS=true;
    _RSSIonSend=true;
    // disabled by default
    _enableCarrierSense=false;
    // DIFS by default
    _send_cad_number=9;

    // end
    _maxRetries = 3;
    packet_sent.retry = _retries;
};



/*
 Function: Sets the module ON.
 Returns: uint8_t setLORA state
*/
uint8_t SX1272::ON()
{
    uint8_t error = 0;
    uint8_t version; 

    // Powering the module
    pinMode(SX1272_SS,OUTPUT);
    digitalWrite(SX1272_SS,HIGH);
    delay(100);



    // MSB, mode 0, 2Mhz 
    SPI.begin();
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV8);
    SPI.setDataMode(SPI_MODE0);
    delay(100);

    // added by C. Pham
    pinMode(SX1272_RST,OUTPUT);

    // request device reset 
    digitalWrite(SX1272_RST,HIGH);
    delay(100);
    digitalWrite(SX1272_RST,LOW);
    delay(100);

    // Read Version 
    version = readRegister(REG_VERSION);
    switch(version)
    {
        case SX1272_REGISTER_VERSION_CODE:
            _board = SX1272Chip;  
            break;

        case SX1276_REGISTER_VERSION_CODE:
            _board = SX1276Chip;  
                Serial.println(F("SX1276Chip"));
            break;
        default:
            error = -1; 
            break; 
    }


    // set LoRa mode
    error = setLORA();
    if(error != 0)
        return error; 

    setMaxCurrent(0x1B);

    // added by C. Pham
    // default sync word for non-LoRaWAN
    setSyncWord(_defaultSyncWord);
 
    _defaultSyncWord=_syncWord;
    //end

    return error;
}

/*
 Function: Sets the module OFF.
 Returns: Nothing
*/
void SX1272::OFF()
{
    SPI.end();
    // Powering the module
    pinMode(SX1272_SS,OUTPUT);
    digitalWrite(SX1272_SS,LOW);
}

/*
 Function: Reads the indicated register.
 Returns: The content of the register
 Parameters:
   address: address register to read from
*/
byte SX1272::readRegister(byte address)
{
    byte value = 0x00;

    digitalWrite(SX1272_SS,LOW);
    bitClear(address, 7);       // Bit 7 cleared to write in registers
    SPI.transfer(address);
    value = SPI.transfer(0x00);
    digitalWrite(SX1272_SS,HIGH);

    return value;
}

/*
 Function: Writes on the indicated register.
 Returns: Nothing
 Parameters:
   address: address register to write in
   data : value to write in the register
*/
void SX1272::writeRegister(byte address, byte data)
{

    digitalWrite(SX1272_SS,LOW);
    bitSet(address, 7);         // Bit 7 set to read from registers
    SPI.transfer(address);
    SPI.transfer(data);
    digitalWrite(SX1272_SS,HIGH);

}

int8_t SX1272::writeReadRegister(byte address, byte data)
{
    int8_t error =0; 
    byte dataRead = 0;
    writeRegister(address, data);

    delay(50);


    dataRead = readRegister(address);

    if(data != dataRead)
        return -1; 
    
    return error; 
}

/*
 Function: Clears the interruption flags
 Returns: Nothing
*/
void SX1272::clearFlags()
{
    int error =0;
    byte st0;

    st0 = readRegister(REG_OP_MODE);        // Save the previous status

    error = writeReadRegister(REG_OP_MODE, LORA_STANDBY_MODE);  // Stdby mode to write in registers
    error = writeReadRegister(REG_IRQ_FLAGS, 0xFF); // LoRa mode flags register
    error = writeReadRegister(REG_OP_MODE, st0);        // Getting back to previous status

    return; 
}

/*
 Function: Sets the module in LoRa mode.
 Returns:  Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::setLORA()
{
    uint8_t error = 0;
    byte st0;
    
    error = writeReadRegister(REG_OP_MODE, (FSK_SLEEP_MODE | LORA_STANDBY_MODE | LORA_SLEEP_MODE) );
    if(error != 0)
    {
        Serial.print("register doesnt match");
        return error; 
    }

    _modem = LORA;
   
    return error;
}


int8_t SX1272::setMode(uint8_t mode)
{
    int8_t error = 0;
    byte st0;
    byte config1 = 0x00;
    byte config2 = 0x00;

    st0 = readRegister(REG_OP_MODE);        // Save the previous status

    writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);  // LoRa standby mode

    // 
    setCR(CR_5);        // CR = 4/5


    switch (mode)
    {
        // mode 1 (better reach, medium time on air)
        case 1:
            
            setSF(SF_12);       // SF = 12
            setBW(BW_125);      // BW = 125 KHz
            break;

        // mode 2 (medium reach, less time on air)
        case 2:
            
            setSF(SF_12);       // SF = 12
            setBW(BW_250);      // BW = 250 KHz
            break;

        // mode 3 (worst reach, less time on air)
        case 3:
            
            setSF(SF_10);       // SF = 10
            setBW(BW_125);      // BW = 125 KHz
            break;

        // mode 4 (better reach, low time on air)
        case 4:
            
            setSF(SF_12);       // SF = 12
            setBW(BW_500);      // BW = 500 KHz
            break;

        // mode 5 (better reach, medium time on air)
        case 5:
            
            setSF(SF_10);       // SF = 10
            setBW(BW_250);      // BW = 250 KHz
            break;

        // mode 6 (better reach, worst time-on-air)
        case 6:
            
            setSF(SF_11);       // SF = 11
            setBW(BW_500);      // BW = 500 KHz
            break;

        // mode 7 (medium-high reach, medium-low time-on-air)
        case 7:
            
            setSF(SF_9);        // SF = 9
            setBW(BW_250);      // BW = 250 KHz
            break;

            // mode 8 (medium reach, medium time-on-air)
        case 8:     
            
            setSF(SF_9);        // SF = 9
            setBW(BW_500);      // BW = 500 KHz
            break;

        // mode 9 (medium-low reach, medium-high time-on-air)
        case 9:
            
            setSF(SF_8);        // SF = 8
            setBW(BW_500);      // BW = 500 KHz
            break;

        // mode 10 (worst reach, less time_on_air)
        case 10:
            
            setSF(SF_7);        // SF = 7
            setBW(BW_500);      // BW = 500 KHz
            break;

        default:    
            error = -1; // The indicated mode doesn't exist

    };

    
    // added by C. Pham
    if (error == 0)
        _loraMode=mode;


    writeRegister(REG_OP_MODE, st0);    // Getting back to previous status
    delay(100);
    return error;
}

/*
 Function: Indicates if module is configured in implicit or explicit header mode.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::getHeader()
{
    int8_t state = 2;

    // added by C. Pham
    uint8_t theHeaderBit;

    if (_board==SX1272Chip)
        theHeaderBit=2;
    else
        theHeaderBit=0;

    // take out bit 2 from REG_MODEM_CONFIG1 indicates ImplicitHeaderModeOn
    if( bitRead(REG_MODEM_CONFIG1, theHeaderBit) == 0 )
    { // explicit header mode (ON)
        _header = HEADER_ON;
        state = 1;
    }
    else
    { // implicit header mode (OFF)
        _header = HEADER_OFF;
        state = 1;
    }

    state = 0;

    return state;
}

/*
 Function: Sets the module in explicit header mode (header is sent).
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
*/
int8_t  SX1272::setHeaderON()
{
    int8_t state = 2;
    byte config1;


    if( _modem == FSK )
    {
        state = -1;     // header is not available in FSK mode
    }
    else
    {
        config1 = readRegister(REG_MODEM_CONFIG1);  // Save config1 to modify only the header bit
        if( _spreadingFactor == 6 )
        {
            state = -1;     // Mandatory headerOFF with SF = 6
        }
        else
        {
            // added by C. Pham
            if (_board==SX1272Chip)
                config1 = config1 & B11111011;      // clears bit 2 from config1 = headerON
            else
                config1 = config1 & B11111110;              // clears bit 0 from config1 = headerON

            writeRegister(REG_MODEM_CONFIG1,config1);   // Update config1
        }

        // added by C. Pham
        uint8_t theHeaderBit;

        if (_board==SX1272Chip)
            theHeaderBit=2;
        else
            theHeaderBit=0;

        if( _spreadingFactor != 6 )
        { // checking headerON taking out bit 2 from REG_MODEM_CONFIG1
            config1 = readRegister(REG_MODEM_CONFIG1);
            // modified by C. Pham
            if( bitRead(config1, theHeaderBit) == HEADER_ON )
            {
                state = 0;
                _header = HEADER_ON;
            }
            else
            {
                state = 1;
            }
        }
        return state;
    }
}

/*
 Function: Sets the module in implicit header mode (header is not sent).
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
*/
int8_t  SX1272::setHeaderOFF()
{
    uint8_t state = 2;
    byte config1;

    if( _modem == FSK )
    { // header is not available in FSK mode
        state = -1;
    }
    else
    {
        config1 = readRegister(REG_MODEM_CONFIG1);  // Save config1 to modify only the header bit

        // modified by C. Pham
        if (_board==SX1272Chip)
            config1 = config1 | B00000100;          // sets bit 2 from REG_MODEM_CONFIG1 = headerOFF
        else
            config1 = config1 | B00000001;                      // sets bit 0 from REG_MODEM_CONFIG1 = headerOFF

        writeRegister(REG_MODEM_CONFIG1,config1);       // Update config1

        config1 = readRegister(REG_MODEM_CONFIG1);

        // added by C. Pham
        uint8_t theHeaderBit;

        if (_board==SX1272Chip)
            theHeaderBit=2;
        else
            theHeaderBit=0;

        if( bitRead(config1, theHeaderBit) == HEADER_OFF )
        { // checking headerOFF taking out bit 2 from REG_MODEM_CONFIG1
            state = 0;
            _header = HEADER_OFF;
        }
        else
        {
            state = 1;
        }
    }
    return state;
}

/*
 Function: Indicates if module is configured with or without checking CRC.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::getCRC()
{
    int8_t state = 2;
    byte value;


    if( _modem == LORA )
    { // LoRa mode

        // added by C. Pham
        uint8_t theRegister;
        uint8_t theCrcBit;

        if (_board==SX1272Chip) {
            theRegister=REG_MODEM_CONFIG1;
            theCrcBit=1;
        }
        else {
            theRegister=REG_MODEM_CONFIG2;
            theCrcBit=2;
        }

        // take out bit 1 from REG_MODEM_CONFIG1 indicates RxPayloadCrcOn
        value = readRegister(theRegister);
        if( bitRead(value, theCrcBit) == CRC_OFF )
        { // CRCoff
            _CRC = CRC_OFF;
            state = 0;
        }
        else
        { // CRCon
            _CRC = CRC_ON;
            state = 0;
        }
    }
    else
    { // FSK mode

        // take out bit 2 from REG_PACKET_CONFIG1 indicates CrcOn
        value = readRegister(REG_PACKET_CONFIG1);
        if( bitRead(value, 4) == CRC_OFF )
        { // CRCoff
            _CRC = CRC_OFF;
            state = 0;
        }
        else
        { // CRCon
            _CRC = CRC_ON;
            state = 0;
        }
    }
    if( state != 0 )
    {
        state = 1;
    }
    return state;
}

/*
 Function: Sets the module with CRC on.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::setCRC_ON()
{
    uint8_t state = 2;
    byte config1;

    if( _modem == LORA )
    { // LORA mode

        // added by C. Pham
        uint8_t theRegister;
        uint8_t theCrcBit;

        if (_board==SX1272Chip) {
            theRegister=REG_MODEM_CONFIG1;
            theCrcBit=1;
        }
        else {
            theRegister=REG_MODEM_CONFIG2;
            theCrcBit=2;
        }

        config1 = readRegister(theRegister);    // Save config1 to modify only the CRC bit

        if (_board==SX1272Chip)
            config1 = config1 | B00000010;              // sets bit 1 from REG_MODEM_CONFIG1 = CRC_ON
        else
            config1 = config1 | B00000100;                               // sets bit 2 from REG_MODEM_CONFIG2 = CRC_ON

        writeRegister(theRegister,config1);

        state = 1;

        config1 = readRegister(theRegister);

        if( bitRead(config1, theCrcBit) == CRC_ON )
        { // take out bit 1 from REG_MODEM_CONFIG1 indicates RxPayloadCrcOn
            state = 0;
            _CRC = CRC_ON;
        }
    }
    else
    { // FSK mode
        config1 = readRegister(REG_PACKET_CONFIG1); // Save config1 to modify only the CRC bit
        config1 = config1 | B00010000;              // set bit 4 and 3 from REG_MODEM_CONFIG1 = CRC_ON
        writeRegister(REG_PACKET_CONFIG1,config1);

        state = 1;

        config1 = readRegister(REG_PACKET_CONFIG1);
        if( bitRead(config1, 4) == CRC_ON )
        { // take out bit 4 from REG_PACKET_CONFIG1 indicates CrcOn
            state = 0;
            _CRC = CRC_ON;
        }
    }
    if( state != 0 )
    {
        state = 1;
    }
    return state;
}

/*
 Function: Sets the module with CRC off.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::setCRC_OFF()
{
    int8_t state = 2;
    byte config1;


    if( _modem == LORA )
    { // LORA mode

        // added by C. Pham
        uint8_t theRegister;
        uint8_t theCrcBit;

        if (_board==SX1272Chip) {
            theRegister=REG_MODEM_CONFIG1;
            theCrcBit=1;
        }
        else {
            theRegister=REG_MODEM_CONFIG2;
            theCrcBit=2;
        }

        config1 = readRegister(theRegister);    // Save config1 to modify only the CRC bit
        if (_board==SX1272Chip)
            config1 = config1 & B11111101;              // clears bit 1 from config1 = CRC_OFF
        else
            config1 = config1 & B11111011;              // clears bit 2 from config1 = CRC_OFF

        writeRegister(theRegister,config1);

        config1 = readRegister(theRegister);
        if( (bitRead(config1, theCrcBit)) == CRC_OFF )
        { // take out bit 1 from REG_MODEM_CONFIG1 indicates RxPayloadCrcOn
            state = 0;
            _CRC = CRC_OFF;
        }
    }
    else
    { // FSK mode
        config1 = readRegister(REG_PACKET_CONFIG1); // Save config1 to modify only the CRC bit
        config1 = config1 & B11101111;              // clears bit 4 from config1 = CRC_OFF
        writeRegister(REG_PACKET_CONFIG1,config1);

        config1 = readRegister(REG_PACKET_CONFIG1);
        if( bitRead(config1, 4) == CRC_OFF )
        { // take out bit 4 from REG_PACKET_CONFIG1 indicates RxPayloadCrcOn
            state = 0;
            _CRC = CRC_OFF;
        }
    }
    if( state != 0 )
    {
        state = 1;
    }
    return state;
}

/*
 Function: Checks if SF is a valid value.
 Returns: Boolean that's 'true' if the SF value exists and
          it's 'false' if the SF value does not exist.
 Parameters:
   spr: spreading factor value to check.
*/
boolean SX1272::isSF(uint8_t spr)
{

    // Checking available values for _spreadingFactor
    switch(spr)
    {
    case SF_6:
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

}

/*
 Function: Gets the SF within the module is configured.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
*/
int8_t  SX1272::getSF()
{
    int8_t state = 2;
    byte config2;

    if( _modem == FSK )
    {
        state = -1;     // SF is not available in FSK mode
    }
    else
    {
        // take out bits 7-4 from REG_MODEM_CONFIG2 indicates _spreadingFactor
        config2 = (readRegister(REG_MODEM_CONFIG2)) >> 4;
        _spreadingFactor = config2;
        state = 1;

        if( (config2 == _spreadingFactor) && isSF(_spreadingFactor) )
        {
            state = 0;
        }
    }
    return state;
}

/*
 Function: Sets the indicated SF in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
 Parameters:
   spr: spreading factor value to set in LoRa modem configuration.
*/
uint8_t SX1272::setSF(uint8_t spr)
{
    byte st0;
    int8_t state = 2;
    byte config1;
    byte config2;

    st0 = readRegister(REG_OP_MODE);    // Save the previous status
    writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);  // LoRa standby mode


    config2 = (readRegister(REG_MODEM_CONFIG2));    // Save config2 to modify SF value (bits 7-4)

    switch(spr)
    {
        case SF_6: 
        {
            config2 = config2 & B01101111;  // clears bits 7 & 4 from REG_MODEM_CONFIG2
            config2 = config2 | B01100000;  // sets bits 6 & 5 from REG_MODEM_CONFIG2
            // Mandatory headerOFF with SF = 6 (Implicit mode)
            setHeaderOFF();

            // Set the bit field DetectionOptimize of
            // register RegLoRaDetectOptimize to value "0b101".
            writeRegister(REG_DETECT_OPTIMIZE, 0x05);

            // Write 0x0C in the register RegDetectionThreshold.
            writeRegister(REG_DETECTION_THRESHOLD, 0x0C);
            break;
        }
        case SF_7:  
        {   
            config2 = config2 & B01111111;  // clears bits 7 from REG_MODEM_CONFIG2
            config2 = config2 | B01110000;  // sets bits 6, 5 & 4
            break;
        }
        case SF_8:  
        {
            config2 = config2 & B10001111;  // clears bits 6, 5 & 4 from REG_MODEM_CONFIG2
            config2 = config2 | B10000000;  // sets bit 7 from REG_MODEM_CONFIG2
            break;
        }
        case SF_9:
        {
            config2 = config2 & B10011111;  // clears bits 6, 5 & 4 from REG_MODEM_CONFIG2
            config2 = config2 | B10010000;  // sets bits 7 & 4 from REG_MODEM_CONFIG2
            break;
        }
        case SF_10: 
        {   config2 = config2 & B10101111;  // clears bits 6 & 4 from REG_MODEM_CONFIG2
            config2 = config2 | B10100000;  // sets bits 7 & 5 from REG_MODEM_CONFIG2
            break;
        }
        case SF_11: 
        {   config2 = config2 & B10111111;  // clears bit 6 from REG_MODEM_CONFIG2
            config2 = config2 | B10110000;  // sets bits 7, 5 & 4 from REG_MODEM_CONFIG2
            break;
        }
        case SF_12: 
        {   config2 = config2 & B11001111;  // clears bits 5 & 4 from REG_MODEM_CONFIG2
            config2 = config2 | B11000000;  // sets bits 7 & 6 from REG_MODEM_CONFIG2
            break;
        }
    }

    // Check if it is neccesary to set special settings for SF=6
    if( spr != SF_6 )
    {
        // added by C. Pham
        setHeaderON();

        // LoRa detection Optimize: 0x03 --> SF7 to SF12
        writeRegister(REG_DETECT_OPTIMIZE, 0x03);

        // LoRa detection threshold: 0x0A --> SF7 to SF12
        writeRegister(REG_DETECTION_THRESHOLD, 0x0A);
    }

    // added by C. Pham
    if (_board==SX1272Chip) {
        // comment by C. Pham
        // bit 9:8 of SymbTimeout are then 11
        // single_chan_pkt_fwd uses 00 and then 00001000
        // why?
        // sets bit 2-0 (AgcAutoOn and SymbTimout) for any SF value
        //config2 = config2 | B00000111;
        // modified by C. Pham
        config2 = config2 | B00000100;
        writeRegister(REG_MODEM_CONFIG1, config1);      // Update config1
    }
    else {
        // set the AgcAutoOn in bit 2 of REG_MODEM_CONFIG3
        uint8_t config3 = (readRegister(REG_MODEM_CONFIG3));
        config3=config3 | B00000100;
        writeRegister(REG_MODEM_CONFIG3, config3);
    }

    // here we write the new SF
    writeRegister(REG_MODEM_CONFIG2, config2);      // Update config2

    delay(100);

    // added by C. Pham
    byte configAgc;
    uint8_t theLDRBit;

    if (_board==SX1272Chip) {
        config1 = (readRegister(REG_MODEM_CONFIG1));    // Save config1 to check update
        config2 = (readRegister(REG_MODEM_CONFIG2));    // Save config2 to check update
        // comment by C. Pham
        // (config2 >> 4) ---> take out bits 7-4 from REG_MODEM_CONFIG2 (=_spreadingFactor)
        // bitRead(config1, 0) ---> take out bits 1 from config1 (=LowDataRateOptimize)
        // config2 is only for the AgcAutoOn
        configAgc=config2;
        theLDRBit=0;
    }
    else {
        config1 = (readRegister(REG_MODEM_CONFIG3));    // Save config1 to check update
        config2 = (readRegister(REG_MODEM_CONFIG2));
        // LowDataRateOptimize is in REG_MODEM_CONFIG3
        // AgcAutoOn is in REG_MODEM_CONFIG3
        configAgc=config1;
        theLDRBit=3;
    }

      writeRegister(REG_OP_MODE, st0);    // Getting back to previous status
    delay(100);

    if( isSF(spr) )
    { // Checking available value for _spreadingFactor
        state = 0;
        _spreadingFactor = spr;
    }

    return state;
}

/*
 Function: Checks if BW is a valid value.
 Returns: Boolean that's 'true' if the BW value exists and
          it's 'false' if the BW value does not exist.
 Parameters:
   band: bandwidth value to check.
*/
boolean SX1272::isBW(uint16_t band)
{


    // Checking available values for _bandwidth
    // added by C. Pham
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

}

/*
 Function: Gets the BW within the module is configured.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
*/
int8_t  SX1272::getBW()
{
    uint8_t state = 2;
    byte config1;


    if( _modem == FSK )
    {
        state = -1;     // BW is not available in FSK mode

    }
    else
    {
        // added by C. Pham
        if (_board==SX1272Chip) {
            // take out bits 7-6 from REG_MODEM_CONFIG1 indicates _bandwidth
            config1 = (readRegister(REG_MODEM_CONFIG1)) >> 6;
        }
        else {
            // take out bits 7-4 from REG_MODEM_CONFIG1 indicates _bandwidth
            config1 = (readRegister(REG_MODEM_CONFIG1)) >> 4;
        }

        _bandwidth = config1;

        if( (config1 == _bandwidth) && isBW(_bandwidth) )
        {
            state = 0;

        }
        else
        {
            state = 1;

        }
    }
    return state;
}

/*
 Function: Sets the indicated BW in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
 Parameters:
   band: bandwith value to set in LoRa modem configuration.
*/
int8_t  SX1272::setBW(uint16_t band)
{
    byte st0;
    int8_t state = 2;
    byte config1;

    if(!isBW(band) )
    {
        state = 1;
        return state;
    }

    st0 = readRegister(REG_OP_MODE);    // Save the previous status

    writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);  // LoRa standby mode
    config1 = (readRegister(REG_MODEM_CONFIG1));    // Save config1 to modify only the BW

    // added by C. Pham for SX1276
    if (_board==SX1272Chip) 
    {
        switch(band)
        {
            case BW_125:  
                config1 = config1 & B00111111;    // clears bits 7 & 6 from REG_MODEM_CONFIG1
                getSF();
                if( _spreadingFactor == 11 )
                { // LowDataRateOptimize (Mandatory with BW_125 if SF_11)
                    config1 = config1 | B00000001;
                }
                if( _spreadingFactor == 12 )
                { // LowDataRateOptimize (Mandatory with BW_125 if SF_12)
                    config1 = config1 | B00000001;
                }
                break;
            case BW_250:  
                config1 = config1 & B01111111;    // clears bit 7 from REG_MODEM_CONFIG1
                config1 = config1 | B01000000;  // sets bit 6 from REG_MODEM_CONFIG1
                break;
            case BW_500: 
                config1 = config1 & B10111111;    //clears bit 6 from REG_MODEM_CONFIG1
                config1 = config1 | B10000000;  //sets bit 7 from REG_MODEM_CONFIG1
                break;
        }
    }
    else if(_board==SX1276Chip)  
    {
        // SX1276
        config1 = config1 & B00001111;  // clears bits 7 - 4 from REG_MODEM_CONFIG1
        switch(band)
        {
            case BW_125:
                // 0111
                config1 = config1 | B01110000;
                getSF();
                if( _spreadingFactor == 11 || _spreadingFactor == 12)
                { // LowDataRateOptimize (Mandatory with BW_125 if SF_11 or SF_12)
                    byte config3=readRegister(REG_MODEM_CONFIG3);
                    config3 = config3 | B0000100;
                    writeRegister(REG_MODEM_CONFIG3,config3);
                }
                break;
            case BW_250:
                // 1000
                config1 = config1 | B10000000;
                break;
            case BW_500:
                // 1001
                config1 = config1 | B10010000;
                break;
        }
    }


    uint8_t error = writeReadRegister(REG_MODEM_CONFIG1,config1);       // Update config1
    if(error ==0)
    {
        _bandwidth = band;
    }
    writeRegister(REG_OP_MODE, st0);    // Getting back to previous status
   
    return state;
}

/*
 Function: Checks if CR is a valid value.
 Returns: Boolean that's 'true' if the CR value exists and
          it's 'false' if the CR value does not exist.
 Parameters:
   cod: coding rate value to check.
*/
boolean SX1272::isCR(uint8_t cod)
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

}

/*
 Function: Indicates the CR within the module is configured.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
*/
int8_t  SX1272::getCR()
{
    int8_t state = 2;
    byte config1;


    if( _modem == FSK )
    {
        state = -1;     // CR is not available in FSK mode
    }
    else
    {
        // added by C. Pham
        if (_board==SX1272Chip) {
            // take out bits 7-3 from REG_MODEM_CONFIG1 indicates _bandwidth & _codingRate
            config1 = (readRegister(REG_MODEM_CONFIG1)) >> 3;
            config1 = config1 & B00000111;  // clears bits 7-3 ---> clears _bandwidth
        }
        else {
            // take out bits 7-1 from REG_MODEM_CONFIG1 indicates _bandwidth & _codingRate
            config1 = (readRegister(REG_MODEM_CONFIG1)) >> 1;
            config1 = config1 & B00000111;  // clears bits 7-3 ---> clears _bandwidth
        }

        _codingRate = config1;
        state = 1;

        if( (config1 == _codingRate) && isCR(_codingRate) )
        {
            state = 0;

        }
    }
    return state;
}

/*
 Function: Sets the indicated CR in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
 Parameters:
   cod: coding rate value to set in LoRa modem configuration.
*/
int8_t  SX1272::setCR(uint8_t cod)
{
    int8_t error = 2;
    byte st0;
    byte config1;

    st0 = readRegister(REG_OP_MODE);        // Save the previous status

    // Set Standby mode to write in registers
    writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);      

    config1 = readRegister(REG_MODEM_CONFIG1);  // Save config1 to modify only the CR

    // added by C. Pham
    if (_board==SX1272Chip)
    {
        switch(cod)
        {
            case CR_5: 
                // clear bits 5 & 4 from REG_MODEM_CONFIG1
                config1 = (config1 | B00001000) & B11001111;  // sets bit 3 from REG_MODEM_CONFIG1
                break;
            case CR_6: 
                // clears bits 5 & 3 from REG_MODEM_CONFIG1
                config1 = (config1 | B00010000) & B11010111;  // sets bit 4 from REG_MODEM_CONFIG1
                break;
            case CR_7: 
                 // clears bit 5 from REG_MODEM_CONFIG1
                config1 = (config1 | B00011000) & B11011111;  // sets bits 4 & 3 from REG_MODEM_CONFIG1
                break;
            case CR_8: 
                // clears bits 4 & 3 from REG_MODEM_CONFIG1
                config1 = (config1 | B00100000) & B11100111;  // sets bit 5 from REG_MODEM_CONFIG1
                break;
        }
    }
    else if(_board==SX1276Chip) 
    {
        // SX1276
        config1 = config1 & B11110001;  // clears bits 3 - 1 from REG_MODEM_CONFIG1
        switch(cod)
        {
            case CR_5:
                config1 = config1 | B00000010;
                break;
            case CR_6:
                config1 = config1 | B00000100;
                break;
            case CR_7:
                config1 = config1 | B00000110;
                break;
            case CR_8:
                config1 = config1 | B00001000;
                break;
        }
    }
    else 
    {
        return -1; 
    }

    writeRegister(REG_MODEM_CONFIG1, config1);      // Update config1


    delay(50);

    if(config1 != readRegister(REG_MODEM_CONFIG1))
    {
        Serial.print("register doesnt match");
    }


    writeRegister(REG_OP_MODE,st0); // Getting back to previous status
    delay(100);
    return error;
}

/*
 Function: Checks if channel is a valid value.
 Returns: Boolean that's 'true' if the CR value exists and
          it's 'false' if the CR value does not exist.
 Parameters:
   ch: frequency channel value to check.
*/
boolean SX1272::isChannel(uint32_t ch)
{


    // Checking available values for _channel
    switch(ch)
    {
    case CH_10_868:
    case CH_11_868:
    case CH_12_868:
    case CH_13_868:
    case CH_14_868:
    case CH_15_868:
    case CH_16_868:
    case CH_17_868:
        //added by C. Pham
    case CH_18_868:
        //end
    case CH_00_900:
    case CH_01_900:
    case CH_02_900:
    case CH_03_900:
    case CH_04_900:
    case CH_05_900:
    case CH_06_900:
    case CH_07_900:
    case CH_08_900:
    case CH_09_900:
    case CH_10_900:
    case CH_11_900:
        return true;
        break;

    default:
        return false;
    }

}

/*
 Function: Indicates the frequency channel within the module is configured.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::getChannel()
{
    uint8_t state = 2;
    uint32_t ch;
    uint8_t freq3;
    uint8_t freq2;
    uint8_t freq1;

    freq3 = readRegister(REG_FRF_MSB);  // frequency channel MSB
    freq2 = readRegister(REG_FRF_MID);  // frequency channel MID
    freq1 = readRegister(REG_FRF_LSB);  // frequency channel LSB
    ch = ((uint32_t)freq3 << 16) + ((uint32_t)freq2 << 8) + (uint32_t)freq1;
    _channel = ch;                      // frequency channel

    if( (_channel == ch) && isChannel(_channel) )
    {
        state = 0;
    }
    else
    {
        state = 1;
    }
    return state;
}

/*
 Function: Sets the indicated channel in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
 Parameters:
   ch: frequency channel value to set in configuration.
*/
int8_t SX1272::setChannel(uint32_t ch)
{
    byte st0;
    int8_t state = 2;
    unsigned int freq3;
    unsigned int freq2;
    uint8_t freq1;
    uint32_t freq;

    // added by C. Pham
    _starttime=millis();

    st0 = readRegister(REG_OP_MODE);    // Save the previous status
    if( _modem == LORA )
    {
        // LoRa Stdby mode in order to write in registers
        writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);
    }
    else
    {
        // FSK Stdby mode in order to write in registers
        writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);
    }

    freq3 = ((ch >> 16) & 0x0FF);       // frequency channel MSB
    freq2 = ((ch >> 8) & 0x0FF);        // frequency channel MIB
    freq1 = (ch & 0xFF);                // frequency channel LSB

    writeRegister(REG_FRF_MSB, freq3);
    writeRegister(REG_FRF_MID, freq2);
    writeRegister(REG_FRF_LSB, freq1);

    // added by C. Pham
    _stoptime=millis();

    delay(100);

    // storing MSB in freq channel value
    freq3 = (readRegister(REG_FRF_MSB));
    freq = (freq3 << 8) & 0xFFFFFF;

    // storing MID in freq channel value
    freq2 = (readRegister(REG_FRF_MID));
    freq = (freq << 8) + ((freq2 << 8) & 0xFFFFFF);

    // storing LSB in freq channel value
    freq = freq + ((readRegister(REG_FRF_LSB)) & 0xFFFFFF);

    if( freq == ch )
    {
        state = 0;
        _channel = ch;

    }
    else
    {
        state = 1;
    }

    if(!isChannel(ch) )
    {
        state = -1;
    }

    writeRegister(REG_OP_MODE, st0);    // Getting back to previous status
    delay(100);
    return state;
}

/*
 Function: Gets the signal power within the module is configured.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::getPower()
{
    uint8_t state = 2;
    byte value = 0x00;


    value = readRegister(REG_PA_CONFIG);
    state = 1;

    // modified by C. Pham
    // get only the OutputPower
    _power = value & B00001111;

    if( (value > -1) & (value < 16) )
    {
        state = 0;
    }

    return state;
}

/*
 Function: Sets the signal power indicated in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
 Parameters:
   p: power option to set in configuration.
*/
int8_t SX1272::setPower(char p)
{
    byte st0;
    int8_t state = 2;
    byte value = 0x00;

    byte RegPaDacReg = (_board==SX1272Chip)?0x5A:0x4D;

    st0 = readRegister(REG_OP_MODE);      // Save the previous status
    
    if( _modem == LORA )
    { // LoRa Stdby mode to write in registers
        writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);
    }

    switch (p)
    {
        // L = low. On SX1272: PA0 on RFO setting
        // H = high. On SX1272: PA0 on RFO setting
        // M = MAX. On SX1272: PA0 on RFO setting

        // x = extreme; added by C. Pham. On SX1272: PA1&PA2 PA_BOOST setting
        // X = eXtreme; added by C. Pham. On SX1272: PA1&PA2 PA_BOOST setting + 20dBm settings

        // added by C. Pham
        //
        case 'x':
        case 'X':
        case 'M':  _power = 0x0F;
            // SX1272: 14dBm
            // SX1276:
            break;

        // modified by C. Pham, set to 0x02 instead of 0x00
        case 'L':  _power = 0x02;
            // SX1272: 1dBm
            // SX1276:
            break;

        case 'H':  _power = 0x07;
            // SX1272: 6dBm
            // SX1276:
            break;

        default:   state = -1;
            break;
    }

    value = _power;

    if (p=='x') 
    {
        // we set only the PA_BOOST pin
        // limit to 14dBm
        value = 0x0C;
        value = value | B10000000;
        // TODO: Have to set RegOcp for OcpOn and OcpTrim
    }
    
    if (p=='X')
     {
        // we set the PA_BOOST pin
        value = value | B10000000;
        // and then set the high output power config with register REG_PA_DAC
        writeRegister(RegPaDacReg, 0x87);
        // TODO: Have to set RegOcp for OcpOn and OcpTrim
    }
    else {
        // disable high power output in all other cases
        writeRegister(RegPaDacReg, 0x84);
    }

    // added by C. Pham
    if (_board==SX1272Chip) {
        writeRegister(REG_PA_CONFIG, value);    // Setting output power value
    }
    else {
        // set MaxPower to 7 -> Pmax=10.8+0.6*MaxPower [dBm] = 15
        value = value | B01110000;
        // then Pout = Pmax-(15-_power[3:0]) if  PaSelect=0 (RFO pin for +13dBm)

        writeRegister(REG_PA_CONFIG, value);
    }

    _power=value;

    value = readRegister(REG_PA_CONFIG);

    if( value == _power )
    {
        state = 0;
    }
    else
    {
        state = 1;
    }

    writeRegister(REG_OP_MODE, st0);    // Getting back to previous status
    delay(100);
    return state;
}

/*
 Function: Sets the signal power indicated in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
 Parameters:
   p: power option to set in configuration.
*/
int8_t SX1272::setPowerNum(uint8_t pow)
{
    byte st0;
    int8_t state = 2;
    byte value = 0x00;


    st0 = readRegister(REG_OP_MODE);      // Save the previous status
    if( _modem == LORA )
    { // LoRa Stdby mode to write in registers
        writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);
    }
    else
    { // FSK Stdby mode to write in registers
        writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);
    }

    if ( (pow >= 0) & (pow < 15) )
    {
        _power = pow;
    }
    else
    {
        state = -1;
    }

    // added by C. Pham
    if (_board==SX1276Chip) {
        value=readRegister(REG_PA_CONFIG);
        // clear OutputPower, but keep current value of PaSelect and MaxPower
        value=value & B11110000;
        value=value + _power;
        _power=value;
    }
    writeRegister(REG_PA_CONFIG, _power);   // Setting output power value
    value = readRegister(REG_PA_CONFIG);

    if( value == _power )
    {
        state = 0;
    }
    else
    {
        state = 1;
    }

    writeRegister(REG_OP_MODE, st0);    // Getting back to previous status
    delay(100);
    return state;
}


/*
 Function: Gets the preamble length from the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::getPreambleLength()
{
    int8_t state = 2;
    uint8_t p_length;


    state = 1;
    if( _modem == LORA )
    { // LORA mode
        p_length = readRegister(REG_PREAMBLE_MSB_LORA);
        // Saving MSB preamble length in LoRa mode
        _preamblelength = (p_length << 8) & 0xFFFF;
        p_length = readRegister(REG_PREAMBLE_LSB_LORA);
        // Saving LSB preamble length in LoRa mode
        _preamblelength = _preamblelength + (p_length & 0xFFFF);
    }
    else
    { // FSK mode
        p_length = readRegister(REG_PREAMBLE_MSB_FSK);
        // Saving MSB preamble length in FSK mode
        _preamblelength = (p_length << 8) & 0xFFFF;
        p_length = readRegister(REG_PREAMBLE_LSB_FSK);
        // Saving LSB preamble length in FSK mode
        _preamblelength = _preamblelength + (p_length & 0xFFFF);
    }
    state = 0;
    return state;
}

/*
 Function: Sets the preamble length in the module
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
 Parameters:
   l: length value to set as preamble length.
*/
uint8_t SX1272::setPreambleLength(uint16_t l)
{
    byte st0;
    uint8_t p_length;
    int8_t state = 2;

    st0 = readRegister(REG_OP_MODE);    // Save the previous status
    state = 1;
    if( _modem == LORA )
    { // LoRa mode
        writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);    // Set Standby mode to write in registers
        p_length = ((l >> 8) & 0x0FF);
        // Storing MSB preamble length in LoRa mode
        writeRegister(REG_PREAMBLE_MSB_LORA, p_length);
        p_length = (l & 0x0FF);
        // Storing LSB preamble length in LoRa mode
        writeRegister(REG_PREAMBLE_LSB_LORA, p_length);
    }
    else
    { // FSK mode
        writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);    // Set Standby mode to write in registers
        p_length = ((l >> 8) & 0x0FF);
        // Storing MSB preamble length in FSK mode
        writeRegister(REG_PREAMBLE_MSB_FSK, p_length);
        p_length = (l & 0x0FF);
        // Storing LSB preamble length in FSK mode
        writeRegister(REG_PREAMBLE_LSB_FSK, p_length);
    }

    state = 0;

    writeRegister(REG_OP_MODE, st0);    // Getting back to previous status
    delay(100);
    return state;
}

/*
 Function: Gets the payload length from the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::getPayloadLength()
{
    uint8_t state = 2;


    if( _modem == LORA )
    { // LORA mode
        // Saving payload length in LoRa mode
        _payloadlength = readRegister(REG_PAYLOAD_LENGTH_LORA);
        state = 1;
    }
    else
    { // FSK mode
        // Saving payload length in FSK mode
        _payloadlength = readRegister(REG_PAYLOAD_LENGTH_FSK);
        state = 1;
    }


    state = 0;
    return state;
}

/*
 Function: Sets the packet length in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
*/
int8_t SX1272::setPacketLength()
{
    uint16_t length;

    length = _payloadlength + OFFSET_PAYLOADLENGTH;
    return setPacketLength(length);
}

/*
 Function: Sets the packet length in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
 Parameters:
   l: length value to set as payload length.
*/
int8_t SX1272::setPacketLength(uint8_t l)
{
    byte st0;
    byte value = 0x00;
    int8_t state = 2;

    st0 = readRegister(REG_OP_MODE);    // Save the previous status
    packet_sent.length = l;

    if( _modem == LORA )
    { // LORA mode
        writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);    // Set LoRa Standby mode to write in registers
        writeRegister(REG_PAYLOAD_LENGTH_LORA, packet_sent.length);
        // Storing payload length in LoRa mode
        value = readRegister(REG_PAYLOAD_LENGTH_LORA);
    }
    else
    { // FSK mode
        writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);    //  Set FSK Standby mode to write in registers
        writeRegister(REG_PAYLOAD_LENGTH_FSK, packet_sent.length);
        // Storing payload length in FSK mode
        value = readRegister(REG_PAYLOAD_LENGTH_FSK);
    }

    if( packet_sent.length == value )
    {
        state = 0;
    }
    else
    {
        state = 1;
    }

    writeRegister(REG_OP_MODE, st0);    // Getting back to previous status
    // comment by C. Pham
    // this delay is included in the send delay overhead
    // TODO: do we really need this delay?
    delay(250);
    return state;
}

/*
 Function: Gets the node address in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::getNodeAddress()
{
    byte st0 = 0;
    uint8_t state = 2;

    if( _modem == LORA )
    { // LoRa mode
        st0 = readRegister(REG_OP_MODE);    // Save the previous status
        // Allowing access to FSK registers while in LoRa standby mode
        writeRegister(REG_OP_MODE, LORA_STANDBY_FSK_REGS_MODE);
    }

    // Saving node address
    _nodeAddress = readRegister(REG_NODE_ADRS);
    state = 1;

    if( _modem == LORA )
    {
        writeRegister(REG_OP_MODE, st0);        // Getting back to previous status
    }

    state = 0;

    return state;
}

/*
 Function: Sets the node address in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
 Parameters:
   addr: address value to set as node address.
*/
int8_t SX1272::setNodeAddress(uint8_t addr)
{
    byte st0;
    byte value;
    uint8_t state = 2;

    if( addr > 255 )
    {
        state = -1;
    }
    else
    {
        // Saving node address
        _nodeAddress = addr;
        st0 = readRegister(REG_OP_MODE);      // Save the previous status

        if( _modem == LORA )
        { // Allowing access to FSK registers while in LoRa standby mode
            writeRegister(REG_OP_MODE, LORA_STANDBY_FSK_REGS_MODE);
        }
        else
        { //Set FSK Standby mode to write in registers
            writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);
        }

        // Storing node and broadcast address
        writeRegister(REG_NODE_ADRS, addr);
        writeRegister(REG_BROADCAST_ADRS, BROADCAST_0);

        value = readRegister(REG_NODE_ADRS);
        writeRegister(REG_OP_MODE, st0);        // Getting back to previous status

        if( value == _nodeAddress )
        {
            state = 0;
        }
        else
        {
            state = 1;
        }
    }
    return state;
}

/*
 Function: Gets the SNR value in LoRa mode.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
*/
int8_t SX1272::getSNR()
{   // getSNR exists only in LoRa mode
    int8_t state = 2;
    byte value;

    if( _modem == LORA )
    { // LoRa mode
        state = 1;
        value = readRegister(REG_PKT_SNR_VALUE);
        if( value & 0x80 ) // The SNR sign bit is 1
        {
            // Invert and divide by 4
            value = ( ( ~value + 1 ) & 0xFF ) >> 2;
            _SNR = -value;
        }
        else
        {
            // Divide by 4
            _SNR = ( value & 0xFF ) >> 2;
        }
        state = 0;
    }
    else
    { // forbidden command if FSK mode
        state = -1;
    }
    return state;
}

/*
 Function: Gets the current value of RSSI.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::getRSSI()
{
    uint8_t state = 2;
    int rssi_mean = 0;
    int total = 5;

    if( _modem == LORA )
    {
        /// LoRa mode
        // get mean value of RSSI
        for(int i = 0; i < total; i++)
        {
            // modified by C. Pham
            // with SX1276 we have to add 20 to OFFSET_RSSI
            _RSSI = -(OFFSET_RSSI+(_board==SX1276Chip?20:0)) + readRegister(REG_RSSI_VALUE_LORA);
            rssi_mean += _RSSI;
        }

        rssi_mean = rssi_mean / total;
        _RSSI = rssi_mean;

        state = 0;
    }
    else
    {
        /// FSK mode
        // get mean value of RSSI
        for(int i = 0; i < total; i++)
        {
            _RSSI = (readRegister(REG_RSSI_VALUE_FSK) >> 1);
            rssi_mean += _RSSI;
        }
        rssi_mean = rssi_mean / total;
        _RSSI = rssi_mean;

        state = 0;

    }
    return state;
}

/*
 Function: Gets the RSSI of the last packet received in LoRa mode.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
*/
int16_t SX1272::getRSSIpacket()
{   // RSSIpacket only exists in LoRa
    int8_t state = 2;

    state = 1;
    if( _modem == LORA )
    { // LoRa mode
        state = getSNR();
        if( state == 0 )
        {
            // added by C. Pham
            _RSSIpacket = readRegister(REG_PKT_RSSI_VALUE);

            if( _SNR < 0 )
            {
                // commented by C. Pham
                //_RSSIpacket = -NOISE_ABSOLUTE_ZERO + 10.0 * SignalBwLog[_bandwidth] + NOISE_FIGURE + ( double )_SNR;

                // added by C. Pham, using Semtech SX1272 rev3 March 2015
                _RSSIpacket = -(OFFSET_RSSI+(_board==SX1276Chip?20:0)) + (double)_RSSIpacket + (double)_SNR*0.25;
                state = 0;
            }
            else
            {
                // commented by C. Pham
                //_RSSIpacket = readRegister(REG_PKT_RSSI_VALUE);
                _RSSIpacket = -(OFFSET_RSSI+(_board==SX1276Chip?20:0)) + (double)_RSSIpacket;
                //end
                state = 0;
            }
        }
    }
    else
    { // RSSI packet doesn't exist in FSK mode
        state = -1;
    }
    return state;
}



/*
 Function: Gets the current supply limit of the power amplifier, protecting battery chemistries.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
 Parameters:
   rate: value to compute the maximum current supply. Maximum current is 45+5*'rate' [mA]
*/
uint8_t SX1272::getMaxCurrent()
{
    int8_t state = 2;
    byte value;

    state = 1;
    _maxCurrent = readRegister(REG_OCP);

    // extract only the OcpTrim value from the OCP register
    _maxCurrent &= B00011111;

    if( _maxCurrent <= 15 )
    {
        value = (45 + (5 * _maxCurrent));
    }
    else if( _maxCurrent <= 27 )
    {
        value = (-30 + (10 * _maxCurrent));
    }
    else
    {
        value = 240;
    }

    _maxCurrent = value;

    state = 0;
    return state;
}

/*
 Function: Limits the current supply of the power amplifier, protecting battery chemistries.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden parameter value for this function
 Parameters:
   rate: value to compute the maximum current supply. Maximum current is 45+5*'rate' [mA]
*/
int8_t SX1272::setMaxCurrent(uint8_t rate)
{
    int8_t state = 2;
    byte st0;



    // Maximum rate value = 0x1B, because maximum current supply = 240 mA
    if (rate > 0x1B)
    {
        state = -1;

    }
    else
    {
        // Enable Over Current Protection
        rate |= B00100000;

        state = 1;
        st0 = readRegister(REG_OP_MODE);    // Save the previous status
        if( _modem == LORA )
        { // LoRa mode
            writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);  // Set LoRa Standby mode to write in registers
        }
        else
        { // FSK mode
            writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);   // Set FSK Standby mode to write in registers
        }
        writeRegister(REG_OCP, rate);       // Modifying maximum current supply
        writeRegister(REG_OP_MODE, st0);        // Getting back to previous status
        state = 0;
    }
    return state;
}


/*
 Function: It truncs the payload length if it is greater than 0xFF.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::truncPayload(uint16_t length16)
{
    uint8_t state = 2;

    state = 1;


    if( length16 > MAX_PAYLOAD )
    {
        _payloadlength = MAX_PAYLOAD;
    }
    else
    {
        _payloadlength = (length16 & 0xFF);
    }
    state = 0;

    return state;
}

/*
 Function: It sets an ACK in FIFO in order to send it.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::setACK()
{
    uint8_t state = 2;

    clearFlags();   // Initializing flags

    if( _modem == LORA )
    { // LoRa mode
        writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);  // Stdby LoRa mode to write in FIFO
    }
    else
    { // FSK mode
        writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);   // Stdby FSK mode to write in FIFO
    }

    // Setting ACK length in order to send it
    state = setPacketLength(ACK_LENGTH);
    if( state == 0 )
    {
        // Setting ACK
        ACK.dst = packet_received.src; // ACK destination is packet source
        ACK.type = PKT_TYPE_ACK;
        ACK.src = packet_received.dst; // ACK source is packet destination
        ACK.packnum = packet_received.packnum; // packet number that has been correctly received
        ACK.length = 2;
        ACK.data[0] = _reception;   // CRC of the received packet
        // added by C. Pham
        // store the SNR
        ACK.data[1]= readRegister(REG_PKT_SNR_VALUE);

        // Setting address pointer in FIFO data buffer
        writeRegister(REG_FIFO_ADDR_PTR, 0x80);

        state = 1;

        // Writing ACK to send in FIFO
        writeRegister(REG_FIFO, ACK.dst);       // Writing the destination in FIFO
        writeRegister(REG_FIFO, ACK.type);
        writeRegister(REG_FIFO, ACK.src);       // Writing the source in FIFO
        writeRegister(REG_FIFO, ACK.packnum);   // Writing the packet number in FIFO
        writeRegister(REG_FIFO, ACK.length);    // Writing the packet length in FIFO
        writeRegister(REG_FIFO, ACK.data[0]);   // Writing the ACK in FIFO
        writeRegister(REG_FIFO, ACK.data[1]);   // Writing the ACK in FIFO

        //#if (SX1272_debug_mode > 0)
        Serial.println(F("## ACK set and written in FIFO ##"));
        // Print the complete ACK if debug_mode
        Serial.println(F("## ACK to send:"));
        Serial.print(F("Destination: "));
        Serial.println(ACK.dst);                // Printing destination
        Serial.print(F("Source: "));
        Serial.println(ACK.src);                // Printing source
        Serial.print(F("ACK number: "));
        Serial.println(ACK.packnum);            // Printing ACK number
        Serial.print(F("ACK length: "));
        Serial.println(ACK.length);             // Printing ACK length
        Serial.print(F("ACK payload: "));
        Serial.println(ACK.data[0]);            // Printing ACK payload
        Serial.print(F("ACK SNR last rcv pkt: "));
        Serial.println(_SNR);
        Serial.println(F("##"));
        Serial.println();
        //#endif

        state = 0;
        _reception = CORRECT_PACKET;        // Updating value to next packet

        // comment by C. Pham
        // TODO: do we really need this delay?
        delay(500);
    }
    return state;
}

/*
 Function: Configures the module to receive information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::receive()
{
    uint8_t state = 1;


    // Initializing packet_received struct
    memset( &packet_received, 0x00, sizeof(packet_received) );

    // Setting Testmode
    // commented by C. Pham
    //writeRegister(0x31,0x43);

    // Set LowPnTxPllOff
    // modified by C. Pham from 0x09 to 0x08
    writeRegister(REG_PA_RAMP, 0x08);

    //writeRegister(REG_LNA, 0x23);         // Important in reception
    // modified by C. Pham
    writeRegister(REG_LNA, LNA_MAX_GAIN);
    writeRegister(REG_FIFO_ADDR_PTR, 0x00);  // Setting address pointer in FIFO data buffer
    // change RegSymbTimeoutLsb
    // comment by C. Pham
    // single_chan_pkt_fwd uses 00 00001000
    // why here we have 11 11111111
    // change RegSymbTimeoutLsb
    //writeRegister(REG_SYMB_TIMEOUT_LSB, 0xFF);

    // modified by C. Pham
    if (_spreadingFactor == SF_10 || _spreadingFactor == SF_11 || _spreadingFactor == SF_12) {
        writeRegister(REG_SYMB_TIMEOUT_LSB,0x05);
    } else {
        writeRegister(REG_SYMB_TIMEOUT_LSB,0x08);
    }
    //end

    writeRegister(REG_FIFO_RX_BYTE_ADDR, 0x00); // Setting current value of reception buffer pointer
    //clearFlags();                     // Initializing flags
    //state = 1;
    if( _modem == LORA )
    { // LoRa mode
        state = setPacketLength(MAX_LENGTH);    // With MAX_LENGTH gets all packets with length < MAX_LENGTH
        writeRegister(REG_OP_MODE, LORA_RX_MODE);     // LORA mode - Rx
    }
    else
    { // FSK mode
        state = setPacketLength();
        writeRegister(REG_OP_MODE, FSK_RX_MODE);  // FSK mode - Rx
    }
    return state;
}

/*
 Function: Configures the module to receive information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::receivePacketMAXTimeout()
{
    return receivePacketTimeout(MAX_TIMEOUT);
}

/*
 Function: Configures the module to receive information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::receivePacketTimeout()
{
    setTimeout();
    return receivePacketTimeout(_sendTime);
}


uint8_t SX1272::receivePacketTimeout(uint16_t wait)
{
    uint8_t state = 2;
    uint8_t state_f = 2;

    state = receive();
    if( state == 0 )
    {
        if( availableData(wait) )
        {
            // If packet received, getPacket
            state_f = getPacket();
        }
        else
        {
            state_f = 1;
        }
    }
    else
    {
        state_f = state;
    }
    return state_f;
}


/*
 Function: Configures the module to receive all the information on air, before MAX_TIMEOUT expires.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::receiveAll()
{
    return receiveAll(MAX_TIMEOUT);
}

/*
 Function: Configures the module to receive all the information on air.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::receiveAll(uint16_t wait)
{
    uint8_t state = 2;
    byte config1;

    if( _modem == FSK )
    { // FSK mode
        writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);       // Setting standby FSK mode
        config1 = readRegister(REG_PACKET_CONFIG1);
        config1 = config1 & B11111001;          // clears bits 2-1 from REG_PACKET_CONFIG1
        writeRegister(REG_PACKET_CONFIG1, config1);     // AddressFiltering = None
    }

    state = receive();  // Setting Rx mode
    if( state == 0 )
    {
        state = getPacket(wait);    // Getting all packets received in wait
    }
    return state;
}

/*
 Function: If a packet is received, checks its destination.
 Returns: Boolean that's 'true' if the packet is for the module and
          it's 'false' if the packet is not for the module.
*/
boolean SX1272::availableData()
{
    return availableData(MAX_TIMEOUT);
}

/*
 Function: If a packet is received, checks its destination.
 Returns: Boolean that's 'true' if the packet is for the module and
          it's 'false' if the packet is not for the module.
 Parameters:
   wait: time to wait while there is no a valid header received.
*/
boolean SX1272::availableData(uint16_t wait)
{
    byte value;
    byte header = 0;
    boolean forme = false;
    boolean _hreceived = false;
    unsigned long previous;

    previous = millis();
    if( _modem == LORA )
    { // LoRa mode
        value = readRegister(REG_IRQ_FLAGS);
        // Wait to ValidHeader interrupt
        while( (bitRead(value, 4) == 0) && (millis() - previous < (unsigned long)wait) )
        {
            value = readRegister(REG_IRQ_FLAGS);
            // Condition to avoid an overflow (DO NOT REMOVE)
            if( millis() < previous )
            {
                previous = millis();
            }
        } // end while (millis)

        if( bitRead(value, 4) == 1 )
        { // header received

            _hreceived = true;


            while( (header == 0) && (millis() - previous < (unsigned long)wait) )
            { // Waiting to read first payload bytes from packet
                header = readRegister(REG_FIFO_RX_BYTE_ADDR);
                // Condition to avoid an overflow (DO NOT REMOVE)
                if( millis() < previous )
                {
                    previous = millis();
                }
            }

            if( header != 0 )
            { // Reading first byte of the received packet

                _destination = readRegister(REG_FIFO);
            }
        }
        else
        {
            forme = false;
            _hreceived = false;
        }
    }
    else
    { // FSK mode
        value = readRegister(REG_IRQ_FLAGS2);
        // Wait to Payload Ready interrupt
        while( (bitRead(value, 2) == 0) && (millis() - previous < wait) )
        {
            value = readRegister(REG_IRQ_FLAGS2);
            // Condition to avoid an overflow (DO NOT REMOVE)
            if( millis() < previous )
            {
                previous = millis();
            }
        }// end while (millis)
        if( bitRead(value, 2) == 1 )    // something received
        {
            _hreceived = true;

            // Reading first byte of the received packet
            _destination = readRegister(REG_FIFO);
        }
        else
        {
            forme = false;
            _hreceived = false;
        }
    }
    // We use _hreceived because we need to ensure that _destination value is correctly
    // updated and is not the _destination value from the previously packet
    if( _hreceived == true )
    { // Checking destination



        // modified by C. Pham
        // if _rawFormat, accept all
        if( (_destination == _nodeAddress) || (_destination == BROADCAST_0) || _rawFormat)

        { // LoRa or FSK mode
            forme = true;

        }
        else
        {
            forme = false;

            if( _modem == LORA )    // STANDBY PARA MINIMIZAR EL CONSUMO
            { // LoRa mode
                //writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);    // Setting standby LoRa mode
            }
            else
            { //  FSK mode
                writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);   // Setting standby FSK mode
            }
        }
    }
    //----else
    //  {
    //  }
    return forme;
}

/*
 Function: It gets and stores a packet if it is received before MAX_TIMEOUT expires.
 Returns:  Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::getPacketMAXTimeout()
{
    return getPacket(MAX_TIMEOUT);
}

/*
 Function: It gets and stores a packet if it is received.
 Returns:  Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
int8_t SX1272::getPacket()
{
    return getPacket(MAX_TIMEOUT);
}

/*
 Function: It gets and stores a packet if it is received before ending 'wait' time.
 Returns:  Integer that determines if there has been any error
   // added by C. Pham
   state = 5  --> The command has been executed with no errors and an ACK is requested
   state = 3  --> The command has been executed but packet has been incorrectly received
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden parameter value for this function
 Parameters:
   wait: time to wait while there is no a valid header received.
*/
int8_t SX1272::getPacket(uint16_t wait)
{
    uint8_t state = 2;
    byte value = 0x00;
    unsigned long previous;
    boolean p_received = false;


    previous = millis();
    if( _modem == LORA )
    { // LoRa mode
        value = readRegister(REG_IRQ_FLAGS);
        // Wait until the packet is received (RxDone flag) or the timeout expires
        while( (bitRead(value, 6) == 0) && (millis() - previous < (unsigned long)wait) )
        {
            value = readRegister(REG_IRQ_FLAGS);
            // Condition to avoid an overflow (DO NOT REMOVE)
            if( millis() < previous )
            {
                previous = millis();
            }
        } // end while (millis)

        if( (bitRead(value, 6) == 1) && (bitRead(value, 5) == 0) )
        { // packet received & CRC correct
            p_received = true;  // packet correctly received
            _reception = CORRECT_PACKET;

        }
        else
        {
            if( bitRead(value, 5) != 0 )
            { // CRC incorrect
                _reception = INCORRECT_PACKET;
                state = 3;

            }
        }
        //writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);    // Setting standby LoRa mode
    }


    if( p_received == true )
    {
        // Store the packet
        if( _modem == LORA )
        {
            // comment by C. Pham
            // set the FIFO addr to 0 to read again all the bytes
            writeRegister(REG_FIFO_ADDR_PTR, 0x00);     // Setting address pointer in FIFO data buffer


            //modified by C. Pham
            if (!_rawFormat)
                packet_received.dst = readRegister(REG_FIFO);   // Storing first byte of the received packet
            else
                packet_received.dst = 0;
        }
        else
        {
            value = readRegister(REG_PACKET_CONFIG1);
            if( (bitRead(value, 2) == 0) && (bitRead(value, 1) == 0) )
            {
                packet_received.dst = readRegister(REG_FIFO); // Storing first byte of the received packet
            }
            else
            {
                packet_received.dst = _destination;         // Storing first byte of the received packet
            }
        }

        // modified by C. Pham
        if (!_rawFormat) {
            packet_received.type = readRegister(REG_FIFO);      // Reading second byte of the received packet
            packet_received.src = readRegister(REG_FIFO);       // Reading second byte of the received packet
            packet_received.packnum = readRegister(REG_FIFO);   // Reading third byte of the received packet
            //packet_received.length = readRegister(REG_FIFO);  // Reading fourth byte of the received packet
        }
        else {
            packet_received.type = 0;
            packet_received.src = 0;
            packet_received.packnum = 0;
        }

        packet_received.length = readRegister(REG_RX_NB_BYTES);

        if( _modem == LORA )
        {
            if (_rawFormat) {
                _payloadlength=packet_received.length;
            }
            else
                _payloadlength = packet_received.length - OFFSET_PAYLOADLENGTH;
        }
        if( packet_received.length > (MAX_LENGTH + 1) )
        {

        }
        else
        {
            for(unsigned int i = 0; i < _payloadlength; i++)
            {
                packet_received.data[i] = readRegister(REG_FIFO); // Storing payload
            }

            // commented by C. Pham
            //packet_received.retry = readRegister(REG_FIFO);

            state = 0;

        }
    }
    else
    {
        state = 1;
        if( (_reception == INCORRECT_PACKET) && (_retries < _maxRetries) )
        {
            // comment by C. Pham
            // what is the purpose of incrementing retries here?
            // bug? not needed?
            _retries++;

        }
    }

    if( _modem == LORA )
    {
        writeRegister(REG_FIFO_ADDR_PTR, 0x00);  // Setting address pointer in FIFO data buffer
    }
    clearFlags();   // Initializing flags
    if( wait > MAX_WAIT )
    {
        state = -1;

    }

    return state;
}

/*
 Function: It sets the packet destination.
 Returns:  Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
 Parameters:
   dest: destination value of the packet sent.
*/
int8_t SX1272::setDestination(uint8_t dest)
{
    int8_t state = 2;
    state = 1;
    _destination = dest; // Storing destination in a global variable
    packet_sent.dst = dest;  // Setting destination in packet structure
    packet_sent.src = _nodeAddress; // Setting source in packet structure
    packet_sent.packnum = _packetNumber;    // Setting packet number in packet structure
    _packetNumber++;
    state = 0;

    return state;
}


/*
 Function: It sets a char array payload packet in a packet struct.
 Returns:  Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::setPayload(char *payload)
{
    uint8_t state = 2;
    uint8_t state_f = 2;
    uint16_t length16;

    state = 1;
    length16 = (uint16_t)strlen(payload);
    state = truncPayload(length16);
    if( state == 0 )
    {
        // fill data field until the end of the string
        for(unsigned int i = 0; i < _payloadlength; i++)
        {
            packet_sent.data[i] = payload[i];
        }
    }
    else
    {
        state_f = state;
    }
    if( ( _modem == FSK ) && ( _payloadlength > MAX_PAYLOAD_FSK ) )
    {
        _payloadlength = MAX_PAYLOAD_FSK;
        state = 1;
    }
    // set length with the actual counter value
    state_f = setPacketLength();    // Setting packet length in packet structure
    return state_f;
}

/*
 Function: It sets an uint8_t array payload packet in a packet struct.
 Returns:  Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::setPayload(uint8_t *payload)
{
    uint8_t state = 2;


    state = 1;
    if( ( _modem == FSK ) && ( _payloadlength > MAX_PAYLOAD_FSK ) )
    {
        _payloadlength = MAX_PAYLOAD_FSK;
        state = 1;
    }
    for(unsigned int i = 0; i < _payloadlength; i++)
    {
        packet_sent.data[i] = payload[i];   // Storing payload in packet structure
    }
    // set length with the actual counter value
    state = setPacketLength();  // Setting packet length in packet structure
    return state;
}

/*
 Function: It sets a packet struct in FIFO in order to send it.
 Returns:  Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::setPacket(uint8_t dest, char *payload)
{
    int8_t state = 2;

    clearFlags();   // Initializing flags

    if( _modem == LORA )
    { // LoRa mode
        writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);  // Stdby LoRa mode to write in FIFO
    }
    else
    { // FSK mode
        writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);   // Stdby FSK mode to write in FIFO
    }

    _reception = CORRECT_PACKET;    // Updating incorrect value
    if( _retries == 0 )
    { // Updating this values only if is not going to re-send the last packet
        state = setDestination(dest);   // Setting destination in packet structure
        packet_sent.retry = _retries;
        if( state == 0 )
        {
            state = setPayload(payload);
        }
    }
    else
    {
        // comment by C. Pham
        // why to increase the length here?
        // bug?
        if( _retries == 1 )
        {
            packet_sent.length++;
        }
        state = setPacketLength();
        packet_sent.retry = _retries;
    }

    // added by C. Pham
    // set the type to be a data packet
    packet_sent.type |= PKT_TYPE_DATA;


    writeRegister(REG_FIFO_ADDR_PTR, 0x80);  // Setting address pointer in FIFO data buffer
    if( state == 0 )
    {
        state = 1;
        // Writing packet to send in FIFO

        writeRegister(REG_FIFO, packet_sent.dst);       // Writing the destination in FIFO
        // added by C. Pham
        writeRegister(REG_FIFO, packet_sent.type);      // Writing the packet type in FIFO
        writeRegister(REG_FIFO, packet_sent.src);       // Writing the source in FIFO
        writeRegister(REG_FIFO, packet_sent.packnum);   // Writing the packet number in FIFO
        // commented by C. Pham
        //writeRegister(REG_FIFO, packet_sent.length);  // Writing the packet length in FIFO
        for(unsigned int i = 0; i < _payloadlength; i++)
        {
            writeRegister(REG_FIFO, packet_sent.data[i]);  // Writing the payload in FIFO
        }
        // commented by C. Pham
        //writeRegister(REG_FIFO, packet_sent.retry);       // Writing the number retry in FIFO
        state = 0;
    }

    return state;
}

/*
 Function: It sets a packet struct in FIFO in order to sent it.
 Returns:  Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::setPacket(uint8_t dest, uint8_t *payload)
{
    int8_t state = 2;
    byte st0;


    st0 = readRegister(REG_OP_MODE);    // Save the previous status
    clearFlags();   // Initializing flags

    if( _modem == LORA )
    { // LoRa mode
        writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);  // Stdby LoRa mode to write in FIFO
    }


    _reception = CORRECT_PACKET;    // Updating incorrect value to send a packet (old or new)
    if( _retries == 0 )
    { // Sending new packet
        state = setDestination(dest);   // Setting destination in packet structure
        packet_sent.retry = _retries;
        if( state == 0 )
        {
            state = setPayload(payload);
        }
    }
    else
    {
        // comment by C. Pham
        // why to increase the length here?
        // bug?
        if( _retries == 1 )
        {
            packet_sent.length++;
        }
        state = setPacketLength();
        packet_sent.retry = _retries;

    }

    // added by C. Pham
    // set the type to be a data packet
    packet_sent.type |= PKT_TYPE_DATA;


    writeRegister(REG_FIFO_ADDR_PTR, 0x80);  // Setting address pointer in FIFO data buffer
    if( state == 0 )
    {
        state = 1;
        // Writing packet to send in FIFO

        writeRegister(REG_FIFO, packet_sent.dst);       // Writing the destination in FIFO
        // added by C. Pham
        writeRegister(REG_FIFO, packet_sent.type);      // Writing the packet type in FIFO
        writeRegister(REG_FIFO, packet_sent.src);       // Writing the source in FIFO
        writeRegister(REG_FIFO, packet_sent.packnum);   // Writing the packet number in FIFO
        // commented by C. Pham
        //writeRegister(REG_FIFO, packet_sent.length);  // Writing the packet length in FIFO
        for(unsigned int i = 0; i < _payloadlength; i++)
        {
            writeRegister(REG_FIFO, packet_sent.data[i]);  // Writing the payload in FIFO
        }
        // commented by C. Pham
        //writeRegister(REG_FIFO, packet_sent.retry);       // Writing the number retry in FIFO
        state = 0;

    }
    writeRegister(REG_OP_MODE, st0);    // Getting back to previous status
    return state;
}


/*
 Function: Configures the module to transmit information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::sendWithTimeout(uint16_t wait)
{
    uint8_t state = 2;
    byte value = 0x00;
    unsigned long previous;


    // wait to TxDone flag
    previous = millis();
    clearFlags();   // Initializing flags

    writeRegister(REG_OP_MODE, LORA_TX_MODE);  // LORA mode - Tx

    value = readRegister(REG_IRQ_FLAGS);
    // Wait until the packet is sent (TX Done flag) or the timeout expires

    // very dumb wait
    while ((bitRead(value, 3) == 0) && (millis() - previous < wait))
    {
        value = readRegister(REG_IRQ_FLAGS);
        // Condition to avoid an overflow (DO NOT REMOVE)
        if( millis() < previous )
        {
            previous = millis();
        }
        delay(1);
    }
    state = 1;
  

    if( bitRead(value, 3) == 1 )
    {
        state = 0;  // Packet successfully sent
    }


    clearFlags();       // Initializing flags

    return state;
}



/*
 Function: Configures the module to transmit information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::sendPacketTimeout(uint8_t dest, uint8_t *payload, uint16_t length16)
{
    uint8_t error = 0;

    
    error = truncPayload(length16);


    error = setPacket(dest, payload); // Setting a packet with 'dest' destination
                                          // and writing it in FIFO.
      setTimeout();
     sendWithTimeout(MAX_TIMEOUT);

   // error = sendWithTimeout();    // Sending the packet

    return error;
}


/*
 Function: It gets the temperature from the measurement block module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::getTemp()
{
    byte st0;
    uint8_t state = 2;

    st0 = readRegister(REG_OP_MODE);    // Save the previous status

    if( _modem == LORA )
    { // Allowing access to FSK registers while in LoRa standby mode
        writeRegister(REG_OP_MODE, LORA_STANDBY_FSK_REGS_MODE);
    }

    state = 1;
    // Saving temperature value
    _temp = readRegister(REG_TEMP);
    if( _temp & 0x80 ) // The SNR sign bit is 1
    {
        // Invert and divide by 4
        _temp = ( ( ~_temp + 1 ) & 0xFF );
    }
    else
    {
        // Divide by 4
        _temp = ( _temp & 0xFF );
    }


    if( _modem == LORA )
    {
        writeRegister(REG_OP_MODE, st0);    // Getting back to previous status
    }

    state = 0;
    return state;
}


void SX1272::setPacketType(uint8_t type)
{
    packet_sent.type=type;

    if (type & PKT_FLAG_ACK_REQ)
        _requestACK=1;
}



/*
 Function: Sets the sync word in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
 Parameters:
   cod: sw is sync word value to set in LoRa modem configuration.
*/
int8_t  SX1272::setSyncWord(uint8_t sw)
{
    byte st0;
    int8_t state = 2;
    byte config1;

    st0 = readRegister(REG_OP_MODE);        // Save the previous status


    writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);      // Set Standby mode to write in registers

    writeRegister(REG_SYNC_WORD, sw);

    delay(100);

    config1 = readRegister(REG_SYNC_WORD);

    if (config1==sw) {
        state=0;
        _syncWord = sw;
    }
    else {
        state=1;

    }

    writeRegister(REG_OP_MODE,st0); // Getting back to previous status
    delay(100);
    return state;
}


/**
 * SetsleepMode
 *     responsible for requesting the SX1272 module is set into sleep mode
 * @return error - non zero state for error
 */
int8_t SX1272::setSleepMode() 
{
    int8_t error = 0;
    byte value;

    // Request module is placed in sleep mode
    writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);
    writeRegister(REG_OP_MODE, LORA_SLEEP_MODE);    
    
    value = readRegister(REG_OP_MODE);

    (value == LORA_SLEEP_MODE) ? ( error = 0) : (error = -1); 

    return error;
}

