/*
 *  Library for LoRa 868 / 915MHz SX1272 LoRa module
 *  
 *  Copyright (C) Libelium Comunicaciones Distribuidas S.L. 
 *  http://www.libelium.com 
 *  
 *  This program is free software: you can redistribute it and/or modify 
 *  it under the terms of the GNU General Public License as published by 
 *  the Free Software Foundation, either version 3 of the License, or 
 *  (at your option) any later version. 
 *  
 *  This program is distributed in the hope that it will be useful, 
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
 *  GNU General Public License for more details.
 *  
 *  You should have received a copy of the GNU General Public License 
 *  along with this program.  If not, see http://www.gnu.org/licenses/. 
 *  
 *  Version:           1.1
 *  Design:            David Gascón 
 *  Implementation:    Covadonga Albiñana & Victor Boria
 */

#ifndef SX1272_h
#define SX1272_h

/******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdlib.h>
#include <stdint.h>
#include "SX172X_defines.h"
#include <stdbool.h>
#include <string.h>

#include "nrf_drv_gpiote.h"
#include "nrf_gpiote.h"
#include "nrf_gpio.h"

#include "nrf_drv_spi.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include "sw_spi.h"
#include "spi.h"

struct SX17X_Packet
{
    uint8_t dst;
    uint8_t type;
    uint8_t src;
    uint8_t packnum;
    uint8_t length;
    uint8_t data[MAX_PAYLOAD];
};


struct SX127X_LoRA_Settings
{
    int8_t power;
    uint32_t channelSet;
    uint8_t mode;
    uint8_t syncWord;
    uint8_t maxCurrent;
};

typedef enum {
    TX_NONE                                      = 0,
    TX_COMPLETE                                  = 1,
    TX_IN_TRANSMISSION                           = 2,
    TX_ERROR                                     = 3,
    TX_REQUEST_SEND                              = 4,
    TX_SETTING_UP_TRANSMISSION                   = 5, 
} SX127X_TX_Packet_States;



/******************************************************************************
 * Class
 ******************************************************************************/

class SX127X
{


public:
    ~SX127X();
    SX127X();

    int8_t  setTX(void);

    /*
     Function: Sets the module ON.
     Returns: uint8_t setLORA state
    */
    uint8_t ON();


    /*
     Function: Sets the module OFF.
     Returns: Nothing
    */
    void OFF();


    /**
     * Function: Responsible for pulling the current power from the register and returns it
     * @return current power 
     */
    uint8_t getPower();

    /**
     * Function: responsible for setting the output power for the antenna
     * @param  p desired power rate
     * @return   0 if error occurs 
     */
    int8_t setPower(char p);

    /**
     * Function: Gets the preamble length from the module.
     * @return preamble length 
     */
    uint16_t getPreambleLength();

    /**
     *  Function: Sets the preamble length in the module
     * @param  l desired preamble length 
     * @return   error state
     */
    int8_t setPreambleLength(uint16_t l);

    /**
     * Function: returns the stored node address 
     * @return node address
     */
    uint8_t getNodeAddress();

    /**
     * Function: responsible for setting up the node address
     * @param  addr desired node addr
     * @return      0 if error occurs
     */
    int8_t setNodeAddress(uint8_t addr);

    /**
     * Function: returns the SNR value 
     * @return SNR value
     */
    int8_t getSNR();

    /**
     * Function: Returns a uint16_t representing the the rssi value of the most recent recieved packet
     * @return RSSI value 
     */
    int16_t getRSSIpacket();


    /**
     * Function: Places SX1272 into RX mode 
     * @return error state 0 if no error 
     */
    uint8_t receive();

    /**
     * Function: Should only be called when DIO_0 is asserted, pulls recieved packet from the registers stores it in global 
     * @return      error state 0 if no error 
     */
    int8_t getPacket(void);

    /**
     * Function: Checks the IRQ register, returns 0 is the TX_DONE_Flag has been set. 
     *         Typically read after DIO_5 is de-asserted. 
     * @return error state
     */
    int8_t checkTransmissionStatus(void);

    /**
     * Function: sets up the registers and places the lora module into TX mode
     * @param  dest    destination address
     * @param  payload pointer to payload
     * @param  length  length of payload
     * @return         returns 0 if no errors 
     */
    int8_t sendPacket(uint8_t dest, uint8_t *payload, uint8_t length);

    /**
     * Function:  writes the desired syncword to the register
     * @param     sw the desired sync word 
     * @return    0 if no error occurs
     */
    int8_t setSyncWord(uint8_t sw);

    /**
     * Function:  places the lora module into sleep mode
     * @return    0 if successful 
     */
    int8_t setSleepMode(); 

private:

    /*
     Function: Reads the indicated register.
     Returns: The content of the register
     Parameters:
       address: address register to read from
    */
    uint8_t readRegister(uint8_t address);

    /*
     Function: Writes on the indicated register.
     Returns: Nothing
     Parameters:
       address: address register to write in
       data : value to write in the register
    */
    void writeRegister(uint8_t address, uint8_t data);

    /**
     * Function: writes & read to the indicated register return 0 if equal
     * @param  address  address
     * @param  data     uint8_t to write
     * @return          0 if equal
     */
    int8_t writeReadRegister(uint8_t address, uint8_t data);

    /**
     * Function: Clears the interruption flags
     * @return Nothing
     */
    int8_t clearFlags();


    /**
     * Function: requests SX127X module to be setup in lora mode 
     * @return error state
     */
    int8_t setLORA();

    /**
     *  Function: Checks if SF is a valid value.
     * @param  spr Spread factor
     * @return     true if aviailbe
     */
    bool isSF(uint8_t spr);

    /**
     * Function: Gets the SF within the module is configured.
     * @return    0 if no error
     */
    int8_t getSF();

    /**
     * Function:   Checks if BW is a valid value.
     * @param  band desired band
     * @return      true if valid
     */
    bool isBW(uint16_t band);

    /**
     * Function: Reads the current BW and returns 0 if read matches the previously set value
     * @return [description]
     */
    int8_t  getBW();

    /**
     *  Function: Checks if CR is a valid value.
     * @param  cod CR
     * @return     1 if valid
     */
    bool isCR(uint8_t cod);

    /**
     *  Function: Indicates the CR within the module is configured.
     * @return 0 if no error 
     */
    int8_t  getCR();

    /**
     * Function: sets the maximum current the SX127X device has access 
     * @param  rate current rate 
     * @return      error state 0 if no error 
     */
    int8_t setMaxCurrent(uint8_t rate);

    /**
     * Function:  responsible for setting up the frequency channel and writing to the registers
     * @param     ch desired channel 
     * @return    0 if no error occurs
     */
    int8_t setChannel(uint32_t ch);

    /**
     * Function: sets up the communications based on a desired LoRA mode
     * @param  mode  desired mode
     * @return       0 if no issue
     */
    int8_t setMode(uint8_t mode);

    /**
     * Function:    Sets the indicated CR in the module.
     * @param  cod  current CR
     * @return      0 if error occurs 
     */
    int8_t  setCR(uint8_t cod);

    /**
     * Function      responsible for writing the desired bandwidth to the resisters
     * @param  band  desired band
     * @return       0 if error 
     */
    int8_t  setBW(uint16_t band);

    /**
     * Function: Sets the indicated SF in the module.
     * @param  spr [description]
     * @return     0 if no error 
     */
    int8_t setSF(uint8_t spr);


public:

    SX17X_Packet _packet_sent;
    SX17X_Packet _packet_received;

    uint16_t _payloadlength;
    uint8_t _nodeAddress; 
    uint8_t _packetNumber;

private:

    uint8_t _board;
    uint8_t _syncWord;
    uint8_t _bandwidth;
    uint8_t _codingRate;
    uint8_t _spreadingFactor;
    uint32_t _channel;
    uint8_t _power;
};


#endif
