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
#include <Arduino.h>

#ifndef inttypes_h
#include <inttypes.h>
#endif

/******************************************************************************
 * Definitions & Declarations
 *****************************************************************************/

// added by C. Pham
#define W_REQUESTED_ACK
//#define W_NET_KEY
#define SX1272_RST  3

//#if defined ARDUINO_AVR_PRO || defined ARDUINO_AVR_NANO || defined ARDUINO_AVR_MINI || defined __MK20DX256__
#define SX1272_SS 10
//#else
//#define SX1272_SS 2
//#endif

#define SX1272Chip  0
#define SX1276Chip  1
// end
#define SX1272_debug_mode 0

//! MACROS //
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)  // read a bit
#define bitSet(value, bit) ((value) |= (1UL << (bit)))    // set bit to '1'
#define bitClear(value, bit) ((value) &= ~(1UL << (bit))) // set bit to '0'


#define SX1272_POWER_LOW            0x02
#define SX1272_POWER_HIGH           0x07
#define SX1272_POWER_MAXIMUM        0x0F

//! REGISTERS //

#define REG_FIFO                                    0x00
#define REG_OP_MODE                                 0x01
#define REG_BITRATE_MSB                             0x02
#define REG_BITRATE_LSB                             0x03
#define REG_FDEV_MSB                                0x04
#define REG_FDEV_LSB                                0x05
#define REG_FRF_MSB                                 0x06
#define REG_FRF_MID                                 0x07
#define REG_FRF_LSB                                 0x08
#define REG_PA_CONFIG                               0x09
#define REG_PA_RAMP                                 0x0A
#define REG_OCP                                     0x0B
#define REG_LNA                                     0x0C
#define REG_RX_CONFIG                               0x0D
#define REG_FIFO_ADDR_PTR                           0x0D
#define REG_RSSI_CONFIG                             0x0E
#define REG_FIFO_TX_BASE_ADDR                       0x0E
#define REG_RSSI_COLLISION                          0x0F
#define REG_FIFO_RX_BASE_ADDR                       0x0F
#define REG_RSSI_THRESH                             0x10
#define REG_FIFO_RX_CURRENT_ADDR                    0x10
#define REG_RSSI_VALUE_FSK                          0x11
#define REG_IRQ_FLAGS_MASK                          0x11
#define REG_RX_BW                                   0x12
#define REG_IRQ_FLAGS                               0x12
#define REG_AFC_BW                                  0x13
#define REG_RX_NB_BYTES                             0x13
#define REG_OOK_PEAK                                0x14
#define REG_RX_HEADER_CNT_VALUE_MSB                 0x14
#define REG_OOK_FIX                                 0x15
#define REG_RX_HEADER_CNT_VALUE_LSB                 0x15
#define REG_OOK_AVG                                 0x16
#define REG_RX_PACKET_CNT_VALUE_MSB                 0x16
#define REG_RX_PACKET_CNT_VALUE_LSB                 0x17
#define REG_MODEM_STAT                              0x18
#define REG_PKT_SNR_VALUE                           0x19
#define REG_AFC_FEI                                 0x1A
#define REG_PKT_RSSI_VALUE                          0x1A
#define REG_AFC_MSB                                 0x1B
#define REG_RSSI_VALUE_LORA                         0x1B
#define REG_AFC_LSB                                 0x1C
#define REG_HOP_CHANNEL                             0x1C
#define REG_FEI_MSB                                 0x1D
#define REG_MODEM_CONFIG1                           0x1D
#define REG_FEI_LSB                                 0x1E
#define REG_MODEM_CONFIG2                           0x1E
#define REG_PREAMBLE_DETECT                         0x1F
#define REG_SYMB_TIMEOUT_LSB                        0x1F
#define REG_RX_TIMEOUT1                             0x20
#define REG_PREAMBLE_MSB_LORA                       0x20
#define REG_RX_TIMEOUT2                             0x21
#define REG_PREAMBLE_LSB_LORA                       0x21
#define REG_RX_TIMEOUT3                             0x22
#define REG_PAYLOAD_LENGTH_LORA                     0x22
#define REG_RX_DELAY                                0x23
#define REG_MAX_PAYLOAD_LENGTH                      0x23
#define REG_OSC                                     0x24
#define REG_HOP_PERIOD                              0x24
#define REG_PREAMBLE_MSB_FSK                        0x25
#define REG_FIFO_RX_BYTE_ADDR                       0x25
#define REG_PREAMBLE_LSB_FSK                        0x26
// added by C. Pham
#define REG_MODEM_CONFIG3                           0x26
// end
#define REG_SYNC_CONFIG                             0x27
#define REG_SYNC_VALUE1                             0x28
#define REG_SYNC_VALUE2                             0x29
#define REG_SYNC_VALUE3                             0x2A
#define REG_SYNC_VALUE4                             0x2B
#define REG_SYNC_VALUE5                             0x2C
#define REG_SYNC_VALUE6                             0x2D
#define REG_SYNC_VALUE7                             0x2E
#define REG_SYNC_VALUE8                             0x2F
#define REG_PACKET_CONFIG1                          0x30
#define REG_PACKET_CONFIG2                          0x31
#define REG_DETECT_OPTIMIZE                         0x31
#define REG_PAYLOAD_LENGTH_FSK                      0x32
#define REG_NODE_ADRS                               0x33
#define REG_BROADCAST_ADRS                          0x34
#define REG_FIFO_THRESH                             0x35
#define REG_SEQ_CONFIG1                             0x36
#define REG_SEQ_CONFIG2                             0x37
#define REG_DETECTION_THRESHOLD                     0x37
#define REG_TIMER_RESOL                             0x38
// added by C. Pham
#define REG_SYNC_WORD                               0x39
//end
#define REG_TIMER1_COEF                             0x39
#define REG_TIMER2_COEF                             0x3A
#define REG_IMAGE_CAL                               0x3B
#define REG_TEMP                                    0x3C
#define REG_LOW_BAT                                 0x3D
#define REG_IRQ_FLAGS1                              0x3E
#define REG_IRQ_FLAGS2                              0x3F
#define REG_DIO_MAPPING1                            0x40
#define REG_DIO_MAPPING2                            0x41
#define REG_VERSION                                 0x42
#define REG_AGC_REF                                 0x43
#define REG_AGC_THRESH1                             0x44
#define REG_AGC_THRESH2                             0x45
#define REG_AGC_THRESH3                             0x46
#define REG_PLL_HOP                                 0x4B
#define REG_TCXO                                    0x58
#define REG_PA_DAC                                  0x5A
#define REG_PLL                                     0x5C
#define REG_PLL_LOW_PN                              0x5E
#define REG_FORMER_TEMP                             0x6C
#define REG_BIT_RATE_FRAC                           0x70

// added by C. Pham
// copied from LoRaMAC-Node
/*!
 * RegImageCal
 */
#define RF_IMAGECAL_AUTOIMAGECAL_MASK               0x7F
#define RF_IMAGECAL_AUTOIMAGECAL_ON                 0x80
#define RF_IMAGECAL_AUTOIMAGECAL_OFF                0x00  // Default

#define RF_IMAGECAL_IMAGECAL_MASK                   0xBF
#define RF_IMAGECAL_IMAGECAL_START                  0x40

#define RF_IMAGECAL_IMAGECAL_RUNNING                0x20
#define RF_IMAGECAL_IMAGECAL_DONE                   0x00  // Default

#define RF_IMAGECAL_TEMPCHANGE_HIGHER               0x08
#define RF_IMAGECAL_TEMPCHANGE_LOWER                0x00

#define RF_IMAGECAL_TEMPTHRESHOLD_MASK              0xF9
#define RF_IMAGECAL_TEMPTHRESHOLD_05                0x00
#define RF_IMAGECAL_TEMPTHRESHOLD_10                0x02  // Default
#define RF_IMAGECAL_TEMPTHRESHOLD_15                0x04
#define RF_IMAGECAL_TEMPTHRESHOLD_20                0x06

#define RF_IMAGECAL_TEMPMONITOR_MASK                0xFE
#define RF_IMAGECAL_TEMPMONITOR_ON                  0x00 // Default
#define RF_IMAGECAL_TEMPMONITOR_OFF                 0x01

// added by C. Pham
// The crystal oscillator frequency of the module
#define RH_LORA_FXOSC                               32000000.0
 
// The Frequency Synthesizer step = RH_LORA_FXOSC / 2^^19
#define RH_LORA_FCONVERT                            (524288 / RH_LORA_FXOSC)


#define CH_10_868                                   0xD84CCC // channel 10, central freq = 865.20MHz
#define CH_11_868                                   0xD86000 // channel 11, central freq = 865.50MHz
#define CH_12_868                                   0xD87333 // channel 12, central freq = 865.80MHz
#define CH_13_868                                   0xD88666 // channel 13, central freq = 866.10MHz
#define CH_14_868                                   0xD89999 // channel 14, central freq = 866.40MHz
#define CH_15_868                                   0xD8ACCC // channel 15, central freq = 866.70MHz
#define CH_16_868                                   0xD8C000 // channel 16, central freq = 867.00MHz
#define CH_17_868                                   0xD90000 // channel 17, central freq = 868.00MHz
#define CH_18_868                                   0xD90666 // 868.1MHz for LoRaWAN test

#define CH_00_900                                   0xE1C51E // channel 00, central freq = 903.08MHz
#define CH_01_900                                   0xE24F5C // channel 01, central freq = 905.24MHz
#define CH_02_900                                   0xE2D999 // channel 02, central freq = 907.40MHz
#define CH_03_900                                   0xE363D7 // channel 03, central freq = 909.56MHz
#define CH_04_900                                   0xE3EE14 // channel 04, central freq = 911.72MHz
#define CH_05_900                                   0xE47851 // channel 05, central freq = 913.88MHz
#define CH_06_900                                   0xE5028F // channel 06, central freq = 916.04MHz
#define CH_07_900                                   0xE58CCC // channel 07, central freq = 918.20MHz
#define CH_08_900                                   0xE6170A // channel 08, central freq = 920.36MHz
#define CH_09_900                                   0xE6A147 // channel 09, central freq = 922.52MHz
#define CH_10_900                                   0xE72B85 // channel 10, central freq = 924.68MHz
#define CH_11_900                                   0xE7B5C2 // channel 11, central freq = 926.84MHz
#define CH_12_900                                   0xE4C000 // default channel 915MHz, the module is configured with it

//LORA BANDWIDTH:
// modified by C. Pham
#define SX1272_BW_125 0x00
#define SX1272_BW_250 0x01
#define SX1272_BW_500 0x02

// use the following constants with setBW()
#define BW_7_8 0x00
#define BW_10_4 0x01
#define BW_15_6 0x02
#define BW_20_8 0x03
#define BW_31_25 0x04
#define BW_41_7 0x05
#define BW_62_5 0x06
#define BW_125 0x07
#define BW_250 0x08
#define BW_500 0x09
// end


//LORA CODING RATE:
#define CR_5 0x01
#define CR_6 0x02
#define CR_7 0x03
#define CR_8 0x04

//LORA SPREADING FACTOR:
#define SF_6 0x06
#define SF_7 0x07
#define SF_8 0x08
#define SF_9 0x09
#define SF_10 0x0A
#define SF_11 0x0B
#define SF_12 0x0C

//LORA MODES:
#define LORA_SLEEP_MODE 0x80
#define LORA_STANDBY_MODE 0x81
#define LORA_TX_MODE 0x83
#define LORA_RX_MODE 0x85

// added by C. Pham
#define LORA_CAD_MODE 0x87
#define LNA_MAX_GAIN                0x23
#define LNA_OFF_GAIN                0x00
#define LNA_LOW_GAIN            0x20
// end

#define LORA_STANDBY_FSK_REGS_MODE 0xC1

//FSK MODES:
#define FSK_SLEEP_MODE 0x00
#define FSK_STANDBY_MODE 0x01
#define FSK_TX_MODE 0x03
#define FSK_RX_MODE 0x05

//OTHER CONSTANTS:

#define HEADER_ON                                   0
#define HEADER_OFF                                  1
#define CRC_ON                                      1
#define CRC_OFF                                     0
#define LORA                                        1
#define BROADCAST_0                                 0x00
#define MAX_LENGTH                                  255
#define MAX_PAYLOAD                                 251
#define MAX_LENGTH_FSK                              64
#define MAX_PAYLOAD_FSK                             60

#define ACK_LENGTH                                  7

#define OFFSET_PAYLOADLENGTH                        4

#define OFFSET_RSSI                                 137
#define NOISE_FIGURE                                6.0
#define NOISE_ABSOLUTE_ZERO                         174.0
#define MAX_TIMEOUT                                 8000      //8000 msec 8.0 sec
#define MAX_WAIT                                    12000        //12000 msec 12.0 sec
#define MAX_RETRIES                                 5
#define CORRECT_PACKET                              0
#define INCORRECT_PACKET                            1

#define PKT_TYPE_MASK                               0xF0
#define PKT_FLAG_MASK                               0x0F
#define PKT_TYPE_DATA                               0x10
#define PKT_TYPE_ACK                                0x20
#define PKT_FLAG_ACK_REQ                            0x08
#define PKT_FLAG_DATA_ENCRYPTED                     0x04
#define PKT_FLAG_DATA_WAPPKEY                       0x02
#define PKT_FLAG_DATA_ISBINARY                      0x01


#define SX1272_REGISTER_VERSION_CODE                0x22
#define SX1276_REGISTER_VERSION_CODE                0x12

#define SX127X_MODES                                11

#define REG_IRQ_RX_TIMEOUT_FLAG                     0x80
#define REG_IRQ_RXDONE_FLAG                         0x40
#define REG_IRQ_PAYLOAD_CRC_ERROR_FLAG              0x20
#define REG_IRQ_VALID_HEADER_FLAG                   0x10

#define REG_IRQ_TX_DONE_FLAG                        0x08
#define REG_IRQ_CADDONE                             0x04
#define REG_IRQ_FHSS_CHANGE_CHAN                    0x02
#define REG_IRQ_CAD_CHAECKED                        0x01




struct pack
{
    uint8_t dst;
    uint8_t type;
    uint8_t src;
    uint8_t packnum;
    uint8_t length;
    uint8_t data[MAX_PAYLOAD];
};

/******************************************************************************
 * Class
 ******************************************************************************/

//! SX1272 Class
/*!
    SX1272 Class defines all the variables and functions used to manage
    SX1272 modules.
 */
class SX1272
{

public:
    ~SX1272();
    SX1272();

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


    /*
     Function: Reads the indicated register.
     Returns: The content of the register
     Parameters:
       address: address register to read from
    */
    byte readRegister(byte address);

    /*
     Function: Writes on the indicated register.
     Returns: Nothing
     Parameters:
       address: address register to write in
       data : value to write in the register
    */
    void writeRegister(byte address, byte data);


    /**
     * Function: writes & read to the indicated register return 0 if equal
     * @param  address  address
     * @param  data     byte to write
     * @return          0 if equal
     */
    int8_t writeReadRegister(byte address, byte data);

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
     * Function: sets up the communications based on a desired LoRA mode
     * @param  mode  desired mode
     * @return       0 if no issue
     */
    int8_t setMode(uint8_t mode);


    /**
     *  Function: Checks if SF is a valid value.
     * @param  spr Spread factor
     * @return     true if aviailbe
     */
    boolean isSF(uint8_t spr);

    /**
     * Function: Gets the SF within the module is configured.
     * @return    0 if no error
     */
    int8_t  getSF();


    /**
     * Function: Sets the indicated SF in the module.
     * @param  spr [description]
     * @return     0 if no error 
     */
    int8_t setSF(uint8_t spr);

    /**
     * Function:   Checks if BW is a valid value.
     * @param  band desired band
     * @return      true if valid
     */
    boolean isBW(uint16_t band);

    /**
     * Function: Reads the current BW and returns 0 if read matches the previously set value
     * @return [description]
     */
    int8_t  getBW();


    /**
     * Function      responsible for writing the desired bandwidth to the resisters
     * @param  band  desired band
     * @return       0 if error 
     */
    int8_t  setBW(uint16_t band);


    /**
     *  Function: Checks if CR is a valid value.
     * @param  cod CR
     * @return     1 if valid
     */
    boolean isCR(uint8_t cod);


    /**
     *  Function: Indicates the CR within the module is configured.
     * @return 0 if no error 
     */
    int8_t  getCR();


    /**
     * Function:    Sets the indicated CR in the module.
     * @param  cod  current CR
     * @return      0 if error occurs 
     */
    int8_t  setCR(uint8_t cod);


    /**
     * Function:  responsible for setting up the frequency channel and writing to the registers
     * @param     ch desired channel 
     * @return    0 if no error occurs
     */
    int8_t setChannel(uint32_t ch);

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
     * Function: sets the maximum current the SX127X device has access 
     * @param  rate current rate 
     * @return      error state 0 if no error 
     */
    int8_t setMaxCurrent(uint8_t rate);


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
    int8_t  setSyncWord(uint8_t sw);

    /**
     * Function:  places the lora module into sleep mode
     * @return    0 if successful 
     */
    int8_t setSleepMode(); 


    pack _packet_sent;
    pack _packet_received;

    uint16_t _payloadlength;
    uint8_t _nodeAddress; 
    uint8_t _packetNumber;
    int8_t _SNR;
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
