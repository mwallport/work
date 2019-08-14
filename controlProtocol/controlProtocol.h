// file crc16.h
#ifndef _CONTROL_PROTOCOL_
#define _CONTROL_PROTOCOL_

#include <stdint.h>

#define __USING_LINUX_USB__
#define __DEBUG_CTRL_PROTO__
#define __DEBUG_CONTROL_PKT_TX__
#define __DEBUG_CONTROL_PKT_RX__
//#define __RUNNING_ON_CONTROLLINO__  // mutually exclusive with __USING_LINUX_USB__

#ifdef __RUNNING_ON_CONTROLLINO__
#include "./util.h"
#endif

//
// loosely based off the Meerstetter protocol in which each frame has
//  - a start character, '#' for master and '!' for response from slave
//  - a sequence number to ensure the answer gotten is the answer to the
//      current request, i.e. the slave will echo back the sequence number
//  - some payload
//  - a 16 bit CRC field
//  - an end of frame character
//
const   uint8_t     MAX_CHILLER_TEMP_LENGH  = 8;    // i.e "-21.5"  or "+100.1" - sign and a float number
const   uint8_t     MAX_TEC_TEMP_LENGH      = 8;    // i.e "-21.5"  or "+100.1" - sign and a float number
const   uint8_t     MAX_HUMIDITY_LENGTH     = 8;    // "34.37" interpreted as percent
const   uint8_t     MAX_BUFF_LENGTH_CP      = 64;   // size of the work m_buffer
const   uint8_t     COMMAND                 = '#';  // start packet byte for commands
const   uint8_t     RESPONSE                = '!';  // start packet byte for responses
const   uint8_t     MSG_NUM_OFFSET          = 4;
const   uint8_t     EOT_BYTE                = 0x8F;


typedef enum _msgID
{
    getStatusCmd = 0x1000,               // fetch the status of chiller, all TECs, and humidity sensor
    getStatusResp,              // get status response
    setHumidityThreshold,       // get the humidity threshold
    setHumidityThresholdResp,   // get the humidity threshold response
    getHumidityThreshold,       // set the humidity threshold
    getHumidityThresholdResp,   // set the humidity threshold response
    getHumidity,                // get current humidity and temperature
    getHumidityResp,            // get current humidity and temperature response
    setTECTemperature,          // target TEC m_address and temp
    setTECTemperatureResp,      // target TEC m_address and temp response
    commandFill0,
    getTECTemperature,          // target TEC m_address and temp
    getTECTemperatureResp,      // target TEC m_address and temp response
    commndFill1,
    setChillerTemperature,      // target TEC m_address and temp
    setChillerTemperatureResp,  // target TEC m_address and temp response
    getChillerTemperature,      // target TEC m_address and temp
    getChillerTemperatureResp,  // target TEC m_address and temp response
    enableTECs,                 // turn on all TECs
    enableTECsResp,             // turn on all TECs response
    disableTECs,                // turn off all TECs
    disableTECsResp,            // turn off all TECs response
    startUpCmd,                    // start up TODO: make this
    startUpCmdResp,                // reponse  TODO: make this
    shutDownCmd,                   // shutdown TODO: make this
    shutDownCmdResp                // shutdown response    TODO: make this
} msgID;


typedef struct _Address
{
    uint16_t    address;    // currently 0 for master control PC, 1 for Conrollino
} Address_t;


typedef uint16_t CRC;


typedef struct _msgHeader
{
    uint16_t    control;        // '#' or '!' - character
    Address_t   address;        // 0 for master, !0 for slave(s) - uint8_t
    uint16_t    seqNum;         // uint16_t
    uint16_t    msgNum;         // uint16_t - this will be the getStatusMsg message
} msgHeader_t;


typedef struct _getStatus
{
    uint16_t    control;        // '#' or '!' - character
    Address_t   address;        // 0 for master, !0 for slave(s) - uint8_t
    uint16_t    seqNum;         // uint16_t
    uint16_t    msgNum;         // uint16_t - this will be the getStatusMsg message
    CRC         crc;            // 16 bit CRC over the packet
    uint16_t    eot;            // end of transmission character/byte
} getStatus_t;
// TODO: better way to exclude crc legnth?
const unsigned int len_getStatus_t    = sizeof(getStatus_t) - sizeof(CRC) - sizeof(uint16_t);


typedef struct _statusReport
{
    uint16_t    humidityAlert;  // 0 - no ; 1 - yes
    uint16_t    TECsRunning;    // 0 - no ; 1 - yes
    uint16_t    chillerOnLine;  // 0 - no ; 1 - yes
} statusReport_t;

typedef struct _getStatusResp
{
    uint16_t    control;        // '#' or '!' - character
    Address_t   address;        // 0 for master, !0 for slave(s) - uint8_t
    uint16_t    seqNum;         // uint16_t
    uint16_t    msgNum;         // uint16_t - this will be the getStatusMsgResp message
    statusReport_t status;        // the status
    CRC         crc;            // 16 bit CRC over the packet
    uint16_t    eot;            // end of transmission character/byte
} getStatusResp_t;
const unsigned int len_getStatusResp_t    = sizeof(getStatusResp_t) - sizeof(CRC) - sizeof(uint16_t);


typedef struct _setHumidityThreshold
{
    uint16_t    control;        // '#' or '!' - character
    Address_t   address;        // 0 for master, !0 for slave(s) - uint8_t
    uint16_t    seqNum;         // uint16_t
    uint16_t    msgNum;         // uint16_t - this will be the setHumidityThreshold message
    uint16_t    threshold;      // uint16_6 - humidity threshold - TODO: make this ASCII char ?
    CRC         crc;            // 16 bit CRC over the packet
    uint16_t    eot;            // end of transmission character/byte
} setHumidityThreshold_t;
const unsigned int len_setHumidityThreshold_t    = sizeof(setHumidityThreshold_t) - sizeof(CRC) - sizeof(uint16_t);


typedef struct _setHumidityThresholdResp
{
    uint16_t    control;        // '#' or '!' - character
    Address_t   address;        // 0 for master, !0 for slave(s) - uint8_t
    uint16_t    seqNum;         // uint16_t
    uint16_t    msgNum;         // uint16_t - this will be the setHumidityThresholdResp message
    uint16_t    result;         // 0 - failed to set; 1 - successfully set
    CRC         crc;            // 16 bit CRC over the packet
    uint16_t    eot;            // end of transmission character/byte
} setHumidityThresholdResp_t;
const unsigned int len_setHumidityThresholdResp_t    = sizeof(setHumidityThresholdResp_t) - sizeof(CRC) - sizeof(uint16_t);


typedef struct _getHumidityThreshold
{
    uint16_t    control;        // '#' or '!' - character
    Address_t   address;        // 0 for master, !0 for slave(s) - uint8_t
    uint16_t    seqNum;         // uint16_t
    uint16_t    msgNum;         // uint16_t - this will be the getHumidityThreshold message
    CRC         crc;            // 16 bit CRC over the packet
    uint16_t    eot;            // end of transmission character/byte
} getHumidityThreshold_t;
const unsigned int len_getHumidityThreshold_t    = sizeof(getHumidityThreshold_t) - sizeof(CRC) - sizeof(uint16_t);


typedef struct _getHumidityThresholdResp
{
    uint16_t    control;            // '#' or '!' - character
    Address_t   address;            // 0 for master, !0 for slave(s) - uint8_t
    uint16_t    seqNum;             // uint16_t
    uint16_t    msgNum;             // uint16_t - this will be the getHumidityThreshold message
    uint16_t    threshold;          // uint16_6 - current humidity threshold - TODO: make this ASCII char?
    CRC         crc;                // 16 bit CRC over the packet
    uint16_t    eot;            // end of transmission character/byte
} getHumidityThresholdResp_t;
const unsigned int len_getHumidityThresholdResp_t    = sizeof(getHumidityThresholdResp_t) - sizeof(CRC) - sizeof(uint16_t);


typedef struct _getHumidity
{
    uint16_t    control;        // '#' or '!' - character
    Address_t   address;        // 0 for master, !0 for slave(s) - uint8_t
    uint16_t    seqNum;         // uint16_t
    uint16_t    msgNum;         // uint16_t - this will be the getHumidityThreshold message
    CRC         crc;            // 16 bit CRC over the packet
    uint16_t    eot;            // end of transmission character/byte
} getHumidity_t;
const unsigned int len_getHumidity_t    = sizeof(getHumidity_t) - sizeof(CRC) - sizeof(uint16_t);


typedef struct _getHumidityResp
{
    uint16_t    control;    // '#' or '!' - character
    Address_t   address;    // 0 for master, !0 for slave(s) - uint8_t
    uint16_t    seqNum;     // uint16_t
    uint16_t    msgNum;     // uint16_t - this will be the getHumidityThreshold message
    uint8_t     humidity[MAX_HUMIDITY_LENGTH];    // float in 32 bits
    CRC         crc;        // 16 bit CRC over the packet
    uint16_t    eot;            // end of transmission character/byte
} getHumidityResp_t;
const unsigned int len_getHumidityResp_t    = sizeof(getHumidityResp_t) - sizeof(CRC) - sizeof(uint16_t);


typedef struct _setTECTemperature
{
    uint16_t    control;        // '#' or '!' - character
    Address_t   address;        // 0 for master, !0 for slave(s) - uint8_t
    uint16_t    seqNum;         // uint16_t
    uint16_t    msgNum;         // uint16_t - this will be the setTECTemperature message
    uint16_t    tec_address;    // uint16_6 - TEC address
    uint8_t     temperature[MAX_TEC_TEMP_LENGH];    // float in 32 bits
    CRC         crc;            // 16 bit CRC over the packet
    uint16_t    eot;            // end of transmission character/byte
} setTECTemperature_t;
const unsigned int len_setTECTemperature_t    = sizeof(setTECTemperature_t) - sizeof(CRC) - sizeof(uint16_t);


typedef struct _setTECTemperatureResp
{
    uint16_t    control;        // '#' or '!' - character
    Address_t   address;        // 0 for master, !0 for slave(s) - uint8_t
    uint16_t    seqNum;         // uint16_t
    uint16_t    msgNum;         // uint16_t - this will be the setTECTemperature message
    uint16_t    tec_address;    // uint16_6 - TEC address
    uint16_t    result;         // 0 - fail ; 1 - success
    CRC         crc;            // 16 bit CRC over the packet
    uint16_t    eot;            // end of transmission character/byte
} setTECTemperatureResp_t;
const unsigned int len_setTECTemperatureResp_t    = sizeof(setTECTemperatureResp_t) - sizeof(CRC) - sizeof(uint16_t);


typedef struct _getTECTemperature
{
    uint16_t    control;        // '#' or '!' - character
    Address_t   address;        // 0 for master, !0 for slave(s) - uint8_t
    uint16_t    seqNum;         // uint16_t
    uint16_t    msgNum;         // uint16_t - this will be the getTECTemperature message
    uint16_t    tec_address;    // uint16_6 - TEC address
    CRC         crc;            // 16 bit CRC over the packet
    uint16_t    eot;            // end of transmission character/byte
} getTECTemperature_t;
const unsigned int len_getTECTemperature_t    = sizeof(getTECTemperature_t) - sizeof(CRC) - sizeof(uint16_t);


typedef struct _getTECTemperatureResp
{
    uint16_t    control;        // '#' or '!' - character
    Address_t   address;        // 0 for master, !0 for slave(s) - uint8_t
    uint16_t    seqNum;         // uint16_t
    uint16_t    msgNum;         // uint16_t - this will be the getTECTemperature message
    uint16_t    tec_address;    // uint16_6 - TEC address
    uint8_t     temperature[MAX_TEC_TEMP_LENGH];    // float in 32 bits
    CRC         crc;            // 16 bit CRC over the packet
    uint16_t    eot;            // end of transmission character/byte
} getTECTemperatureResp_t;
const unsigned int len_getTECTemperatureResp_t    = sizeof(getTECTemperatureResp_t) - sizeof(CRC) - sizeof(uint16_t);


typedef struct _enableTECs
{
    uint16_t    control;        // '#' or '!' - character
    Address_t   address;        // 0 for master, !0 for slave(s) - uint8_t
    uint16_t    seqNum;         // uint16_t
    uint16_t    msgNum;         // uint16_t - this will be the enableTECs message
    CRC         crc;            // 16 bit CRC over the packet
    uint16_t    eot;            // end of transmission character/byte
} enableTECs_t;
const unsigned int len_enableTECs_t    = sizeof(enableTECs_t) - sizeof(CRC) - sizeof(uint16_t);


typedef struct _enableTECsResp
{
    uint16_t    control;        // '#' or '!' - character
    Address_t   address;        // 0 for master, !0 for slave(s) - uint8_t
    uint16_t    seqNum;         // uint16_t
    uint16_t    msgNum;         // uint16_t - this will be the enableTECs message
    uint16_t    result;         // 0 - fail ; 1 - success
    CRC         crc;            // 16 bit CRC over the packet
    uint16_t    eot;            // end of transmission character/byte
} enableTECsResp_t;
const unsigned int len_enableTECsResp_t    = sizeof(enableTECsResp_t) - sizeof(CRC) - sizeof(uint16_t);


typedef struct _disableTECs
{
    uint16_t    control;        // '#' or '!' - character
    Address_t   address;        // 0 for master, !0 for slave(s) - uint8_t
    uint16_t    seqNum;         // uint16_t
    uint16_t    msgNum;         // uint16_t - this will be the enableTECs message
    CRC         crc;            // 16 bit CRC over the packet
    uint16_t    eot;            // end of transmission character/byte
} disableTECs_t;
const unsigned int len_disableTECs_t    = sizeof(disableTECs_t) - sizeof(CRC) - sizeof(uint16_t);


typedef struct _disableTECsResp
{
    uint16_t    control;        // '#' or '!' - character
    Address_t   address;        // 0 for master, !0 for slave(s) - uint8_t
    uint16_t    seqNum;         // uint16_t
    uint16_t    msgNum;         // uint16_t - this will be the enableTECs message
    uint16_t    result;         // 0 - fail ; 1 - success
    CRC         crc;            // 16 bit CRC over the packet } disableTECsResp_t;
    uint16_t    eot;            // end of transmission character/byte
} disableTECsResp_t;
const unsigned int len_disableTECsResp_t    = sizeof(disableTECsResp_t) - sizeof(CRC) - sizeof(uint16_t);

typedef struct _setChillerTemperature
{
    uint16_t    control;        // '#' or '!' - character
    Address_t   address;        // 0 for master, !0 for slave(s) - uint8_t
    uint16_t    seqNum;         // uint16_t
    uint16_t    msgNum;         // uint16_t - this will be the setTECTemperature message
    uint8_t     temperature[MAX_CHILLER_TEMP_LENGH];    // float in 32 bits
    CRC         crc;            // 16 bit CRC over the packet
    uint16_t    eot;            // end of transmission character/byte
} setChillerTemperature_t;
const unsigned int len_setChillerTemperature_t    = sizeof(setChillerTemperature_t) - sizeof(CRC) - sizeof(uint16_t);


typedef struct _setChillerTemperatureResp
{
    uint16_t    control;        // '#' or '!' - character
    Address_t   address;        // 0 for master, !0 for slave(s) - uint8_t
    uint16_t    seqNum;         // uint16_t
    uint16_t    msgNum;         // uint16_t - this will be the setTECTemperature message
    uint16_t    result;         // 0 - fail ; 1 - success
    CRC         crc;            // 16 bit CRC over the packet
    uint16_t    eot;            // end of transmission character/byte
} setChillerTemperatureResp_t;
const unsigned int len_setChillerTemperatureResp_t = sizeof(setChillerTemperatureResp_t) - sizeof(CRC) - sizeof(uint16_t);


typedef struct _getChillerTemperature
{
    uint16_t    control;        // '#' or '!' - character
    Address_t   address;        // 0 for master, !0 for slave(s) - uint8_t
    uint16_t    seqNum;         // uint16_t
    uint16_t    msgNum;         // uint16_t - this will be the getTECTemperature message
    CRC         crc;            // 16 bit CRC over the packet
    uint16_t    eot;            // end of transmission character/byte
} getChillerTemperature_t;
const unsigned int len_getChillerTemperature_t = sizeof(getChillerTemperature_t) - sizeof(CRC) - sizeof(uint16_t);


typedef struct _getChillerTemperatureResp
{
    uint16_t    control;        // '#' or '!' - character
    Address_t   address;        // 0 for master, !0 for slave(s) - uint8_t
    uint16_t    seqNum;         // uint16_t
    uint16_t    msgNum;         // uint16_t - this will be the getTECTemperature message
    uint8_t     temperature[MAX_CHILLER_TEMP_LENGH];    // float in 32 bits
    CRC         crc;            // 16 bit CRC over the packet
    uint16_t    eot;            // end of transmission character/byte
} getChillerTemperatureResp_t;
const unsigned int len_getChillerTemperatureResp_t = sizeof(getChillerTemperatureResp_t) - sizeof(CRC) - sizeof(uint16_t);


typedef struct _startUpCmd
{
    uint16_t    control;        // '#' or '!' - character
    Address_t   address;        // 0 for master, !0 for slave(s) - uint8_t
    uint16_t    seqNum;         // uint16_t
    uint16_t    msgNum;         // uint16_t - this will be the getTECTemperature message
    CRC         crc;            // 16 bit CRC over the packet
    uint16_t    eot;            // end of transmission character/byte
} startUpCmd_t;
const unsigned int len_startUpCmd_t = sizeof(startUpCmd_t) - sizeof(CRC) - sizeof(uint16_t);


typedef struct _startUpCmdResp
{
    uint16_t    control;        // '#' or '!' - character
    Address_t   address;        // 0 for master, !0 for slave(s) - uint8_t
    uint16_t    seqNum;         // uint16_t
    uint16_t    msgNum;         // uint16_t - this will be the getTECTemperature message
    uint16_t    result;         // 0 - failed to set; 1 - successfully set
    CRC         crc;            // 16 bit CRC over the packet
    uint16_t    eot;            // end of transmission character/byte
} startUpCmdResp_t;
const unsigned int len_startUpCmdResp_t = sizeof(startUpCmdResp_t) - sizeof(CRC) - sizeof(uint16_t);


typedef struct _shutDownCmd
{
    uint16_t    control;        // '#' or '!' - character
    Address_t   address;        // 0 for master, !0 for slave(s) - uint8_t
    uint16_t    seqNum;         // uint16_t
    uint16_t    msgNum;         // uint16_t - this will be the getTECTemperature message
    CRC         crc;            // 16 bit CRC over the packet
    uint16_t    eot;            // end of transmission character/byte
} shutDownCmd_t;
const unsigned int len_shutDownCmd_t = sizeof(shutDownCmd_t) - sizeof(CRC) - sizeof(uint16_t);


typedef struct _shutDownCmdResp
{
    uint16_t    control;        // '#' or '!' - character
    Address_t   address;        // 0 for master, !0 for slave(s) - uint8_t
    uint16_t    seqNum;         // uint16_t
    uint16_t    msgNum;         // uint16_t - this will be the getTECTemperature message
    uint16_t    result;         // 0 - failed to set; 1 - successfully set
    CRC         crc;            // 16 bit CRC over the packet
    uint16_t    eot;            // end of transmission character/byte
} shutDownCmdResp_t;
const unsigned int len_shutDownCmdResp_t = sizeof(shutDownCmdResp_t) - sizeof(CRC) - sizeof(uint16_t);



class controlProtocol
{
    public:
    //
    // define function pointers to handle USB and Serial interfaces
    // called like (this.*TxCommand)()
    //
    bool    (controlProtocol::*TxCommand)(uint16_t);
    bool    (controlProtocol::*RxResponse)(uint16_t);
    bool    (controlProtocol::*RxCommand)(uint16_t);
    bool    (controlProtocol::*TxResponse)(uint16_t);

    //
    // functions to hide the member function pointer syntax
    //
    bool    doRxCommand(uint16_t TimeoutMs) { return( (this->*RxCommand)(TimeoutMs) ); };
    bool    doTxResponse(uint16_t length) { return( (this->*TxResponse)(length) ); };
    bool    doTxCommand(uint16_t length) { return( (this->*TxCommand)(length) ); };
    bool    doRxResponse(uint16_t timeout) { return( (this->*RxResponse)(timeout) ); };

    controlProtocol(uint16_t, uint16_t);                // serial : m_myAddress, m_peerAddress
    controlProtocol(uint16_t, uint16_t, const char*);   // USB : m_myAddress, m_peerAddress, USB file
    ~controlProtocol();
    
    bool    StartUpCmd(uint16_t);
    bool    ShutDownCmd(uint16_t);
    bool    GetStatus(uint16_t, uint16_t*, uint16_t*, uint16_t*);
    bool    GetHumidity(uint16_t, float*);
    bool    SetHumidityThreshold(uint16_t, uint16_t);
    bool    GetHumidityThreshold(uint16_t, uint16_t*);
    bool    SetTECTemperature(uint16_t, uint16_t, float);
    bool    GetTECTemperature(uint16_t, uint16_t, float*);
    bool    SetChillerTemperature(uint16_t, float);
    bool    GetChillerTemperature(uint16_t, float*);
    bool    EnableTECs(uint16_t);
    bool    DisableTECs(uint16_t);

    // master - control/test PC USB or serial interface
    bool        TxCommandUSB(uint16_t);    // uses m_buff and m_seqNum
    bool        RxResponseUSB(uint16_t);   // uses m_buff and m_seqNum, msec to wait for response
    // implement these later if needed
    //bool        TxCommandSerial();            // uses m_buff and m_seqNum
    //bool        RxResponseSerial(uint16_t);   // uses m_buff and m_seqNum, msec to wait for response

    // slave - Controllino uC 
    bool        RxCommandSerial(uint16_t);            // uses m_buff and m_seqNum
    bool        TxResponseSerial(uint16_t);           // uses m_buff and m_seqNum
    // implement these later if needed
    //bool        RxCommandUSB();            // uses m_buff and m_seqNum
    //bool        TxResponseUSB();           // uses m_buff and m_seqNum

    // protected:  TODO: protect something !
    uint16_t    m_seqNum;                       // current m_seqNum
    uint16_t    m_myAddress;                    // 'my' Address
    uint16_t    m_peerAddress;                  // peer address, 1 to 1 communication
    uint8_t     m_buff[MAX_BUFF_LENGTH_CP + 1]; // work m_buffer used by all functions
    int         m_fd;                           // for the USB port

    bool        openUSBPort(const char*);
    bool        verifyMessage(uint16_t, uint16_t, uint16_t);
    bool        verifyMessageSeqNum(uint16_t, uint16_t);
    bool        verifyMessageCRC(uint16_t, uint16_t);
    uint16_t    getMsgId();

    uint16_t    Make_startUpCmd(uint16_t, uint8_t*);
    uint16_t    Make_startUpCmdResp(uint16_t, uint8_t*, uint16_t, uint16_t);
    void        Parse_startUpCmdResp(uint8_t*, uint16_t*, uint16_t*);

    uint16_t    Make_shutDownCmd(uint16_t, uint8_t*);
    uint16_t    Make_shutDownCmdResp(uint16_t, uint8_t*, uint16_t, uint16_t);
    void        Parse_shutDownCmdResp(uint8_t*, uint16_t*, uint16_t*);

    uint16_t    Make_getStatus(uint16_t, uint8_t*);
    uint16_t    Make_getStatusResp(uint16_t, uint8_t*, uint16_t, uint16_t, uint16_t, uint16_t);
    void        Parse_getStatusResp(uint8_t*, uint16_t*, uint16_t*, uint16_t*, uint16_t*);

    uint16_t    Make_setHumidityThreshold(uint16_t, uint8_t*, uint16_t);
    uint16_t    Make_setHumidityThresholdResp(uint16_t, uint8_t*, uint16_t, uint16_t);
    void        Parse_setHumidityThresholdResp(uint8_t*, uint16_t*, uint16_t*);

    uint16_t    Make_getHumidityThreshold(uint16_t, uint8_t*);
    uint16_t    Make_getHumidityThresholdResp(uint16_t, uint8_t*, uint16_t, uint16_t);
    void        Parse_getHumidityThresholdResp(uint8_t*, uint16_t*, uint16_t*);

    uint16_t    Make_getHumidity(uint16_t, uint8_t*);
    uint16_t    Make_getHumidityResp(uint16_t, uint8_t*, float, uint16_t);
    void        Parse_getHumidityResp(uint8_t*, float*, uint16_t*);

    uint16_t    Make_setTECTemperature(uint16_t, uint8_t*, uint16_t, float);
    uint16_t    Make_setTECTemperatureResp(uint16_t, uint8_t*, uint16_t, uint16_t, uint16_t);
    void        Parse_setTECTemperatureResp(uint8_t*, uint16_t*, uint16_t*);

    uint16_t    Make_getTECTemperature(uint16_t, uint8_t*, uint16_t);
    uint16_t    Make_getTECTemperatureResp(uint16_t, uint8_t*, uint16_t, float, uint16_t);
    void        Parse_getTECTemperatureResp(uint8_t*, float*, uint16_t*);

    uint16_t    Make_enableTECs(uint16_t, uint8_t*);
    uint16_t    Make_enableTECsResp(uint16_t, uint8_t*, uint16_t, uint16_t);
    void        Parse_enableTECsResp(uint8_t*, uint16_t*, uint16_t*);

    uint16_t    Make_disableTECs(uint16_t, uint8_t*);
    uint16_t    Make_disableTECsResp(uint16_t, uint8_t*, uint16_t, uint16_t);
    void        Parse_disableTECsResp(uint8_t*, uint16_t*, uint16_t*);

    uint16_t    Make_setChillerTemperature(uint16_t, uint8_t*, float);
    uint16_t    Make_setChillerTemperatureResp(uint16_t, uint8_t*, uint16_t, uint16_t);
    void        Parse_setChillerTemperatureResp(uint8_t*, uint16_t*, uint16_t*);

    uint16_t    Make_getChillerTemperature(uint16_t, uint8_t*);
    uint16_t    Make_getChillerTemperatureResp(uint16_t, uint8_t*, float, uint16_t);
    void        Parse_getChillerTemperatureResp(uint8_t*, float*, uint16_t*);
};
#endif
