// file crc16.h
#ifndef _CONTROL_PROTOCOL_
#define _CONTROL_PROTOCOL_

#include <stdint.h>


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
const   uint8_t     COMMAND                 = '#';  // start packet byte for commands
const   uint8_t     RESPONSE                = '!';  // start packet byte for responses
static  uint16_t    seqNum                  = 0;    // TODO: this should be some random number at start up


typedef enum _msgID
{
    getStatus,                  // fetch the status of chiller, all TECs, and humidity sensor
    getStatusResp,              // get status response
    setHumidityThreshold,       // get the humidity threshold
    setHumidityThresholdResp,   // get the humidity threshold response
    getHumidityThreshold,       // set the humidity threshold
    getHumidityThresholdResp,   // set the humidity threshold response
    getHumidity,                // get current humidity and temperature
    getHumidityResp,            // get current humidity and temperature response
    setTECTemperature,          // target TEC address and temp
    setTECTemperatureResp,      // target TEC address and temp response
    getTECTemperature,          // target TEC address and temp
    getTECTemperatureResp,      // target TEC address and temp response
    setChillerTemperature,      // target TEC address and temp
    setChillerTemperatureResp,  // target TEC address and temp response
    getChillerTemperature,      // target TEC address and temp
    getChillerTemperatureResp,  // target TEC address and temp response
    enableTECs,                 // turn on all TECs
    enableTECsResp,             // turn on all TECs response
    disableTECs,                // turn off all TECs
    disableTECsResp             // turn off all TECs response
} msgID;


typedef struct _Address
{
    uint16_t    address;    // currently 0 for master control PC, 1 for Conrollino
} Address_t;


typedef uint16_t CRC;


typedef struct _getStatus
{
    uint16_t    control;        // '#' or '!' - character
    Address_t   address;        // 0 for master, !0 for slave(s) - uint8_t
    uint16_t    seqNum;         // uint16_t
    uint16_t    msgNum;         // uint16_t - this will be the getStatusMsg message
    CRC         crc;            // 16 bit CRC over the packet
    uint8_t     eot;            // end of transmission character/byte
} getStatus_t;
// TODO: better way to exclude crc legnth?
const unsigned int len_getStatus_t    = sizeof(getStatus_t) - sizeof(CRC);


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
    uint8_t     eot;            // end of transmission character/byte
} getStatusResp_t;
const unsigned int len_getStatusResp_t    = sizeof(getStatusResp_t) - sizeof(CRC);


typedef struct _setHumidityThreshold
{
    uint16_t    control;        // '#' or '!' - character
    Address_t   address;        // 0 for master, !0 for slave(s) - uint8_t
    uint16_t    seqNum;         // uint16_t
    uint16_t    msgNum;         // uint16_t - this will be the setHumidityThreshold message
    uint16_t    threshold;      // uint16_6 - humidity threshold - TODO: make this ASCII char ?
    CRC         crc;            // 16 bit CRC over the packet
    uint8_t     eot;            // end of transmission character/byte
} setHumidityThreshold_t;
const unsigned int len_setHumidityThreshold_t    = sizeof(setHumidityThreshold_t) - sizeof(CRC);


typedef struct _setHumidityThresholdResp
{
    uint16_t    control;        // '#' or '!' - character
    Address_t   address;        // 0 for master, !0 for slave(s) - uint8_t
    uint16_t    seqNum;         // uint16_t
    uint16_t    msgNum;         // uint16_t - this will be the setHumidityThresholdResp message
    uint16_t    result;         // 0 - failed to set; 1 - successfully set
    CRC         crc;            // 16 bit CRC over the packet
    uint8_t     eot;            // end of transmission character/byte
} setHumidityThresholdResp_t;
const unsigned int len_setHumidityThresholdResp_t    = sizeof(setHumidityThresholdResp_t) - sizeof(CRC);


typedef struct _getHumidityThreshold
{
    uint16_t    control;        // '#' or '!' - character
    Address_t   address;        // 0 for master, !0 for slave(s) - uint8_t
    uint16_t    seqNum;         // uint16_t
    uint16_t    msgNum;         // uint16_t - this will be the getHumidityThreshold message
    CRC         crc;            // 16 bit CRC over the packet
    uint8_t     eot;            // end of transmission character/byte
} getHumidityThreshold_t;
const unsigned int len_getHumidityThreshold_t    = sizeof(getHumidityThreshold_t) - sizeof(CRC);


typedef struct _getHumidityThresholdResp
{
    uint16_t    control;            // '#' or '!' - character
    Address_t   address;            // 0 for master, !0 for slave(s) - uint8_t
    uint16_t    seqNum;             // uint16_t
    uint16_t    msgNum;             // uint16_t - this will be the getHumidityThreshold message
    uint16_t    threshold;          // uint16_6 - current humidity threshold - TODO: make this ASCII char?
    CRC         crc;                // 16 bit CRC over the packet
    uint8_t     eot;            // end of transmission character/byte
} getHumidityThresholdResp_t;
const unsigned int len_getHumidityThresholdResp_t    = sizeof(getHumidityThresholdResp_t) - sizeof(CRC);


typedef struct _getHumidity
{
    uint16_t    control;        // '#' or '!' - character
    Address_t   address;        // 0 for master, !0 for slave(s) - uint8_t
    uint16_t    seqNum;         // uint16_t
    uint16_t    msgNum;         // uint16_t - this will be the getHumidityThreshold message
    CRC         crc;            // 16 bit CRC over the packet
    uint8_t     eot;            // end of transmission character/byte
} getHumidity_t;
const unsigned int len_getHumidity_t    = sizeof(getHumidity_t) - sizeof(CRC);


typedef struct _getHumidityResp
{
    uint16_t    control;    // '#' or '!' - character
    Address_t   address;    // 0 for master, !0 for slave(s) - uint8_t
    uint16_t    seqNum;     // uint16_t
    uint16_t    msgNum;     // uint16_t - this will be the getHumidityThreshold message
    uint8_t     humidity[MAX_HUMIDITY_LENGTH];    // float in 32 bits
    CRC         crc;        // 16 bit CRC over the packet
    uint8_t     eot;            // end of transmission character/byte
} getHumidityResp_t;
const unsigned int len_getHumidityResp_t    = sizeof(getHumidityResp_t) - sizeof(CRC);


typedef struct _setTECTemperature
{
    uint16_t    control;        // '#' or '!' - character
    Address_t   address;        // 0 for master, !0 for slave(s) - uint8_t
    uint16_t    seqNum;         // uint16_t
    uint16_t    msgNum;         // uint16_t - this will be the setTECTemperature message
    uint16_t    tec_address;    // uint16_6 - TEC address
    uint8_t     temperature[MAX_TEC_TEMP_LENGH];    // float in 32 bits
    CRC         crc;            // 16 bit CRC over the packet
    uint8_t     eot;            // end of transmission character/byte
} setTECTemperature_t;
const unsigned int len_setTECTemperature_t    = sizeof(setTECTemperature_t) - sizeof(CRC);


typedef struct _setTECTemperatureResp
{
    uint16_t    control;        // '#' or '!' - character
    Address_t   address;        // 0 for master, !0 for slave(s) - uint8_t
    uint16_t    seqNum;         // uint16_t
    uint16_t    msgNum;         // uint16_t - this will be the setTECTemperature message
    uint16_t    tec_address;    // uint16_6 - TEC address
    uint16_t    result;         // 0 - fail ; 1 - success
    CRC         crc;            // 16 bit CRC over the packet
    uint8_t     eot;            // end of transmission character/byte
} setTECTemperatureResp_t;
const unsigned int len_setTECTemperatureResp_t    = sizeof(setTECTemperatureResp_t) - sizeof(CRC);


typedef struct _getTECTemperature
{
    uint16_t    control;        // '#' or '!' - character
    Address_t   address;        // 0 for master, !0 for slave(s) - uint8_t
    uint16_t    seqNum;         // uint16_t
    uint16_t    msgNum;         // uint16_t - this will be the getTECTemperature message
    uint16_t    tec_address;    // uint16_6 - TEC address
    CRC         crc;            // 16 bit CRC over the packet
    uint8_t     eot;            // end of transmission character/byte
} getTECTemperature_t;
const unsigned int len_getTECTemperature_t    = sizeof(getTECTemperature_t) - sizeof(CRC);


typedef struct _getTECTemperatureResp
{
    uint16_t    control;        // '#' or '!' - character
    Address_t   address;        // 0 for master, !0 for slave(s) - uint8_t
    uint16_t    seqNum;         // uint16_t
    uint16_t    msgNum;         // uint16_t - this will be the getTECTemperature message
    uint16_t    tec_address;    // uint16_6 - TEC address
    uint8_t     temperature[MAX_TEC_TEMP_LENGH];    // float in 32 bits
    CRC         crc;            // 16 bit CRC over the packet
    uint8_t     eot;            // end of transmission character/byte
} getTECTemperatureResp_t;
const unsigned int len_getTECTemperatureResp_t    = sizeof(getTECTemperatureResp_t) - sizeof(CRC);


typedef struct _enableTECs
{
    uint16_t    control;        // '#' or '!' - character
    Address_t   address;        // 0 for master, !0 for slave(s) - uint8_t
    uint16_t    seqNum;         // uint16_t
    uint16_t    msgNum;         // uint16_t - this will be the enableTECs message
    CRC         crc;            // 16 bit CRC over the packet
    uint8_t     eot;            // end of transmission character/byte
} enableTECs_t;
const unsigned int len_enableTECs_t    = sizeof(enableTECs_t) - sizeof(CRC);


typedef struct _enableTECsResp
{
    uint16_t    control;        // '#' or '!' - character
    Address_t   address;        // 0 for master, !0 for slave(s) - uint8_t
    uint16_t    seqNum;         // uint16_t
    uint16_t    msgNum;         // uint16_t - this will be the enableTECs message
    uint16_t    result;         // 0 - fail ; 1 - success
    CRC         crc;            // 16 bit CRC over the packet
    uint8_t     eot;            // end of transmission character/byte
} enableTECsResp_t;
const unsigned int len_enableTECsResp_t    = sizeof(enableTECsResp_t) - sizeof(CRC);


typedef struct _disableTECs
{
    uint16_t    control;        // '#' or '!' - character
    Address_t   address;        // 0 for master, !0 for slave(s) - uint8_t
    uint16_t    seqNum;         // uint16_t
    uint16_t    msgNum;         // uint16_t - this will be the enableTECs message
    CRC         crc;            // 16 bit CRC over the packet
    uint8_t     eot;            // end of transmission character/byte
} disableTECs_t;
const unsigned int len_disableTECs_t    = sizeof(disableTECs_t) - sizeof(CRC);


typedef struct _disableTECsResp
{
    uint16_t    control;        // '#' or '!' - character
    Address_t   address;        // 0 for master, !0 for slave(s) - uint8_t
    uint16_t    seqNum;         // uint16_t
    uint16_t    msgNum;         // uint16_t - this will be the enableTECs message
    uint16_t    result;         // 0 - fail ; 1 - success
    CRC         crc;            // 16 bit CRC over the packet } disableTECsResp_t;
    uint8_t     eot;            // end of transmission character/byte
} disableTECsResp_t;
const unsigned int len_disableTECsResp_t    = sizeof(disableTECsResp_t) - sizeof(CRC);

typedef struct _setChillerTemperature
{
    uint16_t    control;        // '#' or '!' - character
    Address_t   address;        // 0 for master, !0 for slave(s) - uint8_t
    uint16_t    seqNum;         // uint16_t
    uint16_t    msgNum;         // uint16_t - this will be the setTECTemperature message
    uint8_t     temperature[MAX_TEC_TEMP_LENGH];    // float in 32 bits
    CRC         crc;            // 16 bit CRC over the packet
    uint8_t     eot;            // end of transmission character/byte
} setChillerTemperature_t;
const unsigned int len_setChillerTemperature_t    = sizeof(setChillerTemperature_t) - sizeof(CRC);


typedef struct _setChillerTemperatureResp
{
    uint16_t    control;        // '#' or '!' - character
    Address_t   address;        // 0 for master, !0 for slave(s) - uint8_t
    uint16_t    seqNum;         // uint16_t
    uint16_t    msgNum;         // uint16_t - this will be the setTECTemperature message
    uint16_t    result;         // 0 - fail ; 1 - success
    CRC         crc;            // 16 bit CRC over the packet
    uint8_t     eot;            // end of transmission character/byte
} setChillerTemperatureResp_t;
const unsigned int len_setChillerTemperatureResp_t = sizeof(setChillerTemperatureResp_t) - sizeof(CRC);


typedef struct _getChillerTemperature
{
    uint16_t    control;        // '#' or '!' - character
    Address_t   address;        // 0 for master, !0 for slave(s) - uint8_t
    uint16_t    seqNum;         // uint16_t
    uint16_t    msgNum;         // uint16_t - this will be the getTECTemperature message
    CRC         crc;            // 16 bit CRC over the packet
    uint8_t     eot;            // end of transmission character/byte
} getChillerTemperature_t;
const unsigned int len_getChillerTemperature_t = sizeof(getChillerTemperature_t) - sizeof(CRC);


typedef struct _getChillerTemperatureResp
{
    uint16_t    control;        // '#' or '!' - character
    Address_t   address;        // 0 for master, !0 for slave(s) - uint8_t
    uint16_t    seqNum;         // uint16_t
    uint16_t    msgNum;         // uint16_t - this will be the getTECTemperature message
    uint8_t     temperature[MAX_TEC_TEMP_LENGH];    // float in 32 bits
    CRC         crc;            // 16 bit CRC over the packet
    uint8_t     eot;            // end of transmission character/byte
} getChillerTemperatureResp_t;
const unsigned int len_getChillerTemperatureResp_t = sizeof(getChillerTemperatureResp_t) - sizeof(CRC);


void Make_getStatus(uint16_t, uint8_t*);
void Make_getStatusResp(uint16_t, uint8_t*, uint16_t, uint16_t, uint16_t, uint16_t);
void Parse_getStatusResp(uint8_t*, uint16_t*, uint16_t*, uint16_t*, uint16_t*);
void Make_setHumidityThreshold(uint16_t, uint8_t*, uint16_t);
void Make_setHumidityThresholdResp(uint16_t, uint8_t*, uint16_t, uint16_t);
void Parse_setHumidityThresholdResp(uint8_t*, uint16_t*, uint16_t*);
void Make_getHumidityThreshold(uint16_t, uint8_t*);
void Make_getHumidityThresholdResp(uint16_t, uint8_t*, uint16_t, uint16_t);
void Parse_getHumidityThresholdResp(uint8_t*, uint16_t*, uint16_t*);
void Make_getHumidity(uint16_t, uint8_t*);
void Make_getHumidityResp(uint16_t, uint8_t*, float, uint16_t);
void Parse_getHumidityResp(uint8_t*, float*, uint16_t*);
void Make_setTECTemperature(uint16_t, uint8_t*, uint16_t, float);
void Make_setTECTemperatureResp(uint16_t, uint8_t*, uint16_t, uint16_t, uint16_t);
void Parse_setTECTemperatureResp(uint8_t*, uint16_t*, uint16_t*);
void Make_getTECTemperature(uint16_t, uint8_t*, uint16_t);
void Make_getTECTemperatureResp(uint16_t, uint8_t*, uint16_t, float, uint16_t);
void Parse_getTECTemperatureResp(uint8_t*, float*, uint16_t*);
void Make_enableTECs(uint16_t, uint8_t*);
void Make_enableTECsResp(uint16_t, uint8_t*, uint16_t, uint16_t);
void Parse_enableTECsResp(uint8_t*, uint16_t*, uint16_t*);
void Make_disableTECs(uint16_t, uint8_t*);
void Make_disableTECsResp(uint16_t, uint8_t*, uint16_t, uint16_t);
void Parse_disableTECsResp(uint8_t*, uint16_t*, uint16_t*);
void Make_setChillerTemperature(uint16_t, uint8_t*, float);
void Make_setChillerTemperatureResp(uint16_t, uint8_t*, uint16_t, uint16_t);
void Parse_setChillerTemperatureResp(uint8_t*, uint16_t*, uint16_t*);
void Make_getChillerTemperature(uint16_t, uint8_t*);
void Make_getChillerTemperatureResp(uint16_t, uint8_t*, float, uint16_t);
void Parse_getChillerTemperatureResp(uint8_t*, float*, uint16_t*);
#endif
