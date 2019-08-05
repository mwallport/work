// file controlProtocol.cpp

#include <stdio.h>
#include <arpa/inet.h>
#include "controlProtocol.h"
#include "crc16.h"


//
// send a getStatus message to controllino at Address - future expandability
// TODO: protect against short pBuff by having caller include length ?
//
void Make_getStatus(uint16_t Address, uint8_t* pBuff)
{
    getStatus_t*    msg  = reinterpret_cast<getStatus_t*>(pBuff);
    uint16_t        CRC  = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->control            = COMMAND;
    msg->address.address    = htons(Address);
    msg->seqNum             = htons(seqNum);
    msg->msgNum             = htons(getStatus);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getStatus_t);

    // put the CRC
    msg->crc                = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eot                = '\r';
}


void Make_getStatusResp(uint16_t Address, uint8_t* pBuff, uint16_t humidityAlert,
                    uint16_t TECsRunning, uint16_t chillerOnLine, uint16_t SeqNum)
{
    getStatusResp_t*    msg  = reinterpret_cast<getStatusResp_t*>(pBuff);
    uint16_t            CRC  = 0;


    // create the getStatusResp message in pBuff and CRC16 checksum it
    msg->control                = RESPONSE;
    msg->address.address        = htons(Address);
    msg->seqNum                 = htons(SeqNum);
    msg->msgNum                 = htons(getStatusResp);
    msg->status.humidityAlert   = ntohs(humidityAlert);
    msg->status.TECsRunning     = ntohs(TECsRunning);
    msg->status.chillerOnLine   = ntohs(chillerOnLine);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getStatusResp_t);

    // put the CRC
    msg->crc                = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eot                = '\r';
}


void Parse_getStatusResp(uint8_t* buff, uint16_t* humidityAlert,
        uint16_t* TECsRunning, uint16_t* chillerOnLine, uint16_t* pSeqNum)
{
    getStatusResp_t* pResponse = reinterpret_cast<getStatusResp_t*>(buff);

    //
    // fill in the pReport
    //
    *humidityAlert  = ntohs(pResponse->status.humidityAlert);
    *TECsRunning    = ntohs(pResponse->status.TECsRunning);
    *chillerOnLine  = ntohs(pResponse->status.chillerOnLine);
    *pSeqNum        = ntohs(pResponse->seqNum);
}


void Make_getHumidityThreshold(uint16_t Address, uint8_t* pBuff)
{
    getHumidityThreshold_t*    msg  = reinterpret_cast<getHumidityThreshold_t*>(pBuff);
    uint16_t        CRC  = 0;


    // create the getHumidityThreshold message in pBuff and CRC16 checksum it
    msg->control            = COMMAND;
    msg->address.address    = htons(Address);   // need htons here ?
    msg->seqNum             = htons(seqNum);    // htons ?
    msg->msgNum             = htons(getHumidityThreshold);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getHumidityThreshold_t);

    // put the CRC
    msg->crc                = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eot                = '\r';
}


void Make_getHumidityThresholdResp(uint16_t Address, uint8_t* pBuff, uint16_t threshold, uint16_t SeqNum)
{
    getHumidityThresholdResp_t* msg  = reinterpret_cast<getHumidityThresholdResp_t*>(pBuff);
    uint16_t                    CRC  = 0;


    // create the getHumidityThresholdResp message in pBuff and CRC16 checksum it
    msg->control            = RESPONSE;
    msg->address.address    = htons(Address);   // need htons here ?
    msg->seqNum             = htons(SeqNum);    // htons ?
    msg->msgNum             = htons(getHumidityThresholdResp);
    msg->threshold          = htons(threshold);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getHumidityThresholdResp_t);

    // put the CRC
    msg->crc                = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eot                = '\r';
}


void Parse_getHumidityThresholdResp(uint8_t* buff, uint16_t* threshold, uint16_t* pSeqNum)
{
    getHumidityThresholdResp_t* pResponse = reinterpret_cast<getHumidityThresholdResp_t*>(buff);


    *threshold  = ntohs(pResponse->threshold);
    *pSeqNum    = ntohs(pResponse->seqNum);
}


void Make_setHumidityThreshold(uint16_t Address, uint8_t* pBuff, uint16_t threshold)
{
    setHumidityThreshold_t* msg = reinterpret_cast<setHumidityThreshold_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->control            = COMMAND;
    msg->address.address    = htons(Address);
    msg->seqNum             = htons(seqNum);
    msg->msgNum             = htons(setHumidityThreshold);

    // pass in the humidity threshold to set
    msg->threshold          = htons(threshold);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_setHumidityThreshold_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eot                = '\r';
}


void Make_setHumidityThresholdResp(uint16_t Address, uint8_t* pBuff, uint16_t result, uint16_t SeqNum)
{
    setHumidityThresholdResp_t* msg = reinterpret_cast<setHumidityThresholdResp_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->control            = RESPONSE;
    msg->address.address    = htons(Address);
    msg->seqNum             = htons(SeqNum);
    msg->msgNum             = htons(setHumidityThresholdResp);

    // set the result
    msg->result             = htons(result);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_setHumidityThresholdResp_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eot                = '\r';
}


void Parse_setHumidityThresholdResp(uint8_t* buff, uint16_t* result, uint16_t* pSeqNum)
{
    setHumidityThresholdResp_t* pResponse = reinterpret_cast<setHumidityThresholdResp_t*>(buff);


    *result     = ntohs(pResponse->result);
    *pSeqNum    = ntohs(pResponse->seqNum);
}


void Make_getHumidity(uint16_t Address, uint8_t* pBuff)
{
    getHumidity_t* msg = reinterpret_cast<getHumidity_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->control            = COMMAND;
    msg->address.address    = htons(Address);
    msg->seqNum             = htons(seqNum);
    msg->msgNum             = htons(getHumidity);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getHumidity_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eot                = '\r';
}


void Make_getHumidityResp(uint16_t Address, uint8_t* pBuff, float humidity, uint16_t SeqNum)
{
    getHumidityResp_t* msg = reinterpret_cast<getHumidityResp_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->control            = RESPONSE;
    msg->address.address    = htons(Address);
    msg->seqNum             = htons(SeqNum);
    msg->msgNum             = htons(getHumidityResp);
    snprintf(reinterpret_cast<char*>(msg->humidity), MAX_HUMIDITY_LENGTH, "%-+3.2f", humidity);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getHumidityResp_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eot                = '\r';
}


void Parse_getHumidityResp(uint8_t* buff, float* humidity, uint16_t* pSeqNum)
{
    getHumidityResp_t* pResponse = reinterpret_cast<getHumidityResp_t*>(buff);


    sscanf(reinterpret_cast<char*>(pResponse->humidity), "%6f", humidity);
    *pSeqNum    = ntohs(pResponse->seqNum);
}


void Make_setTECTemperature(uint16_t Address, uint8_t* pBuff, uint16_t tec_address, float temperature)
{
    setTECTemperature_t* msg = reinterpret_cast<setTECTemperature_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->control            = COMMAND;
    msg->address.address    = htons(Address);
    msg->seqNum             = htons(seqNum);
    msg->msgNum             = htons(setTECTemperature);
    msg->tec_address        = htons(tec_address);
    snprintf(reinterpret_cast<char*>(msg->temperature), MAX_TEC_TEMP_LENGH, "%-+3.2f", temperature);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_setTECTemperature_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eot                = '\r';
}


void Make_setTECTemperatureResp(uint16_t Address, uint8_t* pBuff, uint16_t tec_address, uint16_t result, uint16_t SeqNum)
{
    setTECTemperatureResp_t* msg = reinterpret_cast<setTECTemperatureResp_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->control            = RESPONSE;
    msg->address.address    = htons(Address);
    msg->seqNum             = htons(SeqNum);
    msg->msgNum             = htons(setTECTemperatureResp);
    msg->tec_address        = htons(tec_address);
    msg->result             = htons(result);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_setTECTemperatureResp_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eot                = '\r';
}


void Parse_setTECTemperatureResp(uint8_t* buff, uint16_t* result, uint16_t* pSeqNum)
{
    setTECTemperatureResp_t* pResponse = reinterpret_cast<setTECTemperatureResp_t*>(buff);


    *result     = ntohs(pResponse->result);
    *pSeqNum    = ntohs(pResponse->seqNum);
}


void Make_getTECTemperature(uint16_t Address, uint8_t* pBuff, uint16_t tec_address)
{
    getTECTemperature_t* msg = reinterpret_cast<getTECTemperature_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->control            = COMMAND;
    msg->address.address    = htons(Address);
    msg->seqNum             = htons(seqNum);
    msg->msgNum             = htons(getTECTemperature);
    msg->tec_address        = htons(tec_address);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getTECTemperature_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?


    // put the end of transmission byte
    msg->eot                = '\r';
}

void Make_getTECTemperatureResp(uint16_t Address, uint8_t* pBuff, uint16_t tec_address, float temperature, uint16_t SeqNum)
{
    getTECTemperatureResp_t* msg = reinterpret_cast<getTECTemperatureResp_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->control            = RESPONSE;
    msg->address.address    = htons(Address);
    msg->seqNum             = htons(SeqNum);
    msg->msgNum             = htons(getTECTemperatureResp);
    msg->tec_address        = htons(tec_address);
    snprintf(reinterpret_cast<char*>(msg->temperature), MAX_TEC_TEMP_LENGH, "%-+3.2f", temperature);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getTECTemperatureResp_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?


    // put the end of transmission byte
    msg->eot                = '\r';
}

void Parse_getTECTemperatureResp(uint8_t* buff, float* temperature, uint16_t* pSeqNum)
{
    getTECTemperatureResp_t* pResponse = reinterpret_cast<getTECTemperatureResp_t*>(buff);


    sscanf(reinterpret_cast<char*>(pResponse->temperature), "%f", temperature);
    *pSeqNum    = ntohs(pResponse->seqNum);
}


void Make_enableTECs(uint16_t Address, uint8_t* pBuff)
{
    enableTECs_t* msg = reinterpret_cast<enableTECs_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->control            = COMMAND;
    msg->address.address    = htons(Address);
    msg->seqNum             = htons(seqNum);
    msg->msgNum             = htons(enableTECs);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_enableTECs_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eot                = '\r';
}


void Make_enableTECsResp(uint16_t Address, uint8_t* pBuff, uint16_t result, uint16_t SeqNum)
{
    enableTECsResp_t* msg = reinterpret_cast<enableTECsResp_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->control            = RESPONSE;
    msg->address.address    = htons(Address);
    msg->seqNum             = htons(SeqNum);
    msg->msgNum             = htons(enableTECsResp);
    msg->result             = htons(result);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_enableTECsResp_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eot                = '\r';
}


void Parse_enableTECsResp(uint8_t* buff, uint16_t* result, uint16_t* pSeqNum)
{
    enableTECsResp_t* pResponse = reinterpret_cast<enableTECsResp_t*>(buff);


    *result     = ntohs(pResponse->result);
    *pSeqNum    = ntohs(pResponse->seqNum);
}


void Make_disableTECs(uint16_t Address, uint8_t* pBuff)
{
    disableTECs_t* msg = reinterpret_cast<disableTECs_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->control            = COMMAND;
    msg->address.address    = htons(Address);
    msg->seqNum             = htons(seqNum);
    msg->msgNum             = htons(disableTECs);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_disableTECs_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eot                = '\r';
}


void Make_disableTECsResp(uint16_t Address, uint8_t* pBuff, uint16_t result, uint16_t SeqNum)
{
    disableTECsResp_t* msg = reinterpret_cast<disableTECsResp_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->control            = RESPONSE;
    msg->address.address    = htons(Address);
    msg->seqNum             = htons(SeqNum);
    msg->msgNum             = htons(disableTECsResp);
    msg->result             = htons(result);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_disableTECsResp_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eot                = '\r';
}


void Parse_disableTECsResp(uint8_t* buff, uint16_t* result, uint16_t* pSeqNum)
{
    disableTECsResp_t* pResponse = reinterpret_cast<disableTECsResp_t*>(buff);


    *result     = ntohs(pResponse->result);
    *pSeqNum    = ntohs(pResponse->seqNum);
}


void Make_setChillerTemperature(uint16_t Address, uint8_t* pBuff, float temperature)
{
    setChillerTemperature_t* msg = reinterpret_cast<setChillerTemperature_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->control            = COMMAND;
    msg->address.address    = htons(Address);
    msg->seqNum             = htons(seqNum);
    msg->msgNum             = htons(setChillerTemperature);
    snprintf(reinterpret_cast<char*>(msg->temperature), MAX_CHILLER_TEMP_LENGH, "%-+3.2f", temperature);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_setChillerTemperature_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eot                = '\r';
}


void Make_setChillerTemperatureResp(uint16_t Address, uint8_t* pBuff, uint16_t result, uint16_t SeqNum)
{
    setChillerTemperatureResp_t* msg = reinterpret_cast<setChillerTemperatureResp_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->control            = RESPONSE;
    msg->address.address    = htons(Address);
    msg->seqNum             = htons(SeqNum);
    msg->msgNum             = htons(setChillerTemperatureResp);
    msg->result             = htons(result);
    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_setChillerTemperatureResp_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eot                = '\r';
}


void Parse_setChillerTemperatureResp(uint8_t* buff, uint16_t* result, uint16_t* pSeqNum)
{
    setChillerTemperatureResp_t* pResponse = reinterpret_cast<setChillerTemperatureResp_t*>(buff);


    *result     = ntohs(pResponse->result);
    *pSeqNum    = ntohs(pResponse->seqNum);
}


void Make_getChillerTemperature(uint16_t Address, uint8_t* pBuff)
{
    getChillerTemperature_t* msg = reinterpret_cast<getChillerTemperature_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->control            = COMMAND;
    msg->address.address    = htons(Address);
    msg->seqNum             = htons(seqNum);
    msg->msgNum             = htons(getChillerTemperature);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getChillerTemperature_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?


    // put the end of transmission byte
    msg->eot                = '\r';
}

void Make_getChillerTemperatureResp(uint16_t Address, uint8_t* pBuff, float temperature, uint16_t SeqNum)
{
    getChillerTemperatureResp_t* msg = reinterpret_cast<getChillerTemperatureResp_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->control            = RESPONSE;
    msg->address.address    = htons(Address);
    msg->seqNum             = htons(SeqNum);
    msg->msgNum             = htons(getChillerTemperatureResp);
    snprintf(reinterpret_cast<char*>(msg->temperature), MAX_HUMIDITY_LENGTH, "%-+3.2f", temperature);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getChillerTemperatureResp_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?


    // put the end of transmission byte
    msg->eot                = '\r';
}

void Parse_getChillerTemperatureResp(uint8_t* buff, float* temperature, uint16_t* pSeqNum)
{
    getChillerTemperatureResp_t* pResponse = reinterpret_cast<getChillerTemperatureResp_t*>(buff);


    sscanf(reinterpret_cast<char*>(pResponse->temperature), "%f", temperature);
    *pSeqNum    = ntohs(pResponse->seqNum);
}


