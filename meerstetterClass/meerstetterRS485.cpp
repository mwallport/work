 /*#if defined(ARDUINO) && ARDUINO >= 100
      #include "Arduino.h"
    #else
      #include "WProgram.h"
    #endif
*/
#include <SPI.h>
#include <Controllino.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <stdbool.h>
#include <stdlib.h>
#include <time.h>
#include "meerstetterRS485.h"


const uint16_t meerstetterRS485::CRC16_table_C[256] = {
    // CRC-CCIT calculated for every byte between 0x0000 and 0x00FF
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
    0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
    0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
    0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
    0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
    0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
    0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};

uint16_t meerstetterRS485::LastCRC;
uint16_t meerstetterRS485::SequenceNr = 5545; //Initialized to random value
const int8_t meerstetterRS485::cHex[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
int8_t meerstetterRS485::RcvBuf[MEPORT_MAX_RX_BUF_SIZE + 20];
int32_t meerstetterRS485::RcvCtr = -1;
char meerstetterRS485::Buffer[MEPORT_MAX_TX_BUF_SIZE];
int meerstetterRS485::Ctr;
//static void TestAllCommonGetFunctions(uint8_t Address);
//static void TestAllTECGetFunctions(uint8_t Address);


// constructor
meerstetterRS485::meerstetterRS485(uint32_t _SSerialRX, uint32_t _SSerialTX)
// : RS485Serial(_SSerialRX, _SSerialTX)
{
    memset(reinterpret_cast<void*>(&stats), '\0', sizeof(stats));
    MeInt_QueryRcvPayload = MeFrame_RcvFrame.Payload;
    //RS485Serial.begin(9600);

    Controllino_RS485Init();
    Serial3.begin(9600);
    Controllino_RS485RxEnable();
}


// destructor
meerstetterRS485::~meerstetterRS485() {};


/*-----( Define Functions )-----*/
void meerstetterRS485::ComPort_Send(char *in)
{
    size_t size;
    int len = strlen(in);


    //size = RS485Serial.write(in);
    //RS485Serial.flush();
    Controllino_RS485TxEnable();
    size = Serial3.write(in);
    Serial3.flush();
    Controllino_RS485RxEnable();

    if(size != len)
    {
        Serial.print(__PRETTY_FUNCTION__);
        Serial.println(" ERROR got unexp bytes sent, got : ");
        Serial.println(size, DEC);
        stats.pktTxBadLength += 1;
        return; //TODO : return data Write Error
    #ifdef __DEBUG_MS_PKT_TX__
    } else
    {
        Serial.print(__PRETTY_FUNCTION__);
        Serial.print(" sent ");
        Serial.print(size, DEC);
        Serial.println(" bytes");
        Serial.flush();
        for(int x = 0; x < size; x++)
            Serial.print((char)in[x]);
        Serial.println("");
        Serial.flush();
    #endif
    }
}


bool meerstetterRS485::recvData(uint32_t TimeoutMs)
{
    uint8_t Buffer[100];
    bool retVal             = false;
    bool done               = false;
    bool gotSTX             = false;
    bool gotETX             = false;
    bool timedOut           = false;
    int32_t bytes_read      = 0;
    const uint8_t STX       = '!';
    const uint8_t ETX       = '\r';
    unsigned long startTime = millis();

        
    // try to read a packet for a total of TimeoutMs milliseconds
    while( (!done) && (!timedOut) && (bytes_read < 100) )
    {
        // doing the looop this way to enable the addition of stats later
        // i.e. did the application get a timeout on a packet read
        if( ((millis() - startTime) > TimeoutMs) )
        {
            // TODO: testing - never come in here .. hence the false above
            timedOut = true;
            stats.pktRxTimeout += 1;
        } else
        {
            Controllino_RS485RxEnable();
            //if( (RS485Serial.available()) )  //TODO #define the Buffer length of 100
            if( (Serial3.available()) )  //TODO #define the Buffer length of 100
            {
                //Buffer[bytes_read] = RS485Serial.read();
                Buffer[bytes_read] = Serial3.read();

                if( (!gotSTX) )
                {
                    if( (STX == Buffer[bytes_read]) )
                    {
                        // TODO: restart startTime here, give more time to get the packet?
                        gotSTX = true;
                        bytes_read += 1;
                    } // else don't increment bytes_read effectively discarding this byte
                } else
                {
                    if( (!gotETX) )
                    {
                        if( (ETX == Buffer[bytes_read]) )
                        {
                            done        = true;
                            retVal      = true;
                            stats.pktRx += 1;
                        } 

                        // this is a byte in the body of the frame
                        bytes_read += 1;
                    }
                }
            } else
            {
                // TODO: too long, too short ?
                // no data available, wait a bit before checking again
                delay(10);
            }
        }
    }

    // always null terminate just in case we want to dump out for debug
    Buffer[bytes_read] = 0;

    // debug stuff
    #ifdef __DEBUG_MS_PKT_RX__
    Serial.print(__PRETTY_FUNCTION__);
    Serial.print(" rx'ed ");
    Serial.print(bytes_read, DEC);
    Serial.println(" bytes");
    Serial.flush();
    for(int x = 0; x < bytes_read; x++)
        Serial.print((char)Buffer[x]);
    Serial.println("");
    Serial.flush();
    #endif

    if( (done) )
    {
        MePort_ReceiveByte((int8_t*)Buffer);
    }


    return(retVal);
}


/*==============================================================================*/
/** @brief      Reset Device
 *
*/
uint8_t meerstetterRS485::MeCom_ResetDevice(uint8_t Address)
{
    return MeInt_Set('#', Address, 2, (int8_t*)"RS");
}


/*==============================================================================*/
/** @brief      Return IF String
 *
*/
uint8_t meerstetterRS485::MeCom_GetIdentString(uint8_t Address, int8_t *arr)
{
    uint8_t Succeeded = MeInt_Query('#', Address, 3, (int8_t*)"?IF");
    if(Succeeded == 0) 
    {
        *arr = 0;
        return Succeeded;
    }

    for(int32_t i=0; i<20; i++)
    {
        *arr = MeInt_QueryRcvPayload[i];
        arr++;
    }
    *arr = 0;
    return Succeeded;
}


/*==============================================================================*/
/** @brief      Parameter Set, Get and Limit Get Function for INT32 parameters
 *
 *  Please refer to the "Communication Protocol LDD/TEC Controller" to see
 *  if the INT32 or FLOAT32 function must be used.
 *
 *  This Function does 3 things:
 *  - MeGet:        Queries the actual parameter value
 *  - MeSet:        Sets the given parameter value to the new value
 *  - MeGetLimtis:  Queries the corresponding limits of the parameter
 *
*/
uint8_t meerstetterRS485::MeCom_ParValuel(uint8_t Address, uint16_t ParId, uint8_t Inst, MeParLongFields  *Fields, MeParCmd Cmd)
{
    if(Cmd == MeGet)
    {
        int8_t TxData[20];
        TxData[0] = '?'; TxData[1] = 'V'; TxData[2] = 'R'; 
        MeVarConv_AddUsHex(&TxData[3], ParId);
        MeVarConv_AddUcHex(&TxData[7], Inst);

        uint8_t Succeeded = MeInt_Query('#', Address, 9, TxData);
        if(Succeeded == 0) 
        {
            Fields->Value = 0;
            return Succeeded;
        }

        Fields->Value = MeVarConv_HexToSl(&MeInt_QueryRcvPayload[0]);

        return Succeeded;
    }
    else if(Cmd == MeSet)
    {
        int8_t TxData[20];
        TxData[0] = 'V'; TxData[1] = 'S';
        MeVarConv_AddUsHex(&TxData[2], ParId);
        MeVarConv_AddUcHex(&TxData[6], Inst);
        MeVarConv_AddSlHex(&TxData[8], Fields->Value);

        return MeInt_Set('#', Address, 16, TxData);
    }
    else if(Cmd == MeGetLimits)
    {
        int8_t TxData[20];
        TxData[0] = '?'; TxData[1] = 'V'; TxData[2] = 'L'; 
        MeVarConv_AddUsHex(&TxData[3], ParId);
        MeVarConv_AddUcHex(&TxData[7], Inst);

        uint8_t Succeeded = MeInt_Query('#', Address, 9, TxData);
        if(Succeeded == 0) 
        {
            Fields->Min = 0;
            Fields->Max = 0;
            return Succeeded;
        }

        Fields->Min = MeVarConv_HexToSl(&MeInt_QueryRcvPayload[2]);
        Fields->Max = MeVarConv_HexToSl(&MeInt_QueryRcvPayload[10]);

        return Succeeded;
    }
    return 0;
}


/*==============================================================================*/
/** @brief      Parameter Set, Get and Limit Get Function for FLOAT32 parameters
 *
 *  Please refer to the "Communication Protocol LDD/TEC Controller" to see
 *  if the INT32 or FLOAT32 function must be used.
 *
 *  This function just calls the INT32 Function.
 *
*/
uint8_t meerstetterRS485::MeCom_ParValuef(uint8_t Address, uint16_t ParId, uint8_t Inst, MeParFloatFields *Fields, MeParCmd Cmd)
{
    return MeCom_ParValuel(Address, ParId, Inst, (MeParLongFields *)Fields, Cmd);
}
/*==============================================================================*/
/** @file       MeCRC16.c
    @brief      CRC Calculation Type CRC-CCITT (CRC-16)
    @author     Meerstetter Engineering GmbH: Marc Luethi

    This File provides 2 possibilities to calculate the CRC-Checksum:
    - Function with Table: Is faster, but uses more memory
    - Function without Table: Is slower, but uses much less memory
    Basically the Table function is enabled.

*/


/*==============================================================================*/
/** @brief      This function calculates a CRC-CCITT (CRC-16) together with the Table

*/
uint16_t meerstetterRS485::MeCRC16(uint16_t n, uint8_t m)
{
    n = ( (n%256) << 8 ) ^ ( CRC16_table_C[ (n >> 8) ^ m ] );
    return (n);
}


/*==============================================================================*/
/** @brief      This function calculates a CRC-CCITT (CRC-16) without Table
        
    Calculates CRC code of character
    --------------------------------
    'n'         CRC code. A string can be divided into several parts.
                When the CRC of the first part is calculated, 0 has to
                be input as 'n'. When the further parts are calculated,
                the result of the former part has to be input as 'n'.
    return: 'n' CRC code.
*/
/*
uint16_t MeCRC16(uint16_t n, uint8_t m)
{
    uint32_t genPoly = 0x1021; //CCITT CRC-16 Polynomial
    uint32_t uiCharShifted = ((uint32_t)m & 0x00FF) << 8;
    n = n ^ uiCharShifted;
    for (int32_t i = 0; i < 8; i++)
        if ( n & 0x8000 ) n = (n << 1) ^ genPoly;
        else n = n << 1;
    return n;
}
*/


/*==============================================================================*/
/** @file       MeFrame.c
    @brief      This file holds the low level protocol functions
    @author     Meerstetter Engineering GmbH: Marc Luethi

    Frame send and receiving Functions.
*/

/*==============================================================================*/
/** @brief      Frame Send Function
 *
 *  This function packs the Payload Data into the Frame and calculates the CRC.
 *  The Data is directly passed to the Port Send Byte Function.
 *  The Port Send Function receives the start and End of the Frame information.
 *
*/
void meerstetterRS485::MeFrame_Send(int8_t Control, uint8_t Address, uint32_t Length, uint16_t SeqNr, int8_t *Payload)
{
    uint16_t CRC = 0;
    int8_t txc;

    //Control (Source) Byte
    txc = Control;                              MePort_SendByte(txc, MePort_SB_IsFirstByte); CRC = MeCRC16(CRC, txc);

    //Device Address
    txc = MeVarConv_UcToHEX(Address / 16);      MePort_SendByte(txc, MePort_SB_Normal); CRC = MeCRC16(CRC, txc);
    txc = MeVarConv_UcToHEX(Address % 16);      MePort_SendByte(txc, MePort_SB_Normal); CRC = MeCRC16(CRC, txc);

    //Sequence Number
    txc = MeVarConv_UcToHEX(SeqNr>>12);        MePort_SendByte(txc, MePort_SB_Normal); CRC = MeCRC16(CRC, txc);
    txc = MeVarConv_UcToHEX((SeqNr/256)%16);    MePort_SendByte(txc, MePort_SB_Normal); CRC = MeCRC16(CRC, txc);
    txc = MeVarConv_UcToHEX((SeqNr%256)/16);    MePort_SendByte(txc, MePort_SB_Normal); CRC = MeCRC16(CRC, txc);
    txc = MeVarConv_UcToHEX((SeqNr%256)%16);    MePort_SendByte(txc, MePort_SB_Normal); CRC = MeCRC16(CRC, txc);

    for(uint32_t i = 0; i < Length; i++)
    {
        MePort_SendByte(*Payload, MePort_SB_Normal);
        CRC = MeCRC16(CRC, *Payload);
        Payload++;
    }

    txc = MeVarConv_UcToHEX(CRC>>12);          MePort_SendByte(txc, MePort_SB_Normal); 
    txc = MeVarConv_UcToHEX((CRC/256)%16);      MePort_SendByte(txc, MePort_SB_Normal); 
    txc = MeVarConv_UcToHEX((CRC%256)/16);      MePort_SendByte(txc, MePort_SB_Normal); 
    txc = MeVarConv_UcToHEX((CRC%256)%16);      MePort_SendByte(txc, MePort_SB_Normal); 

    MePort_SendByte(0x0D, MePort_SB_IsLastByte);

    meerstetterRS485::LastCRC = CRC;
}


/*==============================================================================*/
/** @brief      Frame Receive Function
 *
 *  This Function is being called by the receiving function of the target system.
 *  It puts the received bytes back into the frame structure and
 *  checks the CRC. If a complete frame has been received,
 *  the corresponding received Flag is set and
 *  the MePort_SemaphorGive function is being called to unlock
 *  the upper level functions.
 *
*/
void meerstetterRS485::MeFrame_Receive(int8_t in)
{
    if(in == '!')
    {
        //Start Indicator --> Reset Receiving Machine
        memset(meerstetterRS485::RcvBuf, 0, sizeof(meerstetterRS485::RcvBuf));
        meerstetterRS485::RcvBuf[0] = in;
        meerstetterRS485::RcvCtr = 1;
        
    }
    else if(in == 0x0D && (meerstetterRS485::RcvCtr >=11))
    {
        //End of a Frame received
        if(meerstetterRS485::RcvCtr == 11)
        {
            //Check CRC of received Frame
            uint16_t RcvCRC = MeVarConv_HexToUs(&meerstetterRS485::RcvBuf[7]);
            if(RcvCRC == meerstetterRS485::LastCRC && MeFrame_RcvFrame.AckReceived == 0)
            {
                MeFrame_RcvFrame.Address    = MeVarConv_HexToUc(&meerstetterRS485::RcvBuf[1]);
                MeFrame_RcvFrame.SeqNr      = MeVarConv_HexToUs(&meerstetterRS485::RcvBuf[3]);
                MeFrame_RcvFrame.AckReceived = 1;
            }
            else meerstetterRS485::RcvCtr = -1; //Error
        }
        else
        {
            //Check CRC of received Frame
            uint16_t RcvCRC, CalcCRC = 0;
            for(int32_t i=0; i < (meerstetterRS485::RcvCtr-4); i++) CalcCRC = MeCRC16(CalcCRC, meerstetterRS485::RcvBuf[i]); //Calculate CRC of received Frame
            RcvCRC = MeVarConv_HexToUs(&meerstetterRS485::RcvBuf[meerstetterRS485::RcvCtr-4]); //Get Frame CRC
            if(RcvCRC == CalcCRC && MeFrame_RcvFrame.DataReceived == 0)
            {
                //CRC is correct and all data has been processed
                MeFrame_RcvFrame.Address    = MeVarConv_HexToUc(&meerstetterRS485::RcvBuf[1]);
                MeFrame_RcvFrame.SeqNr      = MeVarConv_HexToUs(&meerstetterRS485::RcvBuf[3]);
                for(int32_t i = 7; i < (meerstetterRS485::RcvCtr-4);  i++)    MeFrame_RcvFrame.Payload[i-7] = meerstetterRS485::RcvBuf[i];
                MeFrame_RcvFrame.DataReceived = 1;
            }
        }
    }
    else if(meerstetterRS485::RcvCtr >= 0 && meerstetterRS485::RcvCtr < (MEPORT_MAX_RX_BUF_SIZE+15))
    {
        //Write Data to Buffer
        meerstetterRS485::RcvBuf[RcvCtr] = in;
        meerstetterRS485::RcvCtr++;
    }
    else
    {
        //Error 
        meerstetterRS485::RcvCtr = -1;
    }
}


/*==============================================================================*/
/** @file       MeInt.c
    @brief      This file holds the connection oriented  protocol functions
    @author     Meerstetter Engineering GmbH: Marc Luethi

    These Functions do send the given Data down to the Frame level and
    call a wait timeout Function (MePort_SemaphorTake).
    If the timeout has expired without receiving an answer, an error is generated.
    if an answer is received by the lower level functions, the timeout
    function is being unblocked immediately (SemaphoreGive) and the
    received Frame is being checked.

    These functions to try 3 Times. Till a timeout is being generated.

*/

/*==============================================================================*/
/** @brief      Connection Function for Query Commands
 *
*/
uint8_t meerstetterRS485::MeInt_Query(int8_t Control, uint8_t Address, uint32_t Length, int8_t *Payload)
{
    meerstetterRS485::SequenceNr++;

    int32_t Trials = 3;
    while(Trials > 0)
    {
        Trials--;
        MeFrame_RcvFrame.DataReceived = 0;
        MeFrame_RcvFrame.AckReceived = 0;
        MeFrame_Send(Control, Address, Length, meerstetterRS485::SequenceNr, Payload);
        MePort_SemaphorTake(MEPORT_SET_AND_QUERY_TIMEOUT);
        if(MeFrame_RcvFrame.DataReceived == 1 && MeFrame_RcvFrame.Address == Address && MeFrame_RcvFrame.SeqNr == meerstetterRS485::SequenceNr )
        {
            //Correct Data Received -->Check for Error Code
            if(MeFrame_RcvFrame.Payload[0] == '+')
            {
                //Server Error code Received
                MePort_ErrorThrow(MeVarConv_HexToUc(&MeFrame_RcvFrame.Payload[1]));
                return 0;
            }
            return 1;
        }
    }  
    MePort_ErrorThrow(MEPORT_ERROR_QUERY_TIMEOUT);
    return 0;
}


/*==============================================================================*/
/** @brief      Connection Function for Set Commands
 *
*/
uint8_t meerstetterRS485::MeInt_Set(int8_t Control, uint8_t Address, uint32_t Length, int8_t *Payload)
{
    meerstetterRS485::SequenceNr++;

    int32_t Trials = 3;
    while(Trials > 0)
    {
        Trials--;
        MeFrame_RcvFrame.DataReceived = 0;
        MeFrame_RcvFrame.AckReceived = 0;
        MeFrame_Send(Control, Address, Length, meerstetterRS485::SequenceNr, Payload);
        MePort_SemaphorTake(MEPORT_SET_AND_QUERY_TIMEOUT);
        if(MeFrame_RcvFrame.DataReceived == 1 && MeFrame_RcvFrame.Address == Address && MeFrame_RcvFrame.SeqNr == meerstetterRS485::SequenceNr &&
            MeFrame_RcvFrame.Payload[0] == '+')
        {
            //Server Error code Received
            MePort_ErrorThrow(MeVarConv_HexToUc(&MeFrame_RcvFrame.Payload[1]));
            return 0;
        }
        else if(MeFrame_RcvFrame.AckReceived == 1 && MeFrame_RcvFrame.Address == Address && MeFrame_RcvFrame.SeqNr == meerstetterRS485::SequenceNr )
        {
            //Correct ADC received
            return 1;
        }
    }  
    MePort_ErrorThrow(MEPORT_ERROR_SET_TIMEOUT);
    return 0;
}


/*==============================================================================*/
/** @file       MePort_Linux.c
    @brief      This file holds all interface functions to the MeComAPI
    @author     Meerstetter Engineering GmbH: Thomas Braun

    Please do only modify these functions to implement the MeComAPI into
    your system. It should not be necessary to modify any files in the
    private folder.


*/
/*==============================================================================*/
/** @brief      Interface Function: Send Byte
 *
 *  For the example target system, this function collects all the given bytes
 *  and generates a string. If the frame send function sends the last byte
 *  to this function, the string is being given to the Comport function.
 *
 *  For example in case of a microcontroller, it is also possible to
 *  pass every single byte direct to the Comport function.
 *
 *  In case of an RS485 Interface it can be helpful to use the
 *  "MePort_SB_IsFirstByte" case to enable the RS485 TX Signal
 *  and "MePort_SB_IsLastByte" to disable the TX Signal
 *  after the last byte has been sent.
 *
*/
void meerstetterRS485::MePort_SendByte(int8_t in, MePort_SB FirstLast)
{
    switch(FirstLast)
    {
        case MePort_SB_IsFirstByte:
            //This is the first Byte of the Message String 
            meerstetterRS485::Ctr = 0;
            meerstetterRS485::Buffer[Ctr] = in;
            meerstetterRS485::Ctr++;
        break;
        case MePort_SB_Normal:
            //These are some middle Bytes
            if(meerstetterRS485::Ctr < MEPORT_MAX_TX_BUF_SIZE-1)
            {
                meerstetterRS485::Buffer[Ctr] = in;
                meerstetterRS485::Ctr++;
            }
        break;
        case MePort_SB_IsLastByte:
            //This is the last Byte of the Message String
            if(meerstetterRS485::Ctr < MEPORT_MAX_TX_BUF_SIZE-1)
            {
                meerstetterRS485::Buffer[Ctr] = in;
                meerstetterRS485::Ctr++;
                meerstetterRS485::Buffer[Ctr] = 0;
                meerstetterRS485::Ctr++;
                ComPort_Send(Buffer);
            }
        break;
    }
}


/*==============================================================================*/
/** @brief      Interface Function: Receive Byte
 *
 *  For the example target system, this function just calls the function
 *  "MeFrame_Receive" for every received char in the given string.
 *
 *  It is also Possible to modify the function prototype of this function,
 *  to just receive one single byte. (For example in case of an MCU)
*/
void meerstetterRS485::MePort_ReceiveByte(int8_t *arr)
{
    while(*arr)
    {
        if(*arr == '\n') *arr = '\r';
        MeFrame_Receive(*arr);
        arr++;
    }
}


/*==============================================================================*/
/** @brief      Interface Function: SemaphorTake
 *
 *  This function is being called by the Query and Set functions,
 *  while these functions are waiting for an answer of the connected device.
 *
 *  A timeout variable in milliseconds is passed to this function. The user
 *  implementation has to make sure that after this timeout has ran out,
 *  the function ends, even if no data has ben received, otherwise the
 *  system will stock for ever.
 *
 *  For the example target System a Condition Variable is implemented.
 *  The used lock functions are spezified in the POSIX standard.
 *
 *  It is also possible to run this API without an operating system:
 *  - Use a simple delay or timer function of an MCU and the
 *    data receiving function is being called by an interrupt
 *    routine of the UART interface.
 *  - Use a simple delay or timer function of an MCU to have a time base
 *    and poll the UART interface to check if some bytes have been received.
 *
*/
void meerstetterRS485::MePort_SemaphorTake(uint32_t TimeoutMs)
{
    recvData(TimeoutMs);
}


/*==============================================================================*/
/** @brief      Interface Function: SemaphorGive
 *
 *  This function is being called by the Frame receiving function, as soon as a
 *  complete frame has been received.
 *
 *  For the example target System a Condition Variable is implemented.
 *  The used lock functions are spezified in the POSIX standard.
 *
*/
void meerstetterRS485::MePort_SemaphorGive(void)
{
      /*
    pthread_mutex_lock(&Mutex);
    pthread_cond_signal(&Condition);
    pthread_mutex_unlock(&Mutex);
      */
}


/*==============================================================================*/
/** @brief      Interface Function: ErrorThrow
 *
 *  This function is being called by the Query and Set functions when
 *  Something went wrong.
 *
 *  For the example target System a simple console print out has been added.
 *
 *  It is recommended to forward this error Numbers to your error Management system.
 *
*/
void meerstetterRS485::MePort_ErrorThrow(int32_t ErrorNr)
{
    switch(ErrorNr)
    {
        case MEPORT_ERROR_CMD_NOT_AVAILABLE:
            Serial.println("MePort Error: Command not available\n");
        break;

        case MEPORT_ERROR_DEVICE_BUSY:
            Serial.println("MePort Error: Device is Busy\n");
        break;

        case MEPORT_ERROR_GENERAL_COM:
            Serial.println("MePort Error: General Error\n");
        break;

        case MEPORT_ERROR_FORMAT:
            Serial.println("MePort Error: Format Error\n");
        break;

        case MEPORT_ERROR_PAR_NOT_AVAILABLE:
            Serial.println("MePort Error: Parameter not available\n");
        break;

        case MEPORT_ERROR_PAR_NOT_WRITABLE:
            Serial.println("MePort Error: Parameter not writable\n");
        break;

        case MEPORT_ERROR_PAR_OUT_OF_RANGE:
            Serial.println("MePort Error: Parameter out of Range\n");
        break;

        case MEPORT_ERROR_PAR_INST_NOT_AVAILABLE:
            Serial.println("MePort Error: Parameter Instance not available\n");
        break;

        case MEPORT_ERROR_SET_TIMEOUT:
            Serial.println("MePort Error: Set Timeout\n");
        break;

        case MEPORT_ERROR_QUERY_TIMEOUT:
            Serial.println("MePort Error: Query Timeout\n");
        break;
    }
}


/*==============================================================================*/
/** @file       VarConvert.c
    @brief      Converts the variables for the communication
    @author     Meerstetter Engineering GmbH: Marc Luethi

*/
int8_t meerstetterRS485::MeVarConv_UcToHEX   (uint8_t value)
{
    if(value > 0x0F) return 'X';
    return meerstetterRS485::cHex[value];
}


/*======================================================================*/
uint8_t meerstetterRS485::MeVarConv_HexToDigit(int8_t *arr)
{
    return HEXtoNR(*arr);
}


/*======================================================================*/
uint8_t meerstetterRS485::MeVarConv_HexToUc   (int8_t *arr)
{
    return (HEXtoNR(*arr)*16) + HEXtoNR(*(arr+1));
}


/*======================================================================*/
int8_t  meerstetterRS485::MeVarConv_HexToSc   (int8_t *arr)
{
    return (int8_t)(((HEXtoNR(arr[0])&0x0F)*16)+ HEXtoNR(arr[1]));
}


/*======================================================================*/
uint16_t meerstetterRS485::MeVarConv_HexToUs   (int8_t *arr)
{
    return (HEXtoNR(*(arr+0))*4096)+ (HEXtoNR(*(arr+1))*256)+ (HEXtoNR(*(arr+2))*16)+ (HEXtoNR(*(arr+3)));
}


/*======================================================================*/
int16_t  meerstetterRS485::MeVarConv_HexToSs   (int8_t *arr)
{
    return (((int16_t)HEXtoNR(arr[0])<<12)+ ((int16_t)HEXtoNR(arr[1])<<8)+ ((int16_t)HEXtoNR(arr[2])<<4)+ (int16_t)HEXtoNR(arr[3]));
}


/*======================================================================*/
uint32_t meerstetterRS485::MeVarConv_HexToUl   (int8_t *arr)
{
    return 
        (((uint32_t)HEXtoNR(arr[0])<<28)   + ((uint32_t)HEXtoNR(arr[1])<<24)+  
        ((uint32_t)HEXtoNR(arr[2])<<20)    + ((uint32_t)HEXtoNR(arr[3])<<16)+
        ((uint32_t)HEXtoNR(arr[4])<<12)    + ((uint32_t)HEXtoNR(arr[5])<<8)+ 
        ((uint32_t)HEXtoNR(arr[6])<<4)     +  (uint32_t)HEXtoNR(arr[7]));
}


/*======================================================================*/
int32_t  meerstetterRS485::MeVarConv_HexToSl   (int8_t *arr)
{
    return 
        (((int32_t)HEXtoNR(arr[0])<<28)    + ((int32_t)HEXtoNR(arr[1])<<24)+  
        ((int32_t)HEXtoNR(arr[2])<<20)     + ((int32_t)HEXtoNR(arr[3])<<16)+
        ((int32_t)HEXtoNR(arr[4])<<12)     + ((int32_t)HEXtoNR(arr[5])<<8)+ 
        ((int32_t)HEXtoNR(arr[6])<<4)      + (int32_t)HEXtoNR(arr[7]));
}


/*======================================================================*/
float    meerstetterRS485::MeVarConv_HexToFloat(int8_t *arr)
{
    uint32_t temp;
    float fpv;
    temp = MeVarConv_HexToUl(arr);
    memcpy(&fpv, &temp, sizeof(fpv));
    return fpv;
}


/*======================================================================*/
void meerstetterRS485::MeVarConv_AddDigitHex   (int8_t *arr, uint8_t  value)
{
    *arr = meerstetterRS485::cHex[value]; arr++;
}


/*======================================================================*/
void meerstetterRS485::MeVarConv_AddUcHex      (int8_t *arr, uint8_t  value)
{
    *arr = meerstetterRS485::cHex[value/16]; arr++;
    *arr = meerstetterRS485::cHex[value%16]; arr++;
}


/*======================================================================*/
void meerstetterRS485::MeVarConv_AddScHex      (int8_t *arr, int8_t   value)
{
    uint8_t us = (uint8_t)value;
    *arr = meerstetterRS485::cHex[(us>>4)&0x00F]; arr++;
    *arr = meerstetterRS485::cHex[(us   )&0x00F]; arr++;
}


/*======================================================================*/
void meerstetterRS485::MeVarConv_AddUsHex      (int8_t *arr, uint16_t value)
{
    value = value & 0x0000FFFF;
    *arr = meerstetterRS485::cHex[value/4096]; arr++;
    *arr = meerstetterRS485::cHex[(value/256)%16]; arr++;
    *arr = meerstetterRS485::cHex[(value%256)/16]; arr++;
    *arr = meerstetterRS485::cHex[(value%256)%16]; arr++;
}


/*======================================================================*/
void meerstetterRS485::MeVarConv_AddSsHex      (int8_t *arr, int16_t  value)
{
    uint16_t us = (uint16_t)value;
    *arr = meerstetterRS485::cHex[(us>>12)&0x00F]; arr++;
    *arr = meerstetterRS485::cHex[(us>>8)&0x00F]; arr++;
    *arr = meerstetterRS485::cHex[(us>>4)&0x00F]; arr++;
    *arr = meerstetterRS485::cHex[(us   )&0x00F]; arr++;
}


/*======================================================================*/
void meerstetterRS485::MeVarConv_AddUlHex      (int8_t *arr, uint32_t value)
{
    *arr = meerstetterRS485::cHex[value>>28]; arr++;
    *arr = meerstetterRS485::cHex[(value>>24)&0x00F]; arr++;
    *arr = meerstetterRS485::cHex[(value>>20)&0x00F]; arr++;
    *arr = meerstetterRS485::cHex[(value>>16)&0x00F]; arr++;
    *arr = meerstetterRS485::cHex[(value>>12)&0x00F]; arr++;
    *arr = meerstetterRS485::cHex[(value>>8)&0x00F]; arr++;
    *arr = meerstetterRS485::cHex[(value>>4)&0x00F]; arr++;
    *arr = meerstetterRS485::cHex[(value   )&0x00F]; arr++;
}


/*======================================================================*/
void meerstetterRS485::MeVarConv_AddSlHex      (int8_t *arr, int32_t  value)
{
    uint32_t ul = (uint32_t)value;
    *arr = meerstetterRS485::cHex[ul>>28]; arr++;
    *arr = meerstetterRS485::cHex[(ul>>24)&0x00F]; arr++;
    *arr = meerstetterRS485::cHex[(ul>>20)&0x00F]; arr++;
    *arr = meerstetterRS485::cHex[(ul>>16)&0x00F]; arr++;
    *arr = meerstetterRS485::cHex[(ul>>12)&0x00F]; arr++;
    *arr = meerstetterRS485::cHex[(ul>>8)&0x00F]; arr++;
    *arr = meerstetterRS485::cHex[(ul>>4)&0x00F]; arr++;
    *arr = meerstetterRS485::cHex[(ul   )&0x00F]; arr++;
}


/*======================================================================*/
void meerstetterRS485::MeVarConv_AddFloatHex   (int8_t *arr, float    value)
{
    uint32_t lvalue;
    memcpy(&lvalue, &value, sizeof(value));
    MeVarConv_AddUlHex(arr, lvalue);
}


/*======================================================================*/
uint8_t meerstetterRS485::HEXtoNR(int8_t uc)
{
    switch(uc)
    {
        case '0': return 0;
        case '1': return 1;
        case '2': return 2;
        case '3': return 3;
        case '4': return 4;
        case '5': return 5;
        case '6': return 6;
        case '7': return 7;
        case '8': return 8;
        case '9': return 9;
        case 'A': return 10;
        case 'B': return 11;
        case 'C': return 12;
        case 'D': return 13;
        case 'E': return 14;
        case 'F': return 15;
        case 'a': return 10;
        case 'b': return 11;
        case 'c': return 12;
        case 'd': return 13;
        case 'e': return 14;
        case 'f': return 15;
    }

    return (0);
}

//
// helper functions
//
bool meerstetterRS485::StartTEC(uint8_t Address)
{
    bool    retVal      = false;
    uint8_t Instance    = 1;
    MeParLongFields FieldVal;


    //  
    // TODO : make this more generic
    // TODO : find the proper commands
    //
    FieldVal = {2, 0, 0};  // send 2 to enabel LiveOnOff
    if( !(MeCom_TEC_Ope_OutputStageEnable(Address, Instance, &FieldVal, MeSet)) )
    {
        #ifdef __DEBUG_MS_VIA_SERIAL__
        Serial.print(__PRETTY_FUNCTION__);
        Serial.print(" OutputStageEnable failed for addr ");
        Serial.println(Address, DEC);
        #endif
    } else
    {
        FieldVal = {1, 0, 0};  // send 1 to enable ..
        if( !(MeCom_TEC_Oth_LiveEnable(Address, Instance, &FieldVal, MeSet)) )
        {
            #ifdef __DEBUG_MS_VIA_SERIAL__
            Serial.print(__PRETTY_FUNCTION__);
            Serial.print(" LiveEnable failed for addr ");
            Serial.println(Address, DEC);
            #endif
        } else
        {
            //
            // verify the TEC is running
            //
            if( (TECRunning(Address)) )
            {
                retVal  = true;

            #ifdef __DEBUG_MS_VIA_SERIAL__
            } else
            {
                Serial.print(__PRETTY_FUNCTION__);
                Serial.print(" TEC not running for addr ");
                Serial.println(Address, DEC);
            #endif
            }
        }
    }

    return(retVal);
}


bool meerstetterRS485::StopTEC(uint8_t Address)
{
    bool    retVal      = false;
    uint8_t Instance    = 1;
    MeParLongFields FieldVal;


    //  
    // TODO : make this more generic
    // TODO : find the proper commands
    //  
    FieldVal = {0, 0, 0};  // send 0 to disable ..
    if( !(MeCom_TEC_Oth_LiveEnable(Address, Instance, &FieldVal, MeSet)) )
    {
        #ifdef __DEBUG_MS_VIA_SERIAL__
        Serial.print(__PRETTY_FUNCTION__);
        Serial.print(" LiveEnable failed for addr ");
        Serial.println(Address, DEC);
        #endif
    }

    //
    // if MeCom_TEC_Oth_LiveEnable fails continue with the following command anyway
    // trying to shutDown the TEC
    //
    FieldVal = {0, 0, 0};  // send 0 for static off
    if( !(MeCom_TEC_Ope_OutputStageEnable(Address, Instance, &FieldVal, MeSet)) )
    {
        #ifdef __DEBUG_MS_VIA_SERIAL__
        Serial.print(__PRETTY_FUNCTION__);
        Serial.print(" OutputStageEnable failed for addr ");
        Serial.println(Address, DEC);
        #endif
    }

    //
    // check if the TEC is off
    //
    // TODO : this is still bad, TECRunning will return false
    // if communication with the TEC is not possible..
    //
    if( !(TECRunning(Address)) )
        retVal  = true;
    #ifdef __DEBUG_MS_VIA_SERIAL__
    else {
        Serial.print(__PRETTY_FUNCTION__);
        Serial.print(" TECRunning is still true for addr ");
        Serial.println(Address, DEC);
    }
    #endif

    return(retVal);
}


bool meerstetterRS485::TECRunning(uint8_t Address)
{
    bool retVal = false;
    uint8_t Instance    = 1;
    MeParLongFields FieldVal;


    FieldVal = {0, 0, 0};
    if( (MeCom_COM_DeviceStatus(Address, &FieldVal, MeGet)) )
    {
        if( (1 == FieldVal.Value) || (2 == FieldVal.Value) )  //TODO: figure out the TEC running state
        {
            retVal  = true;
        #ifdef __DEBUG_MS_VIA_SERIAL__
        } else
        {
            Serial.print(__PRETTY_FUNCTION__);
            Serial.print(" DeviceStatus not run for Address ");
            Serial.print(Address, DEC);
            Serial.print(", is ");
            Serial.println(FieldVal.Value, DEC);
        #endif
        }
    #ifdef __DEBUG_MS_VIA_SERIAL__
    } else
    {
        Serial.print(__PRETTY_FUNCTION__);
        Serial.print(" unable to MeCom_COM_DeviceStatus for Address ");
        Serial.println(Address, DEC);
    #endif
    }

    return(retVal);
}


bool meerstetterRS485::TECPresent(uint8_t Address)
{
    bool retVal = false;
    uint8_t Instance    = 1;
    MeParLongFields FieldVal;


    FieldVal = {0x7fff, 0, 0};  // TODO: is 0x7fff a valid sentinel value ?
    if( (MeCom_COM_DeviceStatus(Address, &FieldVal, MeGet)) )
    {
        if( (0x7fff != FieldVal.Value) ) // check the sentinel value has changed
        {
            retVal  = true;
        #ifdef __DEBUG_MS_VIA_SERIAL__
        } else
        {
            Serial.print(__PRETTY_FUNCTION__);
            Serial.print(" DeviceStatus not run for Address ");
            Serial.print(Address, DEC);
            Serial.print(", is ");
            Serial.println(FieldVal.Value, DEC);
        #endif
        }
    #ifdef __DEBUG_MS_VIA_SERIAL__
    } else
    {
        Serial.print(__PRETTY_FUNCTION__);
        Serial.print(" unable to MeCom_COM_DeviceStatus for Address ");
        Serial.println(Address, DEC);
    #endif
    }

    return(retVal);
}

bool meerstetterRS485::SetTECTemp(uint8_t Address, float temp)
{
    bool retVal = true;
    uint8_t Instance    = 1; 
    MeParFloatFields FieldVal;

    #ifdef __DEBUG_MS_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    FieldVal.Value = temp;
    FieldVal.Min = FieldVal.Max = 0;

    // TODO: how to handle this if one meerstetter doesn't change temp
    if( !(MeCom_TEC_Tem_TargetObjectTemp(Address, Instance, &FieldVal, MeSet)) )
    {
        #ifdef __DEBUG_MS_VIA_SERIAL__
        Serial.print(__PRETTY_FUNCTION__);
        Serial.flush();
        Serial.print(" failed to MeSet for Address ");
        Serial.flush();
        Serial.println(Address, HEX);
        Serial.flush();
        #endif

        retVal = false;
    } else
    {
        //
        // check what was set
        //
        FieldVal.Value = FieldVal.Min = FieldVal.Max = 0;

        if( !(MeCom_TEC_Tem_TargetObjectTemp(Address, Instance, &FieldVal, MeGet)) )
        {
            #ifdef __DEBUG_MS_VIA_SERIAL__
            Serial.print(__PRETTY_FUNCTION__);
            Serial.flush();
            Serial.print(" failed to MeGet for Address ");
            Serial.flush();
            Serial.println(Address, HEX);
            Serial.flush();
            #endif

            retVal = false;
        } else
        {
            if( (FieldVal.Value !=  temp) )
            {
                #ifdef __DEBUG_MS_VIA_SERIAL__
                Serial.print(__PRETTY_FUNCTION__);
                Serial.flush();
                Serial.print(" retported temps don't match input ");
                Serial.flush();
                Serial.print(temp, 2);
                Serial.flush();
                Serial.print(" fetched ");
                Serial.flush();
                Serial.print(FieldVal.Value, 2);
                Serial.flush();
                Serial.print(" for Address ");
                Serial.println(Address, HEX);
                Serial.flush();
                #endif

                retVal = false;
            } else
            {
                #ifdef __DEBUG_MS_VIA_SERIAL__
                Serial.print(__PRETTY_FUNCTION__);
                Serial.flush();
                Serial.print(" retported temps do match input ");
                Serial.flush();
                Serial.print(temp, 2);
                Serial.flush();
                Serial.print(" fetched ");
                Serial.flush();
                Serial.print(FieldVal.Value, 2);
                Serial.flush();
                Serial.print(" for Address ");
                Serial.println(Address, HEX);
                Serial.flush();
                #endif

                retVal = true;
            }
        }
    }

    return(retVal);
}


//
// fetch the setpoint and the actual
//
// TODO:  find the correct commands
//
bool meerstetterRS485::GetTECTemp(uint8_t Address, float* setPoint, float* actualTemp)
{
    bool retVal = true;
    uint8_t Instance    = 1; 
    MeParFloatFields FieldVal;
    float sp, at;


    #ifdef __DEBUG_MS_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    //
    // fetch the set point
    //
    FieldVal.Value = FieldVal.Min = FieldVal.Max = 0;
    if( !(MeCom_TEC_Mon_TargetObjectTemperature(Address, Instance, &FieldVal, MeGet)) )
    {
        #ifdef __DEBUG_MS_VIA_SERIAL__
        Serial.print(__PRETTY_FUNCTION__);
        Serial.flush();
        Serial.print(" failed to TargetObjectTemp for address ");
        Serial.flush();
        Serial.println(Address, HEX);
        Serial.flush();
        #endif

        retVal = false;
    } else
    {
        sp = FieldVal.Value;

        //
        // fetch the set actual temperature
        //
        FieldVal.Value = FieldVal.Min = FieldVal.Max = 0;
        if( !(MeCom_TEC_Mon_ObjectTemperature(Address, Instance, &FieldVal, MeGet)) )
        {
            #ifdef __DEBUG_MS_VIA_SERIAL__
            Serial.print(__PRETTY_FUNCTION__);
            Serial.flush();
            Serial.print(" failed to ObjectTemp for address ");
            Serial.flush();
            Serial.println(Address, HEX);
            Serial.flush();
            #endif
    
            retVal = false;
        } else
        {
            at = FieldVal.Value;
        }
    }

    if( (retVal) )
    {
        *actualTemp = at;
        *setPoint   = sp;
    }

    return(retVal);
}

