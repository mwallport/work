#include <SoftwareSerial.h>
#include <RS485_protocol.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <stdbool.h>
#include <stdlib.h>
#include <time.h>
#include "header.h"


/*-----( Declare Constants and Pin Numbers )-----*/
#define SSerialRX        10  //Serial Receive pin
#define SSerialTX        11  //Serial Transmit pin
#define RS485Transmit    HIGH
#define RS485Receive     LOW
#define Pin13LED         13


/*-----( Define global / static variables and objects -----*/
SoftwareSerial RS485Serial(SSerialRX, SSerialTX); // RX, TX

    static const uint16_t CRC16_table_C[256] = {
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

struct MeFrame_RcvFrameS MeFrame_RcvFrame;
static uint16_t LastCRC;
int8_t *MeInt_QueryRcvPayload = MeFrame_RcvFrame.Payload;
static uint16_t SequenceNr = 5545; //Initialized to random value
static uint8_t HEXtoNR(int8_t uc);
static const int8_t cHex[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
static void TestAllCommonGetFunctions(uint8_t Address);
static void TestAllTECGetFunctions(uint8_t Address);


/*-----( Define Functions )-----*/
void ComPort_Send(char *in)
{
	size_t size;
	int len = strlen(in);


	size = RS485Serial.write(in); //TODO :  new stuff using RS485Serial, tighten this up

	if(size != len)
        {
          Serial.print("ERROR : ComPort_Send did not get size bytes sent, got : ");
          Serial.println(size, DEC);
          return; //TODO : return data Write Error
        /*} else
        {
          Serial.print("sent ");
          Serial.print(size, DEC);
          Serial.println(" bytes");*/
        }
}


static void* recvData()  // TODO : this was a thread
{
	char Buffer[100];
	int32_t bytes_read = 0;
        int8_t  peekBytes;

        
        // modified to read one byte at a time
        while( (bytes_read < 100) && (RS485Serial.available()) )  //TODO #define the 100
        {
           digitalWrite(Pin13LED, HIGH);  // Show activity
           Buffer[bytes_read++] = RS485Serial.read();    // Read received byte
           digitalWrite(Pin13LED, LOW);  // Show activity 
        }

        // TODO strengthen this
        Buffer[bytes_read] = 0;
        MePort_ReceiveByte((int8_t*)Buffer);
        /*Serial.print("reived ");
        Serial.print(bytes_read, DEC);
        Serial.println(" bytes");*/
}


/*==============================================================================*/
/** @brief      Reset Device
 *
*/
uint8_t MeCom_ResetDevice(uint8_t Address)
{
    return MeInt_Set('#', Address, 2, (int8_t*)"RS");
}


/*==============================================================================*/
/** @brief      Return IF String
 *
*/
uint8_t MeCom_GetIdentString(uint8_t Address, int8_t *arr)
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
uint8_t MeCom_ParValuel(uint8_t Address, uint16_t ParId, uint8_t Inst, MeParLongFields  *Fields, MeParCmd Cmd)
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
uint8_t MeCom_ParValuef(uint8_t Address, uint16_t ParId, uint8_t Inst, MeParFloatFields *Fields, MeParCmd Cmd)
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
uint16_t MeCRC16(uint16_t n, uint8_t m)
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
void MeFrame_Send(int8_t Control, uint8_t Address, uint32_t Length, uint16_t SeqNr, int8_t *Payload)
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

    LastCRC = CRC;
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
void MeFrame_Receive(int8_t in)
{
    static int8_t RcvBuf[MEPORT_MAX_RX_BUF_SIZE + 20];
    static int32_t RcvCtr = -1;

    if(in == '!')
    {
        //Start Indicator --> Reset Receiving Machine
        memset(RcvBuf, 0, sizeof(RcvBuf));
        RcvBuf[0] = in;
        RcvCtr = 1;
        
    }
    else if(in == 0x0D && (RcvCtr >=11))
    {
        //End of a Frame received
        if(RcvCtr == 11)
        {
            //Ack Received

            uint16_t RcvCRC = MeVarConv_HexToUs(&RcvBuf[7]);
            if(RcvCRC == LastCRC && MeFrame_RcvFrame.AckReceived == 0)
            {
                MeFrame_RcvFrame.Address    = MeVarConv_HexToUc(&RcvBuf[1]);
                MeFrame_RcvFrame.SeqNr      = MeVarConv_HexToUs(&RcvBuf[3]);
                MeFrame_RcvFrame.AckReceived = 1;
            }
            else RcvCtr = -1; //Error
        }
        else
        {
            //Data Received 

            //Check CRC of received Frame
            uint16_t RcvCRC, CalcCRC = 0;
            for(int32_t i=0; i < (RcvCtr-4); i++) CalcCRC = MeCRC16(CalcCRC, RcvBuf[i]); //Calculate CRC of received Frame
            RcvCRC = MeVarConv_HexToUs(&RcvBuf[RcvCtr-4]); //Get Frame CRC
            if(RcvCRC == CalcCRC && MeFrame_RcvFrame.DataReceived == 0)
            {
                //CRC is correct and all data has been processed
                MeFrame_RcvFrame.Address    = MeVarConv_HexToUc(&RcvBuf[1]);
                MeFrame_RcvFrame.SeqNr      = MeVarConv_HexToUs(&RcvBuf[3]);
                for(int32_t i = 7; i < (RcvCtr-4);  i++)	MeFrame_RcvFrame.Payload[i-7] = RcvBuf[i];
                MeFrame_RcvFrame.DataReceived = 1;
            }
        }
    }
    else if(RcvCtr >= 0 && RcvCtr < (MEPORT_MAX_RX_BUF_SIZE+15))
    {
        //Write Data to Buffer
        RcvBuf[RcvCtr] = in;
        RcvCtr++;
    }
    else
    {
        //Error 
        RcvCtr = -1;
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
uint8_t MeInt_Query(int8_t Control, uint8_t Address, uint32_t Length, int8_t *Payload)
{
    SequenceNr++;

    int32_t Trials = 3;
    while(Trials > 0)
    {
        Trials--;
        MeFrame_RcvFrame.DataReceived = 0;
        MeFrame_RcvFrame.AckReceived = 0;
        MeFrame_Send(Control, Address, Length, SequenceNr, Payload);
        MePort_SemaphorTake(MEPORT_SET_AND_QUERY_TIMEOUT);
        if(MeFrame_RcvFrame.DataReceived == 1 && MeFrame_RcvFrame.Address == Address && MeFrame_RcvFrame.SeqNr == SequenceNr )
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
uint8_t MeInt_Set(int8_t Control, uint8_t Address, uint32_t Length, int8_t *Payload)
{
    SequenceNr++;

    int32_t Trials = 3;
    while(Trials > 0)
    {
        Trials--;
        MeFrame_RcvFrame.DataReceived = 0;
        MeFrame_RcvFrame.AckReceived = 0;
        MeFrame_Send(Control, Address, Length, SequenceNr, Payload);
        MePort_SemaphorTake(MEPORT_SET_AND_QUERY_TIMEOUT);
        if(MeFrame_RcvFrame.DataReceived == 1 && MeFrame_RcvFrame.Address == Address && MeFrame_RcvFrame.SeqNr == SequenceNr &&
            MeFrame_RcvFrame.Payload[0] == '+')
        {
            //Server Error code Received
            MePort_ErrorThrow(MeVarConv_HexToUc(&MeFrame_RcvFrame.Payload[1]));
            return 0;
        }
        else if(MeFrame_RcvFrame.AckReceived == 1 && MeFrame_RcvFrame.Address == Address && MeFrame_RcvFrame.SeqNr == SequenceNr )
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
void MePort_SendByte(int8_t in, MePort_SB FirstLast)
{
    static char Buffer[MEPORT_MAX_TX_BUF_SIZE];
    static int Ctr;
    
    //Serial.println("in MePort_SendByte...");
    
    switch(FirstLast)
    {
        case MePort_SB_IsFirstByte:
            //This is the first Byte of the Message String 
            Ctr = 0;
            Buffer[Ctr] = in;
            Ctr++;
        break;
        case MePort_SB_Normal:
            //These are some middle Bytes
            if(Ctr < MEPORT_MAX_TX_BUF_SIZE-1)
            {
                Buffer[Ctr] = in;
                Ctr++;
            }
        break;
        case MePort_SB_IsLastByte:
            //This is the last Byte of the Message String
            if(Ctr < MEPORT_MAX_TX_BUF_SIZE-1)
            {
                Buffer[Ctr] = in;
                Ctr++;
                Buffer[Ctr] = 0;
                Ctr++;
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
void MePort_ReceiveByte(int8_t *arr)
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
void MePort_SemaphorTake(uint32_t TimeoutMs)
{
        // polling for 5x the TimeoutMS - TODO : make this better
        uint8_t count = 0;
        uint8_t data_ready = 0;
        
        while( (5 > count++) )
        {
          if( (!RS485Serial.available()) )
          {
            //Serial.println("MePort_SemaphorTake no data on port...");
            delay(TimeoutMs);
          } else
          {
            //Serial.println("MePort_SemaphorTake finds data on port...");
            data_ready = 1;
            break;
          }
        }
        
        // TODO : tighten this up, just calling recvData here after waiting for data to arrive
        // on the port
        if( (data_ready) )
          recvData();
        
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
void MePort_SemaphorGive(void)
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
void MePort_ErrorThrow(int32_t ErrorNr)
{
    switch(ErrorNr)
    {
        case MEPORT_ERROR_CMD_NOT_AVAILABLE:
            //Serial.println("MePort Error: Command not available\n");
        break;

        case MEPORT_ERROR_DEVICE_BUSY:
            //Serial.println("MePort Error: Device is Busy\n");
        break;

        case MEPORT_ERROR_GENERAL_COM:
            //Serial.println("MePort Error: General Error\n");
        break;

        case MEPORT_ERROR_FORMAT:
            //Serial.println("MePort Error: Format Error\n");
        break;

        case MEPORT_ERROR_PAR_NOT_AVAILABLE:
            //Serial.println("MePort Error: Parameter not available\n");
        break;

        case MEPORT_ERROR_PAR_NOT_WRITABLE:
            //Serial.println("MePort Error: Parameter not writable\n");
        break;

        case MEPORT_ERROR_PAR_OUT_OF_RANGE:
            //Serial.println("MePort Error: Parameter out of Range\n");
        break;

        case MEPORT_ERROR_PAR_INST_NOT_AVAILABLE:
            //Serial.println("MePort Error: Parameter Instance not available\n");
        break;

        case MEPORT_ERROR_SET_TIMEOUT:
            //Serial.println("MePort Error: Set Timeout\n");
        break;

        case MEPORT_ERROR_QUERY_TIMEOUT:
            //Serial.println("MePort Error: Query Timeout\n");
        break;
    }
}


/*==============================================================================*/
/** @file       VarConvert.c
    @brief      Converts the variables for the communication
    @author     Meerstetter Engineering GmbH: Marc Luethi

*/
int8_t   MeVarConv_UcToHEX   (uint8_t value)
{
    if(value > 0x0F) return 'X';
    return cHex[value];
}


/*======================================================================*/
uint8_t  MeVarConv_HexToDigit(int8_t *arr)
{
	return HEXtoNR(*arr);
}


/*======================================================================*/
uint8_t  MeVarConv_HexToUc   (int8_t *arr)
{
	return (HEXtoNR(*arr)*16) + HEXtoNR(*(arr+1));
}


/*======================================================================*/
int8_t   MeVarConv_HexToSc   (int8_t *arr)
{
    return (int8_t)(((HEXtoNR(arr[0])&0x0F)*16)+ HEXtoNR(arr[1]));
}


/*======================================================================*/
uint16_t MeVarConv_HexToUs   (int8_t *arr)
{
	return (HEXtoNR(*(arr+0))*4096)+ (HEXtoNR(*(arr+1))*256)+ (HEXtoNR(*(arr+2))*16)+ (HEXtoNR(*(arr+3)));
}


/*======================================================================*/
int16_t  MeVarConv_HexToSs   (int8_t *arr)
{
	return (((int16_t)HEXtoNR(arr[0])<<12)+ ((int16_t)HEXtoNR(arr[1])<<8)+ ((int16_t)HEXtoNR(arr[2])<<4)+ (int16_t)HEXtoNR(arr[3]));
}


/*======================================================================*/
uint32_t MeVarConv_HexToUl   (int8_t *arr)
{
	return 
        (((uint32_t)HEXtoNR(arr[0])<<28)   + ((uint32_t)HEXtoNR(arr[1])<<24)+  
        ((uint32_t)HEXtoNR(arr[2])<<20)    + ((uint32_t)HEXtoNR(arr[3])<<16)+
		((uint32_t)HEXtoNR(arr[4])<<12)    + ((uint32_t)HEXtoNR(arr[5])<<8)+ 
        ((uint32_t)HEXtoNR(arr[6])<<4)     +  (uint32_t)HEXtoNR(arr[7]));
}


/*======================================================================*/
int32_t  MeVarConv_HexToSl   (int8_t *arr)
{
	return 
        (((int32_t)HEXtoNR(arr[0])<<28)    + ((int32_t)HEXtoNR(arr[1])<<24)+  
        ((int32_t)HEXtoNR(arr[2])<<20)     + ((int32_t)HEXtoNR(arr[3])<<16)+
		((int32_t)HEXtoNR(arr[4])<<12)     + ((int32_t)HEXtoNR(arr[5])<<8)+ 
        ((int32_t)HEXtoNR(arr[6])<<4)      + (int32_t)HEXtoNR(arr[7]));
}


/*======================================================================*/
float    MeVarConv_HexToFloat(int8_t *arr)
{
    uint32_t temp;
    float fpv;
    temp = MeVarConv_HexToUl(arr);
    memcpy(&fpv, &temp, sizeof(fpv));
    return fpv;
}


/*======================================================================*/
void MeVarConv_AddDigitHex   (int8_t *arr, uint8_t  value)
{
	*arr = cHex[value]; arr++;
}


/*======================================================================*/
void MeVarConv_AddUcHex      (int8_t *arr, uint8_t  value)
{
	*arr = cHex[value/16]; arr++;
	*arr = cHex[value%16]; arr++;
}


/*======================================================================*/
void MeVarConv_AddScHex      (int8_t *arr, int8_t   value)
{
    uint8_t us = (uint8_t)value;
    *arr = cHex[(us>>4)&0x00F]; arr++;
    *arr = cHex[(us   )&0x00F]; arr++;
}


/*======================================================================*/
void MeVarConv_AddUsHex      (int8_t *arr, uint16_t value)
{
	value = value & 0x0000FFFF;
	*arr = cHex[value/4096]; arr++;
	*arr = cHex[(value/256)%16]; arr++;
	*arr = cHex[(value%256)/16]; arr++;
	*arr = cHex[(value%256)%16]; arr++;
}


/*======================================================================*/
void MeVarConv_AddSsHex      (int8_t *arr, int16_t  value)
{
    uint16_t us = (uint16_t)value;
	*arr = cHex[(us>>12)&0x00F]; arr++;
	*arr = cHex[(us>>8)&0x00F]; arr++;
	*arr = cHex[(us>>4)&0x00F]; arr++;
	*arr = cHex[(us   )&0x00F]; arr++;
}


/*======================================================================*/
void MeVarConv_AddUlHex      (int8_t *arr, uint32_t value)
{
	*arr = cHex[value>>28]; arr++;
	*arr = cHex[(value>>24)&0x00F]; arr++;
	*arr = cHex[(value>>20)&0x00F]; arr++;
	*arr = cHex[(value>>16)&0x00F]; arr++;
	*arr = cHex[(value>>12)&0x00F]; arr++;
	*arr = cHex[(value>>8)&0x00F]; arr++;
	*arr = cHex[(value>>4)&0x00F]; arr++;
	*arr = cHex[(value   )&0x00F]; arr++;
}


/*======================================================================*/
void MeVarConv_AddSlHex      (int8_t *arr, int32_t  value)
{
    uint32_t ul = (uint32_t)value;
	*arr = cHex[ul>>28]; arr++;
	*arr = cHex[(ul>>24)&0x00F]; arr++;
	*arr = cHex[(ul>>20)&0x00F]; arr++;
	*arr = cHex[(ul>>16)&0x00F]; arr++;
	*arr = cHex[(ul>>12)&0x00F]; arr++;
	*arr = cHex[(ul>>8)&0x00F]; arr++;
	*arr = cHex[(ul>>4)&0x00F]; arr++;
	*arr = cHex[(ul   )&0x00F]; arr++;
}


/*======================================================================*/
void MeVarConv_AddFloatHex   (int8_t *arr, float    value)
{
    uint32_t lvalue;
    memcpy(&lvalue, &value, sizeof(value));
    MeVarConv_AddUlHex(arr, lvalue);
}


/*======================================================================*/
static uint8_t HEXtoNR(int8_t uc)
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


/*==============================================================================*/
/** @file       main.c
    @brief      This file is not part of the MeComAPI. Just simple Demo Application.
    @author     Meerstetter Engineering GmbH: Marc Luethi / Thomas Braun
    @version    v0.42

    This is the TOP file of the MeComAPI Demo Application.
    The Demo Application is being used to test the MeComAPI and
    should show the user how the MeComAPI shoud be used.

    The Demo Application has been compiled an the following Product:
    "Microsoft Visual Studio 2010"

    The Demo Application might uses some C++ functions.
    The MeComAPI is written in C (ANSI C99).

    Please refer to the Document 5170-MeComAPI for more information.

*/




void loop()   /****** LOOP: RUNS CONSTANTLY ******/
{
    int32_t Address = 2;    // this is 0 - 255, default is 0
    int8_t Buf[25];


     if(MeCom_ResetDevice(Address))
     {
         Serial.println("Device Reset OK.\n");
     }

     delay(5000);
     

    while(1)
    {
      Serial.println("looping ...");

      if(MeCom_GetIdentString(Address, Buf)) 
      {
         Serial.println("Adress 2 Ident String: ");
         for(int x = 0; x < 25 && Buf[x]; x++)
         {
           Serial.print(Buf[x]);
         }
         
         Serial.println("");
      }          

      SetTargetObjectTemp(Address);

      // delay 10 seconds, let devie come out of reset .. ?
//      delay(5000);
/*      
      TestAllCommonGetFunctions(Address);
      TestAllTECGetFunctions(Address);
*/
      //delay(3000);
       delay(5000);
    }
}


void setup()   /****** SETUP: RUNS ONCE ******/
{
    // Start the built-in serial port, probably to Serial Monitor
  Serial.begin(9600);

  pinMode(Pin13LED, OUTPUT);

  // Start the software serial port, to another device
  RS485Serial.begin(9600);   // set the data rate 
}


/*==============================================================================*/
/** @brief      Test function for all Common Get Functions
 *
*/
static void TestAllCommonGetFunctions(uint8_t Address)
{
    MeParLongFields  lFields;

    if(MeCom_COM_DeviceType(Address, &lFields, MeGet)) 
    {
        Serial.println("MeCom_COM_DeviceType");
        printLongFields(lFields);
    }

    if(MeCom_COM_HardwareVersion(Address, &lFields, MeGet)) 
    {
        Serial.println("MeCom_COM_HardwareVersion");
        printLongFields(lFields);
    }
/*
    if(MeCom_COM_SerialNumber(Address, &lFields, MeGet)) 
    {
        Serial.println("MeCom_COM_SerialNumber");
        printLongFields(lFields);
    }

    if(MeCom_COM_FirmwareVersion(Address, &lFields, MeGet)) 
    {
        Serial.println("MeCom_COM_FirmwareVersion");
        printLongFields(lFields);
    }

    if(MeCom_COM_DeviceStatus(Address, &lFields, MeGet)) 
    {
        Serial.println("MeCom_COM_DeviceStatus");
        printLongFields(lFields);
    }

    if(MeCom_COM_ErrorNumber(Address, &lFields, MeGet)) 
    {
        Serial.println("MeCom_COM_ErrorNumber");
        printLongFields(lFields);
    }

    if(MeCom_COM_ErrorInstance(Address, &lFields, MeGet)) 
    {
        Serial.println("MeCom_COM_ErrorInstance");
        printLongFields(lFields);
    }

    if(MeCom_COM_ErrorParameter(Address, &lFields, MeGet)) 
    {
        Serial.println("MeCom_COM_ErrorParameter");
        printLongFields(lFields);
    }
    if(MeCom_COM_ParameterSystemFlashSaveOff(Address, &lFields, MeGet)) 
    {
        Serial.println("MeCom_COM_ParameterSystemFlashSaveOff");
        printLongFields(lFields);
    }
    if(MeCom_COM_ParameterSystemFlashStatus(Address, &lFields, MeGet)) 
    {
        Serial.println("MeCom_COM_ParameterSystemFlashStatus");
        printLongFields(lFields);
    }
*/
}
/*==============================================================================*/
/** @brief      Test function for all TEC Get Functions
 *
*/
static void TestAllTECGetFunctions(uint8_t Address)
{
    MeParLongFields  lFields;
    MeParFloatFields fFields;


    //Tab Monitor
    if(MeCom_TEC_Mon_ObjectTemperature(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Mon_ObjectTemperature");
        printFloatFields(fFields);
    }
    
    if(MeCom_TEC_Mon_SinkTemperature(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Mon_SinkTemperature");
        printFloatFields(fFields);
    }

    if(MeCom_TEC_Mon_TargetObjectTemperature(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Mon_TargetObjectTemperature");
        printFloatFields(fFields);
    }
/*
    if(MeCom_TEC_Mon_RampNominalObjectTemperature(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Mon_RampNominalObjectTemperature");
        printFloatFields(fFields);
    }

    if(MeCom_TEC_Mon_ThermalPowerModelCurrent(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Mon_ThermalPowerModelCurrent");
        printFloatFields(fFields);
    }

    if(MeCom_TEC_Mon_ActualOutputCurrent(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Mon_ActualOutputCurrent");
        printFloatFields(fFields);
    }

    if(MeCom_TEC_Mon_ActualOutputVoltage(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Mon_ActualOutputVoltage");
        printFloatFields(fFields);
    }

    if(MeCom_TEC_Mon_PIDLowerLimitation(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Mon_PIDLowerLimitation");
        printFloatFields(fFields);
    }

    if(MeCom_TEC_Mon_PIDUpperLimitation(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Mon_PIDUpperLimitation");
        printFloatFields(fFields);
    }

    if(MeCom_TEC_Mon_PIDControlVariable(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Mon_PIDControlVariable");
        printFloatFields(fFields);
    }

    if(MeCom_TEC_Mon_ObjectSensorRawADCValue(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Mon_ObjectSensorRawADCValue");
        printFloatFields(fFields);
    }

    if(MeCom_TEC_Mon_SinkSensorRawADCValue(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Mon_SinkSensorRawADCValue");
        printFloatFields(fFields);
    }

    if(MeCom_TEC_Mon_ObjectSensorResistance(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Mon_ObjectSensorResistance");
        printFloatFields(fFields);
    }

    if(MeCom_TEC_Mon_SinkSensorResitance(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Mon_SinkSensorResitance");
        printFloatFields(fFields);
    }
        
    if(MeCom_TEC_Mon_SinkSensorTemperature(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Mon_SinkSensorTemperature");
        printFloatFields(fFields);
    }
    
    if(MeCom_TEC_Mon_FirmwareVersion(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Mon_FirmwareVersion");
        printFloatFields(fFields);
    }

    if(MeCom_TEC_Mon_FirmwareBuildNumber(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Mon_FirmwareBuildNumber");
        printFloatFields(fFields);
    }

    if(MeCom_TEC_Mon_HardwareVersion(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Mon_HardwareVersion");
        printFloatFields(fFields);
    }

    if(MeCom_TEC_Mon_SerialNumber(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Mon_SerialNumber");
        printFloatFields(fFields);
    }

    if(MeCom_TEC_Mon_DriverInputVoltage(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Mon_DriverInputVoltage");
        printFloatFields(fFields);
    }

    if(MeCom_TEC_Mon_MedVInternalSupply(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Mon_MedVInternalSupply");
        printFloatFields(fFields);
    }

    if(MeCom_TEC_Mon_3_3VInternalSupply(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Mon_3_3VInternalSupply");
        printFloatFields(fFields);
    }

    if(MeCom_TEC_Mon_BasePlateTemperature(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Mon_BasePlateTemperature");
        printFloatFields(fFields);
    }

    if(MeCom_TEC_Mon_ErrorNumber(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Mon_ErrorNumber");
        printFloatFields(fFields);
    }

    if(MeCom_TEC_Mon_ErrorInstance(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Mon_ErrorInstance");
        printFloatFields(fFields);
    }

    if(MeCom_TEC_Mon_ErrorParameter(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Mon_ErrorParameter");
        printFloatFields(fFields);
    }

    if(MeCom_TEC_Mon_ParallelActualOutputCurrent(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Mon_ParallelActualOutputCurrent");
        printFloatFields(fFields);
    }

    if(MeCom_TEC_Mon_DriverStatus(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Mon_DriverStatus");
        printLongFields(lFields);
    }

    if(MeCom_TEC_Mon_FanRelativeCoolingPower(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Mon_FanRelativeCoolingPower");
        printFloatFields(fFields);
    }
    
    if(MeCom_TEC_Mon_FanNominalFanSpeed(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Mon_FanNominalFanSpeed");
        printFloatFields(fFields);
    }
    
    if(MeCom_TEC_Mon_FanActualFanSpeed(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Mon_FanActualFanSpeed");
        printFloatFields(fFields);
    }
    
    if(MeCom_TEC_Mon_FanActualPwmLevel(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Mon_FanActualPwmLevel");
        printFloatFields(fFields);
    }

    if(MeCom_TEC_Mon_BasePlateTemperature(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Mon_BasePlateTemperature");
        printFloatFields(fFields);
    }

    if(MeCom_TEC_Mon_TemperatureIsStable(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Mon_TemperatureIsStable");
        printFloatFields(fFields);
    }
    
    //Tab Operation
    if(MeCom_TEC_Ope_OutputStageInputSelection(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Ope_OutputStageInputSelection");
        printFloatFields(fFields);
    }

    if(MeCom_TEC_Ope_OutputStageEnable(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Ope_OutputStageEnable");
        printLongFields(lFields);
    }

    if(MeCom_TEC_Ope_SetStaticCurrent(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Ope_SetStaticCurrent");
        printFloatFields(fFields);
    }

    if(MeCom_TEC_Ope_SetStaticVoltage(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Ope_SetStaticVoltage");
        printFloatFields(fFields);
    }

    if(MeCom_TEC_Ope_CurrentLimitation(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Ope_CurrentLimitation");
        printFloatFields(fFields);
    }

    if(MeCom_TEC_Ope_VoltageLimitation(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Ope_VoltageLimitation");
        printFloatFields(fFields);
    }

    if(MeCom_TEC_Ope_CurrentErrorThreshold(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Ope_CurrentErrorThreshold");
        printFloatFields(fFields);
    }

    if(MeCom_TEC_Ope_VoltageErrorThreshold(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Ope_VoltageErrorThreshold");
        printFloatFields(fFields);
    }

    if(MeCom_TEC_Ope_GeneralOperatingMode(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Ope_GeneralOperatingMode");
        printLongFields(lFields);
    }

    if(MeCom_TEC_Ope_DeviceAddress(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Ope_DeviceAddress");
        printLongFields(lFields);
    }

    if(MeCom_TEC_Ope_RS485CH1BaudRate(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Ope_RS485CH1BaudRate");
        printLongFields(lFields);
    }

    if(MeCom_TEC_Ope_RS485CH1ResponseDelay(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Ope_RS485CH1ResponseDelay");
        printLongFields(lFields);
    }

    if(MeCom_TEC_Ope_ComWatchDogTimeout(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Ope_ComWatchDogTimeout");
        printFloatFields(fFields);
    }

    //Tab Temperature Control
    
    if(MeCom_TEC_Tem_TargetObjectTemp(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Tem_TargetObjectTemp");
        printFloatFields(fFields);
    }

    if(MeCom_TEC_Tem_CoarseTempRamp(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Tem_CoarseTempRamp");
        printFloatFields(fFields);
    }

    if(MeCom_TEC_Tem_ProximityWidth(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Tem_ProximityWidth");
        printFloatFields(fFields);
    }

    if(MeCom_TEC_Tem_Kp(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Tem_Kp");
        printFloatFields(fFields);
    }

    if(MeCom_TEC_Tem_Ti(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Tem_Ti");
        printFloatFields(fFields);
    }

    if(MeCom_TEC_Tem_Td(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Tem_Td");
        printFloatFields(fFields);
    }

    if(MeCom_TEC_Tem_DPartDampPT1(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Tem_DPartDampPT1");
        printFloatFields(fFields);
    }

    if(MeCom_TEC_Tem_ModelizationMode(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Tem_ModelizationMode");
        printLongFields(lFields);
    }

    if(MeCom_TEC_Tem_PeltierMaxCurrent(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Tem_PeltierMaxCurrent");
        printFloatFields(fFields);
    }

    if(MeCom_TEC_Tem_PeltierMaxVoltage(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Tem_PeltierMaxVoltage");
        printFloatFields(fFields);
    }

    if(MeCom_TEC_Tem_PeltierCoolingCapacity(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Tem_PeltierCoolingCapacity");
        printFloatFields(fFields);
    }

    if(MeCom_TEC_Tem_PeltierDeltaTemperature(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Tem_PeltierDeltaTemperature");
        printFloatFields(fFields);
    }

    if(MeCom_TEC_Tem_PeltierPositiveCurrentIs(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Tem_PeltierPositiveCurrentIs");
        printLongFields(lFields);
    }

    if(MeCom_TEC_Tem_ResistorResistance(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Tem_ResistorResistance");
    }

    if(MeCom_TEC_Tem_ResistorMaxCurrent(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Tem_ResistorMaxCurrent");
    }
    
    //Tab Object Temperature
    if(MeCom_TEC_Obj_TemperatureOffset(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Obj_TemperatureOffset");
    }

    if(MeCom_TEC_Obj_TemperatureGain(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Obj_TemperatureGain");
    }

    if(MeCom_TEC_Obj_LowerErrorThreshold(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Obj_LowerErrorThreshold");
    }

    if(MeCom_TEC_Obj_UpperErrorThreshold(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Obj_UpperErrorThreshold");
    }

    if(MeCom_TEC_Obj_MaxTempChange(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Obj_MaxTempChange");
    }

    if(MeCom_TEC_Obj_NTCLowerPointTemperature(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Obj_NTCLowerPointTemperature");
    }

    if(MeCom_TEC_Obj_NTCLowerPointResistance(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Obj_NTCLowerPointResistance");
    }

    if(MeCom_TEC_Obj_NTCMiddlePointTemperature(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Obj_NTCMiddlePointTemperature");
    }

    if(MeCom_TEC_Obj_NTCMiddlePointResistance(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Obj_NTCMiddlePointResistance");
    }

    if(MeCom_TEC_Obj_NTCUpperPointTemperature(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Obj_NTCUpperPointTemperature");
    }

    if(MeCom_TEC_Obj_NTCUpperPointResistance(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Obj_NTCUpperPointResistance");
    }

    if(MeCom_TEC_Obj_StabilityTemperatureWindow(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Obj_StabilityTemperatureWindow");
    }

    if(MeCom_TEC_Obj_StabilityMinTimeInWindow(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Obj_StabilityMinTimeInWindow");
    }

    if(MeCom_TEC_Obj_StabilityMaxStabiTime(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Obj_StabilityMaxStabiTime");
    }
    
    if(MeCom_TEC_Obj_MeasLowestResistance(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Obj_MeasLowestResistance");
    }

    if(MeCom_TEC_Obj_MeasHighestResistance(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Obj_MeasHighestResistance");
    }

    if(MeCom_TEC_Obj_MeasTempAtLowestResistance(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Obj_MeasTempAtLowestResistance");
    }

    if(MeCom_TEC_Obj_MeasTempAtHighestResistance(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Obj_MeasTempAtHighestResistance");
    }
    
    //Tab Sink Temperature
    
    if(MeCom_TEC_Sin_TemperatureOffset(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Sin_TemperatureOffset");
    }

    if(MeCom_TEC_Sin_TemperatureGain(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Sin_TemperatureGain");
    }

    if(MeCom_TEC_Sin_LowerErrorThreshold(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Sin_LowerErrorThreshold");
    }

    if(MeCom_TEC_Sin_UpperErrorThreshold(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Sin_UpperErrorThreshold");
    }

    if(MeCom_TEC_Sin_MaxTempChange(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Sin_MaxTempChange");
    }

    if(MeCom_TEC_Sin_NTCLowerPointTemperature(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Sin_NTCLowerPointTemperature");
    }

    if(MeCom_TEC_Sin_NTCLowerPointResistance(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Sin_NTCLowerPointResistance");
    }

    if(MeCom_TEC_Sin_NTCMiddlePointTemperature(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Sin_NTCMiddlePointTemperature");
    }

    if(MeCom_TEC_Sin_NTCMiddlePointResistance(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Sin_NTCMiddlePointResistance");
    }

    if(MeCom_TEC_Sin_NTCUpperPointTemperature(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Sin_NTCUpperPointTemperature");
    }

    if(MeCom_TEC_Sin_NTCUpperPointResistance(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Sin_NTCUpperPointResistance");
    }

    if(MeCom_TEC_Sin_SinkTemperatureSelection(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Sin_SinkTemperatureSelection");
    }

    if(MeCom_TEC_Sin_FixedTemperature(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Sin_FixedTemperature");
    }

    if(MeCom_TEC_Sin_MeasLowestResistance(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Sin_MeasLowestResistance");
    }

    if(MeCom_TEC_Sin_MeasHighestResistance(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Sin_MeasHighestResistance");
    }

    if(MeCom_TEC_Sin_MeasTempAtLowestResistance(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Sin_MeasTempAtLowestResistance");
    }

    if(MeCom_TEC_Sin_MeasTempAtHighestResistance(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Sin_MeasTempAtHighestResistance");
    }

    //Tab Expert");
    if(MeCom_TEC_Exp_ObjMeasPGAGain(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Exp_ObjMeasPGAGain");
    }

    if(MeCom_TEC_Exp_ObjMeasCurrentSource(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Exp_ObjMeasCurrentSource");
    }

    if(MeCom_TEC_Exp_ObjMeasADCRs(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Exp_ObjMeasADCRs");
    }

    if(MeCom_TEC_Exp_ObjMeasADCCalibOffset(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Exp_ObjMeasADCCalibOffset");
    }

    if(MeCom_TEC_Exp_ObjMeasADCCalibGain(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Exp_ObjMeasADCCalibGain");
    }

    if(MeCom_TEC_Exp_ObjMeasSensorTypeSelection(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Exp_ObjMeasSensorTypeSelection");
    }

    if(MeCom_TEC_Exp_SinMeasADCRv(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Exp_SinMeasADCRv");
    }

    if(MeCom_TEC_Exp_SinMeasADCVps(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Exp_SinMeasADCVps");
    }

    if(MeCom_TEC_Exp_SinMeasADCCalibOffset(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Exp_SinMeasADCCalibOffset");
    }

    if(MeCom_TEC_Exp_SinMeasADCCalibGain(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Exp_SinMeasADCCalibGain");
    }

    //Tab Expert");
    if(MeCom_TEC_Exp_DisplayType(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Exp_DisplayType");
    }
    if(MeCom_TEC_Exp_DisplayLineDefText(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Exp_DisplayLineDefText");
    }
    if(MeCom_TEC_Exp_DisplayLineAltText(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Exp_DisplayLineAltText");
    }
    if(MeCom_TEC_Exp_DisplayLineAltMode(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Exp_DisplayLineAltMode");
    }
    //Tab Expert: Sub Tab PBC
    for(int32_t Inst=1; Inst<=8; Inst++)
    {
        if(MeCom_TEC_Exp_PbcFunction(Address, Inst, &lFields, MeGet)) 
        {
            Serial.println("MeCom_TEC_Exp_PbcFunction%d");
        }
    }
    for(int32_t Inst=1; Inst<=2; Inst++)
    {
        if(MeCom_TEC_Exp_ChangeButtonLowTemperature(Address, Inst, &fFields, MeGet)) 
        {
            Serial.println("MeCom_TEC_Exp_ChangeButtonLowTemperature%d");
        }
        if(MeCom_TEC_Exp_ChangeButtonHighTemperature(Address, Inst, &fFields, MeGet)) 
        {
            Serial.println("MeCom_TEC_Exp_ChangeButtonHighTemperature%d");
        }
        if(MeCom_TEC_Exp_ChangeButtonStepSize(Address, Inst, &fFields, MeGet)) 
        {
            Serial.println("MeCom_TEC_Exp_ChangeButtonStepSize%d");
        }
    }

    //Tab Expert: Sub Tab Fan
    for(int32_t Inst=1; Inst<=2; Inst++)
    {
        if(MeCom_TEC_Exp_FanControlEnable(Address, Inst, &lFields, MeGet)) 
        {
            Serial.println("MeCom_TEC_Exp_FanControlEnable%d");
        }
        if(MeCom_TEC_Exp_FanActualTempSource(Address, Inst, &lFields, MeGet)) 
        {
            Serial.println("MeCom_TEC_Exp_FanActualTempSource%d");
        }

        if(MeCom_TEC_Exp_FanTargetTemp(Address, Inst, &fFields, MeGet)) 
        {
            Serial.println("MeCom_TEC_Exp_FanTargetTemp%d");
        }
        if(MeCom_TEC_Exp_FanTempKp(Address, Inst, &fFields, MeGet)) 
        {
            Serial.println("MeCom_TEC_Exp_FanTempKp%d");
        }
        if(MeCom_TEC_Exp_FanTempTi(Address, Inst, &fFields, MeGet)) 
        {
            Serial.println("MeCom_TEC_Exp_FanTempTi%d");
        }
        if(MeCom_TEC_Exp_FanTempTd(Address, Inst, &fFields, MeGet)) 
        {
            Serial.println("MeCom_TEC_Exp_FanTempTd%d");
        }
        if(MeCom_TEC_Exp_FanSpeedMin(Address, Inst, &fFields, MeGet)) 
        {
            Serial.println("MeCom_TEC_Exp_FanSpeedMin%d");
        }
        if(MeCom_TEC_Exp_FanSpeedMax(Address, Inst, &fFields, MeGet)) 
        {
            Serial.println("MeCom_TEC_Exp_FanSpeedMax%d");
        }
        if(MeCom_TEC_Exp_FanSpeedKp(Address, Inst, &fFields, MeGet)) 
        {
            Serial.println("MeCom_TEC_Exp_FanSpeedKp%d");
        }
        if(MeCom_TEC_Exp_FanSpeedTi(Address, Inst, &fFields, MeGet)) 
        {
            Serial.println("MeCom_TEC_Exp_FanSpeedTi%d");
        }
        if(MeCom_TEC_Exp_FanSpeedTd(Address, Inst, &fFields, MeGet)) 
        {
            Serial.println("MeCom_TEC_Exp_FanSpeedTd%d");
        }
        if(MeCom_TEC_Exp_FanSpeedBypass(Address, Inst, &lFields, MeGet)) 
        {
            Serial.println("MeCom_TEC_Exp_FanSpeedBypass%d");
        }
    }
    if(MeCom_TEC_Exp_PwmFrequency(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Exp_PwmFrequency");
    }
    
    //Tab Expert");
    if(MeCom_TEC_Exp_MiscActObjectTempSource(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Exp_MiscActObjectTempSource");
    }

    if(MeCom_TEC_Exp_MiscDelayTillReset(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Exp_MiscDelayTillReset");
    }
    
    if(MeCom_TEC_Exp_MiscError108Delay(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Exp_MiscError108Delay");
    }

    //Other Parameters (Not directly displayed in the Service Software)
    if(MeCom_TEC_Oth_LiveEnable(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Oth_LiveEnable");
    }

    if(MeCom_TEC_Oth_LiveSetCurrent(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Oth_LiveSetCurrent");
    }

    if(MeCom_TEC_Oth_LiveSetVoltage(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Oth_LiveSetVoltage");
    }

    if(MeCom_TEC_Oth_SineRampStartPoint(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Oth_SineRampStartPoint");
    }

    if(MeCom_TEC_Oth_ObjectTargetTempSourceSelection(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Oth_ObjectTargetTempSourceSelection");
    }

    if(MeCom_TEC_Oth_ObjectTargetTemperature(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Oth_ObjectTargetTemperature");
    }

    if(MeCom_TEC_Oth_AtmAutoTuningStart(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Oth_AtmAutoTuningStart");
    }

    if(MeCom_TEC_Oth_AtmAutoTuningCancel(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Oth_AtmAutoTuningCancel");
    }

    if(MeCom_TEC_Oth_AtmThermalModelSpeed(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Oth_AtmThermalModelSpeed");
    }

    if(MeCom_TEC_Oth_AtmTuningParameter2A(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Oth_AtmTuningParameter2A");
    }

    if(MeCom_TEC_Oth_AtmTuningParameter2D(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Oth_AtmTuningParameter2D");
    }

    if(MeCom_TEC_Oth_AtmTuningParameterKu(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Oth_AtmTuningParameterKu");
    }

    if(MeCom_TEC_Oth_AtmTuningParameterTu(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Oth_AtmTuningParameterTu");
    }

    if(MeCom_TEC_Oth_AtmPIDParameterKp(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Oth_AtmPIDParameterKp");
    }

    if(MeCom_TEC_Oth_AtmPIDParameterTi(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Oth_AtmPIDParameterTi");
    }

    if(MeCom_TEC_Oth_AtmPIDParameterTd(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Oth_AtmPIDParameterTd");
    }

    if(MeCom_TEC_Oth_AtmSlowPIParameterKp(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Oth_AtmSlowPIParameterKp");
    }

    if(MeCom_TEC_Oth_AtmSlowPIParameterTi(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Oth_AtmSlowPIParameterTi");
    }

    if(MeCom_TEC_Oth_AtmPIDDPartDamping(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Oth_AtmPIDDPartDamping");
    }

    if(MeCom_TEC_Oth_AtmCoarseTempRamp(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Oth_AtmCoarseTempRamp");
    }

    if(MeCom_TEC_Oth_AtmProximityWidth(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Oth_AtmProximityWidth");
    }

    if(MeCom_TEC_Oth_AtmTuningStatus(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Oth_AtmTuningStatus");
    }

    if(MeCom_TEC_Oth_AtmTuningProgress(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Oth_AtmTuningProgress");
    }

    if(MeCom_TEC_Oth_LutTableStart(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Oth_LutTableStart");
    }

    if(MeCom_TEC_Oth_LutTableStop(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Oth_LutTableStop");
    }

    if(MeCom_TEC_Oth_LutTableStatus(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Oth_LutTableStatus");
    }

    if(MeCom_TEC_Oth_LutCurrentTableLine(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Oth_LutCurrentTableLine");
    }
    
    if(MeCom_TEC_Oth_LutTableIDSelection(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Oth_LutTableIDSelection");
    }

    if(MeCom_TEC_Oth_LutNrOfRepetitions(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Oth_LutNrOfRepetitions");
    }

    if(MeCom_TEC_Oth_PbcEnableFunction(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Oth_PbcEnableFunction");
    }

    if(MeCom_TEC_Oth_PbcSetOutputToPushPull(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Oth_PbcSetOutputToPushPull");
    }

    if(MeCom_TEC_Oth_PbcSetOutputStates(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Oth_PbcSetOutputStates");
    }

    if(MeCom_TEC_Oth_PbcReadInputStates(Address, 1, &lFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Oth_PbcReadInputStates");
    }

    if(MeCom_TEC_Oth_ExternalActualObjectTemperature(Address, 1, &fFields, MeGet)) 
    {
        Serial.println("MeCom_TEC_Oth_ExternalActualObjectTemperature");
    }
*/
}


void printFloatFields(MeParFloatFields fields)
{
  Serial.print("Value: ");
  Serial.println(fields.Value, 3);
  Serial.println("");
  Serial.print("Min:");
  Serial.println(fields.Min, 3);
  Serial.print("Max: ");
  Serial.println(fields.Max, 3);
}


void printLongFields(MeParLongFields fields)
{
  Serial.print("Value: 0x");
  Serial.println(fields.Value, HEX);
  Serial.println("");
  /*Serial.print("Min: ");
  Serial.println(fields.Min, 3);
  Serial.print("Max: ");
  Serial.println(fields.Max, 3);*/
}


void SetTargetObjectTemp(uint8_t Address)
{
                int32_t ParId   = 3000; //ConsoleIO_IntInput("Please Enter Parameter ID", 0, 65535, 0);
                int32_t Inst    = 1; //ConsoleIO_IntInput("Please Enter Instance", 1, 255, 1);
                MeParFloatFields Fields;

                if(MeCom_ParValuef(Address, ParId, Inst, &Fields, MeGetLimits))
                {
                    //ConsoleIO_SetColor(ConsoleIO_Reset);
                    Serial.println("SetTargetObjectTemp max min");
                    printFloatFields(Fields);
                    //Fields.Value = ConsoleIO_FloatInput("Please Enter the new float Value", Fields.Min, Fields.Max, 0);
                    Fields.Value = 26.75;  
                    //ConsoleIO_SetColor(ConsoleIO_Red);
                    if(MeCom_ParValuef(Address, ParId, Inst, &Fields, MeSet))
                    {
                        //ConsoleIO_SetColor(ConsoleIO_Green);
                        printf("Parameter ID: %d; Instance: %d; New Value: %f\n", ParId, Inst, Fields.Value);
                    }
                }
}
