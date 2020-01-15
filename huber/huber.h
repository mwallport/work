// file huber.h
#ifndef _HUBER_
#define _HUBER_
#if defined(ARDUINO) && ARDUINO >= 100
     #include "Arduino.h"
   #else
     #include "WProgram.h"
   #endif
#include <SoftwareSerial.h>

//#define __DEBUG_PKT_RX__
//#define __DEBUG_PKT_TX__
//#define __DEBUG_HUBER2__
//#define __DEBUG_HUBER__
//#define __DEBUG_HUBER_ERROR__
#define __DEBUG_PKT_RX2__
#define __DOING_PP_COMMANDS__

const uint8_t MAX_COMMAND_RETRY             = 2;
const uint8_t MAX_BUFF_LENGTH               = 32; 
const uint8_t MAX_SLAVE_ID_LENGTH           = 2;
const uint8_t MAX_SLAVE_NAME_LENGTH         = 20;
const uint8_t MAX_LIMIT_LENGTH              = 4;
const uint8_t COMMAND_QUALIFIER_INDEX       = 4;
const uint8_t ADDRESS_INDEX                 = 2;
const uint8_t LENGTH_INDEX                  = 5;
const uint8_t DATA_START_INDEX              = 7;
const uint8_t CHKSUM_PLUS_ETX_LENGTH        = 3;    // check sum + '\r'
const uint8_t MIN_VERIFY_RESPONSE_LENGTH    = 11;
const uint8_t MIN_LIMIT_RESPONSE_LENGTH     = 20;
const uint8_t MIN_GENERAL_RESPONSE_LENGTH   = 22;
const uint8_t TEMP_CTRL_MODE_INDEX          = 7;
const uint8_t MAX_TEMP_CTRL_MODE_LENGTH     = 1;
const uint8_t ALARM_STATUS_INDEX            = 8;
const uint8_t MAX_ALARM_STATUS_LENGTH       = 1;
const uint8_t SETPOINT_STATUS_INDEX         = 9;
const uint8_t INTERNAL_TEMP_INDEX           = 13;
const uint8_t EXTERNAL_TEMP_INDEX           = 17;

#ifdef __DOING_PP_COMMANDS__
const uint8_t MAX_SET_POINT_LENGTH          = 8;
const uint8_t MAX_INTERNAL_TEMP_LENGTH      = 8;
const uint8_t MAX_EXTERNAL_TEMP_LENGTH      = 8;
#else
const uint8_t MAX_SET_POINT_LENGTH          = 4;
const uint8_t MAX_INTERNAL_TEMP_LENGTH      = 4;
const uint8_t MAX_EXTERNAL_TEMP_LENGTH      = 4;
#endif

//
// data structure to store results of calls to huber's 'General' command
//
typedef struct _HuberData
{
    char tempCtrlMode[MAX_TEMP_CTRL_MODE_LENGTH + 1];   // 
    char alarmStatus[MAX_ALARM_STATUS_LENGTH + 1];      // alarm count, not zero indicates alarms present
    char setPointTemp[MAX_SET_POINT_LENGTH + 1];        // current set point temp
    char internalTemp[MAX_INTERNAL_TEMP_LENGTH + 1];           // current internal temperature
    char externalTemp[MAX_EXTERNAL_TEMP_LENGTH + 1];           // external temp (not sure what this is)
} HuberData;


class huber
{
    public:
    huber(uint32_t);
    virtual ~huber();
    
    //
    // the huber commands
    //
    bool  sendVerifyCommand();
    bool  sendLimitCommand();
    bool  sendGeneralCommand();
    
    //
    // helper functions
    //
    bool  getChillerStatus();
    bool  GetAllChillerInfo();
    bool  StartChiller();
    bool  StopChiller();
    bool  ChillerRunning();
    bool  ChillerPresent();
    bool  SetSetPoint(const char*);
    float GetSetPointFloat() const;
    float GetInternalTempFloat() const;
    float GetExternalTempFloat() const;
    const char* GetSetPoint() const;
    const char* GetInternalTemp() const;
    const char* GetExternalTemp() const;
    const char* GetAlarms() const;
    const char* GetSlaveName() const;
    char GetTempCtrlMode() const;

    #ifdef __DOING_PP_COMMANDS__
    bool StopChiller_PP(uint32_t);
    bool StartChiller_PP(uint32_t);
    bool SetSetPoint_PP(const char*, uint32_t);
    bool GetAllChillerInfo_PP();
    bool DoReadSetPoint_PP(uint32_t);
    bool DoReadInternalActuatlValue_PP(uint32_t);
    bool DoGetTemperatureControlMode_PP(uint32_t);
    #endif

    protected:
    huber();
    huber(const huber&);
    huber operator=(const huber&);

    bool    TxCommand();
    bool    RxResponse(char**, uint32_t TimeoutMs);
    bool    verifyLengthAndCheckSum();
    void    setLengthAndCheckSum();
    void    ClearInputBuff();

    // software serial connection
    //SoftwareSerial mySerial2;

    // working buffers
    char    Buff[MAX_BUFF_LENGTH + 1];
    char    Bkup[MAX_BUFF_LENGTH + 1];  // for packet Tx retry

    // huber chiller data
    char    slaveID[MAX_SLAVE_ID_LENGTH + 1];
    char    slaveName[MAX_SLAVE_NAME_LENGTH + 1];
    char    upperSetPointLimit[MAX_LIMIT_LENGTH + 1];
    char    lowerSetPointLimit[MAX_LIMIT_LENGTH + 1];
    char    upperWorkingRangeLimit[MAX_LIMIT_LENGTH + 1];
    char    lowerWorkingRangeLimit[MAX_LIMIT_LENGTH + 1];
    HuberData   huberData;
};
#endif

