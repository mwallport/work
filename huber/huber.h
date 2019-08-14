// file huber.h
#ifndef _HUBER_
#define _HUBER_
#if defined(ARDUINO) && ARDUINO >= 100
     #include "Arduino.h"
   #else
     #include "WProgram.h"
   #endif
#include <SoftwareSerial.h>

#define MAX_STARTUP_ATTEMPTS 1
#define MAX_SHUTDOWN_ATTEMPTS 1

//#define __DEBUG_PKT_RX__
//#define __DEBUG_PKT_TX__
//#define __DEBUG_FUNC_HUBER__
//#define __DEBUG_HUBER2__
//#define __DEBUG_HUBER__
//#define __DEBUG_HUBER_ERROR__

const uint8_t MAX_COMMAND_RETRY             = 3;
const uint8_t MAX_BUFF_LENGTH               = 30; 
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
const uint8_t MAX_SET_POINT_LENGTH          = 4;
const uint8_t INTERNAL_TEMP_INDEX           = 13;
const uint8_t MAX_INTERNAL_TEMP_LENGTH      = 4;
const uint8_t EXTERNAL_TEMP_INDEX           = 17;
const uint8_t MAX_EXTERNAL_TEMP_LENGTH      = 4;


class debug
{
    public:
    debug()
    {
        Serial.print(__PRETTY_FUNCTION__);
        Serial.flush();
        Serial.println(" enter");
        Serial.flush();
    }

    virtual ~debug()
    {
        Serial.print(__PRETTY_FUNCTION__);
        Serial.flush();
        Serial.println(" exit");
        Serial.flush();
    }
};


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
    huber(uint16_t, uint16_t, uint32_t);
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
    bool  InitChiller();
    bool  StartChiller();
    bool  StopChiller();
    bool  ChillerRunning();
    bool  ChillerPresent();
    bool  SetSetPoint(const char*);
    const char* GetSetPoint() const;
    const char* GetInternalTemp() const;
    const char* GetExternalTemp() const;
    const char* GetAlarms() const;


    protected:
    huber();
    huber(const huber&);
    huber operator=(const huber&);

    bool    TxCommand();
    bool    RxResponse(char**, uint32_t TimeoutMs);
    bool    verifyLengthAndCheckSum();
    void    setLengthAndCheckSum();

    // software serial connection
    //SoftwareSerial mySerial2;

    // working buffer
    char    Buff[MAX_BUFF_LENGTH + 1];

    // bool to determine whether the chiller has been started by this class
    bool    chillerInitialized;

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

