// file RS232LSSeriesChiller.h
#ifndef _RS232_SOFTWARE_SERIAL_
#define _RS232_SOFTWARE_SERIAL_
#if defined(ARDUINO) && ARDUINO >= 100
     #include "Arduino.h"
   #else
     #include "WProgram.h"
   #endif
#include <SoftwareSerial.h>

#define MAX_STARTUP_ATTEMPTS 1
#define MAX_SHUTDOWN_ATTEMPTS 1

#define __DEBUG_LSSERIES_OPERATION__

const uint8_t MAX_BUFF_LENGTH   = 10;


class RS232LSSeriesChiller
{
    public:
    RS232LSSeriesChiller(uint32_t, uint32_t, uint32_t, uint32_t);
    virtual ~RS232LSSeriesChiller();
    
    //
    // helper functions, groups of commands
    //
    bool  StartChiller();
    bool  StopChiller();
    bool  ChillerRunning();
    bool  ChillerPresent(char**);

    //
    // individual commands
    //
    bool  SetCommandEcho(char);
    bool  SetOnOff(char);
    bool  SetSetPoint(char*);
    bool  ReadSetPointTemperature(char**);
    bool  ReadTemperature(char**);
    bool  ReadTemperatureUnits(char**);
    bool  ReadStatus(char**);
    bool  ReadCompressorDischargeTemperature(char**);
    bool  ReadFaultStatus(char**);
    bool  ReadEvaporatorInletTemperature(char**);
    bool  ReadEvaporatorOutletTemperature(char**);
    bool  OutputContinuousDataStream(char);
    
    protected:
    RS232LSSeriesChiller();
    RS232LSSeriesChiller(const RS232LSSeriesChiller&);
    RS232LSSeriesChiller operator=(const RS232LSSeriesChiller&);
    bool    TxCommand();
    bool    RxResponse(char**, uint32_t TimeoutMs);
    char    Buff[MAX_BUFF_LENGTH + 1];
    
    //SoftwareSerial RS232Soft <-- not using this, using Serial2 on the MAXI board
};

#endif

