// file RS232LSSeriesChiller.h
#ifndef _RS232_SOFTWARE_SERIAL_
#define _RS232_SOFTWARE_SERIAL_
#if defined(ARDUINO) && ARDUINO >= 100
     #include "Arduino.h"
   #else
     #include "WProgram.h"
   #endif
#include <SoftwareSerial.h>


const uint8_t MAX_BUFF_LENGTH   = 10;


class RS232LSSeriesChiller
{
    public:
    RS232LSSeriesChiller(uint32_t, uint32_t, uint32_t, uint32_t);
    virtual ~RS232LSSeriesChiller();
    
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
    
    SoftwareSerial RS232Soft;
};

#endif

