#ifndef __V0_3__
#define __V0_3__

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif


//
// enable deubug on Serial vai Serial.print(s)
//
//#define __DEBUG_VIA_SERIAL__


#define HUMIDITY_THRESHOLD 55
#define PIN_HW_ENABLE_n 8
#define SWITCH_PIN 9
#define CHILLER_SHUTOFF_COMMAND "CA@ 00000"
#define MAX_BUFF_LENGHT 10
#define MAX_TEC_ADDRESS 4
#define MAX_SHUTDOWN_ATTEMPTS 1

class debug
{
    public:
    debug() { Serial.println("__pretty_function__ entered"); }
    virtual ~debug() { Serial.println("__pretty_function__ exit"); }
};

#endif
