#ifndef __V0_3__
#define __V0_3__

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define HUMIDITY_THRESHOLD 55
#define PIN_HW_ENABLE_n 8
#define SWITCH_PIN 9
#define CHILLER_SHUTOFF_COMMAND "CA@ 00000"
#define MAX_BUFF_LENGHT 10
#define MAX_TEC_ADDRESS 4

class debug
{
    public:
    debug() { Serial.println("__pretty_function__ entered"); }
    virtual ~debug() { Serial.println("__pretty_function__ exit"); }
};

#endif

