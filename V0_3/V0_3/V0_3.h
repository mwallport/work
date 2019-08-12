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
#define __DEBUG_VIA_SERIAL__


//
// constants
//
#define HUMIDITY_THRESHOLD      80
#define PIN_HW_ENABLE_n         8
#define SWITCH_PIN              9
#define MAX_BUFF_LENGHT         10
#define MAX_TEC_ADDRESS         2
#define MAX_TECs                (MAX_TEC_ADDRESS - 1)
#define MAX_SHUTDOWN_ATTEMPTS   1



//
// functions to paint the LCD screen
//
// enums to be indexes into the lcdDisplay array
//
enum { initializing, notAllDevicesPresent, chillerWarning, initFailed, sensorFailure, tecFailure, chillerFailure, sensorStatus, humidityAlert, currentHumidityAndThreshold, chillerAlertInit, MAX_LCD_FUNC };

void lcd_initializing();
void lcd_notAllDevicesPresent();
void lcd_chillerWarning();
void lcd_initFailed();
void lcd_sensorFailure();
void lcd_tecFailure();
void lcd_chillerFailure();
void lcd_sensorStatus();
void lcd_humidityAlert();
void lcd_currentHumidityAndThreshold();
void lcd_chillerAlertInit();

typedef void (*lcdFunc)(void);     

lcdFunc lcdFaces[MAX_LCD_FUNC] = 
{
    lcd_initializing,
    lcd_notAllDevicesPresent,
    lcd_chillerWarning,
    lcd_initFailed,
    lcd_sensorFailure,
    lcd_tecFailure,
    lcd_chillerFailure,
    lcd_sensorStatus,
    lcd_humidityAlert,
    lcd_currentHumidityAndThreshold,
    lcd_chillerAlertInit
};



//
// running status - updated by getStatus and set commands
//
typedef enum { offline, online, running, stopped, shutdown } runningStates;

typedef struct _chillerState
{
    runningStates   online;         // online or offline
    runningStates   state;          // running or stopped
    float           setpoint;       // current set point temperature
    float           temperature;    // current temperature
} chillerState;

typedef struct _humidityState
{
    runningStates   online;
    float           humidity;
    uint16_t        threshold;
} humidityState;

typedef struct _tecState
{
    runningStates   online;         // online or offline
    runningStates   state;          // running or stopped
    float           setpoint;       // current set point temperature
    float           temperature;    // current temperature
} tecState;

typedef struct _systemState
{
    chillerState    chiller;
    humidityState   sensor;
    tecState        tec[MAX_TEC_ADDRESS];
} systemState;

void initSysStates(systemState& _sysStates)
{
    _sysStates.chiller.online      = offline;
    _sysStates.chiller.state       = stopped;
    _sysStates.chiller.temperature = 100;
    _sysStates.chiller.setpoint    = 100;
    _sysStates.sensor.humidity     = 58;
    _sysStates.sensor.threshold    = HUMIDITY_THRESHOLD;
    _sysStates.sensor.online       = offline;
    for(int i = 2; i < MAX_TEC_ADDRESS; i++)
    {
        _sysStates.tec[i - 2].online          = offline;
        _sysStates.tec[i - 2].state           = stopped;
        _sysStates.tec[i - 2].setpoint        = 100;
        _sysStates.tec[i - 2].temperature     = 100;
    }
}

#endif
