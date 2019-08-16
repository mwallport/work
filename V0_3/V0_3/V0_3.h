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

const int MAX_HUMIDITY_SAMPLES  = 5;
typedef struct _humiditySamples
{
    int    index;
    float  sample[MAX_HUMIDITY_SAMPLES]; 
} humiditySamples_t;

typedef struct _humidityState
{
    runningStates       online;
    float               humidity;
    uint16_t            threshold;
    humiditySamples_t   sampleData;
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

void initSysStates(systemState& states)
{
    // chiller
    states.chiller.online      = offline;
    states.chiller.state       = stopped;
    states.chiller.temperature = 12.34;
    states.chiller.setpoint    = -38.2;

    // sensor
    states.sensor.humidity     = 58;
    states.sensor.threshold    = HUMIDITY_THRESHOLD;
    states.sensor.online       = offline;
    states.sensor.sampleData.index = 0;
    for(int i = 0; i < MAX_HUMIDITY_SAMPLES; i++)
        states.sensor.sampleData.sample[i] = 0.0;

    // tecs
    for(int i = 2; i < MAX_TEC_ADDRESS; i++)
    {
        states.tec[i - 2].online          = offline;
        states.tec[i - 2].state           = stopped;
        states.tec[i - 2].setpoint        = -23.5;
        states.tec[i - 2].temperature     = -10.4;
    }
}

#endif
