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
#define __DEBUG2_VIA_SERIAL__


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
#define SYSTEM_NRML_OFFSET      0   // 2 msgs, good and bad
#define SYSTEM_FAIL_OFFSET      1   // 2 msgs, good and bad
#define TEC_NRML_OFFSET         2   // 2 msgs, good and bad
#define TEC_FAIL_OFFSET         3   // 2 msgs, good and bad
#define CHILLER_NRML_OFFSET     4   // 2 msgs, good and bad
#define CHILLER_FAIL_OFFSET     5   // 2 msgs, good and bad
#define HUMIDITY_NRML_OFFSET    6   // 2 msgs, good and bad
#define HUMIDITY_FAIL_OFFSET    7   // 2 msgs, good and bad
#define MAX_LCD_MSGS            (HUMIDITY_FAIL_OFFSET + 1)
#define MAX_MSG_DISPLAY_TIME    3000    // 3K ms ~= 3 secons per message



//
// functions to paint the LCD screen
//
// enums to be indexes into the lcdDisplay array
//
enum {
    // no status to display
    no_Status,
    // sys LCD status
    sys_Initializing, sys_Ready, sys_Running, sys_Shutdown, sys_StartFailed, sys_Failure,

    // tec status
    tec_Stopped, tec_Running, tec_ComFailure,

    // chiller
    chiller_Running, chiller_Stopped, chiller_ComFailure,

    // sensor
    sensor_humidityAndThreshold,
    sensor_HighHumidity, sensor_Failure,

    MAX_LCD_FUNC
};

// sys
void lcd_initializing();    // power-on, LCD splash screen
void lcd_ready();         // system was started
void lcd_running();         // system was started
void lcd_shutdown();        // system was shutdown
void lcd_startFailed();     // not all devices present or something
void lcd_systemFailure();   // some run-time fail, loss of comm w/ device, etc.

// tec
void lcd_tecsStopped();         // set point and current temp
void lcd_tecsRunning();         // set point and current temp
void lcd_tecComFailure();       // set point and current temp

// chiller
void lcd_chillerRunning();      //set point and current temp
void lcd_chillerStopped();      // running or stopped or fail
void lcd_chillerComFailure();   // can't communicate with the chiller

// sensor
void lcd_humidityAndThreshold();    // current humidity and threshold
void lcd_highHumidity();    // humidity alert, or mechanical failure
void lcd_sensorFailure();   // unable to communicate with the sensor


typedef void (*lcdFunc)(void);     

lcdFunc lcdFaces[MAX_LCD_FUNC] = 
{
    0,                  // no_Status to show
    lcd_initializing,   // system is starting
    lcd_ready,          // started and waiting for startUp command
    lcd_running,        // system is running
    lcd_shutdown,       // shutdown has been done either implicitely or explicitely
    lcd_startFailed,    // not all devices present upon startup
    lcd_systemFailure,  // some run-time failure - check tec, chiller, or humidity status
    lcd_tecsStopped,    // stopped and 3 set points 
    lcd_tecsRunning,    // running and 3 set points
    lcd_tecComFailure,  // can't communicate with one of the TECs asterisks and which TEC
    lcd_chillerRunning, // running - pump is on, etc. and temps
    lcd_chillerStopped, // not running - pump is off and temps
    lcd_chillerComFailure,  // can't communicate with the chiller
    lcd_humidityAndThreshold, // normal humidity
    lcd_highHumidity,   // high humidity
    lcd_sensorFailure
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

const int MAX_HUMIDITY_SAMPLES  = 6;
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

typedef struct _LCDState
{
    uint16_t        lcdFacesIndex[MAX_LCD_MSGS];// index into lcd faces array
    uint16_t        index;                      // index into lcdFacesIndex array
    unsigned long   prior_millis;               // prior time in millis()
} LCDState;

typedef struct _systemState
{
    chillerState    chiller;
    humidityState   sensor;
    tecState        tec[MAX_TEC_ADDRESS];
    LCDState        lcd;
} systemState;


#endif
