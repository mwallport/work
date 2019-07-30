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


#define HUMIDITY_THRESHOLD 55
#define PIN_HW_ENABLE_n 8
#define SWITCH_PIN 9
#define CHILLER_SHUTOFF_COMMAND "CA@ 00000"
#define MAX_BUFF_LENGHT 10
#define MAX_TEC_ADDRESS 2
#define MAX_SHUTDOWN_ATTEMPTS 1


//
// functions to paint the LCD screen
//
// enums to be indexes into the lcdDisplay array
//
enum { initializing, notAllDevicesPresent, chillerWarning, initFailed, sensorFailure, tecFailure, chillerFailure, sensorStatus, humidityAlert, humidityThreshold, chillerAlertInit, MAX_LCD_FUNC };

void lcd_initializing();
void lcd_notAllDevicesPresent();
void lcd_chillerWarning();
void lcd_initFailed();
void lcd_sensorFailure();
void lcd_tecFailure();
void lcd_chillerFailure();
void lcd_sensorStatus();
void lcd_humidityAlert();
void lcd_humidityThreshold();
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
    lcd_humidityThreshold,
    lcd_chillerAlertInit
};





#endif
