#include <SPI.h>
#include <Controllino.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal.h>      // LCD interface library
#include <Wire.h>               // I2C library, needed by SHTSensor
#include "SHTSensor.h"          // SHT sensor interface library
#include <AceButton.h>
#include <controlProtocol.h>
#include <huber.h>              // huber chiller communication library
#include <meerstetterRS485.h>   // meerstetter TEC communication library
#include "V0_3.h" 



//
// system status - updated by getStats and by set commands from control
//
systemState sysStates;

//
// LCD display
//
const int rs = 8, en = 10, d4 = 11, d5 = 12, d6 = 13, d7 = 42, rw=9;
LiquidCrystal lcd(rs, rw, en, d4, d5, d6, d7);

//
// temperature himidity sensor
//
SHTSensor sht;

//
// huber chiller communication
//
huber chiller(9600);

//
// meerstetter communication
//
meerstetterRS485 ms(9600);

//
// control PC communication
// assuming they will be address 0 and this program will be address 1
//
controlProtocol cp(1, 0, 9600);  // my address is 1, control address is 0


//
// configure the ace button
//
// The pin number attached to the button.
const int BUTTON_PIN = 3;
#ifdef ESP32
  // Different ESP32 boards use different pins
  const int LED_PIN = 2;
#else
  const int LED_PIN = LED_BUILTIN;
#endif

// LED states. Some microcontrollers wire their built-in LED the reverse.
const int LED_ON = HIGH;
const int LED_OFF = LOW;

//
// One button wired to the pin at BUTTON_PIN. Automatically uses the default
// ButtonConfig. The alternative is to call the AceButton::init() method in
// setup() below.
//
bool currentButtonOnOff = false;
volatile bool buttonOnOff = false;
ace_button::AceButton button(BUTTON_PIN);
void handleEvent(ace_button::AceButton*, uint8_t, uint8_t);

//
// Using the RTC to prevent getStatus() from failing while the rotary
// knob is being used.  Testing shows that when the ISR runs for the know,
// packet bytes of the chiller or TEC protocols disappear/are-dropped/or-something
// causing protocol failures and bogus 'shutDowns' due to poor communication.
// The idea is to not do getStatus() if the current time is too close to the last time
// the knob was used .. (also will be trying to disable interrupts during get status)
//
volatile int knobTime;


void setup()
{
    //
    // Wire.begin() must appear before the Serial.begin(...)
    // else Serial gets clipped
    //
    #if defined __DEBUG_VIA_SERIAL__ || defined __DEBUG2_VIA_SERIAL__
    Wire.begin();
    Serial.begin(115200);
    #else
    Wire.begin();
    #endif

    //
    // let the Serial port settle after loading Wire.begin()
    //
    delay(1000);


    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println(" -----------------------------------------------------------"); Serial.flush();
    Serial.println(" -------------------------> setup() <-----------------------"); Serial.flush();
    Serial.println(" -----------------------------------------------------------"); Serial.flush();
    #endif


    //
    // initialize the system states /stats - these are
    // used to hold temperatures, humidity, etc. and
    // used in responses to getStatusCmd from control
    // and holds the LCD messages
    //
    initSysStates(sysStates);


    //
    // initialize the real time clock
    //
    startRTC();
    
    
    //
    // start the humidity sensor
    //
    startSHTSensor();


    //
    // start the LCD and paint system initializing
    //
    startLCD();

    
    //
    // register the ISR for the digital encoder
    //
    initRotaryEncoder();
    

    //
    // initialize the button
    //
    initButton();


    //
    // let the Serial port settle after initButton()
    //
    delay(1000);
}


void loop()
{ 
    //
    // Should be called every 4-5ms or faster, for the default debouncing time
    // of ~20ms.
    button.check();


    //
    // getStatus will update LCD and sysStats data structure
    //
    getStatus();


    //
    // take commands from the
    // - the switch on front panel
    // - the controlling PC software
    //
    handleMsgs();


    //
    // update the LCD
    //
    manageLCD();
}


// all LCD calls are void, no way to tell if the LCD is up .. 
bool startLCD()
{
    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    // set up the LCD's number of columns and rows:
    lcd.begin(16, 2);
    lcd.noDisplay();
    lcd.clear();

    // paint something .. look alive
    manageLCD();

    return(true);
}


bool startSHTSensor()
{
    bool retVal = false;

    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif


    if( (sht.init()) && (sht.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM)) )
    {
        #ifdef __DEBUG2_VIA_SERIAL__
        Serial.print(__PRETTY_FUNCTION__);
        Serial.println("started SHT sensor");
        Serial.flush();
        #endif

        
        sysStates.sensor.online = online;
        sysStates.lcd.lcdFacesIndex[HUMIDITY_NRML_OFFSET]  = sensor_humidityAndThreshold;
        retVal  = true;
    } else
    {
        #ifdef __DEBUG_VIA_SERIAL__
        Serial.print(__PRETTY_FUNCTION__);
        Serial.println(" ERROR: unable to start sensor");
        Serial.flush();
        #endif

        // update the sysStates
        sysStates.sensor.online = offline;

        // update the LCD status for the sensor
        sysStates.lcd.lcdFacesIndex[HUMIDITY_FAIL_OFFSET]  = sensor_Failure;

        retVal  = false;
    }

    return(retVal);
}


// ----------------------------------------------------------
// return true only if
// - LCD is up
// - SHTSensor is up
// - Meersteters are up
// - chiller is running
//
bool startUp()
{
    bool retVal = true;


    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    //
    // paint 'Starting' on LCD
    //
    lcd_starting();


    //
    // start the chiller and the TECs
    //
    if( !(startChiller()) )
        retVal  = false;
    else if( !(startTECs()) )
        retVal = false;

    if( (retVal) )
    {
        // set the LCD to running
        sysStates.lcd.lcdFacesIndex[SYSTEM_NRML_OFFSET]    = sys_Running;
    } // else leave the FacesIndex what it was .. ?


    //
    // unable to start for some reason
    // TODO: shutDownSys() or not ?
    //
    //if( (false == retVal) )
        //shutDownSys();

    return(retVal);
}


//
// this function updates the LCD banner w/ failure messages
// to reflect problems
//
// run every 20 seconds
//
void getStatus()
{
    static unsigned long    lastGetStatusTime     = 0;
    unsigned long           currentGetStatusTime  = millis();
    int                     currTime, RTCdiff;


    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    Serial.print("currentGetStatusTime: "); Serial.println(currentGetStatusTime);
    Serial.print("lastGetStatusTime: "); Serial.println(lastGetStatusTime);
    #endif

    //
    // get the chiller and TEC's status every 20 seconds
    //
    if( (10000 < (currentGetStatusTime - lastGetStatusTime)) )
    {
        lastGetStatusTime = currentGetStatusTime;

        //
        // this doesn't matter .. what if the user is grabbing the know 'now' ... lamer ...
        //
        currTime = RTCSum();
        RTCdiff = (currTime - knobTime); // could be negative is the knob is a' turnin' ..

        if( (5 > (currTime - knobTime)) )
            return;

        // disable interrupts when doing protocol stuff - disable the knob state-lessly
        //noInterrupts();

        //
        // hmm I wonder if these use interrupts !!
        //
        getHumidityLevel();
        handleChillerStatus();
        handleTECStatus();

        //
        // if humidity too high, shutdown the chiller
        //
        // TODO: shutdown just the chiller or the TECs too ?
        //
        if( ((sysStates.sensor.humidity > sysStates.sensor.threshold) 
                                || (offline == sysStates.sensor.online)) )
        {
            if( (SHUTDOWN != getSystemStatus()) )
                shutDownSys();
        }

        //
        // set the LCD state for the overall system based on the
        // gathered information and enable/disable interrupts as
        // needed, i.e. shutdown or not
        //
        //if( (SHUTDOWN == getSystemStatus()) )
        //    interrupts();
        getSystemStatus();
    }
}


//
// shut everything down, update the system and LCD status
//
bool shutDownSys()
{
    bool    retVal  = true;


    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    //
    // paint system shutdown on LCD
    //
    lcd_shuttingDown();

    //
    // turn off the chiller
    //
    for(uint8_t i = 0; i < MAX_SHUTDOWN_ATTEMPTS; i++)
    {
        if( !(chiller.StopChiller()) )
            retVal  = false;
        else
        {
            retVal  = true;
            break;
        }
    }


    //
    // turn off the TECs - not checking return value here as we are dying anyway ??
    //
    for(uint8_t Address = 2; (Address <= MAX_TEC_ADDRESS); Address++)
    {
        if( !(ms.StopTEC(Address)) )
            retVal  = false;
    }


    //
    // update the LCD to reflect system shutDown
    //
    sysStates.lcd.lcdFacesIndex[SYSTEM_NRML_OFFSET]    = sys_Shutdown;
    sysStates.lcd.lcdFacesIndex[TEC_NRML_OFFSET]       = tec_Stopped;
    sysStates.lcd.lcdFacesIndex[CHILLER_NRML_OFFSET]   = chiller_Stopped;
    sysStates.lcd.lcdFacesIndex[HUMIDITY_NRML_OFFSET]  = sensor_humidityAndThreshold;

    return(retVal);
}


// ----------------------------------------------------------
// turn off echo and continuous output
// turn on the chiller
// return success if all these happen
//
bool startChiller()
{
    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    bool retVal = true;


    //
    // if humidity is too high, or there is a humidity sensor failure,
    // do not start the chiller - return failure
    //
    if( ((sysStates.sensor.humidity > sysStates.sensor.threshold) 
        || (offline == sysStates.sensor.online)) )
    {
        #ifdef __DEBUG_VIA_SERIAL__
        Serial.print(__PRETTY_FUNCTION__);
        Serial.println(" WARNING: not starting chiller, humidity too high or sensor failure");
        #endif

        retVal  = false;

    } else
    {
        if( !(chiller.StartChiller()) )
        {
            retVal  = false;
    
            #ifdef __DEBUG_VIA_SERIAL__
            Serial.print(__PRETTY_FUNCTION__); Serial.println(" unable to start chiller");
            #endif
    
            // update the sysStates
            sysStates.chiller.state     = stopped;
    
            // update the LCD
            sysStates.lcd.lcdFacesIndex[CHILLER_NRML_OFFSET]   = chiller_Stopped,
            sysStates.lcd.lcdFacesIndex[CHILLER_FAIL_OFFSET]   = chiller_ComFailure;
        } else
        {
            // update the sysStates
            sysStates.chiller.state     = running;
    
            // update the LCD
            sysStates.lcd.lcdFacesIndex[CHILLER_NRML_OFFSET]   = chiller_Running,
            sysStates.lcd.lcdFacesIndex[CHILLER_FAIL_OFFSET]   = no_Status;
        }
    }

    return(retVal);
}


// ---------------------------------------
// find all TECs, there should be 3
// - verify their addresses by fetching hw version
//          expecting ID s2, 3, and 4
// - turn them 'on'
// return true if all these things happen
//
bool startTECs()
{
    bool    retVal      = true;
    bool    TECsRunning = true;


    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif


    //
    // expecting the addresses to be 2, 3, and 4
    // the instance for these commands to be 1
    //
    // initialize the TEC LCD state
    sysStates.lcd.lcdFacesIndex[TEC_NRML_OFFSET]   = tec_Stopped;
    sysStates.lcd.lcdFacesIndex[TEC_FAIL_OFFSET]   = no_Status;

    for(uint8_t Address = 2; Address <= MAX_TEC_ADDRESS; Address++)
    {
        sysStates.tec[(Address - 2)].online   = offline;
        sysStates.tec[(Address - 2)].state    = stopped;    // we don't know

        // set to 2 to enable Live On/Off - otherwise its static on if we send 1 (i.e. always on) ??
        // try the Live On/Off command to enable the TECs .. ?
        if( !(ms.StartTEC(Address)) )
        {
            #ifdef __DEBUG_VIA_SERIAL__
            Serial.print(__PRETTY_FUNCTION__); Serial.print(" unable to start TEC: ");
            Serial.println(Address, DEC);
            Serial.flush();
            #endif

            TECsRunning = false;
            retVal  = false;
        } else
        {
            sysStates.tec[(Address - 2)].online   = online;
            sysStates.tec[(Address - 2)].state    = running;
        }
    }

    if( !(TECsRunning) )
    {
        // update the LCD
        sysStates.lcd.lcdFacesIndex[TEC_FAIL_OFFSET]   = tec_ComFailure;
    } else
    {
        sysStates.lcd.lcdFacesIndex[TEC_NRML_OFFSET]   = tec_Running;
    }

    return(retVal);
}


bool stopTECs()
{
    bool    retVal      = true;
    bool    TECsStopped = true;


    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif


    //
    // expecting the addresses to be 2, 3, and 4
    // the instance for these commands to be 1
    //
    sysStates.lcd.lcdFacesIndex[TEC_NRML_OFFSET]   = tec_Stopped;
    sysStates.lcd.lcdFacesIndex[TEC_FAIL_OFFSET]   = no_Status;
    for(uint8_t Address = 2; (Address <= MAX_TEC_ADDRESS); Address++)
    {
        if( !(ms.StopTEC(Address)) )
        {
            #ifdef __DEBUG_VIA_SERIAL__
            Serial.print("stopTECs unable to stop TEC ");
            Serial.println(Address, DEC);
            #endif

            retVal      = false;
            TECsStopped = false;

            //
            // update the LCD state at least
            //
            sysStates.lcd.lcdFacesIndex[TEC_FAIL_OFFSET]    = tec_ComFailure;
            sysStates.tec[(Address - 2)].online             = offline;
            sysStates.tec[(Address - 2)].state              = stopped;  // we don't know

        } else
        {
            sysStates.tec[(Address - 2)].online   = online;
            sysStates.tec[(Address - 2)].state    = stopped;
        }
    }

    return(retVal);
}


bool setTECTemp(uint16_t tecAddress, float temp)
{
    bool retVal = false;


    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    if( (MAX_TEC_ADDRESS >= tecAddress) )
    {
        if( (ms.SetTECTemp(tecAddress, temp)) )
        {
            retVal  = true;
        #ifdef __DEBUG_VIA_SERIAL__
        } else
        {
            Serial.print(__PRETTY_FUNCTION__);
            Serial.print(" ERROR: failed to set temp for TEC at addr: ");
            Serial.println(tecAddress);
            Serial.flush();
        #endif
        }
    #ifdef __DEBUG_VIA_SERIAL__
    } else
    {
        Serial.print(__PRETTY_FUNCTION__);
        Serial.print(" ERROR: failed to set temp for TEC addr out of range: ");
        Serial.println(tecAddress);
        Serial.flush();
    #endif
    }

    return(retVal);
}


bool setChillerSetPoint(char* temp)
{
    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif


    return(chiller.SetSetPoint(temp));
}


void handleMsgs()
{
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    Serial.flush();
    #endif


    if( (currentButtonOnOff != buttonOnOff) )
    {
        currentButtonOnOff = buttonOnOff;
        
        #ifdef __DEBUG_VIA_SERIAL__
        Serial.print("button press happening switching ");
        if( (true == currentButtonOnOff) ) Serial.println("on"); else Serial.println("off");
        Serial.flush();
        #endif

        if( (true == currentButtonOnOff) )
            startUp();
        else
            shutDownSys();
    }
    
 
    //
    // check for message from control PC
    //
    if( (getMsgFromControl()) )
    {
        //
        // cp.m_buff has the message just received, process that message
        //
        switch( (cp.getMsgId()) )
        {
            case startUpCmd:                // start the chiller, TECs, and sensor
            {
                handleStartUpCmd();
                break;
            }

            case shutDownCmd:               // shutdown the chiller, TECs, and sensor
            {
                handleShutDownCmd();
                break;
            }

            case getStatusCmd:               // fetch the status of chiller, all TECs, and humidity sensor
            {
                handleGetStatusCmd();
                break;
            };

            case setHumidityThreshold:       // get the humidity threshold
            {
                handleSetHumidityThreshold();
                break;
            };

            case getHumidityThreshold:       // set the humidity threshold
            {
                handleGetHumidityThreshold();
                break;
            };

            case getHumidity:                // get current humidity and temperature
            {
                handleGetHumidity();
                break;
            };

            case setTECTemperature:          // target TEC m_address and temp
            {
                handleSetTECTemperature();
                break;
            };

            case getTECTemperature:          // target TEC m_address and temp
            {
                handleGetTECTemperature();
                break;
            };

            case startChillerMsg:
            {
                handleStartChillerMsg();
                break;
            };

            case stopChiller:
            {
                handleStopChiller();
                break;
            };

            case getChillerInfo:
            {
                handleGetChillerInfo();
                break;
            };

            case setChillerTemperature:      // target TEC m_address and temp
            {
                handleSetChillerTemperature();
                break;
            };

            case getChillerTemperature:      // target TEC m_address and temp
            {
                handleGetChillerTemperature();
                break;
            };

            case getTECInfoMsg:
            {
                handlGetTECInfo();
                break;
            };

            case enableTECs:                 // turn on all TECs
            {
                handleEnableTECs();
                break;
            };

            case disableTECs:                // turn off all TECs
            {
                handleDisableTECs();
                break;
            };

            default:
            {
                // send NACK
                sendNACK();
                break;
            }
        }
    } // else no message from control
}
    

void lcd_initializing()
{
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    lcd.noDisplay();
    lcd.clear();
    lcd.home();
    lcd.setCursor(0,1);
    lcd.print("SYS initializing");
    lcd.display();

}


void lcd_starting()
{
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    lcd.noDisplay();
    lcd.clear();
    lcd.home();
    lcd.setCursor(0,0);
    lcd.print("**** SYSTEM ****");
    lcd.setCursor(0,1);
    lcd.print("*** STARTING ***");
    lcd.display();
}


void lcd_ready()
{
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    lcd.noDisplay();
    lcd.clear();
    lcd.home();
    lcd.setCursor(0,1);
    lcd.print("SYS ready");
    lcd.display();
}


void lcd_running()
{
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    lcd.noDisplay();
    lcd.clear();
    lcd.home();
    lcd.setCursor(0,1);
    lcd.print("SYS running");
    lcd.display();
}


void lcd_shutdown()
{
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    lcd.noDisplay();
    lcd.clear();
    lcd.home();
    lcd.setCursor(0,1);
    lcd.print("SYS shutdown");
    lcd.display();
}


void lcd_shuttingDown()
{
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    lcd.noDisplay();
    lcd.clear();
    lcd.home();
    lcd.setCursor(0,0);
    lcd.print("**** SYSTEM ****");
    lcd.setCursor(0,1);
    lcd.print("*SHUTTING DOWN *");
    lcd.display();
}


void lcd_startFailed()
{
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    lcd.noDisplay();
    lcd.clear();
    lcd.home();
    lcd.setCursor(0,1);
    lcd.print("SYS start fail");
    lcd.display();
}


void lcd_systemFailure()
{
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    lcd.noDisplay();
    lcd.clear();
    lcd.home();
    lcd.setCursor(0,0);
    lcd.print("SYS fail");
    lcd.setCursor(0,1);
    lcd.print("failure");
    lcd.display();
}


void lcd_tecsRunning()
{
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    lcd.noDisplay();
    lcd.clear();
    lcd.home();
    lcd.setCursor(0,0);
    lcd.print("TEC run         ");
    lcd.setCursor(9,0);
    lcd.print(sysStates.tec[0].setpoint,1);
    lcd.setCursor(0,1);
    lcd.print(sysStates.tec[1].setpoint,1);
    lcd.setCursor(9,1);
    lcd.print(sysStates.tec[2].setpoint,1);
    lcd.display();
}


void lcd_tecsStopped()
{
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    lcd.noDisplay();
    lcd.clear();
    lcd.home();
    lcd.setCursor(0,0);
    lcd.print("TEC stop        ");
    lcd.setCursor(9,0);
    lcd.print(sysStates.tec[0].setpoint,1);
    lcd.setCursor(0,1);
    lcd.print(sysStates.tec[1].setpoint,1);
    lcd.setCursor(9,1);
    lcd.print(sysStates.tec[2].setpoint,1);
    lcd.display();
}


void lcd_tecComFailure()
{
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    lcd.noDisplay();
    lcd.clear();
    lcd.home();
    lcd.setCursor(0,0);
    lcd.print("TEC COMM");
    lcd.setCursor(0,1);
    lcd.print("FAILURE");
    lcd.display();
}


void lcd_chillerRunning()
{
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif


    lcd.noDisplay();
    lcd.clear();
    lcd.home();
    lcd.setCursor(0,0);
    lcd.print("CHL RUNNING");
    lcd.setCursor(0,1);
    lcd.print("S: ");
    lcd.setCursor(3,1);
    lcd.print(sysStates.chiller.setpoint);
    lcd.setCursor(9,1);
    lcd.print("T: ");
    lcd.setCursor(12,1);
    lcd.print(sysStates.chiller.temperature);
    lcd.display();
}


void lcd_chillerStopped()
{
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif


    lcd.noDisplay();
    lcd.clear();
    lcd.home();
    lcd.setCursor(0,0);
    lcd.print("CHL STOPPED");
    lcd.setCursor(0,1);
    lcd.print("S: ");
    lcd.setCursor(3,1);
    lcd.print(sysStates.chiller.setpoint);
    lcd.setCursor(9,1);
    lcd.print("T: ");
    lcd.setCursor(12,1);
    lcd.print(sysStates.chiller.temperature);
    lcd.display();
}


void lcd_chillerComFailure()
{
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    lcd.noDisplay();
    lcd.clear();
    lcd.home();
    lcd.setCursor(0,1);
    lcd.print("CHL COM FAILURE");
    lcd.display();
}


void lcd_humidityAndThreshold()
{
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif


    lcd.noDisplay();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Humidity:       %");
    lcd.setCursor(11,0);
    lcd.print(sysStates.sensor.humidity);
    lcd.setCursor(0,1);
    lcd.print("Threshold:      %");
    lcd.setCursor(11,1);
    lcd.print(sysStates.sensor.threshold);
    lcd.display();
}


void lcd_highHumidity()
{
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif


    lcd.noDisplay();
    lcd.clear();
    lcd.home();
    lcd.setCursor(0,0);
    lcd.print("*** HUMIDITY ***");
    lcd.setCursor(0,1);
    lcd.print("**** ALERT *****");
    lcd.display();
}


void lcd_sensorFailure()
{
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif


    lcd.noDisplay();
    lcd.clear();
    lcd.home();
    lcd.setCursor(0,1);
    lcd.print("SENSOR FAILURE ");
    lcd.display();
}


bool getMsgFromControl()
{
    uint16_t        seqNum;
    msgHeader_t*    pMsgHeader;
    bool            retVal  = false;

    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    Serial.flush();
    #endif

    //
    // receive a command, wait 1 seconds for a command
    //
    if( (cp.doRxCommand(1000)) )
    {
        retVal  = true;
    }

    return(retVal);
}



void handleStartUpCmd()
{
    startUpCmd_t* pstartUpCmd = reinterpret_cast<startUpCmd_t*>(cp.m_buff);
    uint16_t    respLength; 
    uint16_t    result = 0;


    //
    // verify the received packet, here beause this is a startUpCmdCmd
    // check this is my address and the CRC is correct
    //
    if( (ntohs(pstartUpCmd->header.address.address)) == cp.m_myAddress)
    {
        //
        // verify the CRC
        //
        if( (cp.verifyMessage(len_startUpCmd_t,
                    ntohs(pstartUpCmd->crc), ntohs(pstartUpCmd->eop))) )
        {
            //
            // start the TECs, chiller, sensor ...
            //
            if( (startUp()) )
                result  = 1;

            respLength = cp.Make_startUpCmdResp(cp.m_peerAddress, cp.m_buff,
                result, pstartUpCmd->header.seqNum
            );

            //
            // use the CP object to send the response back
            // this function usese the cp.m_buff created above, just
            // need to send the lenght into the function
            //
            if( !(cp.doTxResponse(respLength)))
            {
                Serial.println(__PRETTY_FUNCTION__); Serial.print(" ERROR: failed to send response");
                Serial.flush();
            #ifdef __DEBUG2_VIA_SERIAL__
            } else
            {
                Serial.println(__PRETTY_FUNCTION__); Serial.print(" sent response");
                Serial.flush();
            #endif
            }
        #ifdef __DEBUG_VIA_SERIAL__
        } else
        {
            Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR: dropping packet bad CRC: ");
            Serial.println(ntohs(pstartUpCmd->crc));
            Serial.flush();
        #endif
        }

    #ifdef __DEBUG_VIA_SERIAL__
    } else
    {
        Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet");
        Serial.println(ntohs(pstartUpCmd->header.address.address));
        Serial.flush();
    #endif
    }
}


void handleShutDownCmd()
{
    shutDownCmd_t* pshutDownCmd = reinterpret_cast<shutDownCmd_t*>(cp.m_buff);
    uint16_t    respLength; 
    uint16_t    result = 0;


    //
    // verify the received packet, here beause this is a shutDownCmdCmd
    // check this is my address and the CRC is correct
    //
    if( (ntohs(pshutDownCmd->header.address.address)) == cp.m_myAddress)
    {
        //
        // verify the CRC
        //
        if( (cp.verifyMessage(len_shutDownCmd_t,
                        ntohs(pshutDownCmd->crc), ntohs(pshutDownCmd->eop))) )
        {
            //
            // call shutDown()
            //
            if( (shutDownSys()) )
                result  = 1;

            respLength = cp.Make_shutDownCmdResp(cp.m_peerAddress, cp.m_buff,
                result, pshutDownCmd->header.seqNum
            );

            //
            // use the CP object to send the response back
            // this function usese the cp.m_buff created above, just
            // need to send the lenght into the function
            //
            if( !(cp.doTxResponse(respLength)))
            {
                Serial.println(__PRETTY_FUNCTION__); Serial.print(" ERROR: failed to send response");
                Serial.flush();
            #ifdef __DEBUG2_VIA_SERIAL__
            } else
            {
                Serial.println(__PRETTY_FUNCTION__); Serial.print(" sent response");
                Serial.flush();
            #endif
            }
        #ifdef __DEBUG_VIA_SERIAL__
        } else
        {
            Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR: dropping packet bad CRC: ");
            Serial.println(ntohs(pshutDownCmd->crc));
            Serial.flush();
        #endif
        }

    #ifdef __DEBUG_VIA_SERIAL__
    } else
    {
        Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet");
        Serial.println(ntohs(pshutDownCmd->header.address.address));
        Serial.flush();
    #endif
    }
}


//
// all message handlers expect the controlProtocol object
// to have the received message
//
void handleGetStatusCmd()
{
    getStatus_t*    pgetStatus = reinterpret_cast<getStatus_t*>(cp.m_buff);
    uint16_t        respLength; 


    //
    // verify the received packet, here beause this is a getStatusCmd
    // check this is my address and the CRC is correct
    //
    if( (ntohs(pgetStatus->header.address.address)) == cp.m_myAddress)
    {
        //
        // verify the CRC
        //
        if( (cp.verifyMessage(len_getStatus_t,
                            ntohs(pgetStatus->crc), ntohs(pgetStatus->eop))) )
        {
            //
            // use the sysStates conentent to respond, send back the received seqNum
            //
            respLength = cp.Make_getStatusResp(cp.m_peerAddress, cp.m_buff,
                ((sysStates.sensor.humidity > sysStates.sensor.threshold) ? 1 : 0), // humidity alert
                ((true == TECsRunning()) ? 1 : 0),                                    // TECs running
                ((running == sysStates.chiller.state) ? 1 : 0),                     // chiller running
                pgetStatus->header.seqNum
            );

            //
            // use the CP object to send the response back
            // this functoin usese the cp.m_buff created above, just
            // need to send the lenght into the function
            //
            if( !(cp.doTxResponse(respLength)))
            {
                Serial.println(__PRETTY_FUNCTION__); Serial.print(" ERROR: failed to send response");
                Serial.flush();
            #ifdef __DEBUG2_VIA_SERIAL__
            } else
            {
                Serial.println(__PRETTY_FUNCTION__); Serial.print(" sent response");
                Serial.flush();
            #endif
            }
        #ifdef __DEBUG_VIA_SERIAL__
        } else
        {
            Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR: dropping packet bad CRC: ");
            Serial.println(ntohs(pgetStatus->crc), HEX);
            Serial.flush();
        #endif
        }

    #ifdef __DEBUG_VIA_SERIAL__
    } else
    {
        Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet: ");
        Serial.println(ntohs(pgetStatus->header.address.address));
        Serial.flush();
    #endif
    }
}



void handleSetHumidityThreshold()
{
    setHumidityThreshold_t* psetHumidityThreshold = reinterpret_cast<setHumidityThreshold_t*>(cp.m_buff);
    uint16_t                respLength; 


    //
    // verify the received packet, here beause this is a setHumidityThreshold
    // check this is my address and the CRC is correct
    //
    if( (ntohs(psetHumidityThreshold->header.address.address)) == cp.m_myAddress)
    {
        //
        // verify the CRC
        //
        if( (cp.verifyMessage(len_setHumidityThreshold_t,
            ntohs(psetHumidityThreshold->crc), ntohs(psetHumidityThreshold->eop))) )
        {
            //
            // pick up the new threshold
            //
            sysStates.sensor.threshold  = ntohs(psetHumidityThreshold->threshold);

            #ifdef __DEBUG2_VIA_SERIAL__
            Serial.print(__PRETTY_FUNCTION__); Serial.print( " set humidity threshold to: ");
            Serial.println(sysStates.sensor.threshold);
            #endif
            
            //
            // use the sysStates conentent to respond, send back the received seqNum
            //
            respLength = cp.Make_setHumidityThresholdResp(cp.m_peerAddress, cp.m_buff, 1, // success
                psetHumidityThreshold->header.seqNum
            );

            //
            // use the CP object to send the response back
            // this functoin usese the cp.m_buff created above, just
            // need to send the lenght into the function
            //
            if( !(cp.doTxResponse(respLength)))
            {
                Serial.println(__PRETTY_FUNCTION__); Serial.print(" ERROR: failed to send response");
                Serial.flush();
            #ifdef __DEBUG2_VIA_SERIAL__
            } else
            {
                Serial.println(__PRETTY_FUNCTION__); Serial.print(" sent response");
                Serial.flush();
            #endif
            }
        #ifdef __DEBUG_VIA_SERIAL__
        } else
        {
            Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR: dropping packet bad CRC: ");
            Serial.println(ntohs(psetHumidityThreshold->crc));
            Serial.flush();
        #endif
        }

    #ifdef __DEBUG_VIA_SERIAL__
    } else
    {
        Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet");
        Serial.println(ntohs(psetHumidityThreshold->header.address.address));
        Serial.flush();
    #endif
    }
}


void handleGetHumidityThreshold()
{
    getHumidityThreshold_t* pgetHumidityThreshold = reinterpret_cast<getHumidityThreshold_t*>(cp.m_buff);
    uint16_t                respLength; 


    //
    // verify the received packet, here beause this is a getHumidityThresholdCmd
    // check this is my address and the CRC is correct
    //
    if( (ntohs(pgetHumidityThreshold->header.address.address)) == cp.m_myAddress)
    {
        //
        // verify the CRC
        //
        if( (cp.verifyMessage(len_getHumidityThreshold_t,
                            ntohs(pgetHumidityThreshold->crc), ntohs(pgetHumidityThreshold->eop))) )
        {
            #ifdef __DEBUG2_VIA_SERIAL__
            Serial.print(__PRETTY_FUNCTION__); Serial.print( " returning humidity threshold: ");
            Serial.println(sysStates.sensor.threshold);
            #endif
            
            //
            // use the sysStates conentent to respond, send back the received seqNum
            //
            respLength = cp.Make_getHumidityThresholdResp(cp.m_peerAddress, cp.m_buff,
                sysStates.sensor.threshold,
                pgetHumidityThreshold->header.seqNum
            );

            //
            // use the CP object to send the response back
            // this functoin usese the cp.m_buff created above, just
            // need to send the lenght into the function
            //
            if( !(cp.doTxResponse(respLength)))
            {
                Serial.println(__PRETTY_FUNCTION__); Serial.print(" ERROR: failed to send response");
                Serial.flush();
            #ifdef __DEBUG2_VIA_SERIAL__
            } else
            {
                Serial.println(__PRETTY_FUNCTION__); Serial.print(" sent response");
                Serial.flush();
            #endif
            }
        #ifdef __DEBUG_VIA_SERIAL__
        } else
        {
            Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR: dropping packet bad CRC: ");
            Serial.println(ntohs(pgetHumidityThreshold->crc));
            Serial.flush();
        #endif
        }

    #ifdef __DEBUG_VIA_SERIAL__
    } else
    {
        Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet");
        Serial.println(ntohs(pgetHumidityThreshold->header.address.address));
        Serial.flush();
    #endif
    }
}


void handleGetHumidity()
{
    getHumidity_t* pgetHumidity = reinterpret_cast<getHumidity_t*>(cp.m_buff);
    uint16_t       respLength; 


    //
    // verify the received packet, here beause this is a getHumidityCmd
    // check this is my address and the CRC is correct
    //
    if( (ntohs(pgetHumidity->header.address.address)) == cp.m_myAddress)
    {
        //
        // verify the CRC
        //
        if( (cp.verifyMessage(len_getHumidity_t,
                        ntohs(pgetHumidity->crc), ntohs(pgetHumidity->eop))) )
        {
            #ifdef __DEBUG2_VIA_SERIAL__
            Serial.print(__PRETTY_FUNCTION__); Serial.print( " returning humidity: ");
            Serial.println(sysStates.sensor.humidity);
            #endif
            
            //
            // use the sysStates content to respond, send back the received seqNum
            //
            respLength = cp.Make_getHumidityResp(cp.m_peerAddress, cp.m_buff,
                sysStates.sensor.humidity,
                pgetHumidity->header.seqNum
            );

            //
            // use the CP object to send the response back
            // this functoin usese the cp.m_buff created above, just
            // need to send the lenght into the function
            //
            if( !(cp.doTxResponse(respLength)))
            {
                Serial.println(__PRETTY_FUNCTION__); Serial.print(" ERROR: failed to send response");
                Serial.flush();
            #ifdef __DEBUG2_VIA_SERIAL__
            } else
            {
                Serial.println(__PRETTY_FUNCTION__); Serial.print(" sent response");
                Serial.flush();
            #endif
            }
        #ifdef __DEBUG_VIA_SERIAL__
        } else
        {
            Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR: dropping packet bad CRC: ");
            Serial.println(ntohs(pgetHumidity->crc));
            Serial.flush();
        #endif
        }

    #ifdef __DEBUG_VIA_SERIAL__
    } else
    {
        Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet");
        Serial.println(ntohs(pgetHumidity->header.address.address));
        Serial.flush();
    #endif
    }
}


void handleSetTECTemperature()
{
    setTECTemperature_t* psetTECTemperature = reinterpret_cast<setTECTemperature_t*>(cp.m_buff);
    uint16_t    respLength; 
    uint16_t    tecAddress;
    uint16_t    result = 0;
    float       setPoint;


    //
    // verify the received packet, here beause this is a setTECTemperatureCmd
    // check this is my address and the CRC is correct
    //
    if( (ntohs(psetTECTemperature->header.address.address)) == cp.m_myAddress)
    {
        //
        // verify the CRC
        //
        if( (cp.verifyMessage(len_setTECTemperature_t,
                            ntohs(psetTECTemperature->crc), ntohs(psetTECTemperature->eop))) )
        {
            //
            // pick up the new temperature
            //
            // if the TECs addresses is out of range, send back failure
            //
            setPoint = atof(reinterpret_cast<char*>(psetTECTemperature->temperature));
            tecAddress = ntohs(psetTECTemperature->tec_address);

            #ifdef __DEBUG2_VIA_SERIAL__
            Serial.print(__PRETTY_FUNCTION__); Serial.print( " found setPoint:tecAddress ");
            Serial.print(setPoint,2);
            Serial.print(":");
            Serial.println(tecAddress);
            Serial.flush();
            #endif

            if( (MAX_TEC_ADDRESS >= tecAddress) )
            {
                if( (setTECTemp(tecAddress, setPoint)) )
                {
                    #ifdef __DEBUG2_VIA_SERIAL__
                    Serial.print(__PRETTY_FUNCTION__); Serial.print( " success set temp for TEC: ");
                    Serial.println(tecAddress);
                    Serial.flush();
                    #endif
                    result  = 1;

                    //
                    // update the TEC's set point temperature
                    //
                    sysStates.tec[(tecAddress - 2)].setpoint = setPoint;

                } else
                {
                    #ifdef __DEBUG_VIA_SERIAL__
                    Serial.print(__PRETTY_FUNCTION__); Serial.print( " ERROR: unable to temp for TEC: ");
                    Serial.println(tecAddress);
                    #endif
                    result  = 0;
                }
            } else
            {
                //
                // send back failure
                //
                result  = 0;

                #ifdef __DEBUG_VIA_SERIAL__
                Serial.print(__PRETTY_FUNCTION__); Serial.print( " ERROR: tec_address out of range: ");
                Serial.println(tecAddress);
                #endif
            }

            #ifdef __DEBUG2_VIA_SERIAL__
            Serial.print(__PRETTY_FUNCTION__); Serial.print( " sending back response for TEC: ");
            Serial.println(tecAddress);
            #endif

            Serial.flush();
            
            //
            // use the sysStates conentent to respond, send back the received seqNum
            //
            respLength = cp.Make_setTECTemperatureResp(cp.m_peerAddress, cp.m_buff,
                tecAddress, result, psetTECTemperature->header.seqNum
            );

            //
            // use the CP object to send the response back
            // this functoin usese the cp.m_buff created above, just
            // need to send the lenght into the function
            //
            if( !(cp.doTxResponse(respLength)))
            {
                Serial.println(__PRETTY_FUNCTION__); Serial.print(" ERROR: failed to send response");
                Serial.flush();
            #ifdef __DEBUG2_VIA_SERIAL__
            } else
            {
                Serial.println(__PRETTY_FUNCTION__); Serial.print(" sent response");
                Serial.flush();
            #endif
            }
        #ifdef __DEBUG_VIA_SERIAL__
        } else
        {
            Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR: dropping packet bad CRC: ");
            Serial.println(ntohs(psetTECTemperature->crc));
            Serial.flush();
        #endif
        }

    #ifdef __DEBUG_VIA_SERIAL__
    } else
    {
        Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet");
        Serial.println(ntohs(psetTECTemperature->header.address.address));
        Serial.flush();
    #endif
    }
}


void handleGetTECTemperature()
{
    getTECTemperature_t* pgetTECTemperature = reinterpret_cast<getTECTemperature_t*>(cp.m_buff);
    uint16_t    respLength; 
    uint16_t    tecAddress;
    uint16_t    result = 0;
    float       setPoint;


    //
    // verify the received packet, here beause this is a getTECTemperatureCmd
    // check this is my address and the CRC is correct
    //
    if( (ntohs(pgetTECTemperature->header.address.address)) == cp.m_myAddress)
    {
        //
        // verify the CRC
        //
        if( (cp.verifyMessage(len_getTECTemperature_t,
                                ntohs(pgetTECTemperature->crc), ntohs(pgetTECTemperature->eop))) )
        {
            //
            // if the TECs addresses is out of range, send back failure
            //
            tecAddress = ntohs(pgetTECTemperature->tec_address);

            if( (MAX_TEC_ADDRESS >= tecAddress) )
            {
                //
                // send back failure
                //
                result  = 0;

                #ifdef __DEBUG_VIA_SERIAL__
                Serial.print(__PRETTY_FUNCTION__); Serial.print( " ERROR: tec_address out of range: ");
                Serial.println(tecAddress);
                #endif
            } else
            {
                result = 1;
            }
            
            //
            // use the sysStates conentent to respond, send back the received seqNum
            //
            respLength = cp.Make_getTECTemperatureResp(cp.m_peerAddress, cp.m_buff,
                tecAddress, sysStates.tec[(tecAddress - 2)].setpoint, pgetTECTemperature->header.seqNum
            );

            //
            // use the CP object to send the response back
            // this function usese the cp.m_buff created above, just
            // need to send the lenght into the function
            //
            if( !(cp.doTxResponse(respLength)))
            {
                Serial.println(__PRETTY_FUNCTION__); Serial.print(" ERROR: failed to send response");
                Serial.flush();
            #ifdef __DEBUG2_VIA_SERIAL__
            } else
            {
                Serial.print(__PRETTY_FUNCTION__); Serial.print(" sent response: ");
                Serial.println(sysStates.tec[(tecAddress - 2)].temperature, 2);
                Serial.flush();
            #endif
            }
        #ifdef __DEBUG_VIA_SERIAL__
        } else
        {
            Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR: dropping packet bad CRC: ");
            Serial.println(ntohs(pgetTECTemperature->crc));
            Serial.flush();
        #endif
        }

    #ifdef __DEBUG_VIA_SERIAL__
    } else
    {
        Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet");
        Serial.println(ntohs(pgetTECTemperature->header.address.address));
        Serial.flush();
    #endif
    }
}


void handleStartChillerMsg()
{
    startChillerMsg_t* pstartChillerMsg = reinterpret_cast<startChillerMsg_t*>(cp.m_buff);
    uint16_t    respLength;
    uint16_t    result = 0;


    //
    // verify the received packet, here beause this is a startChillerMsgCmd
    // check this is my address and the CRC is correct
    //
    if( (ntohs(pstartChillerMsg->header.address.address)) == cp.m_myAddress)
    {
        //
        // verify the CRC
        //
        if( (cp.verifyMessage(len_startChillerMsg_t,
                    ntohs(pstartChillerMsg->crc), ntohs(pstartChillerMsg->eop))) )
        {
            //
            // start the TECs, chiller, sensor ...
            //
            if( (startChiller()) )
            {
                result  = 1;

            } else
            {
                #ifdef __DEBUG_VIA_SERIAL__
                Serial.println(__PRETTY_FUNCTION__); Serial.println(" ERROR: start chiller failed");
                Serial.flush();
                #endif
                result  = 0;
            }

            respLength = cp.Make_startChillerMsgResp(cp.m_peerAddress, cp.m_buff,
                result, pstartChillerMsg->header.seqNum
            );

            //
            // use the CP object to send the response back
            // this function usese the cp.m_buff created above, just
            // need to send the lenght into the function
            //
            if( !(cp.doTxResponse(respLength)))
            {
                Serial.println(__PRETTY_FUNCTION__); Serial.print(" ERROR: failed to send response");
                Serial.flush();
            #ifdef __DEBUG2_VIA_SERIAL__
            } else
            {
                Serial.println(__PRETTY_FUNCTION__); Serial.print(" sent response");
                Serial.flush();
            #endif
            }
        #ifdef __DEBUG_VIA_SERIAL__
        } else
        {
            Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR: dropping packet bad CRC: ");
            Serial.println(ntohs(pstartChillerMsg->crc));
            Serial.flush();
        #endif
        }

    #ifdef __DEBUG_VIA_SERIAL__
    } else
    {
        Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet");
        Serial.println(ntohs(pstartChillerMsg->header.address.address));
        Serial.flush();
    #endif
    }
}


void handleStopChiller()
{
    stopChiller_t* pstopChiller = reinterpret_cast<stopChiller_t*>(cp.m_buff);
    uint16_t    respLength;
    uint16_t    result = 0;


    //
    // verify the received packet, here beause this is a stopChillerCmd
    // check this is my address and the CRC is correct
    //
    if( (ntohs(pstopChiller->header.address.address)) == cp.m_myAddress)
    {
        //
        // verify the CRC
        //
        if( (cp.verifyMessage(len_stopChiller_t,
                    ntohs(pstopChiller->crc), ntohs(pstopChiller->eop))) )
        {
            //
            // stop chiller, this will cause getStatus to shut everything
            // else down too though
            //
            if(chiller.StopChiller())
            {
                result  = 1;
            } else
            {
                #ifdef __DEBUG_VIA_SERIAL__
                Serial.println(__PRETTY_FUNCTION__); Serial.println(" ERROR: stop chiller failed");
                Serial.flush();
                #endif
                result  = 0;
            }

            respLength = cp.Make_stopChillerResp(cp.m_peerAddress, cp.m_buff,
                result, pstopChiller->header.seqNum
            );

            //
            // use the CP object to send the response back
            // this function usese the cp.m_buff created above, just
            // need to send the lenght into the function
            //
            if( !(cp.doTxResponse(respLength)))
            {
                Serial.println(__PRETTY_FUNCTION__); Serial.print(" ERROR: failed to send response");
                Serial.flush();
            #ifdef __DEBUG2_VIA_SERIAL__
            } else
            {
                Serial.println(__PRETTY_FUNCTION__); Serial.print(" sent response");
                Serial.flush();
            #endif
            }
        #ifdef __DEBUG_VIA_SERIAL__
        } else
        {
            Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR: dropping packet bad CRC: ");
            Serial.println(ntohs(pstopChiller->crc));
            Serial.flush();
        #endif
        }

    #ifdef __DEBUG_VIA_SERIAL__
    } else
    {
        Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet");
        Serial.println(ntohs(pstopChiller->header.address.address));
        Serial.flush();
    #endif
    }
}


void handleGetChillerInfo()
{
    getChillerInfo_t* pgetChillerInfo = reinterpret_cast<getChillerInfo_t*>(cp.m_buff);
    uint16_t    respLength;
    uint16_t    result = 0;


    //
    // verify the received packet, here beause this is a getChillerInfoCmd
    // check this is my address and the CRC is correct
    //
    if( (ntohs(pgetChillerInfo->header.address.address)) == cp.m_myAddress)
    {
        //
        // verify the CRC
        //
        if( (cp.verifyMessage(len_getChillerInfo_t,
                    ntohs(pgetChillerInfo->crc), ntohs(pgetChillerInfo->eop))) )
        {
            result  = 1;

            respLength = cp.Make_getChillerInfoResp(cp.m_peerAddress, cp.m_buff,
                result, reinterpret_cast<const uint8_t*>(chiller.GetSlaveName()),
                MAX_SLAVE_NAME_LENGTH, pgetChillerInfo->header.seqNum
            );

            //
            // use the CP object to send the response back
            // this function usese the cp.m_buff created above, just
            // need to send the lenght into the function
            //
            if( !(cp.doTxResponse(respLength)))
            {
                Serial.println(__PRETTY_FUNCTION__); Serial.print(" ERROR: failed to send response");
                Serial.flush();
            #ifdef __DEBUG2_VIA_SERIAL__
            } else
            {
                Serial.println(__PRETTY_FUNCTION__); Serial.print(" sent response");
                Serial.flush();
            #endif
            }
        #ifdef __DEBUG_VIA_SERIAL__
        } else
        {
            Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR: dropping packet bad CRC: ");
            Serial.println(ntohs(pgetChillerInfo->crc));
            Serial.flush();
        #endif
        }

    #ifdef __DEBUG_VIA_SERIAL__
    } else
    {
        Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet");
        Serial.println(ntohs(pgetChillerInfo->header.address.address));
        Serial.flush();
    #endif
    }
}


void handleSetChillerTemperature()
{
    setChillerTemperature_t* psetChillerTemperature = reinterpret_cast<setChillerTemperature_t*>(cp.m_buff);
    uint16_t    respLength; 
    uint16_t    result = 0;
    float       setPoint;


    //
    // verify the received packet, here beause this is a setChillerTemperatureCmd
    // check this is my address and the CRC is correct
    //
    if( (ntohs(psetChillerTemperature->header.address.address)) == cp.m_myAddress)
    {
        //
        // verify the CRC
        //
        if( (cp.verifyMessage(len_setChillerTemperature_t,
                                ntohs(psetChillerTemperature->crc), ntohs(psetChillerTemperature->eop))) )
        {
            //
            // set the new chiller temperature
            //
            if( (setChillerSetPoint(reinterpret_cast<char*>(psetChillerTemperature->temperature))) )
            {
                #ifdef __DEBUG2_VIA_SERIAL__
                Serial.println(__PRETTY_FUNCTION__); Serial.print(" success setChillerSetPoint");
                Serial.flush();
                #endif
                result = 1;
            } else
            {
                #ifdef __DEBUG_VIA_SERIAL__
                Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR: failed to setChillerSetPoint to: ");
                Serial.println(reinterpret_cast<char*>(psetChillerTemperature->temperature));
                Serial.flush();
                #endif
                result  = 0;
            }

            respLength = cp.Make_setChillerTemperatureResp(cp.m_peerAddress, cp.m_buff,
                result, psetChillerTemperature->header.seqNum
            );

            //
            // use the CP object to send the response back
            // this function usese the cp.m_buff created above, just
            // need to send the lenght into the function
            //
            if( !(cp.doTxResponse(respLength)))
            {
                Serial.println(__PRETTY_FUNCTION__); Serial.print(" ERROR: failed to send response");
                Serial.flush();
            #ifdef __DEBUG2_VIA_SERIAL__
            } else
            {
                Serial.println(__PRETTY_FUNCTION__); Serial.print(" sent response");
                Serial.flush();
            #endif
            }
        #ifdef __DEBUG_VIA_SERIAL__
        } else
        {
            Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR: dropping packet bad CRC: ");
            Serial.println(ntohs(psetChillerTemperature->crc));
            Serial.flush();
        #endif
        }

    #ifdef __DEBUG_VIA_SERIAL__
    } else
    {
        Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet");
        Serial.println(ntohs(psetChillerTemperature->header.address.address));
        Serial.flush();
    #endif
    }
}


void handleGetChillerTemperature()
{
    getChillerTemperature_t* pgetChillerTemperature = reinterpret_cast<getChillerTemperature_t*>(cp.m_buff);
    uint16_t    respLength; 
    uint16_t    result = 0;


    //
    // verify the received packet, here beause this is a getChillerTemperatureCmd
    // check this is my address and the CRC is correct
    //
    if( (ntohs(pgetChillerTemperature->header.address.address)) == cp.m_myAddress)
    {
        //
        // verify the CRC
        //
        if( (cp.verifyMessage(len_getChillerTemperature_t,
                                ntohs(pgetChillerTemperature->crc), ntohs(pgetChillerTemperature->eop))) )
        {
            //
            // chiller informaion is gotton during getStatus
            //
            respLength = cp.Make_getChillerTemperatureResp(cp.m_peerAddress, cp.m_buff,
                sysStates.chiller.temperature, pgetChillerTemperature->header.seqNum
            );

            //
            // use the CP object to send the response back
            // this function usese the cp.m_buff created above, just
            // need to send the lenght into the function
            //
            if( !(cp.doTxResponse(respLength)))
            {
                Serial.println(__PRETTY_FUNCTION__); Serial.print(" ERROR: failed to send response");
                Serial.flush();
            #ifdef __DEBUG2_VIA_SERIAL__
            } else
            {
                Serial.println(__PRETTY_FUNCTION__); Serial.print(" sent response");
                Serial.flush();
            #endif
            }
        #ifdef __DEBUG_VIA_SERIAL__
        } else
        {
            Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR: dropping packet bad CRC: ");
            Serial.println(ntohs(pgetChillerTemperature->crc));
            Serial.flush();
        #endif
        }

    #ifdef __DEBUG_VIA_SERIAL__
    } else
    {
        Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet");
        Serial.println(ntohs(pgetChillerTemperature->header.address.address));
        Serial.flush();
    #endif
    }
}


void handlGetTECInfo()
{
    getTECInfoMsg_t*   pgetTECInfo = reinterpret_cast<getTECInfoMsg_t*>(cp.m_buff);
    uint16_t        respLength;
    uint16_t        result = 0;
    uint32_t        deviceType  = 0;
    uint32_t        hwVersion   = 0;
    uint32_t        fwVersion   = 0;
    uint32_t        serialNumber= 0;


    if( (ntohs(pgetTECInfo->header.address.address)) == cp.m_myAddress)
    {
        //
        // verify the CRC
        //
        if( (cp.verifyMessage(len_getTECInfoMsg_t,
                                ntohs(pgetTECInfo->crc), ntohs(pgetTECInfo->eop))) )
        {
            //
            // chiller informaion is gotton during getStatus
            //
            if( (getTECInfo(ntohs(pgetTECInfo->tec_address), &deviceType,
                                &hwVersion, &fwVersion, &serialNumber)) )
            {
                result  = 1;
            } else
            {
                #ifdef __DEBUG_VIA_SERIAL__
                Serial.println(__PRETTY_FUNCTION__); Serial.print(" ERROR: failed to startTECs");
                Serial.flush();
                #endif
                result  = 0;
            }

            respLength = cp.Make_getTECInfoMsgResp(cp.m_peerAddress, cp.m_buff, htons(pgetTECInfo->tec_address), result,
                                deviceType, hwVersion, fwVersion, serialNumber, pgetTECInfo->header.seqNum);

            //
            // use the CP object to send the response back
            // this function usese the cp.m_buff created above, just
            // need to send the lenght into the function
            //
            if( !(cp.doTxResponse(respLength)))
            {
                Serial.println(__PRETTY_FUNCTION__); Serial.print(" ERROR: failed to send response");
                Serial.flush();
            #ifdef __DEBUG2_VIA_SERIAL__
            } else
            {
                Serial.println(__PRETTY_FUNCTION__); Serial.print(" sent response");
                Serial.flush();
            #endif
            }
        #ifdef __DEBUG_VIA_SERIAL__
        } else
        {
            Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR: dropping packet bad CRC: ");
            Serial.println(ntohs(pgetTECInfo->crc));
            Serial.flush();
        #endif
        }

    #ifdef __DEBUG_VIA_SERIAL__
    } else
    {
        Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet");
        Serial.println(ntohs(pgetTECInfo->header.address.address));
        Serial.flush();
    #endif
    }
}


void handleEnableTECs()
{
    enableTECs_t* penableTECs = reinterpret_cast<enableTECs_t*>(cp.m_buff);
    uint16_t    respLength; 
    uint16_t    result = 0;


    //
    // verify the received packet, here beause this is a enableTECsCmd
    // check this is my address and the CRC is correct
    //
    if( (ntohs(penableTECs->header.address.address)) == cp.m_myAddress)
    {
        //
        // verify the CRC
        //
        if( (cp.verifyMessage(len_enableTECs_t,
                                ntohs(penableTECs->crc), ntohs(penableTECs->eop))) )
        {
            //
            // chiller informaion is gotton during getStatus
            //
            if( (startTECs()) )
            {
                result  = 1;
            } else
            {
                #ifdef __DEBUG_VIA_SERIAL__
                Serial.println(__PRETTY_FUNCTION__); Serial.print(" ERROR: failed to startTECs");
                Serial.flush();
                #endif
                result  = 0;
            }

            respLength = cp.Make_enableTECsResp(cp.m_peerAddress, cp.m_buff,
                result, penableTECs->header.seqNum
            );

            //
            // use the CP object to send the response back
            // this function usese the cp.m_buff created above, just
            // need to send the lenght into the function
            //
            if( !(cp.doTxResponse(respLength)))
            {
                Serial.println(__PRETTY_FUNCTION__); Serial.print(" ERROR: failed to send response");
                Serial.flush();
            #ifdef __DEBUG2_VIA_SERIAL__
            } else
            {
                Serial.println(__PRETTY_FUNCTION__); Serial.print(" sent response");
                Serial.flush();
            #endif
            }
        #ifdef __DEBUG_VIA_SERIAL__
        } else
        {
            Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR: dropping packet bad CRC: ");
            Serial.println(ntohs(penableTECs->crc));
            Serial.flush();
        #endif
        }

    #ifdef __DEBUG_VIA_SERIAL__
    } else
    {
        Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet");
        Serial.println(ntohs(penableTECs->header.address.address));
        Serial.flush();
    #endif
    }
}


void handleDisableTECs()
{
    disableTECs_t* pdisableTECs = reinterpret_cast<disableTECs_t*>(cp.m_buff);
    uint16_t    respLength; 
    uint16_t    result = 0;


    //
    // verify the received packet, here beause this is a disableTECsCmd
    // check this is my address and the CRC is correct
    //
    if( (ntohs(pdisableTECs->header.address.address)) == cp.m_myAddress)
    {
        //
        // verify the CRC
        //
        if( (cp.verifyMessage(len_disableTECs_t,
                                ntohs(pdisableTECs->crc), ntohs(pdisableTECs->eop))) )
        {
            //
            // chiller informaion is gotton during getStatus
            //
            if( (stopTECs()) )
            {
                result  = 1;
            } else
            {
                #ifdef __DEBUG_VIA_SERIAL__
                Serial.println(__PRETTY_FUNCTION__); Serial.println(" ERROR: failed to stopTECs");
                Serial.flush();
                #endif
                result  = 0;
            }

            respLength = cp.Make_disableTECsResp(cp.m_peerAddress, cp.m_buff,
                result, pdisableTECs->header.seqNum
            );

            //
            // use the CP object to send the response back
            // this function usese the cp.m_buff created above, just
            // need to send the lenght into the function
            //
            if( !(cp.doTxResponse(respLength)))
            {
                Serial.println(__PRETTY_FUNCTION__); Serial.print(" ERROR: failed to send response");
                Serial.flush();
            #ifdef __DEBUG2_VIA_SERIAL__
            } else
            {
                Serial.println(__PRETTY_FUNCTION__); Serial.print(" sent response");
                Serial.flush();
            #endif
            }
        #ifdef __DEBUG_VIA_SERIAL__
        } else
        {
            Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR: dropping packet bad CRC: ");
            Serial.println(ntohs(pdisableTECs->crc));
            Serial.flush();
        #endif
        }

    #ifdef __DEBUG_VIA_SERIAL__
    } else
    {
        Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet");
        Serial.println(ntohs(pdisableTECs->header.address.address));
        Serial.flush();
    #endif
    }
}


void sendNACK()
{
    msgHeader_t*    pmsgHeader = reinterpret_cast<msgHeader_t*>(cp.m_buff);
    uint16_t        respLength; 
    uint16_t        result = 0;


    respLength = cp.Make_NACK(cp.m_peerAddress, cp.m_buff, pmsgHeader->seqNum);

    //
    // use the CP object to send the response back
    // this function usese the cp.m_buff created above, just
    // need to send the lenght into the function
    //
    if( !(cp.doTxResponse(respLength)))
    {
        Serial.println(__PRETTY_FUNCTION__); Serial.print(" ERROR: failed to send response");
        Serial.flush();
    #ifdef __DEBUG2_VIA_SERIAL__
    } else
    {
        Serial.println(__PRETTY_FUNCTION__); Serial.print(" sent response");
        Serial.flush();
    #endif
    }
}


//
// initialize the system status data to reflect clean start up
// i.e. no failures found yet
//
void initSysStates(systemState& states)
{
    // chiller starts offline until queried via getStatus()
    states.chiller.online      = offline;
    states.chiller.state       = stopped;
    states.chiller.temperature = 0;
    states.chiller.setpoint    = 0;

    // sensor starts offline until discovered to be online via getStatus()
    states.sensor.humidity     = 0;
    states.sensor.threshold    = HUMIDITY_THRESHOLD;
    states.sensor.online       = offline;
    states.sensor.sampleData.index = 0;
    for(int i = 0; i < MAX_HUMIDITY_SAMPLES; i++)
        states.sensor.sampleData.sample[i] = 0.0;

    // tecs starts offline until discovered to be online via getStatus()
    for(int i = 2; i <= MAX_TEC_ADDRESS; i++)
    {
        states.tec[(i - 2)].online          = offline;
        states.tec[(i - 2)].state           = stopped;
        states.tec[(i - 2)].setpoint        = 0;
        states.tec[(i - 2)].temperature     = 0;
    }

    // lcd - initialize all messages
    for(int i = 0; i < MAX_LCD_MSGS; i++)
        sysStates.lcd.lcdFacesIndex[i] = 0;

    // pick up the millis() 'now' - something fishy about this, what will happen when
    // the counter rolls over ?  This program is suppossed to run for a long time..
    // TODO: find a better way - use timeofday or the RTC.. ?
    sysStates.lcd.prior_millis = millis();

    //
    // setup the 'good' lcd functions
    //
    sysStates.lcd.lcdFacesIndex[SYSTEM_NRML_OFFSET]    = sys_Initializing;
    sysStates.lcd.lcdFacesIndex[TEC_NRML_OFFSET]       = tec_Stopped;
    sysStates.lcd.lcdFacesIndex[CHILLER_NRML_OFFSET]   = chiller_Stopped;
    sysStates.lcd.lcdFacesIndex[HUMIDITY_NRML_OFFSET]  = sensor_humidityAndThreshold;

    sysStates.lcd.index = 0;
}


//
// use the global sysStates.lcd data structures to paint
// messages on the LCD
//
void manageLCD()
{
    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.println("---------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    Serial.flush();
    #endif

    //
    // if enough time has passed, display the current index
    //   
    if( (MAX_MSG_DISPLAY_TIME <= (millis() - sysStates.lcd.prior_millis)) )
    {
        //
        // update the LCD with the next message, if the next message
        // had been removed, advance to the next message
        //
        if( (0 == sysStates.lcd.lcdFacesIndex[sysStates.lcd.index]) )
        {
            //
            // find next non-zero message, adjust sysStates.lcd.index
            //
            for(int i = ((((sysStates.lcd.index) + 1) >= MAX_LCD_MSGS) ? 0 : (sysStates.lcd.index + 1));
                (i != sysStates.lcd.index);
                (((i + 1) >= MAX_LCD_MSGS) ? 0 : i + 1) )
            {
                if( (0 != sysStates.lcd.lcdFacesIndex[i]) )
                {
                    //
                    // updagte the index and call the LCD function
                    //
                    sysStates.lcd.index = i;
                    break;
                }
            }
        }

        //
        // update the LCD screen
        //
        // special case the humidity sensor - show humidity status in case of:
        // - humidity is too high
        // - SHT sensor failure
        // in either case the system will be in shutdown mode already or very soon
        //
        if( (sysStates.sensor.humidity > sysStates.sensor.threshold) )
        {
            lcdFaces[sensor_HighHumidity]();

        } else if( (offline == sysStates.sensor.online) )
        {
            lcdFaces[sensor_Failure]();

        } else // paint the current LCD screen
        {
            lcdFaces[sysStates.lcd.lcdFacesIndex[(sysStates.lcd.index)]]();
        }

        //
        // reload the count down timer
        //
        sysStates.lcd.prior_millis  = millis();

        //
        // and advance the index to the next message
        //
        sysStates.lcd.index =
            (((sysStates.lcd.index + 1) >= MAX_LCD_MSGS) ? 0 : (sysStates.lcd.index + 1));
    }
}


bool getHumidityLevel(void)
{
    bool retVal = true;


    //
    // update status and take a reading
    //
    sysStates.sensor.online = online;

    if( (sht.readSample()) )
    {
        //
        // update the sysStates for humidity - take an average to smooth spikes
        //
        sysStates.sensor.sampleData.sample[sysStates.sensor.sampleData.index] = sht.getHumidity();
        sysStates.sensor.humidity = 0;  // this will eventually be an average when have enough samples
        for(int i = 0; i < MAX_HUMIDITY_SAMPLES; i++)
        {
            if( ( 0 != sysStates.sensor.sampleData.sample[i]) )
                sysStates.sensor.humidity += (sysStates.sensor.sampleData.sample[i]);
            else
            {
                sysStates.sensor.humidity = 0;
                break;
            }
        }

        if( (0 != sysStates.sensor.humidity) )
        {
            // have enough samples, make an average
            sysStates.sensor.humidity /= (float)MAX_HUMIDITY_SAMPLES;

            #ifdef __DEBUG_VIA_SERIAL__
            Serial.print(__PRETTY_FUNCTION__); Serial.print(" took an average for humidity: ");
            Serial.print(sysStates.sensor.humidity, 2); Serial.println("%"); Serial.flush();
            #endif

        } else
            // not enough samples - take the raw reading
            sysStates.sensor.humidity = sysStates.sensor.sampleData.sample[sysStates.sensor.sampleData.index];

        //
        // update the index for the next reading
        //
        sysStates.sensor.sampleData.index += 1;

        if( (sysStates.sensor.sampleData.index >= MAX_HUMIDITY_SAMPLES) )
            sysStates.sensor.sampleData.index = 0;

        if (sysStates.sensor.humidity > sysStates.sensor.threshold)
        {
            #ifdef __DEBUG_VIA_SERIAL__
            Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR: found high humidity: ");
            Serial.print(sysStates.sensor.humidity, 2); Serial.println("%"); Serial.flush();
            #endif

            // update the LCD
            sysStates.lcd.lcdFacesIndex[HUMIDITY_FAIL_OFFSET]   = sensor_HighHumidity;
            retVal  = false;
        } else
        {
            sysStates.lcd.lcdFacesIndex[HUMIDITY_FAIL_OFFSET]   = no_Status;
        }
    } else
    {
        #ifdef __DEBUG_VIA_SERIAL__
        Serial.print(__PRETTY_FUNCTION__); Serial.println(" ERROR: sensor not on-line");
        #endif

        //
        // update sysStates
        //
        sysStates.sensor.online = offline;

        // update the LCD
        sysStates.lcd.lcdFacesIndex[HUMIDITY_FAIL_OFFSET]   = sensor_Failure;

        retVal = false;
    }

    return(retVal);
}


bool TECsRunning()
{
    bool retVal = true;


    for(int i = 2; i <= MAX_TEC_ADDRESS; i++)
    {
        if( (running != sysStates.tec[(i - 2)].state) )
            retVal = false;
    }

    return(retVal);
}


void initRotaryEncoder()
{
    // encoder
    pinMode(pinA, INPUT);
    digitalWrite(pinA, HIGH);
    pinMode(pinB, INPUT);
    digitalWrite(pinB, HIGH);

    // cannot use LOW for the interrupt signal, using LOW allows the encoder
    // to hold the whole Controllino hostage if the encoder is held 
    // between 'slots', i.e. if you carefully hold the encoder in a middle
    // postion, not in a notch, the signal to the controllino will be high 
    // (or low) continuously, im guessing causing the ISR routine to be called
    // repeatedly so fast that nothing else runs.
    // But !  If we use CHANGE, HIGH, RISING, FALLING here, the aforementioned
    // problem does not happen but the performance of the knob turn is lessened
    attachInterrupt(digitalPinToInterrupt(pinA), digitalEncoderISR, CHANGE);

    virtualPosition = sysStates.sensor.threshold;
}


void digitalEncoderISR()
{
    static unsigned long  lastInterruptTime = 0;
    unsigned long         interruptTime     = millis();


    // update the guard time
    knobTime = RTCSum();

    // pick up the system value - may have changed from ctrl PC
    virtualPosition = sysStates.sensor.threshold;

    if( (5 < (interruptTime - lastInterruptTime)) )
    {
        // read the pins
        if( (LOW == digitalRead(pinB)) )
            --virtualPosition;
        else
            ++virtualPosition;

        virtualPosition = min(100, max(0, virtualPosition));

        lastInterruptTime = interruptTime;

        // update the system variable
        sysStates.sensor.threshold = virtualPosition;

        // update the LCD
        lcd.noDisplay();
        lcd.clear(); lcd.home();
        lcd.setCursor(0,0);  lcd.print("Humidity");
        lcd.setCursor(0,1);  lcd.print("Threshold: ");
        lcd.setCursor(11,1); lcd.print(virtualPosition);
        lcd.display();
    }
}


// The event handler for the button.
void handleEvent(ace_button::AceButton* /* button */, uint8_t eventType, uint8_t buttonState)
{
    #ifdef __DEBUG_VIA_SERIAL__
    // Print out a message for all events.
    Serial.print(F("handleEvent(): eventType: "));
    Serial.print(eventType);
    Serial.print(F("; buttonState: "));
    Serial.println(buttonState);
    #endif
    
    // Control the LED only for the Pressed and Released events.
    // Notice that if the MCU is rebooted while the button is pressed down, no
    // event is triggered and the LED remains off.
    switch (eventType)
    {
/*
        case AceButton::kEventPressed:
            break;
*/
        case ace_button::AceButton::kEventReleased:
            buttonOnOff != buttonOnOff;
            break;
    }
}


void handleChillerStatus(void)
{
    //
    // get all chiller information
    //
    if( !(chiller.GetAllChillerInfo()) )
    {
        #ifdef __DEBUG_VIA_SERIAL__
        Serial.print(__PRETTY_FUNCTION__);
        Serial.println(" WARINING: unable to GetAllChillerInfo");
        #endif

        //
        // update the chilller state to stopped (as it is not running)
        //
        sysStates.chiller.online                            = offline;
        sysStates.chiller.state                             = stopped;  // offline, we don't know
        sysStates.chiller.temperature                       = 0;
        sysStates.chiller.setpoint                          = 0;
        sysStates.lcd.lcdFacesIndex[CHILLER_NRML_OFFSET]    = chiller_Stopped;
        sysStates.lcd.lcdFacesIndex[CHILLER_FAIL_OFFSET]    = chiller_ComFailure;

    } else
    {
        //
        // update sysStates with what was feteched from GetAllChillerInfo
        //
        sysStates.chiller.online                            = online;
        if( ('O' == chiller.GetTempCtrlMode()) )
        {
          sysStates.chiller.state = stopped;
          sysStates.lcd.lcdFacesIndex[CHILLER_NRML_OFFSET]    = chiller_Stopped;
        }
        else
        {
          sysStates.chiller.state = running;
          sysStates.lcd.lcdFacesIndex[CHILLER_NRML_OFFSET]    = chiller_Running;
        }
        sysStates.chiller.temperature                       = chiller.GetInternalTempFloat();
        sysStates.chiller.setpoint                          = chiller.GetSetPointFloat();
        sysStates.lcd.lcdFacesIndex[CHILLER_FAIL_OFFSET]    = no_Status;

        #ifdef __DEBUG2_VIA_SERIAL__
        Serial.print("stored temperatures "); Serial.print(sysStates.chiller.temperature);
        Serial.print(" : "); Serial.println(sysStates.chiller.setpoint);
        Serial.print("chiller.GetTempCtrlMode(): "); Serial.println(chiller.GetTempCtrlMode());
        Serial.flush();
        #endif
    }
}


void handleTECStatus(void)
{
    bool    TECsOnline  = true;
    bool    TECsRunning = true;


    //
    // check all TECs running - get Device Status, possible status are
    //
    // assume they are running, if one if found not running, mark the group
    // as not running
    //
    for(uint8_t Address = 2; (Address <= MAX_TEC_ADDRESS); Address++)
    {
        sysStates.tec[(Address - 2)].state          = running;
        sysStates.tec[(Address - 2)].online         = online;

        //
        // always fetch the TEC's set point and object temperatures
        //
        if( !(ms.GetTECTemp(Address,
            &sysStates.tec[(Address - 2)].setpoint,
            &sysStates.tec[(Address - 2)].temperature)) )
        {
            #ifdef __DEBUG2_VIA_SERIAL__
            Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR:TEC ");
            Serial.print(Address, DEC); Serial.println(" unable to get temps");
            Serial.flush();
            #endif

            sysStates.tec[(Address - 2)].online   = offline;
            sysStates.tec[(Address - 2)].state    = stopped;
            #ifdef __DEBUG2_VIA_SERIAL__
        } else
        {
            Serial.print(__PRETTY_FUNCTION__); Serial.print(" found ");
            Serial.print(sysStates.tec[(Address - 2)].setpoint, 2);
            Serial.print(" : "); Serial.println(sysStates.tec[(Address - 2)].temperature, 2);
            Serial.flush();
            #endif
        }

        if( !(ms.TECRunning(Address)) )
        {
            #ifdef __DEBUG_VIA_SERIAL__
            Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: TEC ");
            Serial.print(Address); Serial.println(" is not running");
            Serial.flush();
            #endif

            //
            // keep track of whether all TECs are running
            //
            TECsRunning = false;

            // update sysStates
            sysStates.tec[(Address - 2)].state = stopped;

            //
            // check if we can still communicate with TECs
            //
            if( !(ms.TECPresent(Address)) )
            {
                #ifdef __DEBUG_VIA_SERIAL__
                Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR:TEC ");
                Serial.print(Address, DEC); Serial.println(" is not on-line");
                Serial.flush();
                #endif

                TECsOnline  = false;

                // update the sysStates
                sysStates.tec[(Address - 2)].online = offline;
            }
        }
    }


    //
    // if one TEC is down or bad, the overall status is bad
    //
    if( !(TECsOnline) )
        sysStates.lcd.lcdFacesIndex[TEC_FAIL_OFFSET]   = tec_ComFailure;
    else
        sysStates.lcd.lcdFacesIndex[TEC_FAIL_OFFSET]   = no_Status;

    if( !(TECsRunning) )
        sysStates.lcd.lcdFacesIndex[TEC_NRML_OFFSET]   = tec_Stopped;
    else
        sysStates.lcd.lcdFacesIndex[TEC_NRML_OFFSET]   = tec_Running;
}


// careful . . tec_address is a
bool getTECInfo(uint8_t tec_address, uint32_t* deviceType, uint32_t* hwVersion,
                                        uint32_t* fwVersion, uint32_t* serialNumber)
{
    bool retVal = true;


    if( !(ms.GetTECInfo(tec_address, deviceType, hwVersion, fwVersion, serialNumber)) )
    {
        retVal  = false;
        #ifdef __DEBUG_VIA_SERIAL__
        Serial.println(__PRETTY_FUNCTION__); Serial.println(" getTECInfo failed");
        #endif
    }

    return(retVal);
}


void initButton()
{
    // initialize built-in LED as an output
    pinMode(LED_PIN, OUTPUT);

    // Button uses the built-in pull up register.
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    
    // Configure the ButtonConfig with the event handler, and enable all higher
    // level events.
    ace_button::ButtonConfig* buttonConfig = button.getButtonConfig();
    buttonConfig->setEventHandler(handleEvent);
    buttonConfig->setFeature(ace_button::ButtonConfig::kFeatureClick);
    buttonConfig->setFeature(ace_button::ButtonConfig::kFeatureDoubleClick);
    buttonConfig->setFeature(ace_button::ButtonConfig::kFeatureLongPress);
    buttonConfig->setFeature(ace_button::ButtonConfig::kFeatureRepeatPress);

    // Check if the button was pressed while booting
    if (button.isPressedRaw()) {
        Serial.println(F("setup(): button was pressed while booting"));
    }
}


systemStatus getSystemStatus()
{
    systemStatus    retVal      = RUNNING;
    bool            TECsOnline  = true;
    bool            TECsRunning = true;
    

    // get the TECs status'
    for(int i = 2; i <= MAX_TEC_ADDRESS; i++)
    {
        if( (offline == sysStates.tec[(i - 2)].online) )
            TECsOnline = false;

        if( (stopped == sysStates.tec[(i - 2)].state) )
            TECsRunning = false;
    }

    //
    // use the humidity sensor, chiller, and TECs status
    //
    // the other 'bad' status for the other components shoudl already be
    // updated
    //
    if( (offline == sysStates.sensor.online
                    || offline == sysStates.chiller.online || false == TECsOnline) )
    {
        sysStates.lcd.lcdFacesIndex[SYSTEM_NRML_OFFSET]    = sys_Shutdown;
        retVal  = SHUTDOWN;
    } else if( ((online == sysStates.chiller.online && true == TECsOnline)
                && (stopped == sysStates.chiller.state || false == TECsRunning)) )
    {
        sysStates.lcd.lcdFacesIndex[SYSTEM_NRML_OFFSET]    = sys_Ready;
        retVal  = READY;
    } else
    {
        sysStates.lcd.lcdFacesIndex[SYSTEM_NRML_OFFSET]    = sys_Running;
        retVal  = RUNNING;
    }

    return(retVal);
}


void startRTC()
{
    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    
    //
    // real time clock
    //
    Controllino_RTC_init();
    Controllino_SetTimeDate(12,4,1,17,15,41,23); // TODO: compile time variables OK ?

    // initialize these globals here
    knobTime = RTCSum();
}


int RTCSum()
{
    int     vals[7] = {0,0,0,0,0,0,0};
    static int sum = 0;


    vals[0] = Controllino_GetDay();
    vals[1] = Controllino_GetWeekDay();
    vals[2] = Controllino_GetMonth();
    vals[3] = Controllino_GetYear();
    vals[4] = Controllino_GetHour();
    vals[5] = Controllino_GetMinute();
    vals[6] = Controllino_GetSecond();


    for(int i = 0; i < 7; i++)
    {
       sum += vals[i];
    }

    return(sum);
}
