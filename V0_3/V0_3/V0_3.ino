#include <Controllino.h>
#include <SoftwareSerial.h>
#include <huber.h>              // huber chiller communication library
#include <meerstetterRS485.h>   // meerstetter TEC communication library
#include <LiquidCrystal.h>      // LCD interface library
#include <Wire.h>               // I2C library, needed by SHTSensor
#include "SHTSensor.h"          // SHT sensor interface library
#include "V0_3.h" 
#include "controlProtocol.h"

//
// ---- globals ----
//

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
huber chiller(1, 0, 9600);

//
// meerstetter communication
//
meerstetterRS485 ms(14, 15);

//
// control PC communication
// assuming they will be address 0 and this program will be address 1
//
controlProtocol cp(1, 0);  // my address is 1, control address is 0


//
// LED and button states
//
int ledState = LOW;                 // the current state of the output pin
int buttonState = LOW;              // the current reading from the input pin
int lastButtonState = LOW;          // the previous reading from the input pin
unsigned long lastDebounceTime = 0; // the last time the output pin was toggled
unsigned long debounceDelay = 50;   // the debounce time; increase if the output flickers


void setup()
{
    initSysStates(sysStates);

    //
    // this must appear before the Serial.begin(...) else Serial gets clipped
    //
    #ifdef __DEBUG_VIA_SERIAL__
    Wire.begin();
    Serial.begin(9600);
    #else
    Wire.begin();
    #endif


    //
    // let the Serial port settle 
    //
    delay(1000);
    
    //
    // initialize board pins
    //
    pinMode(PIN_HW_ENABLE_n, OUTPUT);           // external LED .. ?
    pinMode(LED_BUILTIN, OUTPUT);               // built on board - defined by board type
    pinMode(SWITCH_PIN, INPUT_PULLUP);          // switch connected to this pin
    digitalWrite(LED_BUILTIN, ledState);        // turn off LED on board
    digitalWrite(PIN_HW_ENABLE_n, ledState);    // turn off external LED


    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.print(__PRETTY_FUNCTION__);
    Serial.println(" doing updateLCD(initializing)");
    Serial.flush();
    #endif
    
    //
    // paint 'Initializing' on the LCD
    //
    updateLCD(initializing);

/*  TODO: put this back

    //
    // verify communication with all devices
    //
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.print(__PRETTY_FUNCTION__);
    Serial.println(" checking allDevicesPresent()");
    #endif
    
    if( !(allDevicesPresent()) )
    {
        #ifdef __DEBUG_VIA_SERIAL__
        Serial.println("---------------------------------------");
        Serial.print(__PRETTY_FUNCTION__);
        Serial.println(" all devices not present, shutting down");
        #endif

        //
        // stop the chiller and the TECs and die
        //
        shutDownSys();

        //
        // TODO: die like this ?
        // update the LCD w/ "ALL DEVICES NOT PRESET", then die ?
        //
        updateLCDAndDie(notAllDevicesPresent);

    #ifdef __DEBUG_VIA_SERIAL__
    } else
    {
        Serial.println("---------------------------------------");
        Serial.print(__PRETTY_FUNCTION__);
        Serial.println(" all devices present");
    #endif
    }

    //
    // always start the chiller
    //
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.print(__PRETTY_FUNCTION__);
    Serial.println(" doing startChiller");
    #endif

    if( !(startChiller()) )
    {
        #ifdef __DEBUG_VIA_SERIAL__
        Serial.print(__PRETTY_FUNCTION__);
        Serial.println(" ERROR: unable to start the chiller, shutting down");
        Serial.flush();
        #endif

        shutDownSys();
        updateLCDAndDie(chillerFailure);    // call this last
    #ifdef __DEBUG_VIA_SERIAL__
    } else
    {
        Serial.print(__PRETTY_FUNCTION__);
        Serial.println(" chiller is started");
        Serial.flush();
    #endif
    }
*/
}


void loop()
{ 
/* TODO: put this  back
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.print(__PRETTY_FUNCTION__);
    Serial.println(" top of loop()");
    #endif


    //
    // getStatus will update LCD w/ what is 'bad', then shutDownAndDie()
    // will loop forever leaving the LCD update with what is 'bad'
    //
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.print(__PRETTY_FUNCTION__);
    Serial.println(" doing getStatus");
    #endif
    if( !(getStatus()) )
    {
        #ifdef __DEBUG_VIA_SERIAL__
        Serial.print(__PRETTY_FUNCTION__);
        Serial.println(" got bad status, shutting down");
        #endif

        shutDownAndDie();
    #ifdef __DEBUG_VIA_SERIAL__
    } else
    {
        Serial.print(__PRETTY_FUNCTION__);
        Serial.println(" getStatus is good");
    #endif
    }


    //
    // throttle loop() to run 1x per second
    //
    // the lcd_currentHumidityAndThreshold delays for 3 seconds, use this
    // delay to throttle the loop() - and always show humidity
    //
    updateLCD(currentHumidityAndThreshold);   


    //
    // take commands from the
    // - the switch
    // - the controlling PC software
    //
*/
    handleMsgs();
}


void lcd_chillerAlertInit()
{
    lcd.display();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("* CHILLER PUMP *");
    lcd.setCursor(0,1);
    lcd.print("  MUST BE ON!!! ");
}



int8_t switchOps()
{
    int8_t retVal   = 0;

    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    // TODO: get rid of this
    return(retVal);

    //
    // read the state of the switch into a local variable:
    //
    int reading = digitalRead(SWITCH_PIN);

    // check to see if you just pressed the button
    // (i.e. the input went from LOW to HIGH), and you've waited long enough
    // since the last press to ignore any noise:

    // If the switch changed, due to noise or pressing:
    if (reading != lastButtonState)
    {
        // reset the debouncing timer
        lastDebounceTime = millis();

        if ((millis() - lastDebounceTime) > debounceDelay)
        {
            // whatever the reading is at, it's been there for longer than the debounce
            // delay, so take it as the actual current state:
    
            // if the button state has changed:
            if (reading != buttonState)
            {
                buttonState = reading;
    
                // only toggle the LED if the new button state is HIGH
                if (buttonState == HIGH)
                {
                    ledState = HIGH;
                    retVal  = 1;  // switched on
                } else
                {
                    ledState = LOW;
                    updateLCD(chillerAlertInit);
                    retVal  = -1;  // switched off
                }
            }

            // set the LED:
            digitalWrite(LED_BUILTIN, ledState);
            digitalWrite(PIN_HW_ENABLE_n, ledState);

            //
            // save the reading. Next time through the loop, it'll be the lastButtonState:
            //
            lastButtonState = reading;
        }
    }  // else no change in the PIN (switch)

    return(retVal);
}


// all LCD calls are void, no way to tell if the LCD is up .. 
bool startLCD()
{
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    // set up the LCD's number of columns and rows:
    lcd.begin(16, 2);
    lcd.display();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("   deftDevise");
    lcd.setCursor(0,1);
    lcd.print("Firmware ver 1.0");

    return(true);
}


// -----------------------------------------
// print a message to the LCD and enter infinite loop (die)
// does not return
//
void updateLCD(uint32_t facesIndex)
{
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    //
    // call the LCD display function
    //
    lcdFaces[facesIndex]();
}


void updateLCDAndDie(uint32_t facesIndex)
{
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    //
    // update the LCD screen
    //
    updateLCD(facesIndex);


    //
    // and die
    //
    while(true) { delay(1000); }    // keep LCD display on and do nothing
}


bool startSHTSensor()
{
    bool retVal = false;

    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif


    if( (sht.init()) && (sht.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM)) ) // only supported by SHT3x
        retVal  = true;

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
    bool retVal = false;    

    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif


    //
    // TODO: chiller is always on - but send the command anyway
    // - start the TECs
    //
    if( (startSHTSensor()) && (startChiller()) && (startTECs()) )
    {
        retVal = true;
    }
    
    return(retVal);
}


//
// return false if any of these are true:
// - chiller not running
// - humidity is too high
// - TECs are not running
//
// this function updates the LCD and returns false
// the calling routine should then 'die' and leave the
// the LCD update in place
//
bool getStatus()
{
    bool    TECsOnline  = true;
    bool    TECsRunning = true;


    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    //
    // check chiller status is running
    //
    // if ChillerPresent is true - chiller is online and running
    //
    sysStates.chiller.state     = running;
    sysStates.chiller.online    = online;
    if( !(chiller.ChillerRunning()) )
    {
        #ifdef __DEBUG_VIA_SERIAL__
        Serial.print(__PRETTY_FUNCTION__);
        Serial.println(" WARINING: chiller not running");
        #endif

        //
        // update the chilller state to stopped (as it is not running)
        //
        sysStates.chiller.state    = stopped;

        //
        // verify the chiller is on-line
        //
        if( !(chiller.ChillerPresent()) )
        {
            #ifdef __DEBUG_VIA_SERIAL__
            Serial.print(__PRETTY_FUNCTION__);
            Serial.println(" ERROR: chiller not present");
            Serial.flush();
            #endif

            sysStates.chiller.online = offline;
        } else
        {
            sysStates.chiller.online = online;
        }

        //
        // update the LCD to show the same
        //
        updateLCD(chillerFailure);

        //
        // keep the LCD as bad chiller status
        //
        // TODO: put this back
        //return(false);
        return(true);
    } else
    {
        //
        // chiller.ChillerRunning does the (G)eneral command which fetches
        // the current set point and internal and external temperatures
        //
        // pick up the current chiller setpoint and running temperature
        //
        // TODO: verify picking up the correct data
        //
        sprintf(chiller.GetInternalTemp(), "%f", &sysStates.chiller.temperature);
        sprintf(chiller.GetSetPoint(), "%f", &sysStates.chiller.setpoint);
    }

    //
    // check all TECs running - get Device Status, possible status are
    //
    // assume they are running, if one if found not running, mark the group
    // as not running
    //
    for(uint8_t Address = 2; (Address <= MAX_TEC_ADDRESS); Address++)
    {
        sysStates.tec[Address - 2].state = running;
        sysStates.tec[Address - 2].online = online;

        if( !(ms.TECRunning(Address)) )
        {
            #ifdef __DEBUG_VIA_SERIAL__
            Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: TEC ");
            Serial.print(Address); Serial.println(" is not running");
            Serial.flush();
            #endif

            TECsRunning = false;
            sysStates.tec[Address - 2].state = stopped;

            //
            // check if we can still communicate with TECs
            //
            if( !(ms.TECPresent(Address)) )
            {
                #ifdef __DEBUG_VIA_SERIAL__
                Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR:TEC ");
                Serial.print(Address, DEC); Serial.println(" is offline");
                Serial.flush();
                #endif

                TECsOnline  = false;
                sysStates.tec[Address - 2].online = offline;
            }
        } else
        {
            sysStates.tec[Address - 2].state    = running;
            sysStates.tec[Address - 2].online   = online;

            //
            // fetch the TEC's set point and object temperatures
            //
            if( !(ms.GetTECTemp(Address, &sysStates.tec[Address - 2].setpoint,
                &sysStates.tec[Address - 2].temperature)) )
            {
                #ifdef __DEBUG_VIA_SERIAL__
                Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR:TEC ");
                Serial.print(Address, DEC); Serial.println(" unable to get temps");
                Serial.flush();
                #endif
            }
        }
    }


    if( !(TECsOnline && TECsRunning) )
    {
        updateLCD(tecFailure);
        return(false);  // keep the LCD as bad TEC status
    }


    //
    // check the humidity/temperature sensor - take an average over time
    //
    sysStates.sensor.online = online;
    if (sht.readSample())
    {
        //
        // update the sysStates
        //
        sysStates.sensor.humidity   = sht.getHumidity();

        if (sysStates.sensor.humidity > sysStates.sensor.threshold)
        {
            #ifdef __DEBUG_VIA_SERIAL__
            Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR: found high humidity: ");
            Serial.print(sysStates.sensor.humidity); Serial.println("%");
            #endif

            updateLCD(sensorStatus);

            //
            // keep the bad humidity status on the LCD
            //
            return(false);
        }
    } else
    {
        #ifdef __DEBUG_VIA_SERIAL__
        Serial.print(__PRETTY_FUNCTION__); Serial.println(" ERROR: sensor offline");
        #endif

        //
        // update sysStates
        //
        sysStates.sensor.online = offline;

        updateLCD(sensorFailure);

        //
        // keep the bad sensor status on the LCD
        //
        return(false);
    }

    return(true);
}


// ---------------------------------------------------------
//
// shutdown the chiller and the TECs
//
// do not update the LCD
void shutDownAndDie()
{
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    shutDownSys();

    // loop forever
    while(true) { delay(1000); };
}


//
// do not update the LCD
//
void shutDownSys()
{
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    char    buff[MAX_BUFF_LENGHT + 1];
    char*   pBuff = buff;


    //
    // turn off the chiller
    //
    for(uint8_t i = 0; i < MAX_SHUTDOWN_ATTEMPTS; i++)
    {
        chiller.StopChiller();
    }


    //
    // turn off the TECs - not checking return value here as we are dying anyway ??
    //
    for(uint8_t Address = 2; (Address <= MAX_TEC_ADDRESS); Address++)
    {
        ms.StopTEC(Address);
    }

    //
    // set the LED:
    //
    // TODO: switchOps handling - switch off the switch ?
    //
    digitalWrite(LED_BUILTIN, ledState);
    digitalWrite(PIN_HW_ENABLE_n, ledState);
}




// ----------------------------------------------------------
// turn off echo and continuous output
// turn on the chiller
// return success if all these happen
//
bool startChiller()
{
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    bool retVal = true;


    if( !(chiller.StartChiller()) )
    {
        //retVal  = false; TODO: put this back
        retVal  = true;

        #ifdef __DEBUG_VIA_SERIAL__
        Serial.println("startChiller unable to start chiller");
        #endif
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

    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif


    //
    // expecting the addresses to be 2, 3, and 4
    // the instance for these commands to be 1
    //
    for(uint8_t Address = 2; Address <= MAX_TEC_ADDRESS; Address++)
    {
        // set to 2 to enable Live On/Off - otherwise its static on if we send 1 (i.e. always on) ??
        // try the Live On/Off command to enable the TECs .. ?
        if( !(ms.StartTEC(Address)) )
        {
            retVal  = false;
            #ifdef __DEBUG_VIA_SERIAL__
            Serial.print("startTECs unable to start TEC ");
            Serial.println(Address, DEC);
            #endif
        } else
        {
            sysStates.tec[Address - 2].online   = online;
            sysStates.tec[Address - 2].state    = running;
        }
    }

    return(retVal);
}


bool stopTECs()
{
    bool    retVal      = true;

    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif


    //
    // expecting the addresses to be 2, 3, and 4
    // the instance for these commands to be 1
    //
    for(uint8_t Address = 2; (Address <= MAX_TEC_ADDRESS); Address++)
    {
        if( !(ms.StopTEC(Address)) )
        {
            retVal  = false;
            #ifdef __DEBUG_VIA_SERIAL__
            Serial.print("stopTECs unable to stop TEC ");
            Serial.println(Address, DEC);
            #endif
        } else
        {
            sysStates.tec[Address - 2].online   = online;
            sysStates.tec[Address - 2].state    = stopped;
        }
    }

    return(retVal);
}

//
// test communication with all devices
//
bool allDevicesPresent()
{
    bool retVal         = true;


    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif


    //
    // SHT sensor
    //
    if( !(sht.init()) )
    {
        #ifdef __DEBUG_VIA_SERIAL__
        Serial.print(__PRETTY_FUNCTION__);
        Serial.println(" ERROR: unable to start sensor");
        Serial.flush();
        #endif

        //
        // updat the sysStates
        //
        sysStates.sensor.online = offline;

        retVal  = false;
    } else
    {
        sysStates.sensor.online = online;
    }


    //
    // LCD
    //
    if(!startLCD())
    {
        #ifdef __DEBUG_VIA_SERIAL__
        Serial.print(__PRETTY_FUNCTION__);
        Serial.println(" ERROR: unable to start LCD");
        Serial.flush();
        #endif

        retVal  = false;
    }


    //
    // chiller
    //
    if( !(chiller.InitChiller()) )
    {
        #ifdef __DEBUG_VIA_SERIAL__
        Serial.print(__PRETTY_FUNCTION__);
        Serial.println(" ERROR: unable to initialize chiller");
        Serial.flush();
        #endif

        //
        // update the sysStates
        //
        sysStates.chiller.online    = offline;
        sysStates.chiller.state     = stopped;

        //retVal  = false; TODO: put this back
        retVal  = true;
    }


    //
    // all TECs
    //
    for(uint8_t Address = 2; (Address <= MAX_TEC_ADDRESS); Address++)
    {
        sysStates.tec[Address - 2].online    = online;
        sysStates.tec[Address - 2].state     = running;

        if( !(ms.TECPresent(Address)) )
        {
            #ifdef __DEBUG_VIA_SERIAL__
            Serial.print(__PRETTY_FUNCTION__);
            Serial.print(" unable to initialize TEC addr ");
            Serial.println(Address, DEC);
            Serial.flush();
            #endif

            //
            // update sysStats
            //
            sysStates.tec[Address - 2].online    = offline;
            sysStates.tec[Address - 2].state     = stopped;

            retVal  = false;
        }
    }

    return(retVal);
}


//
// TODO:  move this meetstetter stuff into the meerstetter library
//
bool setTECTemp(uint16_t tecAddress, float temp)
{
    bool retVal = false;


    #ifdef __DEBUG_VIA_SERIAL__
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
    #ifdef __DEBUG_VIA_SERIAL__
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


    //
    // TODO: implement handleMsgs
    //
    // for now, just do switchOps() and simulate message handling
    //
    // 
    // 0 no change, -1 switched off, 1 switched on
    //
    if( (1 == switchOps()) )
    {
        #ifdef __DEBUG_VIA_SERIAL__
        Serial.println("switchOps - switched on, doing startUp()");
        #endif

        startUp();

    } else if( (-1 == switchOps()) )
    {
        #ifdef __DEBUG_VIA_SERIAL__
        Serial.println("switchOps - switched off, doing shutDownSys()");
        #endif

        shutDownAndDie();
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

    lcd.display();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Initializing");
}


void lcd_notAllDevicesPresent()
{
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    lcd.display();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("ALL DEVICES ARE");
    lcd.setCursor(0,1);
    lcd.print("NOT PRESENT");
}


void lcd_chillerWarning()
{
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    lcd.display();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("  CHILLER PUMP  ");
    lcd.setCursor(0,1);
    lcd.print("   MUST BE ON   ");
}


void lcd_initFailed()
{
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    lcd.display();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(" INITIALIZATION ");
    lcd.setCursor(0,1);
    lcd.print("     FAILED     ");
}


void lcd_sensorFailure()
{
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    lcd.display();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(" SENSOR FAILURE ");
}


void lcd_tecFailure()
{
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    lcd.display();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("  TEC FAILURE  ");
}


void lcd_chillerFailure()
{
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    lcd.display();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("CHILLER FAILURE ");
}

void lcd_sensorStatus()
{
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    if( (sysStates.sensor.humidity > sysStates.sensor.threshold) )
    {
        // flash the alert
        lcd_humidityAlert();
        delay(3000);

        // then show the status
        lcd.display();
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("*Humidity:     %");  // TODO: add asterisk here ?
        lcd.setCursor(13, 0);
        lcd.print(sysStates.sensor.humidity);
        lcd.setCursor(0,1);
        lcd.print("Threshold:     %");
        lcd.setCursor(13,1);
        lcd.print(sysStates.sensor.threshold);
    } else
    {
        //  show the status
        lcd.display();
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Humidity:      %");
        lcd.setCursor(13, 0);
        lcd.print(sysStates.sensor.humidity);
        lcd.setCursor(0,1);
        lcd.print("Threshold:     %");
        lcd.setCursor(13,1);
        lcd.print(sysStates.sensor.threshold);
    }
}


void lcd_humidityAlert()
{
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    lcd.display();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("*** HUMIDITY ***");
    lcd.setCursor(0,1);
    lcd.print("**** ALERT *****");
}


void lcd_currentHumidityAndThreshold()
{
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    lcd.display();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Humidity:      %");
    lcd.setCursor(13, 0);
    lcd.print(sysStates.sensor.humidity);
    lcd.setCursor(0,0);
    lcd.print("Threshold:     %");
    lcd.setCursor(13,1);
    lcd.print(sysStates.sensor.threshold);
}


bool getMsgFromControl()
{
    uint16_t        seqNum;
    msgHeader_t*    pMsgHeader;
    bool            retVal  = false;
    

    //
    // receive a command, wait 5 seconds for a command
    //
    if( (cp.doRxCommand(5000)) )
    {
        retVal  = true;
    #ifdef __DEBUG_VIA_SERIAL__
    } else
    {
        Serial.print(__PRETTY_FUNCTION__);
        Serial.println(": did not get a command from control");
    #endif
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
    if( (ntohs(pstartUpCmd->address.address)) == cp.m_myAddress)
    {
        //
        // verify the CRC
        //
        if( (cp.verifyMessageCRC(len_startUpCmd_t, ntohs(pstartUpCmd->crc))) )
        {
            //
            // start the TECs, chiller, sensor ...
            //
            if( (startUp()) )
            {
                result  = 1;
            } else
            {
                #ifdef __DEBUG_VIA_SERIAL__
                Serial.println(__PRETTY_FUNCTION__); Serial.print(" ERROR: failed startUp");
                Serial.flush();
                #endif
                result  = 0;
            }

            respLength = cp.Make_startUpCmdResp(cp.m_peerAddress, cp.m_buff,
                result, htons(pstartUpCmd->seqNum)
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
            #ifdef __DEBUG_VIA_SERIAL__
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
        Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: not my address, dropping packet");
        Serial.println(ntohs(pstartUpCmd->address.address));
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
    if( (ntohs(pshutDownCmd->address.address)) == cp.m_myAddress)
    {
        //
        // verify the CRC
        //
        if( (cp.verifyMessageCRC(len_shutDownCmd_t, ntohs(pshutDownCmd->crc))) )
        {
            //
            // call shutDown()
            //
            shutDownSys(); // TODO: make this return bool
            result  = 1;

            respLength = cp.Make_shutDownCmdResp(cp.m_peerAddress, cp.m_buff,
                result, htons(pshutDownCmd->seqNum)
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
            #ifdef __DEBUG_VIA_SERIAL__
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
        Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: not my address, dropping packet");
        Serial.println(ntohs(pshutDownCmd->address.address));
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
    bool            TECsRunning = true;


    //
    // verify the received packet, here beause this is a getStatusCmd
    // check this is my address and the CRC is correct
    //
    if( (ntohs(pgetStatus->address.address)) == cp.m_myAddress)
    {
        //
        // verify the CRC
        //
        if( (cp.verifyMessageCRC(len_getStatus_t, ntohs(pgetStatus->crc))) )
        {
            //
            // check all the TECs - if at least one is not running, report
            // TECs not running (as a whole)
            //
            for(int Address = 2; Address < MAX_TEC_ADDRESS; Address++)
            {
                if( (sysStates.tec[Address - 2].state != running) )
                {
                    #ifdef __DEBUG_VIA_SERIAL__
                    Serial.print(__PRETTY_FUNCTION__);
                    Serial.print(" WARNING: TEC addr found not running: ");
                    Serial.println(Address); Serial.flush();
                    #endif

                    TECsRunning = false;
                }
            }

            //
            // use the sysStates conentent to respond, send back the received seqNum
            //
            respLength = cp.Make_getStatusResp(cp.m_peerAddress, cp.m_buff,
                (sysStates.sensor.humidity > sysStates.sensor.threshold ? 1 : 0),   // humidity alert
                (TECsRunning ? 1 : 0),                                              // TECs running
                (sysStates.chiller.state = running ? 1 : 0),                        // chiller running
                htons(pgetStatus->seqNum)
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
            #ifdef __DEBUG_VIA_SERIAL__
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
        Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: not my address, dropping packet: ");
        Serial.println(ntohs(pgetStatus->address.address));
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
    if( (ntohs(psetHumidityThreshold->address.address)) == cp.m_myAddress)
    {
        //
        // verify the CRC
        //
        if( (cp.verifyMessageCRC(len_setHumidityThreshold_t, ntohs(psetHumidityThreshold->crc))) )
        {
            //
            // pick up the new threshold
            //
            sysStates.sensor.threshold  = ntohs(psetHumidityThreshold->threshold);

            #ifdef __DEBUG_VIA_SERIAL__
            Serial.print(__PRETTY_FUNCTION__); Serial.print( " set humidity threshold to: ");
            Serial.println(sysStates.sensor.threshold);
            #endif
            
            //
            // use the sysStates conentent to respond, send back the received seqNum
            //
            respLength = cp.Make_setHumidityThresholdResp(cp.m_peerAddress, cp.m_buff, 1, // success
                htons(psetHumidityThreshold->seqNum)
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
            #ifdef __DEBUG_VIA_SERIAL__
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
        Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: not my address, dropping packet");
        Serial.println(ntohs(psetHumidityThreshold->address.address));
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
    if( (ntohs(pgetHumidityThreshold->address.address)) == cp.m_myAddress)
    {
        //
        // verify the CRC
        //
        if( (cp.verifyMessageCRC(len_getHumidityThreshold_t, ntohs(pgetHumidityThreshold->crc))) )
        {
            #ifdef __DEBUG_VIA_SERIAL__
            Serial.print(__PRETTY_FUNCTION__); Serial.print( " returning humidity threshold: ");
            Serial.println(sysStates.sensor.threshold);
            #endif
            
            //
            // use the sysStates conentent to respond, send back the received seqNum
            //
            respLength = cp.Make_getHumidityThresholdResp(cp.m_peerAddress, cp.m_buff,
                sysStates.sensor.threshold,
                htons(pgetHumidityThreshold->seqNum)
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
            #ifdef __DEBUG_VIA_SERIAL__
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
        Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: not my address, dropping packet");
        Serial.println(ntohs(pgetHumidityThreshold->address.address));
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
    if( (ntohs(pgetHumidity->address.address)) == cp.m_myAddress)
    {
        //
        // verify the CRC
        //
        if( (cp.verifyMessageCRC(len_getHumidity_t, ntohs(pgetHumidity->crc))) )
        {
            #ifdef __DEBUG_VIA_SERIAL__
            Serial.print(__PRETTY_FUNCTION__); Serial.print( " returning humidity: ");
            Serial.println(sysStates.sensor.humidity);
            #endif
            
            //
            // use the sysStates content to respond, send back the received seqNum
            //
            respLength = cp.Make_getHumidityResp(cp.m_peerAddress, cp.m_buff,
                sysStates.sensor.humidity,
                htons(pgetHumidity->seqNum)
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
            #ifdef __DEBUG_VIA_SERIAL__
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
        Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: not my address, dropping packet");
        Serial.println(ntohs(pgetHumidity->address.address));
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
    if( (ntohs(psetTECTemperature->address.address)) == cp.m_myAddress)
    {
        //
        // verify the CRC
        //
        if( (cp.verifyMessageCRC(len_setTECTemperature_t, ntohs(psetTECTemperature->crc))) )
        {
            //
            // pick up the new temperature
            //
            // if the TECs addresses is out of range, send back failure
            //
            //sscanf(reinterpret_cast<char*>(psetTECTemperature->temperature), "%f", &setPoint);
            setPoint = atof(psetTECTemperature->temperature);
            tecAddress = ntohs(psetTECTemperature->tec_address);

            #ifdef __DEBUG_VIA_SERIAL__
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
                    #ifdef __DEBUG_VIA_SERIAL__
                    Serial.print(__PRETTY_FUNCTION__); Serial.print( " success set temp for TEC: ");
                    Serial.println(tecAddress);
                    Serial.flush();
                    #endif
                    result  = 1;

                    //
                    // update the TEC's set point temperature
                    //
                    sysStates.tec[tecAddress - 2].setpoint = setPoint;

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

            Serial.flush();
            
            //
            // use the sysStates conentent to respond, send back the received seqNum
            //
            respLength = cp.Make_setTECTemperatureResp(cp.m_peerAddress, cp.m_buff,
                tecAddress, result, htons(psetTECTemperature->seqNum)
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
            #ifdef __DEBUG_VIA_SERIAL__
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
        Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: not my address, dropping packet");
        Serial.println(ntohs(psetTECTemperature->address.address));
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
    if( (ntohs(pgetTECTemperature->address.address)) == cp.m_myAddress)
    {
        //
        // verify the CRC
        //
        if( (cp.verifyMessageCRC(len_getTECTemperature_t, ntohs(pgetTECTemperature->crc))) )
        {
            //
            // if the TECs addresses is out of range, send back failure
            //
            tecAddress = ntohs(pgetTECTemperature->tec_address);

            if( (MAX_TEC_ADDRESS > tecAddress) )
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
                tecAddress, sysStates.tec[tecAddress - 2].temperature, htons(pgetTECTemperature->seqNum)
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
            #ifdef __DEBUG_VIA_SERIAL__
            } else
            {
                Serial.print(__PRETTY_FUNCTION__); Serial.print(" sent response: ");
                Serial.println(sysStates.tec[tecAddress - 2].temperature, 2);
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
        Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: not my address, dropping packet");
        Serial.println(ntohs(pgetTECTemperature->address.address));
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
    if( (ntohs(psetChillerTemperature->address.address)) == cp.m_myAddress)
    {
        //
        // verify the CRC
        //
        if( (cp.verifyMessageCRC(len_setChillerTemperature_t, ntohs(psetChillerTemperature->crc))) )
        {
            //
            // set the new chiller temperature
            //
            if( (setChillerSetPoint(reinterpret_cast<char*>(psetChillerTemperature->temperature))) )
            {
                #ifdef __DEBUG_VIA_SERIAL__
                Serial.println(__PRETTY_FUNCTION__); Serial.print(" success setChillerSetPoint");
                Serial.flush();
                #endif
                result = 1;
            } else
            {
                #ifdef __DEBUG_VIA_SERIAL__
                Serial.println(__PRETTY_FUNCTION__); Serial.print(" ERROR: failed to setChillerSetPoint");
                Serial.flush();
                #endif
                result  = 0;
            }

            respLength = cp.Make_setChillerTemperatureResp(cp.m_peerAddress, cp.m_buff,
                result, htons(psetChillerTemperature->seqNum)
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
            #ifdef __DEBUG_VIA_SERIAL__
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
        Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: not my address, dropping packet");
        Serial.println(ntohs(psetChillerTemperature->address.address));
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
    if( (ntohs(pgetChillerTemperature->address.address)) == cp.m_myAddress)
    {
        //
        // verify the CRC
        //
        if( (cp.verifyMessageCRC(len_getChillerTemperature_t, ntohs(pgetChillerTemperature->crc))) )
        {
            //
            // chiller informaion is gotton during getStatus
            //
            respLength = cp.Make_getChillerTemperatureResp(cp.m_peerAddress, cp.m_buff,
                sysStates.chiller.temperature, htons(pgetChillerTemperature->seqNum)
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
            #ifdef __DEBUG_VIA_SERIAL__
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
        Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: not my address, dropping packet");
        Serial.println(ntohs(pgetChillerTemperature->address.address));
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
    if( (ntohs(penableTECs->address.address)) == cp.m_myAddress)
    {
        //
        // verify the CRC
        //
        if( (cp.verifyMessageCRC(len_enableTECs_t, ntohs(penableTECs->crc))) )
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
                result, htons(penableTECs->seqNum)
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
            #ifdef __DEBUG_VIA_SERIAL__
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
        Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: not my address, dropping packet");
        Serial.println(ntohs(penableTECs->address.address));
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
    if( (ntohs(pdisableTECs->address.address)) == cp.m_myAddress)
    {
        //
        // verify the CRC
        //
        if( (cp.verifyMessageCRC(len_disableTECs_t, ntohs(pdisableTECs->crc))) )
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
                Serial.println(__PRETTY_FUNCTION__); Serial.print(" ERROR: failed to stopTECs");
                Serial.flush();
                #endif
                result  = 0;
            }

            respLength = cp.Make_disableTECsResp(cp.m_peerAddress, cp.m_buff,
                result, htons(pdisableTECs->seqNum)
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
            #ifdef __DEBUG_VIA_SERIAL__
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
        Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: not my address, dropping packet");
        Serial.println(ntohs(pdisableTECs->address.address));
        Serial.flush();
    #endif
    }
}


void sendNACK()
{

}
