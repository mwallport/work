#include <SoftwareSerial.h>
#include <RS232LSSeriesChiller.h>
#include <meerstetterRS485.h>
#include <LiquidCrystal.h>        //http://www.arduino.cc/en/Tutorial/LiquidCrystalHelloWorld
#include <Wire.h>
#include <SHTSensor.h>
#include "V0_3.h"


//
// ---- globals ----
//
// LCD display
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);


// temperature himidity sensor
SHTSensor sht;


// chiller communication - SoftwareSerial on pins 10 and 11 assuming RS232 module there
RS232LSSeriesChiller chiller(10, 11, 9600, SERIAL_8N1);


// meerstetter communication - Software Serial on pings 12 and 13 assuming RS485 module there
meerstetterRS485 ms(12, 13);


// LED and button states
int ledState = LOW;                 // the current state of the output pin
int buttonState;                    // the current reading from the input pin
int lastButtonState = LOW;          // the previous reading from the input pin
unsigned long lastDebounceTime = 0; // the last time the output pin was toggled
unsigned long debounceDelay = 50;   // the debounce time; increase if the output flickers


// humidity sensor
boolean isHumidityOk=true,
        isHumidityOkPrevious = isHumidityOk,
        fault = false;
float   temp, humidity;     // temperature and humidity from SHT sensor


// getStatus
enum    { initStr, chillerStr, chillerComStr, TECStr, TECComStr, SHTSensorStr, SHTSensorComStr };
const char* statusString;       // will point at one of the statusStrings
const char* statusStrings[] = { // returned by getStatus used to update LCD, max 16 chars
            "init(): fail"
            "Chiller fail",
            "Chiller com fail",
            "TEC fail",
            "TEC com fail",
            "Sensor fail",
            "Sensor com fail"
        };



uint8_t msgCounter = 0; // used for testing, will pretend we're getting messages



void setup()
{
#ifdef __DEBUG_VIA_SERIAL__
Serial.begin(9600);
debug dbg();
#endif


    // initialize board pins
    pinMode(PIN_HW_ENABLE_n, OUTPUT);           // external LED .. ?
    pinMode(LED_BUILTIN, OUTPUT);               // built on board - defined by board type
    pinMode(SWITCH_PIN, INPUT_PULLUP);          // switch connected to this pin
    digitalWrite(LED_BUILTIN, ledState);        // turn off LED on board
    digitalWrite(PIN_HW_ENABLE_n, ledState);    // turn off external LED


    //
    // verify communication with the following  -  if all not available, fail and die
    // - sensor
    // - chiller
    // - all TECs
    // - LCD
    //
    if(0 != (statusString = checkPresence()))
    {
        shutDown();
        updateLCDAndDie(statusString);      // hold the last LCD message, not
    }                                       // more processing, done, dead, kaput


    //
    // start the chiller and the SHT sensor and get the LCD ready
    //
    // TODO: always start the chiller or make this stateful, i.e. after switch on 
    // or startup message received ?
    //
    //
    // following call pauses 1.5 mins for chiller as required by documentation
    if(0 != (statusString = initializeSystem()) )
    {
        shutDown();
        updateLCDAndDie(statusString);
    }


    // 
    // all devices are present, chiller is running
    // update the LCD w/ himidity and temperature and enter main loop
    //
    displayInit();
}


void loop()
{ 
#ifdef __DEBUG_VIA_SERIAL__
debug dbg();
#endif

    //
    // handleMsgs
    // - switchOps() is a message - same as startUp
    // - input from test PC is a message
    //      - startUp - same as switchOps
    //      - shutDown
    //      - setTECTemp
    //      - setChillerSetPoint
    //      - setHumidityThreshold - eventually
    //      - setTemperatureThreshold - eventually
    //
    handleMsgs();
    /*
    if(0 != (statusString = handleMsgs()))
    {
        shutDown();
        updateLCDAndDie(statusString);      // hold the last LCD message, not
    }                                       // more processing, done, dead, kaput
    */
  
    //
    // getStatus - check facilities and shutDown if needed and pause
    // - check humidity sensor
    // - check if chiller running
    // - check if TECs are running
    // - if any of these are bad, shutDown
    //
    if(0 != (statusString = getStatus()))    // 0 means there is no failure
    {
        shutDown();
        updateLCDAndDie(statusString);      // hold the last LCD message, not
    }                                       // more processing, done, dead, kaput


    //
    // TODO: pause a little or just keep looping ?
    //
    delay(3000);
}


void displayChillerAlertInit()
{
#ifdef __DEBUG_VIA_SERIAL__
debug dbg();
#endif

    lcd.display();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("* CHILLER PUMP *");
    lcd.setCursor(0,1);
    lcd.print("  MUST BE ON!!! ");
    delay(3000);
}


void displayInit()
{
#ifdef __DEBUG_VIA_SERIAL__
debug dbg();
#endif

    lcd.display();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Humidity:      %");
    lcd.setCursor(0,1);
    lcd.print("Threshold:     %");
    lcd.setCursor(13,1);
    lcd.print(HUMIDITY_THRESHOLD);
}


void displayAlertInit()
{
#ifdef __DEBUG_VIA_SERIAL__
debug dbg();
#endif

    lcd.display();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("*** HUMIDITY ***");
    lcd.setCursor(0,1);
    lcd.print("**** ALERT *****");
}


void displayUpdate()
{
#ifdef __DEBUG_VIA_SERIAL__
debug dbg();
#endif

  static boolean toggle = false;


    if (isHumidityOk)
    {
        lcd.setCursor(10, 0);
        lcd.print(humidity);

        if (fault)
        {
            lcd.setCursor(14, 0);

            if (toggle)
            {
                lcd.print("*");
            } else
            {
                lcd.print(" ");
            }

            toggle = !toggle;
        }

    } else
    {
        if (toggle)
            lcd.display();
        else
            lcd.noDisplay();

        toggle = !toggle;
    }
}


void switchOps()
{
#ifdef __DEBUG_VIA_SERIAL__
debug dbg();
#endif

    // read the state of the switch into a local variable:
    int reading = digitalRead(SWITCH_PIN);

    // check to see if you just pressed the button
    // (i.e. the input went from LOW to HIGH), and you've waited long enough
    // since the last press to ignore any noise:

    // If the switch changed, due to noise or pressing:
    if (reading != lastButtonState)
    {
        // reset the debouncing timer
        lastDebounceTime = millis();
    }

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
            } else
            {
                ledState = LOW;
                displayChillerAlertInit();
                displayInit();
            }
        }
    }

    // set the LED:
    digitalWrite(LED_BUILTIN, ledState || fault);
    digitalWrite(PIN_HW_ENABLE_n, ledState || fault);

    // save the reading. Next time through the loop, it'll be the lastButtonState:
    lastButtonState = reading;
}


// all LCD calls are void, no way to tell if the LCD is up .. 
bool startLCD()
{
#ifdef __DEBUG_VIA_SERIAL__
debug dbg();
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
void updateLCDAndDie(const char* displayStr)
{
#ifdef __DEBUG_VIA_SERIAL__
debug dbg();
#endif

    lcd.display();
    lcd.clear();
    lcd.print(displayStr);

    while(true) { delay(1000); }    // keep LCD display on and do nothing
}


bool startSHTSensor()
{
#ifdef __DEBUG_VIA_SERIAL__
debug dbg();
#endif

    bool retVal = false;


    Wire.begin();

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
#ifdef __DEBUG_VIA_SERIAL__
debug dbg();
#endif

    bool retVal = false;    


    //
    // start the chiller(should always be on)
    // and start the TECs
    //
    if( (startChiller() && startTECs()) )
    {
        retVal = true;
    }
    
    return(retVal);
}


const char* getStatus()
{
#ifdef __DEBUG_VIA_SERIAL__
debug dbg();
#endif

    char    buff[MAX_BUFF_LENGHT + 1];
    char*   pBuff = buff;
    uint8_t Instance    = 1;
    MeParLongFields FieldVal;


    //
    // check chiller status is running
    //
    memset(buff, '\0', MAX_BUFF_LENGHT + 1);
    if(chiller.ReadStatus(&pBuff))
    {
        if('1' != buff[0])  // 1 is on, 0 is off
            return(statusStrings[chillerStr]);
    } else
    {
        // can't talk to chiller, this is bad
        return(statusStrings[chillerComStr]);
    }


    //
    // check all TECs running - get Device Status, possible status are
    //
    /*
        0: Init
        1: Ready
        2: Run      <-- looking for this for all TECs
        3: Error
        4: Bootloader
        5: Device will Reset within next 200ms
    */
    for(uint8_t Address = 2; (Address <= MAX_TEC_ADDRESS); Address++)
    {
        FieldVal = {0, 0, 0};  // initialize return variable
        if( (ms.MeCom_COM_DeviceStatus(Address, &FieldVal, MeGet)) )
        {
            if( (2 != FieldVal.Value) )
                return(statusStrings[TECStr]);

        } else  // this will never hit due to the way the meerstetter protocol works
        {       // seems like we will always be able to communicate with TEC .. ?
            return(statusStrings[TECComStr]);
        }
    }


    //
    // check the humidity/temperature sensor - take an average over time
    //
    // TODO : keep a running average - or make this more lenient ?
    //
    if (sht.readSample())
    {
        temp        = sht.getTemperature();
        humidity    = sht.getHumidity();

        if (humidity > HUMIDITY_THRESHOLD)
        {
            isHumidityOk = false;
            fault = true;
        }

        if (isHumidityOk != isHumidityOkPrevious)
        {
            if (isHumidityOk)
                displayInit();
            else
            {
                displayAlertInit();
                return(statusStrings[SHTSensorStr]);
            }
        }

        displayUpdate();

        isHumidityOkPrevious = isHumidityOk;

    } else
    {
        return(statusStrings[SHTSensorComStr]);
    }

    return(0);
}


// ---------------------------------------------------------
//
//  shutdown the chiller and the TECs
//
void shutDown()
{
#ifdef __DEBUG_VIA_SERIAL__
debug dbg();
#endif

    char    buff[MAX_BUFF_LENGHT + 1];
    char*   pBuff = buff;
    uint8_t Instance    = 1;
    MeParLongFields FieldVal;

    
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
}




// ----------------------------------------------------------
// turn off echo and continuous output
// turn on the chiller
// return success if all these happen
//
bool startChiller()
{
#ifdef __DEBUG_VIA_SERIAL__
debug dbg();
#endif

    bool retVal = true;


    if( !(chiller.StartChiller()) )
    {
        retVal  = false;

        #ifdef __DEBUG_VIA_SERIAL__
        Serial.print("__PRETTY_FUNCTION__ unable to start chiller ");
        Serial.println(Address, DEC);
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
#ifdef __DEBUG_VIA_SERIAL__
debug dbg();
#endif

    bool    retVal      = true;


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
            Serial.print("__PRETTY_FUNCTION__ unable to start TEC ");
            Serial.println(Address, DEC);
            #endif
        }
    }

    return(retVal);
}


char* checkPresence()
{
    bool retVal  = false;     // always thinking positive ..


    //
    // check chiller is present
    //
    if( !(chiller.ChillerPresent(0)))
    {
        // return chiller-comm-bad string
        return(statusStrings[chillerComStr]);
    }


    //
    // check all TECs present
    //
    for(uint8_t Address = 2; (Address <= MAX_TEC_ADDRESS); Address++)
    {
        if( !(ms.TECPresent(Address)) )
        {
            return(statusStrings[TECComStr]);
        }
    }


    //
    // check the SHT sensor is present
    //
    if( !(sht.init()) )
    {
        return(statusStrings[SHTSensorComStr]);
    }

    return(0);
}


char* initializeSystem()
{
    // this pauses 1.5 minutes per chiller documentation
    if(!chiller.StartChiller())
    {
        return(statusStrings[chillerStr]);
    }


    // this will never fail
    if(!startLCD())
    {
        return("LCD failure");  // hmm if LCD is not present, what to, but this will never fail
    }


    if(!startSHTSensor())
    {
        return(statusStrings[SHTSensorStr]);
    }

    return(0);
}


bool setTECTemp(float temp)
{
    bool retVal = true;
    char    buff[MAX_BUFF_LENGHT + 1];
    char*   pBuff = buff;
    uint8_t Instance    = 1;
    MeParFloatFields FieldVal;


    for(uint8_t Address = 2; (Address <= MAX_TEC_ADDRESS); Address++)
    {
        FieldVal = {temp, 0, 0};
        // TODO: how to handle this if one meerstetter doesn't change temp
        if( !(ms.MeCom_TEC_Tem_TargetObjectTemp(Address, Instance, &FieldVal, MeSet)) )
            retVal = false;

        // TODO: fetch the object temp to verify the setting
    }

    return(retVal);
}


bool setChillerSetPoint(char* temp)
{
    return(chiller.SetSetPoint(temp));
}


void handleMsgs()
{
#ifdef __DEBUG_VIA_SERIAL__
debug dbg();
#endif


    // TODO: for simulated testing, remove
    static uint8_t counter  = 0;    // for looping through the following temps
    char* chillerSetPointTemps[] = {"-10", "-05", "+15", "+20"};  // -20 to +60C is valid
    float TECTemps[] = {-21.5, -22.5, -23.5, -24.5};  // -20 to +60C is valid

    //
    // TODO: implement handleMsgs
    //
    // for now, just do switchOps() and simulate message handling
    //
    //switchOps();

    //
    // simulate messages using a counter
    //
    switch(msgCounter++)
    {
        case 0:     // startUp message
        {
            startUp();
            /*
            if( startUp() )
            {
                sendStartUpACK();
            } else
            {
                sendStartUpNACK();
            }
            */
            break;
        };
        case 1:     // shutDown message
        {
            shutDown();
            break;
        };
        case 2:     // setTECTemp message
        {
            setTECTemp(TECTemps[counter]);
            counter = (counter > 3 ? 0 : counter++);
            break;
        };
        case 3:     // setChillerTemp message
        {
            setChillerSetPoint(chillerSetPointTemps[counter]);
            counter = (counter > 3 ? 0 : counter++);
            break;
        };
        /*
        case 4:     // setSensorTemp message
        {
            break;
        };
        case 5:     // setSensorHumidity message
        {
            break;
        };
        */
        default:
        {
            msgCounter = 0;
            break;
        }
    }
}
    
