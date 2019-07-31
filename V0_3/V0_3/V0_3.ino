#include <Controllino.h>
#include <SoftwareSerial.h>
#include <huber.h>              // huber chiller communication library
#include <meerstetterRS485.h>   // meerstetter TEC communication library
#include <LiquidCrystal.h>      // LCD interface library
//#include <Wire.h>               // I2C library, needed by SHTSensor TODO: put this back
//#include <SHTSensor.h>          // SHT sensor interface library TODO: put this back
#include "V0_3.h" 


//
// ---- globals ----
//
// LCD display
/* TODO: put this back
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
*/

// temperature himidity sensor
//SHTSensor sht;
float temp, humidity;

// chiller communication
huber chiller(9600);

// meerstetter communication
meerstetterRS485 ms(14, 15);

// LED and button states
int ledState = LOW;                 // the current state of the output pin
int buttonState = LOW;              // the current reading from the input pin
int lastButtonState = LOW;          // the previous reading from the input pin
unsigned long lastDebounceTime = 0; // the last time the output pin was toggled
unsigned long debounceDelay = 50;   // the debounce time; increase if the output flickers



//
// for testing
//
int msgCounter = 0;


void setup()
{
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.begin(9600);
    #endif

    //
    // TODO: remove
    //
    delay(10000);


    // initialize board pins

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
    //updateLCD(initializing);


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
        shutDown();

        //
        // TODO: die like this ?
        // update the LCD w/ "ALL DEVICES NOT PRESET", then die ?
        //
        updateLCDAndDie(lcd_notAllDevicesPresent);

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
        Serial.println("unable to start the chiller, shutting down");
        #endif

        Serial.println("doing shutDown");
        shutDown();
        updateLCDAndDie(chillerFailure);    // call this last
    #ifdef __DEBUG_VIA_SERIAL__
    } else
    {
        Serial.println("chiller is started");
    #endif
    }
}


void loop()
{ 
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.print(__PRETTY_FUNCTION__);
    Serial.println("top of loop()");
    #endif


    //
    // throttle loop() to run 1x per second
    //
    // the lcd_humidityThreshold delays for 3 seconds, use this
    // delay to throttle the loop() - and always show humidity
    //
    updateLCD(lcd_humidityThreshold);   


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
    // take commands from the
    // - the switch
    // - the controlling PC software
    //
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.print(__PRETTY_FUNCTION__);
    Serial.println(" doing handlMsgs()");
    #endif
    handleMsgs();


    //
    // TODO: pause a little or just keep looping ?
    //
    delay(3000);
}


void lcd_chillerAlertInit()
{
    /* TODO: put this back
    lcd.display();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("* CHILLER PUMP *");
    lcd.setCursor(0,1);
    lcd.print("  MUST BE ON!!! ");
    delay(3000);
    */
}



void displayAlertInit()
{
    /* TODO: put this back
    lcd.display();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("*** HUMIDITY ***");
    lcd.setCursor(0,1);
    lcd.print("**** ALERT *****");
    */
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
                    updateLCD(lcd_chillerAlertInit);
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

    /* TODO:put this back
    // set up the LCD's number of columns and rows:
    lcd.begin(16, 2);
    lcd.display();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("   deftDevise");
    lcd.setCursor(0,1);
    lcd.print("Firmware ver 1.0");
    */

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
    // TODO: put this back
    //lcdFaces[facesIndex]();
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
    // TODO: put this back
    //updateLCD(facesIndex);

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


/* TODO: put this back

    Wire.begin();

    if( (sht.init()) && (sht.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM)) ) // only supported by SHT3x
        retVal  = true;

    return(retVal);
*/

    return(true);
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
    if( (startChiller()) && (startTECs()) )
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
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    //
    // check chiller status is running
    //
    if( !(chiller.ChillerRunning()) )
    {
        #ifdef __DEBUG_VIA_SERIAL__
        Serial.println("gotStatus thinks the chiller is not running .. !");
        #endif

        updateLCD(chillerFailure);
        return(false);  // keep the LCD as bad chiller status
    }


    //
    // check all TECs running - get Device Status, possible status are
    //
    for(uint8_t Address = 2; (Address <= MAX_TEC_ADDRESS); Address++)
    {
        if( !(ms.TECRunning(Address)) )
        {
            #ifdef __DEBUG_VIA_SERIAL__
            Serial.println("gotStatus thinks TEC is not running .. !");
            #endif
            updateLCD(tecFailure);
            return(false);  // keep the LCD as bad TEC status
        }
    }


    //
    // check the humidity/temperature sensor - take an average over time
    //
    /* TODO: put this back
    if (sht.readSample())
    {
        //
        // update globals
        //
        temp        = sht.getTemperature();
        humidity    = sht.getHumidity();

        if (humidity > HUMIDITY_THRESHOLD)
        {
            updateLCD(sensorStatus);
            return(false);  // keep the bad humidity status on the LCD
        }
    } else
    {
        updateLCD(sensorFailure);
        return(false);  // keep the bad sensor status on the LCD
    }
    */

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

    shutDown();

    // loop forever
    while(true) { delay(1000); };
}


// do not update the LCD
void shutDown()
{
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
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
        retVal  = false;

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
        }
    }

    return(retVal);
}


//
// test communication with all devices
//
bool allDevicesPresent()
{
    bool retVal  = true;

    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif


    //
    // SHT sensor
    //
    /* TODO: put this back - this is killing the Serial output
    if( !(sht.init()) )
    {
        #ifdef __DEBUG_VIA_SERIAL__
        Serial.println("allDevicesPresent unable to start sensor");
        #endif
        retVal  = false;
    }
    */


    //
    // LCD
    //
    if(!startLCD())
    {
        #ifdef __DEBUG_VIA_SERIAL__
        Serial.println("allDevicesPresent unable to start LCD");
        #endif
        retVal  = false;
    }


    //
    // chiller
    //
    if( !(chiller.InitChiller()) )
    {
        #ifdef __DEBUG_VIA_SERIAL__
        Serial.println("allDevicesPresent unable to initialize chiller");
        #endif
        retVal  = false;
    }


    //
    // all TECs
    //
    for(uint8_t Address = 2; (Address <= MAX_TEC_ADDRESS); Address++)
    {
        if( !(ms.TECPresent(Address)) )
        {
            #ifdef __DEBUG_VIA_SERIAL__
            Serial.print("allDevicesPresent unable to initialize TEC addr ");
            Serial.println(Address, DEC);
            #endif
            retVal  = false;
        }
    }


    return(retVal);
}


//
// TODO:  move this meetstetter stuff into the meerstetter library
//
bool setTECTemp(float temp)
{
    bool retVal = true;
    char    buff[MAX_BUFF_LENGHT + 1];
    char*   pBuff = buff;
    uint8_t Instance    = 1;
    MeParFloatFields FieldVal;

    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif


    for(uint8_t Address = 2; (Address <= MAX_TEC_ADDRESS); Address++)
    {
        FieldVal = {temp, 0, 0};

        //
        // TODO:  move this meetstetter stuff into the meerstetter library
        //

        // TODO: how to handle this if one meerstetter doesn't change temp
        if( !(ms.MeCom_TEC_Tem_TargetObjectTemp(Address, Instance, &FieldVal, MeSet)) )
        {
            #ifdef __DEBUG_VIA_SERIAL__
            Serial.print(__PRETTY_FUNCTION__);
            Serial.flush();
            Serial.print(" failed to MeSet for Address ");
            Serial.flush();
            Serial.println(Address, HEX);
            Serial.flush();
            #endif

            retVal = false;
        } else
        {
            //
            // check what was set
            //
            FieldVal = {0, 0, 0};
            if( !(ms.MeCom_TEC_Tem_TargetObjectTemp(Address, Instance, &FieldVal, MeGet)) )
            {
                #ifdef __DEBUG_VIA_SERIAL__
                Serial.print(__PRETTY_FUNCTION__);
                Serial.flush();
                Serial.print(" failed to MeGet for Address ");
                Serial.flush();
                Serial.println(Address, HEX);
                Serial.flush();
                #endif

                retVal = false;
            } else
            {
                if( (FieldVal.Value !=  temp) )
                {
                    #ifdef __DEBUG_VIA_SERIAL__
                    Serial.print(__PRETTY_FUNCTION__);
                    Serial.flush();
                    Serial.print(" retported temps don't match input ");
                    Serial.flush();
                    Serial.print(temp, 2);
                    Serial.flush();
                    Serial.print(" fetched ");
                    Serial.flush();
                    Serial.print(FieldVal.Value, 2);
                    Serial.flush();
                    Serial.print(" for Address ");
                    Serial.println(Address, HEX);
                    Serial.flush();
                    #endif

                    retVal = false;
                } else
                {
                    #ifdef __DEBUG_VIA_SERIAL__
                    Serial.print(__PRETTY_FUNCTION__);
                    Serial.flush();
                    Serial.print(" retported temps do match input ");
                    Serial.flush();
                    Serial.print(temp, 2);
                    Serial.flush();
                    Serial.print(" fetched ");
                    Serial.flush();
                    Serial.print(FieldVal.Value, 2);
                    Serial.flush();
                    Serial.print(" for Address ");
                    Serial.println(Address, HEX);
                    Serial.flush();
                    #endif

                    retVal = true;
                }
            }
        }
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
    //
    // TODO: for simulated testing
    //
    static uint8_t counter  = 0;    // for looping through the following temps
    char* chillerSetPointTemps[] = {"0010", "0020", "0030", "0040"};  // -20 to +60C is valid
    float TECTemps[] = {-21.5, -22.5, -23.5, -24.5};  // -20 to +60C is valid

    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif


    //
    // TODO: implement handleMsgs
    //
    // for now, just do switchOps() and simulate message handling
    //
    // 
    //
    if( (1 == switchOps()) ) // 0 no change, -1 off, 1 on
    {
        #ifdef __DEBUG_VIA_SERIAL__
        Serial.println("switchOps - switched on, doing startUp()");
        #endif

        startUp();

    } else if( (-1 == switchOps()) )
    {
        #ifdef __DEBUG_VIA_SERIAL__
        Serial.println("switchOps - switched off, doing shutDown()");
        #endif

        shutDownAndDie();
    }
    // else no change in button state


    //
    // simulate messages using a counter
    //
    switch(msgCounter++)
    {
        case 0:     // startUp message
        {
            #ifdef __DEBUG_VIA_SERIAL__
            Serial.println("");
            Serial.println("======================================");
            Serial.println("simulated doing startUp()");
            #endif

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

        case 1:     // setTECTemp message
        {
            #ifdef __DEBUG_VIA_SERIAL__
            Serial.println("");
            Serial.println("======================================");
            Serial.println("simulated doing setTECTemp()");
            #endif

            for(int i = 0; i < 4; i++)
            {
                #ifdef __DEBUG_VIA_SERIAL__
                Serial.println("");
                Serial.print("simulated doing setTECTemp(");
                Serial.print(TECTemps[i]);
                Serial.println(")");
                #endif
                setTECTemp(TECTemps[i]);
            }
                
            //counter = (counter > 4 ? 0 : counter++);
            break;
        };

        case 2:     // setChillerTemp message
        {
            #ifdef __DEBUG_VIA_SERIAL__
            Serial.println("");
            Serial.println("======================================");
            Serial.println("simulated doing setChillerSetPoint()");
            #endif

            for(int i = 0; i < 4; i++)
            {
                #ifdef __DEBUG_VIA_SERIAL__
                Serial.println("");
                Serial.print("simulated doing setChillerSetPoint(");
                Serial.print(chillerSetPointTemps[i]);
                Serial.println(")");
                #endif
                setChillerSetPoint(chillerSetPointTemps[i]);
            }
                
            //counter = (counter > 4 ? 0 : counter++);
            break;
        };

        case 3:     // shutDown message
        {
            #ifdef __DEBUG_VIA_SERIAL__
            Serial.println("");
            Serial.println("======================================");
            Serial.println("simulated doing shutDown()");
            #endif

            shutDown();
            //counter = (counter > 4 ? 0 : counter++);
            break;
        };

        case 4:     // initialize system .. ?
        {
            #ifdef __DEBUG_VIA_SERIAL__
            Serial.println("");
            Serial.println("======================================");
            Serial.println("simulated doing allDevicesPresent()");
            #endif

            allDevicesPresent();
            //counter = (counter > 4 ? 0 : counter++);
            break;
        };

        case 5:     // startUp message
        {
            #ifdef __DEBUG_VIA_SERIAL__
            Serial.println("");
            Serial.println("======================================");
            Serial.println("simulated doing startUp() again...");
            #endif

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

        /*
        case 6:     // setSensorHumidity message
        {
            break;
        };
        */

        default:
        {
            #ifdef __DEBUG_VIA_SERIAL__
            Serial.println("");
            Serial.println("======================================");
            Serial.println("done with handleMsgs simulation...");
            #endif
            break;
        }
    }
}
    

void lcd_initializing()
{
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    /* TODO: put this back
    lcd.display();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Initializing");
    delay(3000);
    */
}


void lcd_notAllDevicesPresent()
{
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    /* TODO: put this back
    lcd.display();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("ALL DEVICES ARE");
    lcd.setCursor(0,1);
    lcd.print("NOT PRESENT");
    delay(3000);
    */
}


void lcd_chillerWarning()
{
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    /* TODO: put this back
    lcd.display();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("  CHILLER PUMP  ");
    lcd.setCursor(0,1);
    lcd.print("   MUST BE ON   ");
    delay(3000);
    */
}


void lcd_initFailed()
{
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    /* TODO: put this back
    lcd.display();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(" INITIALIZATION ");
    lcd.setCursor(0,1);
    lcd.print("     FAILED     ");
    delay(3000);
    */
}


void lcd_sensorFailure()
{
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    /* TODO: put this back
    lcd.display();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(" SENSOR FAILURE ");
    delay(3000);
    */
}


void lcd_tecFailure()
{
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    /* TODO: put this back
    lcd.display();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("  TEC FAILURE  ");
    delay(3000);
    */
}


void lcd_chillerFailure()
{
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    /* TODO: put this back
    lcd.display();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("CHILLER FAILURE ");
    delay(3000);
    */
}

void lcd_sensorStatus()
{
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    /* TODO: put this back

    if( (humidity > HUMIDITY_THRESHOLD) )
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
        lcd.print(humidity);
        lcd.setCursor(0,1);
        lcd.print("Threshold:     %");
        lcd.setCursor(13,1);
        lcd.print(HUMIDITY_THRESHOLD);
    } else
    {
        //  show the status
        lcd.display();
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Humidity:      %");
        lcd.setCursor(13, 0);
        lcd.print(humidity);
        lcd.setCursor(0,1);
        lcd.print("Threshold:     %");
        lcd.setCursor(13,1);
        lcd.print(HUMIDITY_THRESHOLD);
    }

    delay(3000);
    */
}


void lcd_humidityAlert()
{
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    /* TODO: put this back
    lcd.display();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("*** HUMIDITY ***");
    lcd.setCursor(0,1);
    lcd.print("**** ALERT *****");
    delay(3000);
    */
}


void lcd_humidityThreshold()
{
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    /* TODO: put this back
    lcd.display();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Humidity:      %");
    lcd.setCursor(0,1);
    lcd.print("Threshold:     %");
    lcd.setCursor(13,1);
    lcd.print(HUMIDITY_THRESHOLD);
    */
}
