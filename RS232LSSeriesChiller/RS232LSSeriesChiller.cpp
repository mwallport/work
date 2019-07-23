#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include <SoftwareSerial.h>
#include "RS232LSSeriesChiller.h"



// gobals
//RS232LSSeriesChiller rs(15, 14, 9600, 1);  // assuming the mega, for Uno would be 0(rx), 1(tx)
                                           // and the config does not work, can't pass SERIAL_8N1


/*
SEi[CR]
Echo: i = 1
No Echo: i = 0
![CR]
*/
// echo is not supported do not enable it
bool  RS232LSSeriesChiller::SetCommandEcho(char OnOff)
{
    // build command
    uint8_t count   = 0;
    bool retVal     = false;


    Buff[count++]   = 'S';
    Buff[count++]   = 'E';
    Buff[count++]   = OnOff;
    Buff[count++]   = '\r';
    Buff[count++]   = '\0';
    
    // uses class' Buff which was set up above
    if( (TxCommand()) )
    {
        Serial.println("Tx succes");

        // puts the response into the class' Buff
        if( (RxResponse(0, 3000)) ) // 3000ms and no retBuff
        {
            Serial.println("SetCommandEcho Rx success");

            //
            // parse Buff - this is speciffic to this function
            // should just be ![CR] or ?[CR]
            // this is all debug crap, most can be deleted after verified
            //
            if( ('!' == Buff[0]) )
            {
                Serial.println("SetCommandEcho got !, success");
                retVal = true;
            } else if( ('?' == Buff[0]) )
                Serial.println("SetCommandEcho got ?, failed");

            // expecting total length of 2
            if( (2 == strlen(Buff)) )
                Serial.println("SetCommandEcho got expected length");
            else
                Serial.println("SetCommandEcho got unexpected length");
        } else
        {
            Serial.println("SetCommandEcho Rx failed");
        }
    } else
    {
        Serial.println("SetCommandEcho Tx failed");
    }

    return(retVal);
}

/*
SOi[CR]
i = 1 to turn on
i = 0 to turn off
![CR]
*/
bool  RS232LSSeriesChiller::SetOnOff(char OnOff)
{
    // build command
    uint8_t count   = 0;
    bool retVal     = false;


    Buff[count++]   = 'S';
    Buff[count++]   = 'O';
    Buff[count++]   = OnOff;
    Buff[count++]   = '\r';
    Buff[count++]   = '\0';
    
    // uses class' Buff which was set up above
    if( (TxCommand()) )
    {
        Serial.println("SetOnOff Tx succes");

        // puts the response into the class' Buff
        if( (RxResponse(0, 3000)) ) // 3000ms and no retBuff
        {
            Serial.println("SetOnOff Rx success");

            //
            // parse Buff - this is speciffic to this function
            // should just be ![CR] or ?[CR]
            // this is all debug crap, most can be deleted after verified
            //
            if( ('!' == Buff[0]) )
            {
                Serial.println("got !, success");
                retVal = true;
            } else if( ('?' == Buff[0]) )
                Serial.println("SetOnOff got ?, failed");

            // expecting total length of 2
            if( (2 == strlen(Buff)) )
                Serial.println("SetOnOff got expected length");
            else
                Serial.println("SetOnOff got unexpected length");
        } else
        {
            Serial.println("SetOnOff Rx failed");
        }
    } else
    {
        Serial.println("SetOnOff Tx failed");
    }

    return(retVal);
}


/*
SSxxx[CR]
x = ASCII digit
![CR]
*/
bool  RS232LSSeriesChiller::SetSetPoint(char* temp) // where temp is 3 ASCII digits
                                                    // not clear what this parameter
{
    // build command
    uint8_t count   = 0;
    bool retVal     = false;


    Buff[count++]   = 'S';
    Buff[count++]   = 'S';
    for(int i = 0; i < strlen(temp); i++)
        Buff[count++]   = temp[i];
    Buff[count++]   = '\r';
    Buff[count++]   = '\0';
    
    // uses class' Buff which was set up above
    if( (TxCommand()) )
    {
        Serial.println("SetSetPoint Tx succes");

        // puts the response into the class' Buff
        if( (RxResponse(0, 3000)) ) // 3000ms and no retBuff
        {
            Serial.println("SetSetPoint Rx success");

            //
            // parse Buff - this is speciffic to this function
            // should just be ![CR] or ?[CR]
            // this is all debug crap, most can be deleted after verified
            //
            if( ('!' == Buff[0]) )
            {
                Serial.println("SetSetPoint got !, success");

                //
                // TODO:  read the setpoint back and verify
                //

                retVal = true;
            } else if( ('?' == Buff[0]) )
                Serial.println("SetSetPoint got ?, failed");

            // expecting total length of 2
            if( (2 == strlen(Buff)) )
                Serial.println("SetSetPoint got expected length");
            else
                Serial.println("SetSetPoint got unexpected length");
        } else
        {
            Serial.println("SetSetPoint Rx failed");
        }
    } else
    {
        Serial.println("SetSetPoint Tx failed");
    }

    return(retVal);
}

/*
RS[CR]
+xx.x[CR] or -xx.x[CR]
*/
bool  RS232LSSeriesChiller::ReadSetPointTemperature(char** ppRetStr)
{
    // build command
    uint8_t count   = 0;
    bool retVal     = false;
    char* pRetStr   = (ppRetStr ? 0 : *ppRetStr);

    Buff[count++]   = 'R';
    Buff[count++]   = 'S';
    Buff[count++]   = '\r';
    Buff[count++]   = '\0';
    
    // uses class' Buff which was set up above
    if( (TxCommand()) )
    {
        Serial.println("ReadSetPointTemperature Tx succes");

        // puts the response into the class' Buff
        if( (RxResponse(0, 3000)) ) // 3000ms and no retBuff
        {
            Serial.println("ReadSetPointTemperature Rx success");

            //
            // parse Buff - this is speciffic to this function
            // should just be ![CR] or ?[CR]
            // this is all debug crap, most can be deleted after verified
            //
            if( ('!' == Buff[0]) )
            {
                Serial.println("ReadSetPointTemperature got !, success");
                retVal = true;
            } else if( ('?' == Buff[0]) )
                Serial.println("ReadSetPointTemperature got ?, failed");

            // expecting total length of 6
            if( (6 == strlen(Buff)) )
                Serial.println("ReadSetPointTemperature got expected length");
            else
                Serial.println("ReadSetPointTemperature got unexpected length");

            // set the return parameter
            if( 0 != pRetStr )
                strcpy(pRetStr, Buff);
        } else
        {
            Serial.println("ReadSetPointTemperature Rx failed");
        }
    } else
    {
        Serial.println("ReadSetPointTemperature Tx failed");
    }

    return(retVal);
}


/*
RT[CR]
+xx.x[CR] or -xx.x[CR]]
*/
bool  RS232LSSeriesChiller::ReadTemperature(char** ppRetStr)
{
    // build command
    uint8_t count   = 0;
    bool retVal     = false;
    char* pRetStr    = (ppRetStr ? 0 : *ppRetStr);


    Buff[count++]   = 'R';
    Buff[count++]   = 'T';
    Buff[count++]   = '\r';
    Buff[count++]   = '\0';
    
    // uses class' Buff which was set up above
    if( (TxCommand()) )
    {
        Serial.println("ReadTemperature Tx succes");

        // puts the response into the class' Buff
        if( (RxResponse(0, 3000)) ) // 3000ms and no retBuff
        {
            Serial.println("ReadTemperature Rx success");

            //
            // parse Buff - this is speciffic to this function
            // should just be ![CR] or ?[CR]
            // this is all debug crap, most can be deleted after verified
            //
            if( ('!' == Buff[0]) )
            {
                Serial.println("ReadTemperature got !, success");
                retVal = true;
            } else if( ('?' == Buff[0]) )
                Serial.println("ReadTemperature got ?, failed");

            // expecting total length of 6
            if( (6 == strlen(Buff)) )
                Serial.println("ReadTemperature got expected length");
            else
                Serial.println("ReadTemperature got unexpected length");

            // set the return parameter
            if( 0 != pRetStr )
                strcpy(pRetStr, Buff);
        } else
        {
            Serial.println("ReadTemperature Rx failed");
        }
    } else
    {
        Serial.println("ReadTemperature Tx failed");
    }

    return(retVal);
}


/*
RU[CR]
C[CR] or F[CR]
*/
bool  RS232LSSeriesChiller::ReadTemperatureUnits(char** ppRetStr)
{
    // build command
    uint8_t count   = 0;
    bool retVal     = false;
    char* pRetStr    = (ppRetStr ? 0 : *ppRetStr);


    Buff[count++]   = 'R';
    Buff[count++]   = 'U';
    Buff[count++]   = '\r';
    Buff[count++]   = '\0';
    
    // uses class' Buff which was set up above
    if( (TxCommand()) )
    {
        Serial.println("ReadTemperatureUnits Tx succes");

        // puts the response into the class' Buff
        if( (RxResponse(0, 3000)) ) // 3000ms and no retBuff
        {
            Serial.println("ReadTemperatureUnits Rx success");

            //
            // parse Buff - this is speciffic to this function
            // should just be ![CR] or ?[CR]
            // this is all debug crap, most can be deleted after verified
            //
            if( ('!' == Buff[0]) )
            {
                Serial.println("ReadTemperatureUnits got !, success");
                retVal = true;
            } else if( ('?' == Buff[0]) )
                Serial.println("ReadTemperatureUnits got ?, failed");

            // expecting total length of 6
            if( (2 == strlen(Buff)) )
                Serial.println("ReadTemperatureUnits got expected length");
            else
                Serial.println("ReadTemperatureUnits got unexpected length");

            // set the return parameter
            if( 0 != pRetStr )
                strcpy(pRetStr, Buff);
        } else
        {
            Serial.println("ReadTemperatureUnits Rx failed");
        }
    } else
    {
        Serial.println("ReadTemperatureUnits Tx failed");
    }

    return(retVal);
}


/*
RW[CR]
1[CR] or 0[CR]
*/
bool  RS232LSSeriesChiller::ReadStatus(char** ppRetStr)
{
    // build command
    uint8_t count   = 0;
    bool retVal     = false;
    char* pRetStr    = (ppRetStr ? 0 : *ppRetStr);


    Buff[count++]   = 'R';
    Buff[count++]   = 'W';
    Buff[count++]   = '\r';
    Buff[count++]   = '\0';
    
    // uses class' Buff which was set up above
    if( (TxCommand()) )
    {
        Serial.println("ReadStatus Tx succes");

        // puts the response into the class' Buff
        if( (RxResponse(0, 3000)) ) // 3000ms and no retBuff
        {
            Serial.println("ReadStatus Rx success");

            //
            // parse Buff - this is speciffic to this function
            // should just be ![CR] or ?[CR]
            // this is all debug crap, most can be deleted after verified
            //
            if( ('!' == Buff[0]) )
            {
                Serial.println("ReadStatus got !, success");
                retVal = true;
            } else if( ('?' == Buff[0]) )
                Serial.println("ReadStatus got ?, failed");

            // expecting total length of 2
            if( (2 == strlen(Buff)) )
                Serial.println("ReadStatus got expected length");
            else
                Serial.println("ReadStatus got unexpected length");

            // set the return parameter
            if( 0 != pRetStr )
                strcpy(pRetStr, Buff);
        } else
        {
            Serial.println("ReadStatus Rx failed");
        }
    } else
    {
        Serial.println("ReadStatus Tx failed");
    }

    return(retVal);
}


/*
RUT[CR]
xxx.x[CR]
*/
bool  RS232LSSeriesChiller::ReadCompressorDischargeTemperature(char** ppRetStr)
{
    // build command
    uint8_t count   = 0;
    bool retVal     = false;
    char* pRetStr    = (ppRetStr ? 0 : *ppRetStr);


    Buff[count++]   = 'R';
    Buff[count++]   = 'U';
    Buff[count++]   = 'T';
    Buff[count++]   = '\r';
    Buff[count++]   = '\0';
    
    // uses class' Buff which was set up above
    if( (TxCommand()) )
    {
        Serial.println("ReadCompressorDischargeTemperature Tx succes");

        // puts the response into the class' Buff
        if( (RxResponse(0, 3000)) ) // 3000ms and no retBuff
        {
            Serial.println("ReadCompressorDischargeTemperature Rx success");

            //
            // parse Buff - this is speciffic to this function
            // should just be ![CR] or ?[CR]
            // this is all debug crap, most can be deleted after verified
            //
            if( ('!' == Buff[0]) )
            {
                Serial.println("ReadCompressorDischargeTemperature got !, success");
                retVal = true;
            } else if( ('?' == Buff[0]) )
                Serial.println("ReadCompressorDischargeTemperature got ?, failed");

            // expecting total length of 6
            if( (6 == strlen(Buff)) )
                Serial.println("ReadCompressorDischargeTemperature got expected length");
            else
                Serial.println("ReadCompressorDischargeTemperature got unexpected length");

            // set the return parameter
            if( 0 != pRetStr )
                strcpy(pRetStr, Buff);
        } else
        {
            Serial.println("ReadCompressorDischargeTemperature Rx failed");
        }
    } else
    {
        Serial.println("ReadCompressorDischargeTemperature Tx failed");
    }

    return(retVal);
}


/*
RF[CR]
![CR]
00 = System OK      <-- this one is not clear what the return will be
Faults
*/
bool  RS232LSSeriesChiller::ReadFaultStatus(char** ppRetStr)
{
    // build command
    uint8_t count   = 0;
    bool retVal     = false;
    char* pRetStr    = (ppRetStr ? 0 : *ppRetStr);


    Buff[count++]   = 'R';
    Buff[count++]   = 'F';
    Buff[count++]   = '\r';
    Buff[count++]   = '\0';
    
    // uses class' Buff which was set up above
    if( (TxCommand()) )
    {
        Serial.println("ReadFaultStatus Tx succes");

        // puts the response into the class' Buff
        if( (RxResponse(0, 3000)) ) // 3000ms and no retBuff
        {
            Serial.println("ReadFaultStatus Rx success");

            //
            // parse Buff - this is speciffic to this function
            // should just be ![CR] or ?[CR]
            // this is all debug crap, most can be deleted after verified
            //
            if( ('!' == Buff[0]) )
            {
                Serial.println("ReadFaultStatus got !, success");
                retVal = true;
            } else if( ('?' == Buff[0]) )
                Serial.println("ReadFaultStatus got ?, failed");

            // expecting total length of 4 TODO:what is this expected length
            if( (4 == strlen(Buff)) )
                Serial.println("ReadFaultStatus got expected length");
            else
                Serial.println("ReadFaultStatus got unexpected length");

            // set the return parameter
            if( 0 != pRetStr )
                strcpy(pRetStr, Buff);

        } else
        {
            Serial.println("ReadFaultStatus Rx failed");
        }
    } else
    {
        Serial.println("ReadFaultStatus Tx failed");
    }

    return(retVal);
}


/*
REI[CR]
xxx.x[CR]
*/
bool  RS232LSSeriesChiller::ReadEvaporatorInletTemperature(char** ppRetStr)
{
    // build command
    uint8_t count   = 0;
    bool retVal     = false;
    char* pRetStr    = (ppRetStr ? 0 : *ppRetStr);


    Buff[count++]   = 'R';
    Buff[count++]   = 'E';
    Buff[count++]   = 'I';
    Buff[count++]   = '\r';
    Buff[count++]   = '\0';
    
    // uses class' Buff which was set up above
    if( (TxCommand()) )
    {
        Serial.println("ReadEvaporatorInletTemperature Tx succes");

        // puts the response into the class' Buff
        if( (RxResponse(0, 3000)) ) // 3000ms and no retBuff
        {
            Serial.println("ReadEvaporatorInletTemperature Rx success");

            //
            // parse Buff - this is speciffic to this function
            // should just be ![CR] or ?[CR]
            // this is all debug crap, most can be deleted after verified
            //
            if( ('!' == Buff[0]) )
            {
                Serial.println("ReadEvaporatorInletTemperature got !, success");
                retVal = true;
            } else if( ('?' == Buff[0]) )
                Serial.println("ReadEvaporatorInletTemperature got ?, failed");

            // expecting total length of 6
            if( (6 == strlen(Buff)) )
                Serial.println("ReadEvaporatorInletTemperature got expected length");
            else
                Serial.println("ReadEvaporatorInletTemperature got unexpected length");

            // set the return parameter
            if( 0 != pRetStr )
                strcpy(pRetStr, Buff);
        } else
        {
            Serial.println("ReadEvaporatorInletTemperature Rx failed");
        }
    } else
    {
        Serial.println("ReadEvaporatorInletTemperature Tx failed");
    }

    return(retVal);
}


/*
REO[CR]
xxx.x[CR]
*/
bool  RS232LSSeriesChiller::ReadEvaporatorOutletTemperature(char** ppRetStr)
{
    // build command
    uint8_t count   = 0;
    bool retVal     = false;
    char* pRetStr    = (ppRetStr ? 0 : *ppRetStr);


    Buff[count++]   = 'R';
    Buff[count++]   = 'E';
    Buff[count++]   = 'O';
    Buff[count++]   = '\r';
    Buff[count++]   = '\0';
    
    // uses class' Buff which was set up above
    if( (TxCommand()) )
    {
        Serial.println("ReadEvaporatorOutletTemperature Tx succes");

        // puts the response into the class' Buff
        if( (RxResponse(0, 3000)) ) // 3000ms and no retBuff
        {
            Serial.println("ReadEvaporatorOutletTemperature Rx success");

            //
            // parse Buff - this is speciffic to this function
            // should just be ![CR] or ?[CR]
            // this is all debug crap, most can be deleted after verified
            //
            if( ('!' == Buff[0]) )
            {
                Serial.println("ReadEvaporatorOutletTemperature got !, success");
                retVal = true;
            } else if( ('?' == Buff[0]) )
                Serial.println("ReadEvaporatorOutletTemperature got ?, failed");

            // expecting total length of 6
            if( (6 == strlen(Buff)) )
                Serial.println("ReadEvaporatorOutletTemperature got expected length");
            else
                Serial.println("ReadEvaporatorOutletTemperature got unexpected length");

            // set the return parameter
            if( 0 != pRetStr )
                strcpy(pRetStr, Buff);
        } else
        {
            Serial.println("ReadEvaporatorOutletTemperature Rx failed");
        }
    } else
    {
        Serial.println("ReadEvaporatorOutletTemperature Tx failed");
    }

    return(retVal);
}


/*
RDi[CR]
i = 1 to turn on
i = 0 to turn off
![CR]
*/
// continuous output is not supported, only turn if off !
bool  RS232LSSeriesChiller::OutputContinuousDataStream(char OnOff)
{
    // build command
    uint8_t count   = 0;
    bool retVal     = false;


    Buff[count++]   = 'R';
    Buff[count++]   = 'D';
    Buff[count++]   = OnOff;
    Buff[count++]   = '\r';
    Buff[count++]   = '\0';
    
    // uses class' Buff which was set up above
    if( (TxCommand()) )
    {
        Serial.println("OutputContinuousDataStream Tx succes");

        // puts the response into the class' Buff
        if( (RxResponse(0, 3000)) ) // 3000ms and no retBuff
        {
            Serial.println("OutputContinuousDataStream Rx success");

            //
            // parse Buff - this is speciffic to this function
            // should just be ![CR] or ?[CR]
            // this is all debug crap, most can be deleted after verified
            //
            if( ('!' == Buff[0]) )
            {
                Serial.println("OutputContinuousDataStream got !, success");
                retVal = true;
            } else if( ('?' == Buff[0]) )
                Serial.println("OutputContinuousDataStream got ?, failed");

            // expecting total length of 2
            if( (2 == strlen(Buff)) )
                Serial.println("OutputContinuousDataStream got expected length");
            else
                Serial.println("OutputContinuousDataStream got unexpected length");
        } else
        {
            Serial.println("OutputContinuousDataStream Rx failed");
        }
    } else
    {
        Serial.println("OutputContinuousDataStream Tx failed");
    }

    return(retVal);
}


bool RS232LSSeriesChiller::RS232LSSeriesChiller::TxCommand()
{
    uint8_t lenWritten;
    bool    retVal  = true;


    // class member Buff is filled in by the member functions
    lenWritten = Serial2.write(Buff);

    if( (lenWritten != strlen(Buff)) )
    {
        Serial.println("__PRETTY_FUNCTION__ failed");
        retVal  = false;
    #ifdef __DEBUG_PKT_TX__
    } else
    {
        Serial.println("RS232LSSeriesChiller::TxCommand success");
    #endif
    }
    
    return(retVal);
}


bool RS232LSSeriesChiller::RxResponse(char** retBuff, uint32_t TimeoutMs)
{
    bool retVal             = false;
    bool done               = false;
    bool gotSTX             = true;     // TODO: for now dont' find a defined start char
    bool gotETX             = false;
    bool timedOut           = false;
    int32_t bytes_read      = 0;
    const uint8_t STX       = '!';
    const uint8_t ETX       = '\r';
    unsigned long startTime = millis();
    char* pBuff             = (0 == retBuff ? 0 : *retBuff);


    // try to read a packet for a total of TimeoutMs milliseconds
    while( (!done) && (!timedOut) && (bytes_read < MAX_BUFF_LENGTH) )
    {
        if( ((millis() - startTime) > TimeoutMs) )
        {
            timedOut = true;
        } else
        {
            if( (Serial2.available()) )
            {
                Buff[bytes_read] = Serial2.read();

                if( (!gotSTX) )
                {
                    if( (STX == Buff[bytes_read]) )
                    {
                        // TODO: restart startTime here, give more time to get the packet?
                        gotSTX = true;
                        bytes_read += 1;
                    } // else don't increment bytes_read effectively discarding this byte
                } else
                {
                    if( (!gotETX) )
                    {
                        if( (ETX == Buff[bytes_read]) )
                        {
                            done        = true;
                            retVal      = true;
                        }

                        // this is a byte in the body of the frame
                        bytes_read += 1;
                    }
                }
            } else
            {
                // TODO: too long, too short ?
                // no data available, wait a bit before checking again
                delay(10);
            }
        }
    }

    // always null terminate just in case we want to dump out for debug
    Buff[bytes_read] = 0;

    // debug stuff
    #ifdef __DEBUG_PKT_RX__
    Serial.print("__PRETTY_FUNCTION__ received ");
    Serial.print(bytes_read, DEC);
    Serial.println(" bytes");
    #endif

    if( (0 != pBuff) )
        memcpy(pBuff, Buff, bytes_read);

    return(retVal);
}


//
// class definitions
//
//for modes see : https://www.arduino.cc/reference/en/language/functions/communication/serial/begin/
//
RS232LSSeriesChiller::RS232LSSeriesChiller(uint32_t SSerialRX, uint32_t SSerialTX, uint32_t Speed, uint32_t Config)
{
    // set the configuration upon start
    Serial2.begin(Speed);  // can't supply the config parameter, default is 8N1 though
}
    

RS232LSSeriesChiller::~RS232LSSeriesChiller() { };


bool RS232LSSeriesChiller::StartChiller()
{
    bool    retVal  = false;


    for(uint8_t i = 0; i < MAX_STARTUP_ATTEMPTS; i++)
    {
        //
        // turn off echo and turn off continuous data stream
        //
        if(SetCommandEcho('0') && (OutputContinuousDataStream('0')) )
        {
            // chiller is on-line, start the pump
            if(SetOnOff('1'))
            {
                //
                // documentation for the LS Series reads the pump will start 90 seconds after startup
                // .. so lets wait ??
                //
                #ifdef __DEBUG_LSSERIES_OPERATION__
                Serial.println("__PRETTY_FUNCTION__ sleeping 95 seconds while chiller starts");
                #endif
                delay(95000);
    
                //
                // verify the chiller status is 'Running'
                //
                if( (ChillerRunning()) )
                    retVal = true;
    
            #ifdef __DEBUG_LSSERIES_OPERATION__
            } else
            {
                Serial.println("__PRETTY_FUNCTION__ unable to SetOnOff");
            #endif
            }
    
        #ifdef __DEBUG_LSSERIES_OPERATION__
        } else
        {
            Serial.println("__PRETTY_FUNCTION__ unable to stop echo and cmd stream");
        #endif
        }
    }

    return(retVal);
}


bool RS232LSSeriesChiller::ChillerRunning()
{
    bool    retVal  = false;
    char    buff[MAX_BUFF_LENGTH + 1];
    char*   pBuff = buff;


    memset(buff, '\0', MAX_BUFF_LENGTH + 1);
    if(ReadStatus(&pBuff))
    {
        if('1' == buff[0])  // 1 is on, 0 is off
            retVal  = true;
        #ifdef __DEBUG_LSSERIES_OPERATION__
        else
        {
            Serial.print("__PRETTY_FUNCTION__ ReadStatus got ");
            Serial.println(buff[0]);
        }
        #endif
    #ifdef __DEBUG_LSSERIES_OPERATION__
    } else
    {
        Serial.println("__PRETTY_FUNCTION__ unable to ReadStatus");
    #endif
    }

    return(retVal);
}


bool RS232LSSeriesChiller::StopChiller()
{
    bool    retVal  = false;


    for(uint8_t i = 0; i < MAX_SHUTDOWN_ATTEMPTS; i++)
    {
        if( (SetOnOff('0')) )  // 0 is off, 1 is on
        {
            // TODO : test to find out whether the following delay is needed
            // or is too long or too short
            delay(5000);

            if( !(ChillerRunning()) )
                retVal = true;

        #ifdef __DEBUG_LSSERIES_OPERATION__
        } else
        {
            Serial.println("__PRETTY_FUNCTION__ unable to SetOnOff");
        #endif
        }
    }

    return(retVal);
}


bool RS232LSSeriesChiller::ChillerPresent(char** ppRetStr)
{
    bool    retVal    = false;
    char*   pRetStr   = (ppRetStr ? 0 : *ppRetStr);


    if( (ReadStatus(&pRetStr)) )
    {
        retVal = true;
    }

    return(retVal);
}

