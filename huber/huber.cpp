#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include <SoftwareSerial.h>
#include "huber.h"
#include <stdlib.h>
#include <string.h>


//SoftwareSerial mySerial2(0, 1);


//
// class definitions
//
huber::huber(uint32_t Speed)
{
    //
    // initialize slaveID to '01'
    //
    memset(reinterpret_cast<void*>(slaveID), '\0', MAX_SLAVE_ID_LENGTH + 1);
    strcpy(slaveID, "01");

    //
    // initialize the huberData
    //
    memset(reinterpret_cast<void*>(&huberData), '\0', sizeof(huberData));

    //
    // set the configuration upon start
    //
    //mySerial2.begin(Speed);
    Serial2.begin(Speed, SERIAL_8N1);
}
    

huber::~huber()
{
};


bool huber::TxCommand()
{
    uint8_t lenWritten;
    bool    retVal  = true;


    // clear the input buffer prior to sending to clear
    // out any old responses - happens on link fail tests
    ClearInputBuff(); 
    lenWritten = Serial2.write(Buff);
    Serial2.flush();

    if( (lenWritten != strlen(Buff)) )
    {
        #ifdef __DEBUG_HUBER_ERROR__
        Serial.println("TxCommand failed");
        #endif
        retVal  = false;
    #ifdef __DEBUG_PKT_TX__
    } else
    {
        Serial.println("TxCommand success");
    #endif
    }
    
    return(retVal);
}


//
// verify
//  - packet starts with  '[S'  for slave response
//  - lenght and check sum are correct
//
bool huber::RxResponse(char** retBuff, uint32_t TimeoutMs)
{
    bool retVal             = false;
    bool done               = false;
    bool gotSTX             = false;     // TODO: for now dont' find a defined start char
    bool gotETX             = false;
    bool timedOut           = false;
    int32_t bytes_read      = 0;
    const uint8_t STX       = '[';
    const uint8_t ETX       = '\r';
    unsigned long startTime = millis();
    char* pBuff             = (0 == retBuff ? 0 : *retBuff);


    memset(reinterpret_cast<void*>(Buff), '\0', MAX_BUFF_LENGTH + 1);

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
                        }

                        // this is a byte in the body of the frame
                        bytes_read += 1;
                    }
                }
            } else
            {
                // TODO: too long, too short ?
                // no data available, wait a bit before checking again
                //Serial.println("Serial2 no bytes available");
                delay(100);
            }
        }
    }

    // always null terminate just in case we want to dump out for debug
    Buff[bytes_read] = 0;

    //
    // verify received packet
    //
    if( (verifyLengthAndCheckSum()) )
    {
        retVal  = true;
    }

    // debug stuff
    #ifdef __DEBUG_PKT_RX__
    Serial.print(__PRETTY_FUNCTION__);
    Serial.flush();
    Serial.print(" received ");
    Serial.flush();
    Serial.print(bytes_read, DEC);
    Serial.flush();
    Serial.println(" bytes");
    Serial.flush();
    Serial.println(Buff);
    Serial.flush();
    #endif

    #ifdef __DEBUG_HUBER_ERROR__
    if( !(retVal) )
    {
        Serial.println("RxResponse found bad formatted packet");
        Serial.flush();
    }
    #endif

    if( (0 != pBuff) )
        memcpy(pBuff, Buff, bytes_read);

    return(retVal);
}


//
// send the verify command
// - pick up the slave ID and name
//
bool huber::sendVerifyCommand()
{
    bool    retVal      = false;
    bool    cmdTxAckRx  = false;
    uint8_t count       = 0;
    uint8_t retry       = 0;


    do
    {
        //
        // build the Verify command
        //
        memset(reinterpret_cast<void*>(Buff), '\0', MAX_BUFF_LENGTH + 1);
        count = 0;
        Buff[count++] = '[';
        Buff[count++] = 'M';
        Buff[count++] = slaveID[0];    // assuming slave address is "01"
        Buff[count++] = slaveID[1];
        Buff[count++] = 'V';
        Buff[count++] = 'x';    // setLengthAndCheckSum to fill in
        Buff[count++] = 'x';    // setLengthAndCheckSum to fill in
        Buff[count++] = 'x';    // setLengthAndCheckSum to fill in
        Buff[count++] = 'x';    // setLengthAndCheckSum to fill in
        Buff[count++] = '\r';
        Buff[count] = '\0';
    
        //
        // set the length of data and checksum
        //
        setLengthAndCheckSum();
    
        #ifdef __DEBUG_PKT_TX__
        Serial.print(__PRETTY_FUNCTION__);
        Serial.flush();
        Serial.print(" sending ");
        Serial.flush();
        Serial.print(count, DEC);
        Serial.flush();
        Serial.println(" bytes");
        Serial.flush();
        Serial.println(Buff);
        Serial.flush();
        #endif
    
        //
        // Tx the command
        //
        if( (TxCommand()) )
        {
            //
            // RxResponse also verifies the length and checksum
            //
            if( (RxResponse(0, 3500)) ) // wait max 3500ms for reply and no retBuff
            {
                //
                // packet length must at least be 11 bytes long - this is a one charater name
                //
                if( (strlen(Buff) >= MIN_VERIFY_RESPONSE_LENGTH) )
                {
                    //
                    // this is a validated packet, OK to proceed
                    // - pick up the slave ID and name
                    //
                    // verify command qualifier - should be 'V'
                    //
                    if( ('V' == Buff[COMMAND_QUALIFIER_INDEX]) )
                    {
                        //
                        // pick up the slave's address
                        //
                        strncpy(slaveID, &Buff[ADDRESS_INDEX], MAX_SLAVE_ID_LENGTH);
        
                        //
                        // pick up the name of the unit - should be the whole data section
                        //
                        strncpy(slaveName, &Buff[DATA_START_INDEX],
                            (strlen(Buff) - (DATA_START_INDEX + CHKSUM_PLUS_ETX_LENGTH)) );
        
                        #ifdef __DEBUG_HUBER__
                        Serial.print("sendVerifyCommand found slaveId ");
                        Serial.flush();
                        Serial.println(slaveID);
                        Serial.flush();
                        Serial.print("sendVerifyCommand found name ");
                        Serial.flush();
                        Serial.println(slaveName);
                        Serial.flush();
                        #endif
    
                        retVal      = true;
                        cmdTxAckRx  = true;
        
                    #ifdef __DEBUG_HUBER_ERROR__
                    } else
                    {
                        Serial.println("ERROR: sendVerifyCommand fail, did not get \'V\'");
                    #endif
                    }
                #ifdef __DEBUG_HUBER_ERROR__
                } else
                {
                    Serial.println("ERROR: sendVerifyCommand Rx'ed Verify response packet is too short");
                #endif
                }
            #ifdef __DEBUG_HUBER_ERROR__
            } else
            {
                Serial.println("ERROR: sendVerifyCommand RxResponse failed");
            #endif
            }
        #ifdef __DEBUG_HUBER_ERROR__
        } else
        {
            Serial.print(__PRETTY_FUNCTION__);
            Serial.println("ERROR: TxCommand failed");
        #endif
        }

        // re-load Buff with the original command
        // as a partial response could be in Buff
        if( !(cmdTxAckRx) )
        {
            #ifdef __DEBUG_HUBER_ERROR__
            Serial.println("--------- resending packet ----------");
            Serial.flush();
            #endif
        }

    } while( (!cmdTxAckRx) && (++retry < MAX_COMMAND_RETRY) );

    return(retVal);
}


//
// fetch the liimits of the chiller
//
// The Limit command should be used to query for the presence of a slave
//
bool huber::sendLimitCommand()
{
    bool    retVal      = false;
    bool    cmdTxAckRx  = false;
    uint8_t count       = 0;
    uint8_t retry       = 0;


    do
    {
        //
        // build the Limit command
        //
        // master: [M01L0F********1B\r
        // slave:  [S01L17F4484E20F4484E2045\r
        memset(reinterpret_cast<void*>(Buff), '\0', MAX_BUFF_LENGTH + 1);
        count = 0;
        Buff[count++] = '[';
        Buff[count++] = 'M';
        Buff[count++] = slaveID[0];    // assuming slave address is "01"
        Buff[count++] = slaveID[1];
        Buff[count++] = 'L';
        Buff[count++] = 'x';    // setLengthAndCheckSum to fill in
        Buff[count++] = 'x';    // setLengthAndCheckSum to fill in
        Buff[count++] = '*';    // asterisk is 'no change, give me the limits
        Buff[count++] = '*';
        Buff[count++] = '*';
        Buff[count++] = '*';
        Buff[count++] = '*';
        Buff[count++] = '*';
        Buff[count++] = '*';
        Buff[count++] = '*';
        Buff[count++] = 'x';    // setLengthAndCheckSum to fill in
        Buff[count++] = 'x';    // setLengthAndCheckSum to fill in
        Buff[count++] = '\r';
        Buff[count] = '\0';
    
        //
        // set the length of data and checksum
        //
        setLengthAndCheckSum();
    
        #ifdef __DEBUG_PKT_TX__
        Serial.print("sendLimitCommand is sending: ");
        Serial.flush();
        Serial.println(Buff);
        Serial.flush();
        #endif
    
        //
        // Tx the command
        //
        if( (TxCommand()) )
        {
            //
            // RxResponse also verifies the length and checksum
            //
            if( (RxResponse(0, 3500)) ) // wait max 3500ms for reply and no retBuff
            {
                //
                // packet length must at least be 11 bytes long - this is a one charater name
                //
                if( (strlen(Buff) >= MIN_LIMIT_RESPONSE_LENGTH) )
                {
                    //
                    // this is a validated packet, OK to proceed
                    // - pick up the slave ID and name
                    //
                    // verify command qualifier - should be 'V'
                    //
                    // slave:  [S01L17 F448 4E20 F448 4E20 45\r
                    if( ('L' == Buff[COMMAND_QUALIFIER_INDEX]) )
                    {
                        //
                        // pick up the lower setpoint limit, upper setpoint limit,
                        // lower working range limit, upper working range limit
                        //
                        strncpy(lowerSetPointLimit, &Buff[DATA_START_INDEX], MAX_LIMIT_LENGTH);
                        strncpy(upperSetPointLimit, &Buff[DATA_START_INDEX + 4], MAX_LIMIT_LENGTH);
                        strncpy(upperWorkingRangeLimit, &Buff[DATA_START_INDEX + 8], MAX_LIMIT_LENGTH);
                        strncpy(lowerWorkingRangeLimit, &Buff[DATA_START_INDEX + 12], MAX_LIMIT_LENGTH);
        
                        #ifdef __DEBUG_HUBER__
                        Serial.print("sendLimitCommand found following limits 0x");
                        Serial.flush();
                        Serial.print(lowerSetPointLimit);
                        Serial.flush();
                        Serial.print(" 0x");
                        Serial.flush();
                        Serial.println(upperSetPointLimit);
                        Serial.flush();
                        Serial.print(" 0x");
                        Serial.flush();
                        Serial.println(upperWorkingRangeLimit);
                        Serial.flush();
                        Serial.print(" 0x");
                        Serial.flush();
                        Serial.println(lowerWorkingRangeLimit);
                        Serial.flush();
                        #endif
    
                        retVal      = true;
                        cmdTxAckRx  = true;
        
                    #ifdef __DEBUG_HUBER_ERROR__
                    } else
                    {
                        Serial.println("ERROR: sendLimitCommand fail, did not get \'L\'");
                    #endif
                    }
                #ifdef __DEBUG_HUBER_ERROR__
                } else
                {
                    Serial.println("ERROR: sendLimitCommand Rx'ed Limit response packet is too short");
                #endif
                }
            #ifdef __DEBUG_HUBER_ERROR__
            } else
            {
                Serial.println("ERROR: sendLimitCommand RxResponse failed");
            #endif
            }
        #ifdef __DEBUG_HUBER_ERROR__
        } else
        {
            Serial.print(__PRETTY_FUNCTION__);
            Serial.println("ERROR: TxCommand failed");
        #endif
        }

        // re-load Buff with the original command
        // as a partial response could be in Buff
        if( !(cmdTxAckRx) )
        {
            #ifdef __DEBUG_HUBER_ERROR__
            Serial.println("--------- resending packet ----------");
            Serial.flush();
            #endif
        }

    } while( (!cmdTxAckRx) && (++retry < MAX_COMMAND_RETRY) );

    return(retVal);
}


//
// this will only send the General command as formatted
// by other functions, i.e. start/stop chiller commands
// and set up the huberData structure, the calling function
// will query the huberData structure for results . nah nah nah poo poo
//
// this expects the Buff to have the command to send already formatted for send
//
bool huber::sendGeneralCommand()
{
    bool    retVal      = false;
    bool    cmdTxAckRx  = false;
    uint8_t retry       = 0;


    // save a copy of the Tx packet for re-trasmits
    memcpy(static_cast<void*>(Bkup), static_cast<void*>(Buff), MAX_BUFF_LENGTH + 1);


    do
    {
        #ifdef __DEBUG_PKT_TX__
        Serial.print("sendGeneralCommand is sending: ");
        Serial.flush();
        Serial.println(Buff);
        Serial.flush();
        #endif

        //
        // Tx the command
        //
        if( (TxCommand()) )
        {
            //
            // RxResponse also verifies the length and checksum
            //
            if( (RxResponse(0, 3500)) ) // wait max 3500ms for reply and no retBuff
            {
                //
                // packet length must at least be 11 bytes long - this is a one charater name
                //
                if( (strlen(Buff) >= MIN_GENERAL_RESPONSE_LENGTH) )
                {
                    //
                    // example from doc ...
                    //
                    // *    - temp control mode and alarm status should remain unchanged 
                    // FE70 - and a setpoint of -4.00C is to be set
                    // master: [M01G 0D * * FE70 0A\r
                    //
                    // slave : [S01G 15 O 0 FE70 09A4 C504 E7\r
                    //  'O' - temperature control is turned off
                    //  '0' - there is no alarm
                    //  FE70 - set point of -4.00C was set
                    //  09A4 - the actual value is 24.68C
                    //  C504 - corresponds to -151.00C and indicates that no external temp sensor is install or connected
                    //
                    if( ('G' == Buff[COMMAND_QUALIFIER_INDEX]) )
                    {
                        //
                        // reset and fill in the huberData structure
                        //
                        //memset(reinterpret_cast<void*>(&huberData), '\0', sizeof(huberData));
    
                        //
                        // fill in the huberData structure
                        //
    
                        // get the temperature control mode
                        strncpy(huberData.tempCtrlMode, &Buff[TEMP_CTRL_MODE_INDEX],
                            MAX_TEMP_CTRL_MODE_LENGTH);
    
                        // get the alarm status
                        strncpy(huberData.alarmStatus, &Buff[ALARM_STATUS_INDEX],
                            MAX_ALARM_STATUS_LENGTH);
    
                        // get the set point status
                        strncpy(huberData.setPointTemp, &Buff[SETPOINT_STATUS_INDEX],
                            MAX_SET_POINT_LENGTH);
    
                        // get the internal actual value
                        strncpy(huberData.internalTemp, &Buff[INTERNAL_TEMP_INDEX],
                            MAX_INTERNAL_TEMP_LENGTH);
    
                        // get the external temperature
                        strncpy(huberData.externalTemp, &Buff[EXTERNAL_TEMP_INDEX],
                            MAX_EXTERNAL_TEMP_LENGTH);
    
                        #ifdef __DEBUG_HUBER__
                        Serial.println("sendGeneralCommand found the following:");
                        Serial.flush();
                        Serial.print("huberData.tempCtrlMode ");
                        Serial.flush();
                        Serial.println(huberData.tempCtrlMode);
                        Serial.flush();
                        Serial.print("huberData.alarmStatus 0x");
                        Serial.flush();
                        Serial.println(huberData.alarmStatus);
                        Serial.flush();
                        Serial.print("huberData.setPointTemp 0x");
                        Serial.flush();
                        Serial.println(huberData.setPointTemp);
                        Serial.flush();
                        Serial.print("huberData.internalTemp 0x");
                        Serial.flush();
                        Serial.println(huberData.internalTemp);
                        Serial.flush();
                        Serial.print("huberData.externalTemp 0x");
                        Serial.flush();
                        Serial.println(huberData.externalTemp);
                        Serial.flush();
                        #endif
    
                        retVal      = true;
                        cmdTxAckRx  = true;
        
                    #ifdef __DEBUG_HUBER_ERROR__
                    } else
                    {
                        Serial.println("ERROR: sendGeneralCommand fail, did not get \'G\'");
                    #endif
                    }
                #ifdef __DEBUG_HUBER_ERROR__
                } else
                {
                    Serial.println("ERROR: sendGeneralCommand Rx'ed Limit response packet is too short");
                #endif
                }
            #ifdef __DEBUG_HUBER_ERROR__
            } else
            {
                Serial.println("ERROR: sendGeneralCommand RxResponse failed");
            #endif
            }
        #ifdef __DEBUG_HUBER_ERROR__
        } else
        {
            Serial.print(__PRETTY_FUNCTION__);
            Serial.println("ERROR: TxCommand failed");
        #endif
        }

        // re-load Buff with the original command
        // as a partial response could be in Buff
        if( !(cmdTxAckRx) )
        {
            #ifdef __DEBUG_HUBER_ERROR__
            Serial.println("--------- resending packet ----------");
            Serial.flush();
            #endif
            memcpy(static_cast<void*>(Buff), static_cast<void*>(Bkup), MAX_BUFF_LENGTH + 1);
        }

    } while( (!cmdTxAckRx) && (++retry < MAX_COMMAND_RETRY) );

    return(retVal);
}



//
// helper functions
//
bool huber::GetAllChillerInfo()
{
    bool    retVal  = true;


    // try to get all stats
    if( !(sendVerifyCommand()) )
    {
        #ifdef __DEBUG_HUBER_ERROR_
        Serial.println("GetAllChillerInfo unable to sendVerifyCommand");
        #endif
        retVal = false;
    }

    if( !(sendLimitCommand()) )
    {
        #ifdef __DEBUG_HUBER_ERROR_
        Serial.println("GetAllChillerInfo unable to sendLimitCommand");
        #endif
        retVal = false;
    }

    if( !(getChillerStatus()) )  // general command, all '*'
    {
        #ifdef __DEBUG_HUBER_ERROR_
        Serial.println("GetAllChillerInfo unable to getChillerStatus");
        #endif
        retVal = false;
    }

    return(retVal);
}


//
// send all '*' using G command 
//
bool huber::getChillerStatus()
{
    bool    retVal  = false;
    uint8_t count   = 0;


    //
    // send '*' for temp control, alarm query, and set point - just
    // pick up the values, update the huberData structure
    //
    memset(reinterpret_cast<void*>(Buff), '\0', MAX_BUFF_LENGTH + 1);
    Buff[count++] = '[';
    Buff[count++] = 'M';
    Buff[count++] = slaveID[0];    // assuming slave address is "01"
    Buff[count++] = slaveID[1];
    Buff[count++] = 'G';
    Buff[count++] = 'x';    // length setLengthAndCheckSum to fill in
    Buff[count++] = 'x';    // length setLengthAndCheckSum to fill in
    Buff[count++] = '*';    // Temp control mode - turn on internal temperature control
    Buff[count++] = '*';    // Alarms query, don't change current state
    Buff[count++] = '*';    // Query set point
    Buff[count++] = '*';    // Query set point
    Buff[count++] = '*';    // Query set point
    Buff[count++] = '*';    // Query set point
    Buff[count++] = 'x';    // check sum setLengthAndCheckSum to fill in
    Buff[count++] = 'x';    // check sum setLengthAndCheckSum to fill in
    Buff[count++] = '\r';
    Buff[count] = '\0';

    setLengthAndCheckSum();

    if( (sendGeneralCommand()) )
    {
        retVal = true;
    #ifdef __DEBUG_HUBER_ERROR_
    } else
    {
        Serial.println("ERROR: getChillerStatus failed sendGeneralCommand");
        Serial.flush();
    #endif
    }

    return(retVal);
}


// TODO: do both of these to 'turn on the chiller'? :
//  -  'C' enable fluid circulation
//  -  'I' enable internal temperature control
//
bool huber::StartChiller()
{
    bool    retVal  = false;
    uint8_t count   = 0;


    //
    // enable the internal temp control and the pump for StartChiller
    //

    //
    // enable internal temperature control
    //
    memset(reinterpret_cast<void*>(Buff), '\0', MAX_BUFF_LENGTH + 1);
    Buff[count++] = '[';
    Buff[count++] = 'M';
    Buff[count++] = slaveID[0];    // assuming slave address is "01"
    Buff[count++] = slaveID[1];
    Buff[count++] = 'G';
    Buff[count++] = 'x';    // length setLengthAndCheckSum to fill in
    Buff[count++] = 'x';    // length setLengthAndCheckSum to fill in
    Buff[count++] = 'I';    // Temp control mode - turn on internal temperature control
    Buff[count++] = '*';    // Alarms query, don't change current state
    Buff[count++] = '*';    // Query set point
    Buff[count++] = '*';    // Query set point
    Buff[count++] = '*';    // Query set point
    Buff[count++] = '*';    // Query set point
    Buff[count++] = 'x';    // check sum setLengthAndCheckSum to fill in
    Buff[count++] = 'x';    // check sum setLengthAndCheckSum to fill in
    Buff[count++] = '\r';
    Buff[count] = '\0';

    setLengthAndCheckSum();

    if( (sendGeneralCommand()) )
    {
        //
        //  TODO : test with real Huber chiller !
        // check the huberData structure, it was updated by the sendGeneralCommand
        // 
        #ifdef __DEBUG_HUBER__
        if( ('I' == huberData.tempCtrlMode[0]) )
            Serial.println("StartChiller found \'I\' assuming internal temp ctrl is on");
        #endif

        //
        // switch circulation on
        //
        memset(reinterpret_cast<void*>(Buff), '\0', MAX_BUFF_LENGTH + 1);
        count = 0;
        Buff[count++] = '[';
        Buff[count++] = 'M';
        Buff[count++] = slaveID[0];    // assuming slave address is "01"
        Buff[count++] = slaveID[1];
        Buff[count++] = 'G';
        Buff[count++] = 'x';    // length setLengthAndCheckSum to fill in
        Buff[count++] = 'x';    // length setLengthAndCheckSum to fill in
        Buff[count++] = 'C';    // Temp control mode - turn on internal temperature control
        Buff[count++] = '*';    // Alarms query, don't change current state
        Buff[count++] = '*';    // Query set point
        Buff[count++] = '*';    // Query set point
        Buff[count++] = '*';    // Query set point
        Buff[count++] = '*';    // Query set point
        Buff[count++] = 'x';    // check sum setLengthAndCheckSum to fill in
        Buff[count++] = 'x';    // check sum setLengthAndCheckSum to fill in
        Buff[count++] = '\r';
        Buff[count] = '\0';

        setLengthAndCheckSum();

        if( (sendGeneralCommand()) )
        {
            //
            //  TODO : test with real Huber chiller !
            // check the huberData structure, it was updated by the sendGeneralCommand
            // 
            #ifdef __DEBUG_HUBER_ERROR__
            if( ('C' == huberData.tempCtrlMode[0]) )
                Serial.println("StartChiller found \'C\' assuming circulation is on");
            #endif

            retVal  = true;

        #ifdef __DEBUG_HUBER_ERROR__
        } else
        {
            Serial.println("StartChiller failed sendGeneralCommand to enable circulation");
        #endif
        }

    #ifdef __DEBUG_HUBER_ERROR__
    } else
    {
        Serial.println("StartChiller failed sendGeneralCommand for internal temp ctrl");
    #endif
    }

    return(retVal);
}


bool huber::ChillerRunning()
{
    bool    retVal  = false;


    //
    // fetch the chiller's status and update the huberData structure
    //
    if( (getChillerStatus()) )
    {
        //
        // check the huberData structure, it is updated on every
        // call to sendGeneralCommand
        //
        if( ('O' != huberData.tempCtrlMode[0]) )
            retVal = true;
        #ifdef __DEBUG_HUBER_ERROR__
        else
        {
            Serial.print("ERROR :: ChillerRunning fail, got mode: ");
            Serial.println(huberData.tempCtrlMode);
        }
        #endif
    #ifdef __DEBUG_HUBER_ERROR__
    } else
    {
        Serial.println("ChillerRunning unable to getChillerStatus()");
    #endif
    }

    
    #ifdef __DEBUG_HUBER__
    Serial.print("ChillerRunning returning ");
    Serial.println(retVal, DEC);
    #endif
    return(retVal);
}


bool huber::StopChiller()
{
    bool    retVal  = false;
    uint8_t count   = 0;


    //
    // turn temperature control off
    //
    memset(reinterpret_cast<void*>(Buff), '\0', MAX_BUFF_LENGTH + 1);
    Buff[count++] = '[';
    Buff[count++] = 'M';
    Buff[count++] = slaveID[0];    // assuming slave address is "01"
    Buff[count++] = slaveID[1];
    Buff[count++] = 'G';
    Buff[count++] = 'x';    // length setLengthAndCheckSum to fill in
    Buff[count++] = 'x';    // length setLengthAndCheckSum to fill in
    Buff[count++] = 'O';    // Temp control mode - turn on internal temperature control
    Buff[count++] = '*';    // Alarms query, don't change current state
    Buff[count++] = '*';    // Query set point
    Buff[count++] = '*';    // Query set point
    Buff[count++] = '*';    // Query set point
    Buff[count++] = '*';    // Query set point
    Buff[count++] = 'x';    // check sum setLengthAndCheckSum to fill in
    Buff[count++] = 'x';    // check sum setLengthAndCheckSum to fill in
    Buff[count++] = '\r';
    Buff[count] = '\0';

    setLengthAndCheckSum();

    if( (sendGeneralCommand()) )
    {
        //
        //  TODO : test with real Huber chiller !
        // check the huberData structure, it was updated by the sendGeneralCommand
        // 
        if( ('O' == huberData.tempCtrlMode[0]) )
        {
            retVal = true;
        #ifdef __DEBUG_HUBER_ERROR__
        } else
        {
            Serial.println("StopChiller did not found \'O\' assuming chiller is still running");
        #endif
        }

    #ifdef __DEBUG_HUBER_ERROR__
    } else
    {
        Serial.println("StopChiller failed sendGeneralCommand for internal temp ctrl");
    #endif
    }

    return(retVal);
}


bool huber::ChillerPresent()
{
    //
    // sendVerifyCommand()
    //
    return(sendVerifyCommand());
}


bool huber::SetSetPoint(const char* pSetPoint)
{
    bool        retVal  = false;
    uint8_t     count   = 0;
    float       setTemp;
    uint16_t    int1, int2;
    int         written;


    //
    // use sendGeneralCommand
    //
    memset(reinterpret_cast<void*>(Buff), '\0', MAX_BUFF_LENGTH + 1);
    Buff[count++] = '[';
    Buff[count++] = 'M';
    Buff[count++] = slaveID[0];    // assuming slave address is "01"
    Buff[count++] = slaveID[1];
    Buff[count++] = 'G';
    Buff[count++] = 'x';    // length setLengthAndCheckSum to fill in
    Buff[count++] = 'x';    // length setLengthAndCheckSum to fill in
    Buff[count++] = '*';    // Temp control mode - query only
    Buff[count++] = '*';    // Alarms query, don't change current state


    // convert pSetPoint to ASCII HEX representation
    setTemp = atof(pSetPoint);
    int1    = setTemp * (float)100; // rid the funky float conversion past 2 decimals

    written = snprintf(reinterpret_cast<char*>(&Buff[count]), 5, "%X", int1);

    if( (1 > written) ) // all zeros
    {
        Buff[count++] = '0'; Buff[count++] = '0'; Buff[count++] = '0'; Buff[count++] = '0';
    } else if( (2 > written) ) // shift 3
    {
        Buff[count++] = '0'; Buff[count++] = '0'; Buff[count++] = '0';
        snprintf(reinterpret_cast<char*>(&Buff[count]), 2, "%X", int1);
        count += 1;
    } else if( (3 > written) ) // shift 2
    {
        Buff[count++] = '0'; Buff[count++] = '0';
        snprintf(reinterpret_cast<char*>(&Buff[count]), 3, "%X", int1);
        count += 2;
    }
    else if( (4 > written) ) //shift 1
    {
        Buff[count++] = '0';
        snprintf(reinterpret_cast<char*>(&Buff[count]), 4, "%X", int1);
        count += 3;
    } else
    {
        count += 4;
    }

    Buff[count++] = 'x';    // check sum setLengthAndCheckSum to fill in
    Buff[count++] = 'x';    // check sum setLengthAndCheckSum to fill in
    Buff[count++] = '\r';
    Buff[count] = '\0';

    setLengthAndCheckSum();

    if( (sendGeneralCommand()) )
    {
        //
        //  TODO : test with real Huber chiller !
        // check the huberData structure, it was updated by the sendGeneralCommand
        // 
        int2    = GetSetPointFloat() * (float)100; // rid the funky float conversion past 2 decimals

        if( (int1 == int2) )
        {
            #ifdef __DEBUG_HUBER2__
            Serial.print(__PRETTY_FUNCTION__);
            Serial.print(" SetSetPoint success, changed to 0x");
            Serial.println(huberData.setPointTemp);
            #endif

            retVal = true;

        #ifdef __DEBUG_HUBER__
        } else
        {
            Serial.print("ERROR: SetSetPoint failed, huber still reporting set point of 0x");
            Serial.println(huberData.setPointTemp);
        #endif
        }

    #ifdef __DEBUG_HUBER_ERROR__
    } else
    {
        Serial.print("ERROR: SetSetPoint failed, sendGeneralCommand failed");
    #endif
    }
    return(retVal);
}


const char* huber::GetSetPoint() const
{
    return(huberData.setPointTemp);
}


const char* huber::GetInternalTemp() const
{
    return(huberData.internalTemp);
}


const char* huber::GetExternalTemp() const
{
    return(huberData.externalTemp);
}


char huber::GetTempCtrlMode() const
{
    return(huberData.tempCtrlMode[0]);
}


float huber::GetSetPointFloat() const
{
    float retVal;
    int16_t converted;


    //
    // i find sptintf support not great on uC, going
    // with atof for now
    //
    sscanf(reinterpret_cast<const char*>(huberData.setPointTemp),"%04x", &converted);
    retVal = (float)converted / (float)100.00;

    return(retVal);
}


float huber::GetInternalTempFloat() const
{
    float retVal;
    int16_t converted;


    //
    // i find sptintf support not great on uC, going
    // with atof for now
    //
    sscanf(reinterpret_cast<const char*>(huberData.internalTemp),"%04x", &converted);
    retVal = (float)converted / (float)100.00;

    return(retVal);
}


float huber::GetExternalTempFloat() const
{
    float retVal;
    int16_t converted;


    //
    // i find sptintf support not great on uC, going
    // with atof for now
    //
    sscanf(reinterpret_cast<const char*>(huberData.externalTemp),"%04x", &converted);
    retVal = (float)converted / (float)100.00;

    return(retVal);
}


const char* huber::GetAlarms() const
{
    return(huberData.alarmStatus);
}


const char* huber::GetSlaveName() const
{
    return(slaveName);
}


void huber::setLengthAndCheckSum()
{
    char        intStr[5]   = {0, 0, 0, 0, 0};
    int16_t     chkSum      = 0;
    int16_t     chkSumIndex;

    //
    // convert integer to char[], chkSumIndex is buffLen - chkSum len - '\r'
    //
    chkSumIndex = (strlen(Buff) > 3 ? strlen(Buff) - 3 : 0);
    sprintf(intStr, "%02X", chkSumIndex);


    #ifdef __DEBUG_HUBER__
    Serial.print("setLenghthAndCheckSum setting length to: ");
    Serial.flush();
    Serial.println(intStr);
    Serial.flush();
    Serial.print("where first two bytes are: ");
    Serial.flush();
    Serial.print(intStr[0]);
    Serial.flush();
    Serial.println(intStr[1]);
    Serial.flush();
    #endif

    //
    // set the length  - always bytes 5 and 6 - zero based count
    //
    Buff[LENGTH_INDEX]      = intStr[0];
    Buff[LENGTH_INDEX + 1]  = intStr[1];


    //
    // now calculate the checkSum including the length bytes just put in
    //
    for(int i = 0; i < chkSumIndex; i++)
        chkSum += static_cast<uint8_t>(Buff[i]);

    //
    // convert chkSum to a string
    //
    memset(reinterpret_cast<void*>(intStr), '\0', 5);
    sprintf(intStr, "%04X", chkSum);  // a hex number you numb-skull !

    #ifdef __DEBUG_HUBER__
    Serial.print("setLengthAndCheckSum setting chkSum to lower byte of: 0x");
    Serial.println(chkSum, HEX);
    Serial.print("where the first 4 bytes are: ");
    Serial.flush();
    Serial.print(intStr[0]);
    Serial.flush();
    Serial.print(intStr[1]);
    Serial.flush();
    Serial.print(intStr[2]);
    Serial.flush();
    Serial.println(intStr[3]);
    Serial.flush();
    #endif

    //
    // depending on the magnitude of the cheksum, take the
    // lower byte
    //
    if( (chkSum < 0x0100) )
    {
        Buff[chkSumIndex++] = intStr[0];
        Buff[chkSumIndex++] = intStr[1];
    } else
    {
        Buff[chkSumIndex++] = intStr[2];
        Buff[chkSumIndex++] = intStr[3];
    }
}


bool huber::verifyLengthAndCheckSum()
{
    int32_t     buffLength      = strlen(Buff);
    int32_t     packetLength    = 0;    // lenght from the packet
    uint16_t    packetChkSum    = 0;    // check sum from packet
    uint16_t    chkSum          = 0;    // calculated check sum
    int32_t     chkSumIndex     = (buffLength > 3 ? buffLength - 3 : 0);
    bool        lengthValid     = false;
    bool        checkSumValid   = false;
    bool        retVal          = false;
    char        intStr[5]       = {0, 0, 0, 0, 0};


    //
    // length should be total packet length minus check sum bytes and trailing '\r'
    //
    // pick up length from the packet bytes - it is hex
    //
    sscanf(reinterpret_cast<char*>(&Buff[LENGTH_INDEX]), "%2hX", &packetLength);

    #ifdef __DEBUG_HUBER__
    Serial.print(__PRETTY_FUNCTION__);
    Serial.flush();
    Serial.print(" found length from packet: ");
    Serial.flush();
    Serial.println(packetLength, HEX);
    Serial.flush();
    #endif

    //
    // strlenth(Buff) - 3   should equal reportedLength
    //
    if( (packetLength != strlen(Buff) - 3) )
    {
        #ifdef __DEBUG_HUBER__
        Serial.print(__PRETTY_FUNCTION__);
        Serial.flush();
        Serial.print(" ERROR: length mismatch : 0x" );
        Serial.flush();
        Serial.println( (strlen(Buff) - 3), HEX);
        Serial.flush();
        #endif
    } else
    {
        lengthValid = true;
    }

    //
    // calculate the packet's check sum
    //
    for(int i = 0; i < chkSumIndex; i++)
        chkSum += static_cast<uint8_t>(Buff[i]);

    #ifdef __DEBUG_HUBER__
    Serial.print("verifyLengthAndCheckSum calc'ed chkSum is: 0x");
    Serial.println(chkSum, HEX);
    #endif
    
    //
    // convert the checksum from the packet to an integer, this is the lower byte of a word value
    //
    sscanf(reinterpret_cast<char*>(&Buff[chkSumIndex]), "%02hX", &packetChkSum);

    //
    // crap, I don't know if the processor is big-endian or little endian ..
    // so, take the long way, convert to string, take the lower string nibble
    //
    memset(reinterpret_cast<void*>(intStr), '\0', 5);
    sprintf(intStr, "%04X", chkSum);
    
    //
    // using 0 extended sprintf, always take the lower two chars as calc'ed check sum
    //
    sscanf(reinterpret_cast<char*>(&intStr[2]), "%2hX", &chkSum);

    //
    // now have calculated chkSum and packetChkSum, compare those f*ckers !
    //
    #ifdef __DEBUG_HUBER__
    Serial.print("verifyLengthAndCheckSum comparing chkSum and packetChkSum: 0x");
    Serial.flush();
    Serial.print(chkSum, HEX);
    Serial.flush();
    Serial.print(" 0x");
    Serial.flush();
    Serial.println(packetChkSum, HEX);
    Serial.flush();
    #endif

    if( (chkSum == packetChkSum) )
        checkSumValid = true;

    if( (checkSumValid && lengthValid) )
        retVal  = true;

    return(retVal);
}


void huber::ClearInputBuff()
{
    while( (Serial2.available()) )
        Serial2.read();
}

