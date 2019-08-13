// file controlProtocol.cpp
#include "controlProtocol.h"

#ifdef __RUNNING_ON_CONTROLLINO__
    #if defined(ARDUINO) && ARDUINO >= 100
        #include "Arduino.h"
    #else
        #include "WProgram.h"
    #endif
#endif


#include <stdio.h>
#include <stdlib.h>
#ifdef __RUNNING_ON_CONTROLLINO__
#else
#include <arpa/inet.h>
#endif

#include "crc16.h"
#ifdef __USING_LINUX_USB__
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#endif



bool controlProtocol::openUSBPort(const char* usbPort)
{
    bool    retVal  = false;

#ifdef __USING_LINUX_USB__

    struct  termios options;


    m_fd = open(usbPort, O_RDWR | O_NOCTTY | O_NDELAY);
    if( (m_fd == -1) )
    {
        fprintf(stderr, "%s unable to open %s\n", __PRETTY_FUNCTION__, usbPort);
        retVal  = false;
    } else
    {
        // FNDELAY makes the fd non-blocking, doing a blocking calls
        fcntl(m_fd, F_SETFL, 0);
        retVal  = true;
    }

    //
    // and set 9600N81
    //
    tcgetattr(m_fd, &options);

    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);

    // Enable the receiver and set local mode...
    options.c_cflag |= (CLOCAL | CREAD);

    // 8 data bits
    options.c_cflag &= ~CSIZE; /* Mask the character size bits */
    options.c_cflag |= CS8;    /* Select 8 data bits */

    // no parity
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    // no hardware flow control
    options.c_cflag &= ~CRTSCTS;

    // use raw input
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // no software flow control
    options.c_iflag &= ~(IXON | IXOFF | IXANY);

    // raw output
    options.c_oflag &= ~OPOST;

    // have 5 second timeout
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME]= 50;

    //Set the new options for the port...
    tcsetattr(m_fd, TCSANOW, &options);
#endif
    return(retVal);
}


//
// TODO: throw if this fails
//
controlProtocol::controlProtocol(uint16_t myAddress, uint16_t peerAddress, const char* usbPort)
    : m_seqNum(0x0000), m_myAddress(myAddress), m_peerAddress(peerAddress)
{
    //
    // only coding this for the master for now .. 
    //
    TxCommand   = &controlProtocol::TxCommandUSB;
    RxResponse  = &controlProtocol::RxResponseUSB;

    //
    // assuming this always passes TODO: don't assume
    //
    openUSBPort(usbPort);
};


//
// TODO: throw if this fails
//
controlProtocol::controlProtocol(uint16_t myAddress, uint16_t peerAddress)
    : m_seqNum(0x0000), m_myAddress(myAddress), m_peerAddress(peerAddress)
{
    //
    // for now this will always be the Controllino
    //
    RxCommand   = &controlProtocol::RxCommandSerial;
    TxResponse  = &controlProtocol::TxResponseSerial;

    #ifdef __RUNNING_ON_CONTROLLINO__
    Serial2.begin(9600, SERIAL_8N1);
    #endif
};


controlProtocol::~controlProtocol()
{
#ifdef __USING_LINUX_USB__
    close(m_fd);
#endif
};


bool controlProtocol::TxCommandUSB(uint16_t length)
{

#ifdef __USING_LINUX_USB__

    #ifdef __DEBUG_CTRL_PROTO__
    printf(__PRETTY_FUNCTION__);
    printf("\n");
    printf("writing %u bytes: ", length);
    for(int i = 0; i < length; i++)
    {
        printf("0x%02X ", m_buff[i]);
    }
    printf("\n");
    #endif

    int n = write(m_fd, m_buff, length);

    if( (n < 0) )
    {
        printf("%s failed with 0x%x\n", __PRETTY_FUNCTION__, n);
    }

#endif

    return(true);
}


bool controlProtocol::RxResponseUSB(uint16_t timeout)
{
    uint32_t    nbytes  = 0;
    uint8_t*    bufptr;

#ifdef __USING_LINUX_USB__
    struct  termios options;

    printf(__PRETTY_FUNCTION__);
    printf("\n");

    //
    // set the passed in timeout - VTIME wants 10ths of a second
    // TODO: assuming the caller will provide milliseconds !
    //
    tcgetattr(m_fd, &options);          // get the attribures (all)
    options.c_cc[VMIN] = 0;             // update the VMIN and VTIME
    options.c_cc[VTIME]= timeout / 100; 
    tcsetattr(m_fd, TCSANOW, &options); // set the attributes

    //
    // clear da buffa' bra'
    //
    memset(m_buff, '\0', MAX_BUFF_LENGTH_CP + 1);

    bufptr = m_buff;
    while ((nbytes = read(m_fd, bufptr, m_buff + sizeof(m_buff) - bufptr - 1)) > 0)
    {
      bufptr += nbytes;
      if (bufptr[-1] == '\n' || bufptr[-1] == '\r')
        break;
    }

   /* nul terminate the string and see if we got an OK response */
    *bufptr = '\0';

#endif

    return(true);
}


bool controlProtocol::RxCommandSerial(uint16_t TimeoutMs)
{
    printf(__PRETTY_FUNCTION__);
    printf("\n");

    bool retVal             = false;
#ifdef __RUNNING_ON_CONTROLLINO__
    bool done               = false;
    bool gotSTX             = false;     // TODO: for now dont' find a defined start char
    bool gotETX             = false;
    bool timedOut           = false;
    int32_t bytes_read      = 0;
    const uint8_t STX       = COMMAND;
    const uint8_t ETX       = '\r';
    unsigned long startTime = millis();


    memset(reinterpret_cast<void*>(m_buff), '\0', MAX_BUFF_LENGTH_CP + 1);

    // try to read a packet for a total of TimeoutMs milliseconds
    while( (!done) && (!timedOut) && (bytes_read < MAX_BUFF_LENGTH_CP) )
    {
        if( ((millis() - startTime) > TimeoutMs) )
        {
            timedOut = true;
        } else
        {
            if( (Serial2.available()) )
            {
                m_buff[bytes_read] = Serial2.read();

                if( (!gotSTX) )
                {
                    if( (STX == m_buff[bytes_read]) )
                    {
                        // TODO: restart startTime here, give more time to get the packet?
                        gotSTX = true;
                        bytes_read += 1;
                    } // else don't increment bytes_read effectively discarding this byte
                } else
                {
                    if( (!gotETX) )
                    {
                        if( (ETX == m_buff[bytes_read]) )
                        {
                            done        = true;
                            retVal  = true; // TODO: only true of lenght and checksum are good

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
    m_buff[bytes_read] = 0;

    //
    // TODO: verify received packet
    //
/*
    if( (verifyLengthAndCheckSum()) )
    {
        retVal  = true;
    }
*/

    // debug stuff
    #ifdef __DEBUG_CONTROL_PKT_RX__
    Serial.print(__PRETTY_FUNCTION__);
    Serial.flush();
    Serial.print(" received ");
    Serial.flush();
    Serial.print(bytes_read, DEC);
    Serial.flush();
    Serial.println(" bytes");
    Serial.flush();
    for(int i = 0; i < bytes_read; i++)
    {
        Serial.print(m_buff[i], HEX);
        Serial.print(" ");
    }
    Serial.println("");
    Serial.flush();
    #endif

    #ifdef __DEBUG_CONTROL_ERROR__
    if( !(retVal) )
    {
        Serial.println("RxCommand found bad formatted packet");
        Serial.flush();
    }
    #endif

#endif
    return(retVal);
}


bool controlProtocol::TxResponseSerial(uint16_t length)
{
    printf(__PRETTY_FUNCTION__);
    printf("\n");

    bool    retVal  = true;
#ifdef __RUNNING_ON_CONTROLLINO__
    uint8_t lenWritten;

    // class member Buff is filled in by the member functions
    lenWritten = Serial2.write(m_buff, length);
    Serial2.flush();

    if( (lenWritten != length) )
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

#endif

    return(retVal);
}


bool controlProtocol::GetStatus(uint16_t destAddress, uint16_t* humidityAlert,
                            uint16_t* TECsRunning,  uint16_t* chillerOnLine)
{
    bool                retVal  = false;
    uint16_t            seqNum;
    msgHeader_t*        pMsgHeader;
    getStatusResp_t*    pgetStatusResp;


    //
    // increment the sequence number for this transaction
    //
    ++m_seqNum;

    if( (doTxCommand(Make_getStatus(destAddress, m_buff))) )
    {   
        //
        // save the seqNum
        //
        pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
        seqNum  = ntohs(pMsgHeader->seqNum);

        // get the return packet
        if( (doRxResponse(5000)) )
        {
            #ifdef __DEBUG_CTRL_PROTO__
            //
            // dump out what we got
            //
            for(uint16_t i = 0; i < sizeof(getStatusResp_t); i++)
            {
                printf("0x%02X ", m_buff[i]);
            }
            printf("\n");
            #endif

            //
            // check got the expected message number
            //
            pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
            if( (getStatusResp != ntohs(pMsgHeader->msgNum)) )
            {
                fprintf(stderr, "ERROR: %s got unexpected msg %hu\n",
                    __PRETTY_FUNCTION__, ntohs(pMsgHeader->msgNum));

                //
                // no need to continue processing
                //
                return(false);
            }

            //
            // cast into the buffer, pick up the CRC
            //
            pgetStatusResp = reinterpret_cast<getStatusResp_t*>(m_buff);
            

            //
            // verify seqNum and CRC
            //
            if( !(verifyMessage(len_getStatusResp_t,
                        ntohs(pgetStatusResp->crc), seqNum)) )
            {
                // TODO: drop the packet
                fprintf(stderr, "ERROR: %s CRC bad, seqNum mismatch, or wrong address\n",
                        __PRETTY_FUNCTION__);

                //
                // no need to continue processing
                //
                return(false);
            }


            //
            // report the health 
            //
            Parse_getStatusResp(m_buff, humidityAlert, TECsRunning, chillerOnLine, &seqNum);

            printf("found in packet humidityAlert %u TECsRunning %u, chillerOnLine %u, seqNumer 0x%02x\n",
                *humidityAlert, *TECsRunning, *chillerOnLine, seqNum);
            
            retVal  = true;
        } else
        {
            printf("ERROR: did not get a m_buffer back\n");
        }

    } else
    {
        printf("ERROR: unable to Make_getStatus\n");
    }

    return(retVal);
}


bool controlProtocol::GetHumidity(uint16_t destAddress, float* humidity)
{
    bool                retVal  = false;
    uint16_t            seqNum;
    msgHeader_t*        pMsgHeader;
    getHumidityResp_t*    pgetHumidityResp;


    //
    // increment the sequence number for this transaction
    //
    ++m_seqNum;

    if( (doTxCommand(Make_getHumidity(destAddress, m_buff))) )
    {   
        //
        // save the seqNum
        //
        pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
        seqNum  = ntohs(pMsgHeader->seqNum);

        // get the return packet
        if( (doRxResponse(5000)) )
        {
            #ifdef __DEBUG_CTRL_PROTO__
            //
            // dump out what we got
            //
            for(uint16_t i = 0; i < sizeof(getHumidityResp_t); i++)
            {
                printf("0x%02X ", m_buff[i]);
            }
            printf("\n");
            #endif

            //
            // check got the expected message number
            //
            pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
            if( (getHumidityResp != ntohs(pMsgHeader->msgNum)) )
            {
                fprintf(stderr, "ERROR: %s got unexpected msg %hu\n",
                    __PRETTY_FUNCTION__, ntohs(pMsgHeader->msgNum));

                //
                // no need to continue processing
                //
                return(false);
            }

            //
            // cast into the buffer, pick up the CRC
            //
            pgetHumidityResp = reinterpret_cast<getHumidityResp_t*>(m_buff);
            

            //
            // verify seqNum and CRC
            //
            if( !(verifyMessage(len_getHumidityResp_t,
                        ntohs(pgetHumidityResp->crc), seqNum)) )
            {
                // TODO: drop the packet
                fprintf(stderr, "ERROR: %s CRC bad, seqNum mismatch, or wrong address\n",
                        __PRETTY_FUNCTION__);

                //
                // no need to continue processing
                //
                return(false);
            }


            Parse_getHumidityResp(m_buff, humidity, &seqNum);

            printf("found in packet humidity %f, seqNumer 0x%02x\n", *humidity, seqNum);
            
            retVal  = true;
        } else
        {
            printf("ERROR: did not get a m_buffer back\n");
        }

    } else
    {
        printf("ERROR: unable to Make_getStatus\n");
    }

    return(retVal);
}


bool controlProtocol::SetHumidityThreshold(uint16_t destAddress, uint16_t threshold)
{
    bool                retVal  = false;
    uint16_t            seqNum;
    uint16_t            result;
    msgHeader_t*        pMsgHeader;
    setHumidityThresholdResp_t*    psetHumidityThresholdResp;


    //
    // increment the sequence number for this transaction
    //
    ++m_seqNum;

    if( (doTxCommand(Make_setHumidityThreshold(destAddress, m_buff, threshold))) )
    {   
        //
        // save the seqNum
        //
        pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
        seqNum  = ntohs(pMsgHeader->seqNum);

        // get the return packet
        if( (doRxResponse(5000)) )
        {
            #ifdef __DEBUG_CTRL_PROTO__
            //
            // dump out what we got
            //
            for(uint16_t i = 0; i < sizeof(setHumidityThresholdResp_t); i++)
            {
                printf("0x%02X ", m_buff[i]);
            }
            printf("\n");
            #endif

            //
            // check got the expected message number
            //
            pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
            if( (setHumidityThresholdResp != ntohs(pMsgHeader->msgNum)) )
            {
                fprintf(stderr, "ERROR: %s got unexpected msg %hu\n",
                    __PRETTY_FUNCTION__, ntohs(pMsgHeader->msgNum));

                //
                // no need to continue processing
                //
                return(false);
            }

            //
            // cast into the buffer, pick up the CRC
            //
            psetHumidityThresholdResp = reinterpret_cast<setHumidityThresholdResp_t*>(m_buff);
            

            //
            // verify seqNum and CRC
            //
            if( !(verifyMessage(len_setHumidityThresholdResp_t,
                        ntohs(psetHumidityThresholdResp->crc), seqNum)) )
            {
                // TODO: drop the packet
                fprintf(stderr, "ERROR: %s CRC bad, seqNum mismatch, or wrong address\n",
                        __PRETTY_FUNCTION__);

                //
                // no need to continue processing
                //
                return(false);
            }


            //
            // report the health 
            //
            Parse_setHumidityThresholdResp(m_buff, &result, &seqNum);

            printf("found in packet result %d seqNumer 0x%02x\n", result, seqNum);
            
            retVal  = true;
        } else
        {
            printf("ERROR: did not get a m_buffer back\n");
        }

    } else
    {
        printf("ERROR: unable to Make_getStatus\n");
    }

    return(retVal);
}


bool controlProtocol::GetHumidityThreshold(uint16_t destAddress, uint16_t* threshold)
{
    bool                retVal  = false;
    uint16_t            seqNum;
    msgHeader_t*        pMsgHeader;
    getHumidityThresholdResp_t*    pgetHumidityThresholdResp;


    //
    // increment the sequence number for this transaction
    //
    ++m_seqNum;

    if( (doTxCommand(Make_getHumidityThreshold(destAddress, m_buff))) )
    {   
        //
        // save the seqNum
        //
        pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
        seqNum  = ntohs(pMsgHeader->seqNum);

        // get the return packet
        if( (doRxResponse(5000)) )
        {
            #ifdef __DEBUG_CTRL_PROTO__
            //
            // dump out what we got
            //
            for(uint16_t i = 0; i < sizeof(getHumidityThresholdResp_t); i++)
            {
                printf("0x%02X ", m_buff[i]);
            }
            printf("\n");
            #endif

            //
            // check got the expected message number
            //
            pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
            if( (getHumidityThresholdResp != ntohs(pMsgHeader->msgNum)) )
            {
                fprintf(stderr, "ERROR: %s got unexpected msg %hu\n",
                    __PRETTY_FUNCTION__, ntohs(pMsgHeader->msgNum));

                //
                // no need to continue processing
                //
                return(false);
            }

            //
            // cast into the buffer, pick up the CRC
            //
            pgetHumidityThresholdResp = reinterpret_cast<getHumidityThresholdResp_t*>(m_buff);
            

            //
            // verify seqNum and CRC
            //
            if( !(verifyMessage(len_getHumidityThresholdResp_t,
                        ntohs(pgetHumidityThresholdResp->crc), seqNum)) )
            {
                // TODO: drop the packet
                fprintf(stderr, "ERROR: %s CRC bad, seqNum mismatch, or wrong address\n",
                        __PRETTY_FUNCTION__);

                //
                // no need to continue processing
                //
                return(false);
            }


            //
            // report the health 
            //
            Parse_getHumidityThresholdResp(m_buff, threshold, &seqNum);

            printf("found in packet threshold %d seqNumer 0x%02x\n", *threshold, seqNum);
            
            retVal  = true;
        } else
        {
            printf("ERROR: did not get a m_buffer back\n");
        }

    } else
    {
        printf("ERROR: unable to Make_getStatus\n");
    }

    return(retVal);
}


bool controlProtocol::SetTECTemperature(uint16_t destAddress, uint16_t tec_address, float temperature)
{
    bool                retVal  = false;
    uint16_t            seqNum;
    uint16_t            result;
    msgHeader_t*        pMsgHeader;
    setTECTemperatureResp_t*    psetTECTemperatureResp;


    //
    // increment the sequence number for this transaction
    //
    ++m_seqNum;

    if( (doTxCommand(Make_setTECTemperature(destAddress, m_buff, tec_address, temperature))) )
    {   
        //
        // save the seqNum
        //
        pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
        seqNum  = ntohs(pMsgHeader->seqNum);

        // get the return packet
        if( (doRxResponse(5000)) )
        {
            #ifdef __DEBUG_CTRL_PROTO__
            //
            // dump out what we got
            //
            for(uint16_t i = 0; i < sizeof(setTECTemperatureResp_t); i++)
            {
                printf("0x%02X ", m_buff[i]);
            }
            printf("\n");
            #endif

            //
            // check got the expected message number
            //
            pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
            if( (setTECTemperatureResp != ntohs(pMsgHeader->msgNum)) )
            {
                fprintf(stderr, "ERROR: %s got unexpected msg %hu\n",
                    __PRETTY_FUNCTION__, ntohs(pMsgHeader->msgNum));

                //
                // no need to continue processing
                //
                return(false);
            }

            //
            // cast into the buffer, pick up the CRC
            //
            psetTECTemperatureResp = reinterpret_cast<setTECTemperatureResp_t*>(m_buff);
            

            //
            // verify seqNum and CRC
            //
            if( !(verifyMessage(len_setTECTemperatureResp_t,
                        ntohs(psetTECTemperatureResp->crc), seqNum)) )
            {
                // TODO: drop the packet
                fprintf(stderr, "ERROR: %s CRC bad, seqNum mismatch, or wrong address\n",
                        __PRETTY_FUNCTION__);

                //
                // no need to continue processing
                //
                return(false);
            }


            //
            // report the health 
            //
            Parse_setTECTemperatureResp(m_buff, &result, &seqNum);

            printf("found in packet result %d seqNumer 0x%02x\n", result, seqNum);
            
            retVal  = true;
        } else
        {
            printf("ERROR: did not get a m_buffer back\n");
        }

    } else
    {
        printf("ERROR: unable to Make_getStatus\n");
    }

    return(retVal);
}


bool controlProtocol::GetTECTemperature(uint16_t destAddress, uint16_t tec_address, float* temperature)
{
    bool                retVal  = false;
    uint16_t            seqNum;
    msgHeader_t*        pMsgHeader;
    getTECTemperatureResp_t*    pgetTECTemperatureResp;


    //
    // increment the sequence number for this transaction
    //
    ++m_seqNum;

    if( (doTxCommand(Make_getTECTemperature(destAddress, m_buff, tec_address))) )
    {   
        //
        // save the seqNum
        //
        pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
        seqNum  = ntohs(pMsgHeader->seqNum);

        // get the return packet
        if( (doRxResponse(5000)) )
        {
            #ifdef __DEBUG_CTRL_PROTO__
            //
            // dump out what we got
            //
            for(uint16_t i = 0; i < sizeof(getTECTemperatureResp_t); i++)
            {
                printf("0x%02X ", m_buff[i]);
            }
            printf("\n");
            #endif

            //
            // check got the expected message number
            //
            pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
            if( (getTECTemperatureResp != ntohs(pMsgHeader->msgNum)) )
            {
                fprintf(stderr, "ERROR: %s got unexpected msg %hu\n",
                    __PRETTY_FUNCTION__, ntohs(pMsgHeader->msgNum));

                //
                // no need to continue processing
                //
                return(false);
            }

            //
            // cast into the buffer, pick up the CRC
            //
            pgetTECTemperatureResp = reinterpret_cast<getTECTemperatureResp_t*>(m_buff);
            

            //
            // verify seqNum and CRC
            //
            if( !(verifyMessage(len_getTECTemperatureResp_t,
                        ntohs(pgetTECTemperatureResp->crc), seqNum)) )
            {
                // TODO: drop the packet
                fprintf(stderr, "ERROR: %s CRC bad, seqNum mismatch, or wrong address\n",
                        __PRETTY_FUNCTION__);

                //
                // no need to continue processing
                //
                return(false);
            }


            //
            // report the health 
            //
            Parse_getTECTemperatureResp(m_buff, temperature, &seqNum);

            printf("found in packet temperature %f seqNumer 0x%02x\n", *temperature, seqNum);
            
            retVal  = true;
        } else
        {
            printf("%s ERROR: did not get a m_buffer back\n", __PRETTY_FUNCTION__);
        }

    } else
    {
        printf("%s ERROR: unable to Make_getStatus\n", __PRETTY_FUNCTION__);
    }

    return(retVal);
}


bool controlProtocol::SetChillerTemperature(uint16_t destAddress, float temperature)
{
    bool                retVal  = false;
    uint16_t            seqNum;
    uint16_t            result;
    msgHeader_t*        pMsgHeader;
    setChillerTemperatureResp_t*    psetChillerTemperatureResp;


    //
    // increment the sequence number for this transaction
    //
    ++m_seqNum;

    if( (doTxCommand(Make_setChillerTemperature(destAddress, m_buff, temperature))) )
    {   
        //
        // save the seqNum
        //
        pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
        seqNum  = ntohs(pMsgHeader->seqNum);

        // get the return packet
        if( (doRxResponse(5000)) )
        {
            #ifdef __DEBUG_CTRL_PROTO__
            //
            // dump out what we got
            //
            for(uint16_t i = 0; i < sizeof(setChillerTemperatureResp_t); i++)
            {
                printf("0x%02X ", m_buff[i]);
            }
            printf("\n");
            #endif

            //
            // check got the expected message number
            //
            pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
            if( (setChillerTemperatureResp != ntohs(pMsgHeader->msgNum)) )
            {
                fprintf(stderr, "ERROR: %s got unexpected msg %hu\n",
                    __PRETTY_FUNCTION__, ntohs(pMsgHeader->msgNum));

                //
                // no need to continue processing
                //
                return(false);
            }

            //
            // cast into the buffer, pick up the CRC
            //
            psetChillerTemperatureResp = reinterpret_cast<setChillerTemperatureResp_t*>(m_buff);
            

            //
            // verify seqNum and CRC
            //
            if( !(verifyMessage(len_setChillerTemperatureResp_t,
                        ntohs(psetChillerTemperatureResp->crc), seqNum)) )
            {
                // TODO: drop the packet
                fprintf(stderr, "ERROR: %s CRC bad, seqNum mismatch, or wrong address\n",
                        __PRETTY_FUNCTION__);

                //
                // no need to continue processing
                //
                return(false);
            }


            //
            // report the health 
            //
            Parse_setChillerTemperatureResp(m_buff, &result, &seqNum);

            printf("found in packet result %d seqNumer 0x%02x\n", result, seqNum);
            
            retVal  = true;
        } else
        {
            printf("%s ERROR: did not get a m_buffer back\n", __PRETTY_FUNCTION__);
        }

    } else
    {
        printf("%s ERROR: unable to Make_getStatus\n", __PRETTY_FUNCTION__);
    }

    return(retVal);
}


bool controlProtocol::GetChillerTemperature(uint16_t destAddress, float* temperature)
{
    bool                retVal  = false;
    uint16_t            seqNum;
    msgHeader_t*        pMsgHeader;
    getChillerTemperatureResp_t*    pgetChillerTemperatureResp;


    //
    // increment the sequence number for this transaction
    //
    ++m_seqNum;

    if( (doTxCommand(Make_getChillerTemperature(destAddress, m_buff))) )
    {   
        //
        // save the seqNum
        //
        pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
        seqNum  = ntohs(pMsgHeader->seqNum);

        // get the return packet
        if( (doRxResponse(5000)) )
        {
            #ifdef __DEBUG_CTRL_PROTO__
            //
            // dump out what we got
            //
            for(uint16_t i = 0; i < sizeof(getChillerTemperatureResp_t); i++)
            {
                printf("0x%02X ", m_buff[i]);
            }
            printf("\n");
            #endif

            //
            // check got the expected message number
            //
            pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
            if( (getChillerTemperatureResp != ntohs(pMsgHeader->msgNum)) )
            {
                fprintf(stderr, "ERROR: %s got unexpected msg %hu\n",
                    __PRETTY_FUNCTION__, ntohs(pMsgHeader->msgNum));

                //
                // no need to continue processing
                //
                return(false);
            }

            //
            // cast into the buffer, pick up the CRC
            //
            pgetChillerTemperatureResp = reinterpret_cast<getChillerTemperatureResp_t*>(m_buff);
            

            //
            // verify seqNum and CRC
            //
            if( !(verifyMessage(len_getChillerTemperatureResp_t,
                        ntohs(pgetChillerTemperatureResp->crc), seqNum)) )
            {
                // TODO: drop the packet
                fprintf(stderr, "ERROR: %s CRC bad, seqNum mismatch, or wrong address\n",
                        __PRETTY_FUNCTION__);

                //
                // no need to continue processing
                //
                return(false);
            }


            //
            // report the health 
            //
            Parse_getChillerTemperatureResp(m_buff, temperature, &seqNum);

            printf("found in packet temperature %f seqNumer 0x%02x\n", *temperature, seqNum);
            
            retVal  = true;
        } else
        {
            printf("%s ERROR: did not get a m_buffer back\n", __PRETTY_FUNCTION__);
        }

    } else
    {
        printf("%s ERROR: unable to Make_getStatus\n", __PRETTY_FUNCTION__);
    }

    return(retVal);
}


bool controlProtocol::EnableTECs(uint16_t destAddress)
{
    bool                retVal  = false;
    uint16_t            seqNum;
    uint16_t            result;
    msgHeader_t*        pMsgHeader;
    enableTECsResp_t*    penableTECsResp;


    //
    // increment the sequence number for this transaction
    //
    ++m_seqNum;

    if( (doTxCommand(Make_enableTECs(destAddress, m_buff))) )
    {   
        //
        // save the seqNum
        //
        pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
        seqNum  = ntohs(pMsgHeader->seqNum);

        // get the return packet
        if( (doRxResponse(5000)) )
        {
            #ifdef __DEBUG_CTRL_PROTO__
            //
            // dump out what we got
            //
            for(uint16_t i = 0; i < sizeof(enableTECsResp_t); i++)
            {
                printf("0x%02X ", m_buff[i]);
            }
            printf("\n");
            #endif

            //
            // check got the expected message number
            //
            pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
            if( (enableTECsResp != ntohs(pMsgHeader->msgNum)) )
            {
                fprintf(stderr, "ERROR: %s got unexpected msg %hu\n",
                    __PRETTY_FUNCTION__, ntohs(pMsgHeader->msgNum));

                //
                // no need to continue processing
                //
                return(false);
            }

            //
            // cast into the buffer, pick up the CRC
            //
            penableTECsResp = reinterpret_cast<enableTECsResp_t*>(m_buff);
            

            //
            // verify seqNum and CRC
            //
            if( !(verifyMessage(len_enableTECsResp_t,
                        ntohs(penableTECsResp->crc), seqNum)) )
            {
                // TODO: drop the packet
                fprintf(stderr, "ERROR: %s CRC bad, seqNum mismatch, or wrong address\n",
                        __PRETTY_FUNCTION__);

                //
                // no need to continue processing
                //
                return(false);
            }


            //
            // report the health 
            //
            Parse_enableTECsResp(m_buff, &result, &seqNum);

            printf("found in packet result %d seqNumer 0x%02x\n", result, seqNum);
            
            retVal  = true;
        } else
        {
            printf("%s ERROR: did not get a m_buffer back\n", __PRETTY_FUNCTION__);
        }

    } else
    {
        printf("%s ERROR: unable to Make_getStatus\n", __PRETTY_FUNCTION__);
    }

    return(retVal);
}


bool controlProtocol::DisableTECs(uint16_t destAddress)
{
    bool                retVal  = false;
    uint16_t            seqNum;
    uint16_t            result;
    msgHeader_t*        pMsgHeader;
    disableTECsResp_t*    pdisableTECsResp;


    //
    // increment the sequence number for this transaction
    //
    ++m_seqNum;

    if( (doTxCommand(Make_disableTECs(destAddress, m_buff))) )
    {   
        //
        // save the seqNum
        //
        pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
        seqNum  = ntohs(pMsgHeader->seqNum);

        // get the return packet
        if( (doRxResponse(5000)) )
        {
            #ifdef __DEBUG_CTRL_PROTO__
            //
            // dump out what we got
            //
            for(uint16_t i = 0; i < sizeof(disableTECsResp_t); i++)
            {
                printf("0x%02X ", m_buff[i]);
            }
            printf("\n");
            #endif

            //
            // check got the expected message number
            //
            pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
            if( (disableTECsResp != ntohs(pMsgHeader->msgNum)) )
            {
                fprintf(stderr, "ERROR: %s got unexpected msg %hu\n",
                    __PRETTY_FUNCTION__, ntohs(pMsgHeader->msgNum));

                //
                // no need to continue processing
                //
                return(false);
            }

            //
            // cast into the buffer, pick up the CRC
            //
            pdisableTECsResp = reinterpret_cast<disableTECsResp_t*>(m_buff);
            

            //
            // verify seqNum and CRC
            //
            if( !(verifyMessage(len_disableTECsResp_t,
                        ntohs(pdisableTECsResp->crc), seqNum)) )
            {
                // TODO: drop the packet
                fprintf(stderr, "ERROR: %s CRC bad, seqNum mismatch, or wrong address\n",
                        __PRETTY_FUNCTION__);

                //
                // no need to continue processing
                //
                return(false);
            }


            //
            // report the health 
            //
            Parse_disableTECsResp(m_buff, &result, &seqNum);

            printf("found in packet result %d seqNumer 0x%02x\n", result, seqNum);
            
            retVal  = true;
        } else
        {
            printf("%s ERROR: did not get a m_buffer back\n", __PRETTY_FUNCTION__);
        }

    } else
    {
        printf("%s ERROR: unable to Make_getStatus\n", __PRETTY_FUNCTION__);
    }

    return(retVal);
}


bool controlProtocol::StartUpCmd(uint16_t destAddress)
{
    bool                retVal  = false;
    uint16_t            seqNum;
    uint16_t            result;
    msgHeader_t*        pMsgHeader;
    startUpCmdResp_t*   pstartUpCmdResp;


    //
    // increment the sequence number for this transaction
    //
    ++m_seqNum;

    if( (doTxCommand(Make_startUpCmd(destAddress, m_buff))) )
    {   
        //
        // save the seqNum
        //
        pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
        seqNum  = ntohs(pMsgHeader->seqNum);

        // get the return packet
        if( (doRxResponse(5000)) )
        {
            #ifdef __DEBUG_CTRL_PROTO__
            //
            // dump out what we got
            //
            for(uint16_t i = 0; i < sizeof(startUpCmdResp_t); i++)
            {
                printf("0x%02X ", m_buff[i]);
            }
            printf("\n");
            #endif

            //
            // check got the expected message number
            //
            pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
            if( (startUpCmdResp != ntohs(pMsgHeader->msgNum)) )
            {
                fprintf(stderr, "ERROR: %s got unexpected msg %hu\n",
                    __PRETTY_FUNCTION__, ntohs(pMsgHeader->msgNum));

                //
                // no need to continue processing
                //
                return(false);
            }

            //
            // cast into the buffer, pick up the CRC
            //
            pstartUpCmdResp = reinterpret_cast<startUpCmdResp_t*>(m_buff);
            

            //
            // verify seqNum and CRC
            //
            if( !(verifyMessage(len_startUpCmdResp_t,
                        ntohs(pstartUpCmdResp->crc), seqNum)) )
            {
                // TODO: drop the packet
                fprintf(stderr, "ERROR: %s CRC bad, seqNum mismatch, or wrong address\n",
                        __PRETTY_FUNCTION__);

                //
                // no need to continue processing
                //
                return(false);
            }


            //
            // report the health 
            //
            Parse_startUpCmdResp(m_buff, &result, &seqNum);

            printf("found in packet result %d seqNumer 0x%02x\n", result, seqNum);
            
            retVal  = true;
        } else
        {
            printf("%s ERROR: did not get a m_buffer back\n", __PRETTY_FUNCTION__);
        }

    } else
    {
        printf("%s ERROR: unable to Make_getStatus\n", __PRETTY_FUNCTION__);
    }

    return(retVal);
}


bool controlProtocol::ShutDownCmd(uint16_t destAddress)
{
    bool                retVal  = false;
    uint16_t            seqNum;
    uint16_t            result;
    msgHeader_t*        pMsgHeader;
    shutDownCmdResp_t*  pshutDownCmdResp;


    //
    // increment the sequence number for this transaction
    //
    ++m_seqNum;

    if( (doTxCommand(Make_shutDownCmd(destAddress, m_buff))) )
    {   
        //
        // save the seqNum
        //
        pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
        seqNum  = ntohs(pMsgHeader->seqNum);

        // get the return packet
        if( (doRxResponse(5000)) )
        {
            #ifdef __DEBUG_CTRL_PROTO__
            //
            // dump out what we got
            //
            for(uint16_t i = 0; i < sizeof(shutDownCmdResp_t); i++)
            {
                printf("0x%02X ", m_buff[i]);
            }
            printf("\n");
            #endif

            //
            // check got the expected message number
            //
            pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
            if( (shutDownCmdResp != ntohs(pMsgHeader->msgNum)) )
            {
                fprintf(stderr, "ERROR: %s got unexpected msg %hu\n",
                    __PRETTY_FUNCTION__, ntohs(pMsgHeader->msgNum));

                //
                // no need to continue processing
                //
                return(false);
            }

            //
            // cast into the buffer, pick up the CRC
            //
            pshutDownCmdResp = reinterpret_cast<shutDownCmdResp_t*>(m_buff);
            

            //
            // verify seqNum and CRC
            //
            if( !(verifyMessage(len_shutDownCmdResp_t,
                        ntohs(pshutDownCmdResp->crc), seqNum)) )
            {
                // TODO: drop the packet
                fprintf(stderr, "ERROR: %s CRC bad, seqNum mismatch, or wrong address\n",
                        __PRETTY_FUNCTION__);

                //
                // no need to continue processing
                //
                return(false);
            }


            //
            // report the health 
            //
            Parse_shutDownCmdResp(m_buff, &result, &seqNum);

            printf("found in packet result %d seqNumer 0x%02x\n", result, seqNum);
            
            retVal  = true;
        } else
        {
            printf("%s ERROR: did not get a m_buffer back\n", __PRETTY_FUNCTION__);
        }

    } else
    {
        printf("%s ERROR: unable to Make_getStatus\n", __PRETTY_FUNCTION__);
    }

    return(retVal);
}


uint16_t controlProtocol::Make_startUpCmd(uint16_t Address, uint8_t* pBuff)
{
    startUpCmd_t*    msg  = reinterpret_cast<startUpCmd_t*>(pBuff);
    uint16_t      CRC  = 0;


    // create the startUpCmd message in pBuff and CRC16 checksum it
    msg->control            = COMMAND;
    msg->address.address    = htons(Address);
    msg->seqNum             = htons(m_seqNum);
    msg->msgNum             = htons(startUpCmd);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_startUpCmd_t);

    // put the CRC
    msg->crc                = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eot                = '\r';

    return(sizeof(startUpCmd_t));
}


uint16_t controlProtocol::Make_startUpCmdResp(uint16_t Address, uint8_t* pBuff, uint16_t result, uint16_t SeqNum)
{
    startUpCmdResp_t*  msg  = reinterpret_cast<startUpCmdResp_t*>(pBuff);
    uint16_t        CRC  = 0;


    // create the startUpCmdResp message in pBuff and CRC16 checksum it
    msg->control            = RESPONSE;
    msg->address.address    = htons(Address);
    msg->seqNum             = htons(SeqNum);
    msg->msgNum             = htons(startUpCmdResp);

    // set the result
    msg->result             = htons(result);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_startUpCmdResp_t);

    // put the CRC
    msg->crc                = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eot                = '\r';

    return(sizeof(startUpCmdResp_t));
}


void controlProtocol::Parse_startUpCmdResp(uint8_t* m_buff, uint16_t* result, uint16_t* pSeqNum)
{
    startUpCmdResp_t* pResponse = reinterpret_cast<startUpCmdResp_t*>(m_buff);


    *result     = ntohs(pResponse->result);
    *pSeqNum    = ntohs(pResponse->seqNum);
}


uint16_t controlProtocol::Make_shutDownCmd(uint16_t Address, uint8_t* pBuff)
{
    shutDownCmd_t* msg  = reinterpret_cast<shutDownCmd_t*>(pBuff);
    uint16_t    CRC  = 0;


    // create the shutDownCmdResp message in pBuff and CRC16 checksum it
    msg->control            = COMMAND;
    msg->address.address    = htons(Address);
    msg->seqNum             = htons(m_seqNum);
    msg->msgNum             = htons(shutDownCmd);

    // calculate the CRC
    CRC = calcCRC16(pBuff, len_shutDownCmd_t);

    // put the CRC
    msg->crc                = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eot                = '\r';

    return(sizeof(shutDownCmd_t));
}


uint16_t controlProtocol::Make_shutDownCmdResp(uint16_t Address, uint8_t* pBuff, uint16_t result, uint16_t SeqNum)
{
    shutDownCmdResp_t* msg  = reinterpret_cast<shutDownCmdResp_t*>(pBuff);
    uint16_t        CRC  = 0;


    // create the shutDownCmdResp message in pBuff and CRC16 checksum it
    msg->control            = RESPONSE;
    msg->address.address    = htons(Address);
    msg->seqNum             = htons(SeqNum);
    msg->msgNum             = htons(shutDownCmdResp);

    // set the result
    msg->result             = htons(result);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_shutDownCmdResp_t);

    // put the CRC
    msg->crc                = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eot                = '\r';

    return(sizeof(shutDownCmdResp_t));
}


void controlProtocol::Parse_shutDownCmdResp(uint8_t* m_buff, uint16_t* result, uint16_t* pSeqNum)
{
    shutDownCmdResp_t* pResponse = reinterpret_cast<shutDownCmdResp_t*>(m_buff);


    *result     = ntohs(pResponse->result);
    *pSeqNum    = ntohs(pResponse->seqNum);
}


//
// send a getStatus message to controllino at Address - future expandability
// TODO: protect against short pBuff by having caller include length ?
//
uint16_t controlProtocol::Make_getStatus(uint16_t Address, uint8_t* pBuff)
{
    getStatus_t*    msg  = reinterpret_cast<getStatus_t*>(pBuff);
    uint16_t        CRC  = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->control            = COMMAND;
    msg->address.address    = htons(Address);
    msg->seqNum             = htons(m_seqNum);
    msg->msgNum             = htons(getStatusCmd);


    #ifndef __RUNNING_ON_CONTROLLINO__
    printf("input bytes: ");
    for(unsigned int i = 0; i < len_getStatus_t; i++)
        printf("0x%X ", reinterpret_cast<uint8_t>(pBuff[i]));
    printf("\n");
    #else
    Serial.print("input bytes: ");
    for(int i = 0; i < len_getStatus_t; i++)
        Serial.print(reinterpret_cast<uint8_t>(pBuff[i]), HEX);
    Serial.println("");
    Serial.flush();
    #endif


    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getStatus_t);

    // put the CRC
    msg->crc                = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eot                = '\r';

    return(sizeof(getStatus_t));
}


uint16_t controlProtocol::Make_getStatusResp(uint16_t Address, uint8_t* pBuff, uint16_t humidityAlert,
                    uint16_t TECsRunning, uint16_t chillerOnLine, uint16_t SeqNum)
{
    getStatusResp_t*    msg  = reinterpret_cast<getStatusResp_t*>(pBuff);
    uint16_t            CRC  = 0;


    // create the getStatusResp message in pBuff and CRC16 checksum it
    msg->control                = RESPONSE;
    msg->address.address        = htons(Address);
    msg->seqNum                 = htons(SeqNum);
    msg->msgNum                 = htons(getStatusResp);
    msg->status.humidityAlert   = ntohs(humidityAlert);
    msg->status.TECsRunning     = ntohs(TECsRunning);
    msg->status.chillerOnLine   = ntohs(chillerOnLine);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getStatusResp_t);

    // put the CRC
    msg->crc                = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eot                = '\r';

    return(sizeof(getStatusResp_t));
}


void controlProtocol::Parse_getStatusResp(uint8_t* m_buff, uint16_t* humidityAlert,
        uint16_t* TECsRunning, uint16_t* chillerOnLine, uint16_t* pSeqNum)
{
    getStatusResp_t* pResponse = reinterpret_cast<getStatusResp_t*>(m_buff);

    //
    // fill in the pReport
    //
    *humidityAlert  = ntohs(pResponse->status.humidityAlert);
    *TECsRunning    = ntohs(pResponse->status.TECsRunning);
    *chillerOnLine  = ntohs(pResponse->status.chillerOnLine);
    *pSeqNum        = ntohs(pResponse->seqNum);
}


uint16_t controlProtocol::Make_getHumidityThreshold(uint16_t Address, uint8_t* pBuff)
{
    getHumidityThreshold_t*    msg  = reinterpret_cast<getHumidityThreshold_t*>(pBuff);
    uint16_t        CRC  = 0;


    // create the getHumidityThreshold message in pBuff and CRC16 checksum it
    msg->control            = COMMAND;
    msg->address.address    = htons(Address);   // need htons here ?
    msg->seqNum             = htons(m_seqNum);    // htons ?
    msg->msgNum             = htons(getHumidityThreshold);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getHumidityThreshold_t);

    // put the CRC
    msg->crc                = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eot                = '\r';

    return(sizeof(getHumidityThreshold_t));
}


uint16_t controlProtocol::Make_getHumidityThresholdResp(uint16_t Address, uint8_t* pBuff, uint16_t threshold, uint16_t SeqNum)
{
    getHumidityThresholdResp_t* msg  = reinterpret_cast<getHumidityThresholdResp_t*>(pBuff);
    uint16_t                    CRC  = 0;


    // create the getHumidityThresholdResp message in pBuff and CRC16 checksum it
    msg->control            = RESPONSE;
    msg->address.address    = htons(Address);   // need htons here ?
    msg->seqNum             = htons(SeqNum);    // htons ?
    msg->msgNum             = htons(getHumidityThresholdResp);
    msg->threshold          = htons(threshold);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getHumidityThresholdResp_t);

    // put the CRC
    msg->crc                = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eot                = '\r';

    return(sizeof(getHumidityThresholdResp_t));
}


void controlProtocol::Parse_getHumidityThresholdResp(uint8_t* m_buff, uint16_t* threshold, uint16_t* pSeqNum)
{
    getHumidityThresholdResp_t* pResponse = reinterpret_cast<getHumidityThresholdResp_t*>(m_buff);


    *threshold  = ntohs(pResponse->threshold);
    *pSeqNum    = ntohs(pResponse->seqNum);
}


uint16_t controlProtocol::Make_setHumidityThreshold(uint16_t Address, uint8_t* pBuff, uint16_t threshold)
{
    setHumidityThreshold_t* msg = reinterpret_cast<setHumidityThreshold_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->control            = COMMAND;
    msg->address.address    = htons(Address);
    msg->seqNum             = htons(m_seqNum);
    msg->msgNum             = htons(setHumidityThreshold);

    // pass in the humidity threshold to set
    msg->threshold          = htons(threshold);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_setHumidityThreshold_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eot                = '\r';

    return(sizeof(setHumidityThreshold_t));
}


uint16_t controlProtocol::Make_setHumidityThresholdResp(uint16_t Address, uint8_t* pBuff, uint16_t result, uint16_t SeqNum)
{
    setHumidityThresholdResp_t* msg = reinterpret_cast<setHumidityThresholdResp_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->control            = RESPONSE;
    msg->address.address    = htons(Address);
    msg->seqNum             = htons(SeqNum);
    msg->msgNum             = htons(setHumidityThresholdResp);

    // set the result
    msg->result             = htons(result);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_setHumidityThresholdResp_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eot                = '\r';

    return(sizeof(setHumidityThresholdResp_t));
}


void controlProtocol::Parse_setHumidityThresholdResp(uint8_t* m_buff, uint16_t* result, uint16_t* pSeqNum)
{
    setHumidityThresholdResp_t* pResponse = reinterpret_cast<setHumidityThresholdResp_t*>(m_buff);


    *result     = ntohs(pResponse->result);
    *pSeqNum    = ntohs(pResponse->seqNum);
}


uint16_t controlProtocol::Make_getHumidity(uint16_t Address, uint8_t* pBuff)
{
    getHumidity_t* msg = reinterpret_cast<getHumidity_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->control            = COMMAND;
    msg->address.address    = htons(Address);
    msg->seqNum             = htons(m_seqNum);
    msg->msgNum             = htons(getHumidity);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getHumidity_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eot                = '\r';

    return(sizeof(getHumidity_t));
}


uint16_t controlProtocol::Make_getHumidityResp(uint16_t Address, uint8_t* pBuff, float humidity, uint16_t SeqNum)
{
    getHumidityResp_t* msg = reinterpret_cast<getHumidityResp_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->control            = RESPONSE;
    msg->address.address    = htons(Address);
    msg->seqNum             = htons(SeqNum);
    msg->msgNum             = htons(getHumidityResp);

    #ifdef __RUNNING_ON_CONTROLLINO__
    //
    // use the dostrf function, left justified, max 7 characters total, max 2 decimal places
    //
    dtostrf(humidity, -(MAX_HUMIDITY_LENGTH), 2, reinterpret_cast<char*>(msg->humidity));
    #else
    //
    // use the snprintf function, 
    //
    snprintf(reinterpret_cast<char*>(msg->humidity), MAX_HUMIDITY_LENGTH, "%-+3.2f", humidity);
    #endif

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getHumidityResp_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eot                = '\r';

    return(sizeof(getHumidityResp_t));
}


void controlProtocol::Parse_getHumidityResp(uint8_t* m_buff, float* humidity, uint16_t* pSeqNum)
{
    getHumidityResp_t* pResponse = reinterpret_cast<getHumidityResp_t*>(m_buff);


    #ifdef __RUNNING_ON_CONTROLLINO__
    //
    // on uC use atof, sscanf support is dodgy
    //
    *humidity = atof(reinterpret_cast<char*>(pResponse->humidity));
    #else
    sscanf(reinterpret_cast<char*>(pResponse->humidity), "%6f", humidity);
    #endif

    *pSeqNum    = ntohs(pResponse->seqNum);
}


uint16_t controlProtocol::Make_setTECTemperature(uint16_t Address, uint8_t* pBuff, uint16_t tec_address, float temperature)
{
    setTECTemperature_t* msg = reinterpret_cast<setTECTemperature_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->control            = COMMAND;
    msg->address.address    = htons(Address);
    msg->seqNum             = htons(m_seqNum);
    msg->msgNum             = htons(setTECTemperature);
    msg->tec_address        = htons(tec_address);
    #ifdef __RUNNING_ON_CONTROLLINO__
    //
    // use the dostrf function, left justified, max 7 characters total, max 2 decimal places
    //
    dtostrf(temperature, -(MAX_TEC_TEMP_LENGH), 2, reinterpret_cast<char*>(msg->temperature));
    #else
    //
    // use the snprintf function, 
    //
    snprintf(reinterpret_cast<char*>(msg->temperature), MAX_TEC_TEMP_LENGH, "%-+3.2f", temperature);
    #endif

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_setTECTemperature_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eot                = '\r';

    return(sizeof(setTECTemperature_t));
}


uint16_t controlProtocol::Make_setTECTemperatureResp(uint16_t Address, uint8_t* pBuff, uint16_t tec_address, uint16_t result, uint16_t SeqNum)
{
    setTECTemperatureResp_t* msg = reinterpret_cast<setTECTemperatureResp_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->control            = RESPONSE;
    msg->address.address    = htons(Address);
    msg->seqNum             = htons(SeqNum);
    msg->msgNum             = htons(setTECTemperatureResp);
    msg->tec_address        = htons(tec_address);
    msg->result             = htons(result);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_setTECTemperatureResp_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eot                = '\r';

    return(sizeof(setTECTemperatureResp_t));
}


void controlProtocol::Parse_setTECTemperatureResp(uint8_t* m_buff, uint16_t* result, uint16_t* pSeqNum)
{
    setTECTemperatureResp_t* pResponse = reinterpret_cast<setTECTemperatureResp_t*>(m_buff);


    *result     = ntohs(pResponse->result);
    *pSeqNum    = ntohs(pResponse->seqNum);
}


uint16_t controlProtocol::Make_getTECTemperature(uint16_t Address, uint8_t* pBuff, uint16_t tec_address)
{
    getTECTemperature_t* msg = reinterpret_cast<getTECTemperature_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->control            = COMMAND;
    msg->address.address    = htons(Address);
    msg->seqNum             = htons(m_seqNum);
    msg->msgNum             = htons(getTECTemperature);
    msg->tec_address        = htons(tec_address);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getTECTemperature_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?


    // put the end of transmission byte
    msg->eot                = '\r';

    return(sizeof(getTECTemperature_t));
}

uint16_t controlProtocol::Make_getTECTemperatureResp(uint16_t Address, uint8_t* pBuff, uint16_t tec_address, float temperature, uint16_t SeqNum)
{
    getTECTemperatureResp_t* msg = reinterpret_cast<getTECTemperatureResp_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->control            = RESPONSE;
    msg->address.address    = htons(Address);
    msg->seqNum             = htons(SeqNum);
    msg->msgNum             = htons(getTECTemperatureResp);
    msg->tec_address        = htons(tec_address);
    #ifdef __RUNNING_ON_CONTROLLINO__
    //
    // use the dostrf function, left justified, max 7 characters total, max 2 decimal places
    //
    dtostrf(temperature, -(MAX_TEC_TEMP_LENGH), 2, reinterpret_cast<char*>(msg->temperature));
    #else
    //
    // use the snprintf function, 
    //
    snprintf(reinterpret_cast<char*>(msg->temperature), MAX_TEC_TEMP_LENGH, "%-+3.2f", temperature);
    #endif

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getTECTemperatureResp_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?


    // put the end of transmission byte
    msg->eot                = '\r';

    return(sizeof(getTECTemperatureResp_t));
}

void controlProtocol::Parse_getTECTemperatureResp(uint8_t* m_buff, float* temperature, uint16_t* pSeqNum)
{
    getTECTemperatureResp_t* pResponse = reinterpret_cast<getTECTemperatureResp_t*>(m_buff);


    #ifdef __RUNNING_ON_CONTROLLINO__
    //
    // on uC use atof, sscanf support is dodgy
    //
    *temperature = atof(reinterpret_cast<char*>(pResponse->temperature));
    #else
    sscanf(reinterpret_cast<char*>(pResponse->temperature), "%f", temperature);
    #endif

    *pSeqNum    = ntohs(pResponse->seqNum);
}


uint16_t controlProtocol::Make_enableTECs(uint16_t Address, uint8_t* pBuff)
{
    enableTECs_t* msg = reinterpret_cast<enableTECs_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->control            = COMMAND;
    msg->address.address    = htons(Address);
    msg->seqNum             = htons(m_seqNum);
    msg->msgNum             = htons(enableTECs);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_enableTECs_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eot                = '\r';

    return(sizeof(enableTECs_t));
}


uint16_t controlProtocol::Make_enableTECsResp(uint16_t Address, uint8_t* pBuff, uint16_t result, uint16_t SeqNum)
{
    enableTECsResp_t* msg = reinterpret_cast<enableTECsResp_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->control            = RESPONSE;
    msg->address.address    = htons(Address);
    msg->seqNum             = htons(SeqNum);
    msg->msgNum             = htons(enableTECsResp);
    msg->result             = htons(result);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_enableTECsResp_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eot                = '\r';

    return(sizeof(enableTECsResp_t));
}


void controlProtocol::Parse_enableTECsResp(uint8_t* m_buff, uint16_t* result, uint16_t* pSeqNum)
{
    enableTECsResp_t* pResponse = reinterpret_cast<enableTECsResp_t*>(m_buff);


    *result     = ntohs(pResponse->result);
    *pSeqNum    = ntohs(pResponse->seqNum);
}


uint16_t controlProtocol::Make_disableTECs(uint16_t Address, uint8_t* pBuff)
{
    disableTECs_t* msg = reinterpret_cast<disableTECs_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->control            = COMMAND;
    msg->address.address    = htons(Address);
    msg->seqNum             = htons(m_seqNum);
    msg->msgNum             = htons(disableTECs);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_disableTECs_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eot                = '\r';

    return(sizeof(disableTECs_t));
}


uint16_t controlProtocol::Make_disableTECsResp(uint16_t Address, uint8_t* pBuff, uint16_t result, uint16_t SeqNum)
{
    disableTECsResp_t* msg = reinterpret_cast<disableTECsResp_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->control            = RESPONSE;
    msg->address.address    = htons(Address);
    msg->seqNum             = htons(SeqNum);
    msg->msgNum             = htons(disableTECsResp);
    msg->result             = htons(result);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_disableTECsResp_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eot                = '\r';

    return(sizeof(disableTECsResp_t));
}


void controlProtocol::Parse_disableTECsResp(uint8_t* m_buff, uint16_t* result, uint16_t* pSeqNum)
{
    disableTECsResp_t* pResponse = reinterpret_cast<disableTECsResp_t*>(m_buff);


    *result     = ntohs(pResponse->result);
    *pSeqNum    = ntohs(pResponse->seqNum);
}


uint16_t controlProtocol::Make_setChillerTemperature(uint16_t Address, uint8_t* pBuff, float temperature)
{
    setChillerTemperature_t* msg = reinterpret_cast<setChillerTemperature_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->control            = COMMAND;
    msg->address.address    = htons(Address);
    msg->seqNum             = htons(m_seqNum);
    msg->msgNum             = htons(setChillerTemperature);
    #ifdef __RUNNING_ON_CONTROLLINO__
    //
    // use the dostrf function, left justified, max 7 characters total, max 2 decimal places
    //
    dtostrf(temperature, -(MAX_CHILLER_TEMP_LENGH), 2, reinterpret_cast<char*>(msg->temperature));
    #else
    //
    // use the snprintf function, 
    //
    snprintf(reinterpret_cast<char*>(msg->temperature), MAX_CHILLER_TEMP_LENGH, "%-+3.2f", temperature);
    #endif

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_setChillerTemperature_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eot                = '\r';

    return(sizeof(setChillerTemperature_t));
}


uint16_t controlProtocol::Make_setChillerTemperatureResp(uint16_t Address, uint8_t* pBuff, uint16_t result, uint16_t SeqNum)
{
    setChillerTemperatureResp_t* msg = reinterpret_cast<setChillerTemperatureResp_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->control            = RESPONSE;
    msg->address.address    = htons(Address);
    msg->seqNum             = htons(SeqNum);
    msg->msgNum             = htons(setChillerTemperatureResp);
    msg->result             = htons(result);
    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_setChillerTemperatureResp_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eot                = '\r';

    return(sizeof(setChillerTemperatureResp_t));
}


void controlProtocol::Parse_setChillerTemperatureResp(uint8_t* m_buff, uint16_t* result, uint16_t* pSeqNum)
{
    setChillerTemperatureResp_t* pResponse = reinterpret_cast<setChillerTemperatureResp_t*>(m_buff);


    *result     = ntohs(pResponse->result);
    *pSeqNum    = ntohs(pResponse->seqNum);
}


uint16_t controlProtocol::Make_getChillerTemperature(uint16_t Address, uint8_t* pBuff)
{
    getChillerTemperature_t* msg = reinterpret_cast<getChillerTemperature_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->control            = COMMAND;
    msg->address.address    = htons(Address);
    msg->seqNum             = htons(m_seqNum);
    msg->msgNum             = htons(getChillerTemperature);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getChillerTemperature_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?


    // put the end of transmission byte
    msg->eot                = '\r';

    return(sizeof(getChillerTemperature_t));
}

uint16_t controlProtocol::Make_getChillerTemperatureResp(uint16_t Address, uint8_t* pBuff, float temperature, uint16_t SeqNum)
{
    getChillerTemperatureResp_t* msg = reinterpret_cast<getChillerTemperatureResp_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->control            = RESPONSE;
    msg->address.address    = htons(Address);
    msg->seqNum             = htons(SeqNum);
    msg->msgNum             = htons(getChillerTemperatureResp);
    #ifdef __RUNNING_ON_CONTROLLINO__
    //
    // use the dostrf function, left justified, max 7 characters total, max 2 decimal places
    //
    dtostrf(temperature, -(MAX_CHILLER_TEMP_LENGH), 2, reinterpret_cast<char*>(msg->temperature));
    #else
    //
    // use the snprintf function, 
    //
    snprintf(reinterpret_cast<char*>(msg->temperature), MAX_CHILLER_TEMP_LENGH, "%-+3.2f", temperature);
    #endif

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getChillerTemperatureResp_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?


    // put the end of transmission byte
    msg->eot                = '\r';

    return(sizeof(getChillerTemperatureResp_t));
}

void controlProtocol::Parse_getChillerTemperatureResp(uint8_t* m_buff, float* temperature, uint16_t* pSeqNum)
{
    getChillerTemperatureResp_t* pResponse = reinterpret_cast<getChillerTemperatureResp_t*>(m_buff);


    #ifdef __RUNNING_ON_CONTROLLINO__
    //
    // on uC use atof, sscanf support is dodgy
    //
    *temperature = atof(reinterpret_cast<char*>(pResponse->temperature));
    #else
    sscanf(reinterpret_cast<char*>(pResponse->temperature), "%f", temperature);
    #endif

    *pSeqNum    = ntohs(pResponse->seqNum);
}


//
// uses class members:
// 1. m_buff    - has the received message in it
// 2. m_seqNum  - is the current sequence number expected in this reply
//
// the calling function supplies the length of the message in m_buff
//
bool controlProtocol::verifyMessage(uint16_t buffLength, uint16_t pktCRC, uint16_t expSeqNum)
{
    if( (verifyMessageSeqNum(buffLength, expSeqNum)) && (verifyMessageCRC(buffLength, pktCRC)) )
    {
        return(true);
    } else
    {
        return(false);
    }
}


bool controlProtocol::verifyMessageSeqNum(uint16_t buffLength, uint16_t expSeqNum)
{
    bool            retVal      = true;
    msgHeader_t*    pMsgHeader  = reinterpret_cast<msgHeader_t*>(m_buff);


    //
    // check seqNum match
    //
    if( (expSeqNum != ntohs(pMsgHeader->seqNum)) )
    {
        fprintf(stderr, "ERROR: %s seqNum mismatch\n", __PRETTY_FUNCTION__);
        retVal  = false;
    }

    return(retVal);
}


bool controlProtocol::verifyMessageCRC(uint16_t buffLength, uint16_t pktCRC)
{
    bool            retVal      = true;
    uint16_t        CRC         = calcCRC16(m_buff, buffLength);


    #ifndef __RUNNING_ON_CONTROLLINO__
    printf("input bytes: ");
    for(int i = 0; i < buffLength; i++)
        printf("0x%X ", reinterpret_cast<uint8_t>(m_buff[i]));
    printf("\n");
    #else
    Serial.print("input bytes: ");
    for(int i = 0; i < buffLength; i++)
        Serial.print(reinterpret_cast<uint8_t>(m_buff[i]), HEX);
    Serial.println("");
    Serial.flush();
    #endif


    if( (CRC != pktCRC) ) 
    {
        #ifndef __RUNNING_ON_CONTROLLINO__
        fprintf(stderr, "ERROR: %s CRC mismatch 0x%x:0x%x\n", __PRETTY_FUNCTION__, CRC, pktCRC);
        #else
        Serial.print(__PRETTY_FUNCTION__);
        Serial.print(" ERROR: CRC mismatch: 0x");
        Serial.print(CRC, HEX);
        Serial.print(":");
        Serial.println(pktCRC, HEX);
        Serial.flush();
        #endif
        retVal  = false;
    }

    return(retVal);
}


uint16_t controlProtocol::getMsgId()
{
    msgHeader_t* pmsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);

    return( ntohs(pmsgHeader->msgNum) );
}

