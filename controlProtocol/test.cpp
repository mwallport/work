// file test.cpp
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "controlProtocol.h"

int main(int argc, char** argv)
{
    uint8_t     buff[100];
    uint16_t    SeqNum = 0x2112;
    uint16_t    humidityAlert;
    uint16_t    TECsRunning;
    uint16_t    chillerOnLine;
    uint16_t    threshold;
    uint16_t    result;
    uint16_t    destAddress = 1;
    float       humidity    = 39.04;
    float       temperature = 0;
    controlProtocol cpUSB(0, 1, "/dev/ttyUSB0"); // my address, peer address, usb file


    if( (cpUSB.StartUpCmd(1)) )
        printf("++++++++++++++++++++++++StartUpCmd good\n");
    else
        printf("-------------------StartUpCmd bad\n");

    printf("\n\n");
    if( (cpUSB.GetStatus(destAddress, &humidityAlert, &TECsRunning, &chillerOnLine)) )
    {
        printf("++++++++++++++++++++got: humidityAlert %hu, TECsRunning %hu, chillerOnLine %hu\n",
            humidityAlert, TECsRunning, chillerOnLine);
    } else
    {
        printf("---------------------failed to GetStatus\n");
    }

    printf("\n\n");
    if( (cpUSB.GetHumidity(1, &humidity)) )
        printf("+++++++++++++++++++++got: humidity %f\n", humidity);
    else
        printf("---------------------failed to get humidity\n");

    printf("\n\n");
    if( (cpUSB.SetHumidityThreshold(1, 47)) )
        printf("+++++++++++++++++++++success on set humidity threshold\n");
    else
        printf("---------------------fail on set humidity threshold\n");

    printf("\n\n");
    if( (cpUSB.GetHumidityThreshold(1, &threshold)) )
        printf("+++++++++++++++++++++got humidity threshold: %d\n", threshold);
    else
        printf("---------------------failed to get humidity threshold\n");
    printf("\n\n");

    if( (cpUSB.SetTECTemperature(1, 2, -21.12)) )            // problem - works and flakey/crashy
        printf("+++++++++++++++++++success on set TEC temperature\n");
    else
        printf("-------------------fail on set TEC temperature\n");

    printf("\n\n");
    if( (cpUSB.GetTECTemperature(1, 2, &temperature)) ) // always sends back 0.00 (and is correct in pkt)
        printf("++++++++++++++++++++++got TEC temperature %f\n", temperature);// works if you program in non-zero
    else
        printf("----------------------failed to get TEC temperature\n");

    printf("\n\n");
    if( (cpUSB.SetChillerTemperature(1, 23.3)) )
        printf("++++++++++++++++++++++success for set chiller temperature\n");
    else
        printf("----------------------fail on set chiller temperature\n");

    printf("\n\n");
    if( (cpUSB.GetChillerTemperature(1, &temperature)) )
        printf("+++++++++++++++++++++got chiller temperature %f\n", temperature);
    else
        printf("---------------------failed to get chiller temperature\n");
    
    printf("\n\n");
    if( (cpUSB.EnableTECs(1)) )
        printf("+++++++++++++++++++++success on enable TECs\n");
    else
        printf("---------------------fail on enable TECs\n");
    
    printf("\n\n");
    if( (cpUSB.DisableTECs(1)) )
        printf("+++++++++++++++++++++success on disable TECs\n");
    else
        printf("---------------------failure on disable TECs\n");

    printf("\n\n");
    if( (cpUSB.ShutDownCmd(1)) )
        printf("+++++++++++++++++++++ShutDownCmd good\n");
    else
        printf("---------------------ShutDownCmd bad\n");

    return(0);
}

