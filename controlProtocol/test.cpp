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
    uint32_t    deviceType;
    uint32_t    serialNumber;
    uint32_t    fwVersion;
    uint32_t    hwVersion;
    char        ChillerInfo[100];
    float       humidity    = 39.04;
    float       temperature = 0;
    controlProtocol cpUSB(0, 1, "/dev/ttyUSB0", 9600); // my address, peer address, usb file



/*
    if( (cpUSB.GetChillerInfo(1, ChillerInfo, 100)) ) // works
    {
        printf("++++++++++++++++++++++++GetChillerInfo cmd good\n");
        printf("got info: %s\n", ChillerInfo);
    } else
    {
        printf("-------------------GetChillerInfo bad\n");
    }

    printf("\n\n");
    sleep(5);
*/
    if( (cpUSB.GetTECInfo(1, 1, &deviceType, &hwVersion, &fwVersion, &serialNumber)) ) // works
    {
        printf("++++++++++++++++++++++++GetTECInfo cmd good\n");
        printf("deviceType 0x%04X, hwVersion 0x%04X, fwVersion 0x%04X, serialNumber 0x%04X", 
            deviceType, hwVersion, fwVersion, serialNumber);
    } else
        printf("-------------------GetTECInfo bad\n");
    printf("\n\n");

    if( (cpUSB.GetTECInfo(1, 2, &deviceType, &hwVersion, &fwVersion, &serialNumber)) ) // works
    {
        printf("++++++++++++++++++++++++GetTECInfo cmd good\n");
        printf("deviceType 0x%04X, hwVersion 0x%04X, fwVersion 0x%04X, serialNumber 0x%04X", 
            deviceType, hwVersion, fwVersion, serialNumber);
    } else
        printf("-------------------GetTECInfo bad\n");
    printf("\n\n");

    if( (cpUSB.GetTECInfo(1, 3, &deviceType, &hwVersion, &fwVersion, &serialNumber)) ) // works
    {
        printf("++++++++++++++++++++++++GetTECInfo cmd good\n");
        printf("deviceType 0x%04X, hwVersion 0x%04X, fwVersion 0x%04X, serialNumber 0x%04X", 
            deviceType, hwVersion, fwVersion, serialNumber);
    } else
        printf("-------------------GetTECInfo bad\n");
    printf("\n\n");
/*
    sleep(5);

    if( (cpUSB.StartChiller(1)) ) // works
        printf("++++++++++++++++++++++++StartChiller cmd good\n");
    else
        printf("------------------- StartChiller cmd bad\n");
    printf("\n\n");
    sleep(5);
    
    if( (cpUSB.StopChiller(1)) ) // works
        printf("++++++++++++++++++++++++StopChiller cmd good\n");
    else
        printf("------------------- StopChiller cmd bad\n");
    printf("\n\n");
    sleep(5);

    if( (cpUSB.StartUpCmd(1)) ) // working- takes too long, this program times out waiting for reply
        printf("++++++++++++++++++++++++StartUpCmd good\n");
    else
        printf("-------------------StartUpCmd bad\n");
    printf("\n\n");
    sleep(5);

    if( (cpUSB.GetStatus(destAddress, &humidityAlert, &TECsRunning, &chillerOnLine)) ) // not working
    {
        printf("++++++++++++++++++++got: humidityAlert %hu, TECsRunning %hu, chillerOnLine %hu\n",
            humidityAlert, TECsRunning, chillerOnLine);
    } else
    {
        printf("---------------------failed to GetStatus\n");
    }
    printf("\n\n");

    sleep(2);

    if( (cpUSB.GetHumidity(1, &humidity)) )     // working
        printf("+++++++++++++++++++++got: humidity %f\n", humidity);
    else
        printf("---------------------failed to get humidity\n");
    printf("\n\n");

    sleep(2);

    if( (cpUSB.SetHumidityThreshold(1, 68)) )  // working
        printf("+++++++++++++++++++++success on set humidity threshold\n");
    else
        printf("---------------------fail on set humidity threshold\n");
    printf("\n\n");

    sleep(2);

    if( (cpUSB.GetHumidityThreshold(1, &threshold)) ) // working
        printf("+++++++++++++++++++++got humidity threshold: %d\n", threshold);
    else
        printf("---------------------failed to get humidity threshold\n");
    printf("\n\n");

    sleep(2);

    if( (cpUSB.GetHumidity(1, &humidity)) ) // working
        printf("+++++++++++++++++++++got: humidity %f\n", humidity);
    else
        printf("---------------------failed to get humidity\n");
    printf("\n\n");

    sleep(2);

    if( (cpUSB.SetTECTemperature(1, 1, -1.11)) )   // working
        printf("+++++++++++++++++++success on set TEC temperature\n");
    else
        printf("-------------------fail on set TEC temperature\n");
    printf("\n\n");

    sleep(2);


    if( (cpUSB.GetTECTemperature(1, 1, &temperature)) ) // showing number that is too big
        printf("++++++++++++++++++++++got TEC temperature %f\n", temperature);// works if you program in non-zero
    else
        printf("----------------------failed to get TEC temperature\n");
    printf("\n\n");

    if( (cpUSB.SetTECTemperature(1, 2, -2.22)) )   // working
        printf("+++++++++++++++++++success on set TEC temperature\n");
    else
        printf("-------------------fail on set TEC temperature\n");
    printf("\n\n");

    sleep(2);


    if( (cpUSB.GetTECTemperature(1, 2, &temperature)) ) // showing number that is too big
        printf("++++++++++++++++++++++got TEC temperature %f\n", temperature);// works if you program in non-zero
    else
        printf("----------------------failed to get TEC temperature\n");
    printf("\n\n");

    if( (cpUSB.SetTECTemperature(1, 3, -3.33)) )   // working
        printf("+++++++++++++++++++success on set TEC temperature\n");
    else
        printf("-------------------fail on set TEC temperature\n");
    printf("\n\n");

    sleep(2);


    if( (cpUSB.GetTECTemperature(1, 3, &temperature)) ) // showing number that is too big
        printf("++++++++++++++++++++++got TEC temperature %f\n", temperature);// works if you program in non-zero
    else
        printf("----------------------failed to get TEC temperature\n");
    printf("\n\n");

    sleep(2);

    if( (cpUSB.SetChillerTemperature(1, -11.10)) ) // not working
        printf("++++++++++++++++++++++success for set chiller temperature\n");
    else
        printf("----------------------fail on set chiller temperature\n");
    printf("\n\n");

    sleep(2);

    if( (cpUSB.GetChillerTemperature(1, &temperature)) ) // not working
        printf("+++++++++++++++++++++got chiller temperature %f\n", temperature);
    else
        printf("---------------------failed to get chiller temperature\n");
    printf("\n\n");
    sleep(2);

    if( (cpUSB.EnableTECs(1)) )
        printf("+++++++++++++++++++++success on enable TECs\n");
    else
        printf("---------------------fail on enable TECs\n");
    
    printf("\n\n");
    sleep(2);

    if( (cpUSB.DisableTECs(1)) )    // LCD shows running still - not working
        printf("+++++++++++++++++++++success on disable TECs\n");
    else
        printf("---------------------failure on disable TECs\n");
    printf("\n\n");
    sleep(2);

    if( (cpUSB.ShutDownCmd(1)) )
        printf("+++++++++++++++++++++ShutDownCmd good\n");
    else
        printf("---------------------ShutDownCmd bad\n");
    printf("\n\n");

    sleep(2);

*/
    return(0);

}

