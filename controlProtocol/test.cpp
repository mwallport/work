// file test.cpp
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
    unsigned int speed;
    
    
    if( (3 > argc) )
    {
        printf("Please supply USB device and speed\n");
        printf("i.e. test.exe /dev/tty/USB0 19200\n");
        return(-1);
    }
    
    sscanf(argv[2], "%u", &speed);             // assuming this works
    printf("trying usb: %s, speed %u\n", argv[1], speed);
    
    #ifdef __USING_WINDOWS_USB__
    WORD wVersionRequested;
    WSADATA wsaData;
    int err;

    /* Use the MAKEWORD(lowbyte, highbyte) macro declared in Windef.h */
    wVersionRequested = MAKEWORD(2, 2);

    err = WSAStartup(wVersionRequested, &wsaData);
    if (err != 0) {
        /* Tell the user that we could not find a usable */
        /* Winsock DLL.                                  */
        printf("WSAStartup failed with error: %d\n", err);
        return 1;
    }
    #endif

    controlProtocol cpUSB(0, 1, argv[1], speed); // my address, peer address, usb file

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


    if( (cpUSB.GetTECInfo(1, 1, &deviceType, &hwVersion, &fwVersion, &serialNumber)) ) // works
    {
        printf("++++++++++++++++++++++++GetTECInfo cmd good\n");
        printf("deviceType 0x%04X, hwVersion 0x%04X, fwVersion 0x%04X, serialNumber 0x%04X", 
            deviceType, hwVersion, fwVersion, serialNumber);
    } else
        printf("-------------------GetTECInfo bad\n");


    printf("\n\n");
    sleep(5);


    if( (cpUSB.GetTECInfo(1, 2, &deviceType, &hwVersion, &fwVersion, &serialNumber)) ) // works
    {
        printf("++++++++++++++++++++++++GetTECInfo cmd good\n");
        printf("deviceType 0x%04X, hwVersion 0x%04X, fwVersion 0x%04X, serialNumber 0x%04X", 
            deviceType, hwVersion, fwVersion, serialNumber);
    } else
        printf("-------------------GetTECInfo bad\n");


    printf("\n\n");
    sleep(5);


    if( (cpUSB.GetTECInfo(1, 3, &deviceType, &hwVersion, &fwVersion, &serialNumber)) ) // works
    {
        printf("++++++++++++++++++++++++GetTECInfo cmd good\n");
        printf("deviceType 0x%04X, hwVersion 0x%04X, fwVersion 0x%04X, serialNumber 0x%04X", 
            deviceType, hwVersion, fwVersion, serialNumber);
    } else
        printf("-------------------GetTECInfo bad\n");


    printf("\n\n");
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
    sleep(5);


    if( (cpUSB.GetHumidity(1, &humidity)) )     // working
        printf("+++++++++++++++++++++got: humidity %f\n", humidity);
    else
        printf("---------------------failed to get humidity\n");


    printf("\n\n");
    sleep(5);

    if( (cpUSB.SetHumidityThreshold(1, 70)) )  // working
        printf("+++++++++++++++++++++success on set humidity threshold\n");
    else
        printf("---------------------fail on set humidity threshold\n");


    printf("\n\n");
    sleep(5);


    if( (cpUSB.GetHumidityThreshold(1, &threshold)) ) // working
        printf("+++++++++++++++++++++got humidity threshold: %d\n", threshold);
    else
        printf("---------------------failed to get humidity threshold\n");


    printf("\n\n");
    sleep(5);


    if( (cpUSB.GetHumidity(1, &humidity)) ) // working
        printf("+++++++++++++++++++++got: humidity %f\n", humidity);
    else
        printf("---------------------failed to get humidity\n");


    printf("\n\n");
    sleep(5);

*/

    if( (cpUSB.SetTECTemperature(1, 1, -8.23)) )   // working
        printf("+++++++++++++++++++success on set TEC temperature\n");
    else
        printf("-------------------fail on set TEC temperature\n");


    printf("\n\n");
    sleep(5);


    if( (cpUSB.GetTECTemperature(1, 1, &result, &temperature)) ) // showing number that is too big
        printf("++++++++++++++++++++++got TEC temperature %f\n", temperature);// works if you program in non-zero
    else
        printf("----------------------failed to get TEC temperature\n");


    printf("\n\n");
    sleep(5);


    if( (cpUSB.SetTECTemperature(1, 2, -9.56)) )   // working
        printf("+++++++++++++++++++success on set TEC temperature\n");
    else
        printf("-------------------fail on set TEC temperature\n");


    printf("\n\n");
    sleep(5);


    if( (cpUSB.GetTECTemperature(1, 2, &result, &temperature)) ) // showing number that is too big
        printf("++++++++++++++++++++++got TEC temperature %f\n", temperature);// works if you program in non-zero
    else
        printf("----------------------failed to get TEC temperature\n");


    printf("\n\n");
    sleep(5);


    if( (cpUSB.SetTECTemperature(1, 3, -10.89)) )   // working
        printf("+++++++++++++++++++success on set TEC temperature\n");
    else
        printf("-------------------fail on set TEC temperature\n");


    printf("\n\n");
    sleep(5);


    if( (cpUSB.GetTECTemperature(1, 3, &result, &temperature)) ) // showing number that is too big
        printf("++++++++++++++++++++++got TEC temperature %f\n", temperature);// works if you program in non-zero
    else
        printf("----------------------failed to get TEC temperature\n");


    printf("\n\n");
    sleep(5);


    if( (cpUSB.SetChillerTemperature(1, -11.10)) ) // not working
        printf("++++++++++++++++++++++success for set chiller temperature\n");
    else
        printf("----------------------fail on set chiller temperature\n");


    printf("\n\n");
    sleep(5);


    if( (cpUSB.GetChillerTemperature(1, &temperature)) ) // not working
        printf("+++++++++++++++++++++got chiller temperature %f\n", temperature);
    else
        printf("---------------------failed to get chiller temperature\n");


    printf("\n\n");
    sleep(5);


    if( (cpUSB.EnableTECs(1)) )
        printf("+++++++++++++++++++++success on enable TECs\n");
    else
        printf("---------------------fail on enable TECs\n");


    printf("\n\n");
    sleep(5);


    if( (cpUSB.DisableTECs(1)) )    // LCD shows running still - not working
        printf("+++++++++++++++++++++success on disable TECs\n");
    else
        printf("---------------------failure on disable TECs\n");


    printf("\n\n");
    sleep(5);


    if( (cpUSB.ShutDownCmd(1)) )
        printf("+++++++++++++++++++++ShutDownCmd good\n");
    else
        printf("---------------------ShutDownCmd bad\n");
    printf("\n\n");


    return(0);
}

