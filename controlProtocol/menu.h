#ifndef __MENU__
#define __MENU__
#include <unistd.h>
#include <iostream>
#include <string>
#include <cstdint>
#include "controlProtocol.h"

    

using namespace std;

typedef bool (controlProtocol::*pStartUpCmd_t)(uint16_t);
typedef bool (controlProtocol::*pShutDownCmd_t)(uint16_t);
typedef bool (controlProtocol::*pGetStatus_t)(uint16_t, uint16_t*, uint16_t*, uint16_t*);
typedef bool (controlProtocol::*pGetHumidity_t)(uint16_t, float*);
typedef bool (controlProtocol::*pSetHumidityThreshold_t)(uint16_t, uint16_t);
typedef bool (controlProtocol::*pGetHumidityThreshold_t)(uint16_t, uint16_t*);
typedef bool (controlProtocol::*pSetTECTemperature_t)(uint16_t, uint16_t, float);
typedef bool (controlProtocol::*pGetTECTemperature_t)(uint16_t, uint16_t, uint16_t*, float*);
typedef bool (controlProtocol::*pGetTECObjTemperature_t)(uint16_t, uint16_t, uint16_t*, float*);
typedef bool (controlProtocol::*pStartChiller_t)(uint16_t);
typedef bool (controlProtocol::*pStopChiller_t)(uint16_t);
typedef bool (controlProtocol::*pGetChillerInfo_t)(uint16_t, char*, uint8_t);
typedef bool (controlProtocol::*pSetChillerTemperature_t)(uint16_t, float);
typedef bool (controlProtocol::*pGetChillerTemperature_t)(uint16_t, float*);
typedef bool (controlProtocol::*pGetChillerObjTemperature_t)(uint16_t, float*);
typedef bool (controlProtocol::*pEnableTECs_t)(uint16_t);
typedef bool (controlProtocol::*pDisableTECs_t)(uint16_t);
typedef bool (controlProtocol::*pGetTECInfo_t)(uint16_t, uint16_t, uint32_t*, uint32_t*,
                      uint32_t*, uint32_t*, uint32_t*, uint32_t*, uint32_t*, uint32_t*);
typedef bool (controlProtocol::*pSetRTCCmd_t)(uint16_t);
typedef bool (controlProtocol::*pGetRTCCmd_t)(uint16_t, struct tm*);
typedef bool (controlProtocol::*pClrEventLogCmd_t)(uint16_t);
typedef bool (controlProtocol::*pGetEventLogCmd_t)(uint16_t, elogentry*);
typedef bool (controlProtocol::*pGetTempCmd_t)(uint16_t, uint16_t*);


class menuItemBase
{
    public:
    string  m_name;			            // name for the command to list in a menu
    string  m_description;	            // breif description of the command
    uint16_t m_destId;

    menuItemBase(const string& name, const string& description, const uint16_t destId = 1)
        : m_name(name), m_description(description), m_destId(destId) {};
    virtual ~menuItemBase() {};
    virtual void getParameters(void) {};    // prompt for parameters for cmd - derived as needed
    void executeTest(controlProtocol* pCP) {cout.flush(); cout << "Not implemented.\n"; cout.flush(); };
    virtual void execute(controlProtocol*) = 0;
    friend ostream& operator<<(ostream& str, const menuItemBase& item)
    {
        str << setw(30) << item.m_name << ":" << item.m_description;
        return(str);
    }
    
    private:
    menuItemBase();
    menuItemBase(const menuItemBase&);
    menuItemBase& operator=(const menuItemBase&);
};


// bool    StartUpCmd(uint16_t);
class menuStartUpCmd : public menuItemBase
{
    public:
    pStartUpCmd_t m_pStartUpCmd  = &controlProtocol::StartUpCmd;

    menuStartUpCmd()
        :   menuItemBase("startup system", "start TCUs and chiller"),
            m_pStartUpCmd(&controlProtocol::StartUpCmd) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pStartUpCmd)(m_destId) )
            cout << "\nstartup successful" << endl;
        else
            cout << "\nstartup failed" << endl;
    }
    
    private:
    menuStartUpCmd(const menuItemBase&);
    menuStartUpCmd& operator=(const menuItemBase&);
};


// bool    ShutDownCmd(uint16_t);
class menuShutDownCmd : public menuItemBase
{
    public:
    pShutDownCmd_t m_pShutDownCmd;

    menuShutDownCmd()
        :   menuItemBase("shutdown system", "stop TCUs, chiller not affected"),
            m_pShutDownCmd(&controlProtocol::ShutDownCmd) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pShutDownCmd)(m_destId) )
            cout << "\nshutdown successful" << endl;
        else
            cout << "\nshutdown failed" << endl;
    }
    
    private:
    menuShutDownCmd(const menuItemBase&);
    menuShutDownCmd& operator=(const menuItemBase&);
};


// bool    GetStatus(uint16_t, uint16_t*, uint16_t*, uint16_t*);
class menuGetStatus : public menuItemBase
{
    public:
    pGetStatus_t m_pGetStatus;

    menuGetStatus()
        :   menuItemBase("get status", "report humidity alert, TEC states, chiller state"),
            m_pGetStatus(&controlProtocol::GetStatus) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pGetStatus)(m_destId, &humidityAlert, &TECsRunning, &chillerRunning) )
        {
            cout << "\nhumidity alert: " << humidityAlert <<
                    " TECs running: " << TECsRunning <<
                    " chiller running: " << chillerRunning << endl;
        } else
        {
            cout << "\nunable to get status" << endl;
        }
    }

    uint16_t humidityAlert;
    uint16_t TECsRunning;
    uint16_t chillerRunning;
    
    private:
    menuGetStatus(const menuItemBase&);
    menuGetStatus& operator=(const menuItemBase&);
};


// bool    GetHumidity(uint16_t, float*);
class menuGetHumidity : public menuItemBase
{
    public:
    pGetHumidity_t m_pGetHumidity;

    menuGetHumidity()
        :   menuItemBase("get humidity", "get current humidity measurement"),
            m_pGetHumidity(&controlProtocol::GetHumidity) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pGetHumidity)(m_destId, &humidity) )
        {
            cout << "\nhumidity: " << humidity << endl;
        } else
        {
            cout << "\nunable to get humidity" << endl;
        }
    }

    float humidity;
    
    private:
    menuGetHumidity(const menuItemBase&);
    menuGetHumidity& operator=(const menuItemBase&);
};


//bool    SetHumidityThreshold(uint16_t, uint16_t);
class menuSetHumidityThreshold : public menuItemBase
{
    public:
    pSetHumidityThreshold_t m_pSetHumidityThreshold;
    void getParameters(void)
    {
        cout << "enter humidity threshold: "; cin >> humidityThreshold;
    }

    menuSetHumidityThreshold()
        :   menuItemBase("set humidity threshold", "set running humidity threshold"),
            m_pSetHumidityThreshold(&controlProtocol::SetHumidityThreshold) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pSetHumidityThreshold)(m_destId, humidityThreshold) )
            cout << "\nset humidity threshold successful" << endl;
        else
            cout << "\nset humidity threshold failed" << endl;
    }

    float humidityThreshold;
    
    private:
    menuSetHumidityThreshold(const menuItemBase&);
    menuSetHumidityThreshold& operator=(const menuItemBase&);
};


//bool (controlProtocol::*pGetHumidityThreshold_t)(uint16_t, uint16_t*);
class menuGetHumidityThreshold : public menuItemBase
{
    public:
    pGetHumidityThreshold_t m_pGetHumidityThreshold;

    menuGetHumidityThreshold()
        :   menuItemBase("get humidity threshold", "get current humidity threshold"),
            m_pGetHumidityThreshold(&controlProtocol::GetHumidityThreshold) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pGetHumidityThreshold)(m_destId, &humidityThreshold) )
        {
            cout << "\nhumidity threshold: " << humidityThreshold << endl;
        } else
        {
            cout << "\nget humidity threshold failed" << endl;
        }
    }

    uint16_t humidityThreshold;
    
    private:
    menuGetHumidityThreshold(const menuItemBase&);
    menuGetHumidityThreshold& operator=(const menuItemBase&);
};


// bool    SetTECTemperature(uint16_t, uint16_t, float);
class menuSetTECTemperature : public menuItemBase
{
    public:
    pSetTECTemperature_t m_pSetTECTemperature;
    void getParameters(void)
    {
        cout << "enter TEC address (i.e. 1, 2, or 3):  "; cin >> TECAddress;
        cout << "enter temperature (i.e. 5.0 or -5.0): "; cin >> temperature;
    }

    menuSetTECTemperature()
        :   menuItemBase("set TEC temperature", "set a TCU temperature"),
            m_pSetTECTemperature(&controlProtocol::SetTECTemperature) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pSetTECTemperature)(m_destId, TECAddress, temperature) )
            cout << "\nset TEC temperature succesful" << endl;
          else
            cout << "\nset TEC temperature failed" << endl;
    }

    uint16_t TECAddress;
    float temperature; 
    
    private:
    menuSetTECTemperature(const menuItemBase&);
    menuSetTECTemperature& operator=(const menuItemBase&);
};


//    bool    GetTECTemperature(uint16_t, uint16_t, uint16_t*, float*);
class menuGetTECTemperature : public menuItemBase
{
    public:
    pGetTECTemperature_t m_pGetTECTemperature;
    void getParameters(void)
    {
        cout << "enter TEC address:  "; cin >> TECAddress;
    }

    menuGetTECTemperature()
        :   menuItemBase("get TEC temperature", "get a TCU set-point temperature"),
            m_pGetTECTemperature(&controlProtocol::GetTECTemperature) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pGetTECTemperature)(m_destId, TECAddress, &result, &temperature) )
        {
            if( (result) )
                cout << "\nTEC temperature: " << temperature << endl;
            else
                cout << "\nget TEC temperature failed to return temp" << endl;
        } else
            cout << "\nget TEC temperature failed" << endl;
    }

    uint16_t TECAddress;
    uint16_t result;
    float temperature; 
    
    private:
    menuGetTECTemperature(const menuItemBase&);
    menuGetTECTemperature& operator=(const menuItemBase&);
};


class menuGetTECObjTemperature : public menuItemBase
{
    public:
    pGetTECObjTemperature_t m_pGetTECObjTemperature;
    
    void getParameters(void)
    {
        cout << "enter TEC address:  "; cin >> TECAddress;
    }

    menuGetTECObjTemperature()
        :   menuItemBase("get TEC obj temperature", "get a TCU object temperature"),
            m_pGetTECObjTemperature(&controlProtocol::GetTECObjTemperature) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pGetTECObjTemperature)(m_destId, TECAddress, &result, &temperature) )
        {
            if( (result) )
                cout << "\nTEC obj temperature: " << temperature << endl;
            else
                cout << "\nget TEC object temperature failed to return temp" << endl;
        } else
            cout << "\nget TEC object temperature failed" << endl;
    }

    uint16_t TECAddress;
    uint16_t result;
    float temperature; 
    
    private:
    menuGetTECObjTemperature(const menuItemBase&);
    menuGetTECObjTemperature& operator=(const menuItemBase&);
};


//    bool    StartChiller(uint16_t);
class menuStartChiller : public menuItemBase
{
    public:
    pStartChiller_t m_pStartChiller = &controlProtocol::StartChiller;

    menuStartChiller()
        :   menuItemBase("start chiller", "start the chiller"),
            m_pStartChiller(&controlProtocol::StartChiller) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pStartChiller)(m_destId) )
            cout << "\nstart chiller successful" << endl;
        else
            cout << "\nstart chiller failed" << endl;
    }
    
    private:
    menuStartChiller(const menuItemBase&);
    menuStartChiller& operator=(const menuItemBase&);
};


//    bool    StopChiller(uint16_t);
class menuStopChiller : public menuItemBase
{
    public:
    pStopChiller_t m_pStopChiller  = &controlProtocol::StopChiller;

    menuStopChiller()
        :   menuItemBase("stop chiller", "stop the chiller, also stops TECs"),
            m_pStopChiller(&controlProtocol::StopChiller) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pStopChiller)(m_destId) )
            cout << "\nstop chiller successful" << endl;
        else
            cout << "\nstop chiller failed" << endl;
    }
    
    private:
    menuStopChiller(const menuItemBase&);
    menuStopChiller& operator=(const menuItemBase&);
};


//    bool    GetChillerInfo(uint16_t, char*, uint8_t);
class menuGetChillerInfo : public menuItemBase
{
    public:
    pGetChillerInfo_t m_pGetChillerInfo;

    menuGetChillerInfo()
        :   menuItemBase("get chiller info", "get the chiller's name"),
            m_pGetChillerInfo(&controlProtocol::GetChillerInfo) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pGetChillerInfo)(m_destId, chillerInfo, 64) )
            cout << "\nget chiller info: " << chillerInfo << endl;
        else
            cout << "\nget chiller info failed" << endl;
    }

    char chillerInfo[64];
    
    private:
    menuGetChillerInfo(const menuItemBase&);
    menuGetChillerInfo& operator=(const menuItemBase&);
};


//    bool    SetChillerTemperature(uint16_t, float);
class menuSetChillerTemperature : public menuItemBase
{
    public:
    pSetChillerTemperature_t m_pSetChillerTemperature  = &controlProtocol::SetChillerTemperature;
    void getParameters(void)
    {
        cout << "enter temperature (i.e. 24.0 or -10.5): "; cin >> temperature;
    }

    menuSetChillerTemperature()
        :   menuItemBase("set chiller temperature", "set the chiller set-point temp"),
            m_pSetChillerTemperature(&controlProtocol::SetChillerTemperature) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pSetChillerTemperature)(m_destId, temperature) )
            cout << "\nset chiller temperature successful" << endl;
        else
            cout << "\nset chiller temperature failed" << endl;
    }

    float   temperature;

    private:
    menuSetChillerTemperature(const menuItemBase&);
    menuSetChillerTemperature& operator=(const menuItemBase&);
};


//    bool    GetChillerTemperature(uint16_t, float*);
class menuGetChillerTemperature : public menuItemBase
{
    public:
    pGetChillerTemperature_t m_pGetChillerTemperature;

    menuGetChillerTemperature()
        :   menuItemBase("get chiller temperature", "get the chiller set-point temp"),
            m_pGetChillerTemperature(&controlProtocol::GetChillerTemperature) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pGetChillerTemperature)(m_destId, &temperature) )
            cout << "\nchiller set-point temperature: " << temperature << endl;
        else
            cout << "\nget chiller temperature failed" << endl;
    }

    float   temperature;

    private:
    menuGetChillerTemperature(const menuItemBase&);
    menuGetChillerTemperature& operator=(const menuItemBase&);
};


class menuGetChillerObjTemperature : public menuItemBase
{
    public:
    pGetChillerObjTemperature_t m_pGetChillerObjTemperature;
    
    menuGetChillerObjTemperature()
        :   menuItemBase("get chiller obj temperature", "get the chiller internal temp"),
            m_pGetChillerObjTemperature(&controlProtocol::GetChillerObjTemperature) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pGetChillerObjTemperature)(m_destId, &temperature) )
            cout << "\nchiller internal temperature: " << temperature << endl;
        else
            cout << "\nget chiller obj temperature failed" << endl;
    }

    float   temperature;

    private:
    menuGetChillerObjTemperature(const menuItemBase&);
    menuGetChillerObjTemperature& operator=(const menuItemBase&);
};


//    bool    EnableTECs(uint16_t);
class menuEnableTECs : public menuItemBase
{
    public:
    pEnableTECs_t m_pEnableTECs;

    menuEnableTECs()
        :   menuItemBase("start TECs", "start all TECs"),
            m_pEnableTECs(&controlProtocol::EnableTECs) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pEnableTECs)(m_destId) )
            cout << "\nstart TECs successful" << endl;
        else
            cout << "\nstart TECs failed" << endl;
    }
    
    private:
    menuEnableTECs(const menuItemBase&);
    menuEnableTECs& operator=(const menuItemBase&);
};


//    bool    DisableTECs(uint16_t);
class menuDisableTECs : public menuItemBase
{
    public:
    pDisableTECs_t m_pDisableTECs;

    menuDisableTECs()
        :   menuItemBase("stop TECs", "stop all TECs"),
            m_pDisableTECs(&controlProtocol::DisableTECs) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pDisableTECs)(m_destId) )
            cout << "\nstop TECs successful" << endl;
        else
            cout << "\nstop TECs failed" << endl;
    }
    
    private:
    menuDisableTECs(const menuItemBase&);
    menuDisableTECs& operator=(const menuItemBase&);
};


//    bool    GetTECInfo(uint16_t, uint16_t, uint32_t*, uint32_t*, uint32_t*, uint32_t*);
class menuGetTECInfo : public menuItemBase
{
    public:
    pGetTECInfo_t m_pGetTECInfo;
    void getParameters(void)
    {
        cout << "enter TEC address:  "; cin >> tec_address;
    }

    menuGetTECInfo()
        :   menuItemBase("get TEC info", "get a TEC's h/w & s/w version, model, and serial number"),
            m_pGetTECInfo(&controlProtocol::GetTECInfo) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pGetTECInfo)(m_destId, tec_address, &deviceType, &hwVersion,
                    &fwVersion, &serialNum, &deviceStatus, &errNumber, &errInstance, &errParameter) )
        {
          printf("\ndeviceType: %zu hwVersion: %zu fwVersion: %zu serialNum: %zu\n",
            htons(deviceType), htons(hwVersion), htons(fwVersion), htons(serialNum));

          printf("deviceStatus: %zu errNumber: %zu errInstance: %zu errParameter: %zu\n",
            htons(deviceStatus), htons(errNumber), htons(errInstance), htons(errParameter));

        } else
        {
            cout << "\nget TEC info failed" << endl;
        }
    }

    uint16_t tec_address;
    uint32_t deviceType;
    uint32_t hwVersion;
    uint32_t fwVersion;
    uint32_t serialNum;
    uint32_t deviceStatus;
    uint32_t errNumber;
    uint32_t errInstance;
    uint32_t errParameter;
    
    private:
    menuGetTECInfo(const menuItemBase&);
    menuGetTECInfo& operator=(const menuItemBase&);
};


// bool    SetRTCCmd(uint16_t);
class menuSetRTCCmd : public menuItemBase
{
    public:
    pSetRTCCmd_t m_pSetRTCCmd;

    menuSetRTCCmd()
        :   menuItemBase("set clock", "set the clock"),
            m_pSetRTCCmd(&controlProtocol::SetRTCCmd) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pSetRTCCmd)(m_destId) )
            cout << "\nset RTC successful" << endl;
        else
            cout << "\nset RTC failed" << endl;
    }
    
    private:
    menuSetRTCCmd(const menuItemBase&);
    menuSetRTCCmd& operator=(const menuItemBase&);
};


// bool    GetRTCCmd(uint16_t);
class menuGetRTCCmd : public menuItemBase
{
    public:
    pGetRTCCmd_t m_pGetRTCCmd;

    menuGetRTCCmd()
        :   menuItemBase("get clock", "get the clock"),
            m_pGetRTCCmd(&controlProtocol::GetRTCCmd) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pGetRTCCmd)(m_destId, &ltime) )
        {
            // output the time 
            cout << "time : " << asctime(&ltime) << endl;
        }
        else
            cout << "\nget RTC failed" << endl;
    }
    
    private:
    struct tm ltime;


    menuGetRTCCmd(const menuItemBase&);
    menuGetRTCCmd& operator=(const menuItemBase&);
};


// bool    CleEventLogCmd(uint16_t);
class menuClrEventLogCmd : public menuItemBase
{
    public:
    pClrEventLogCmd_t m_pClrEventLogCmd  = &controlProtocol::ClrEventLogCmd;

    menuClrEventLogCmd()
        :   menuItemBase("clear eventlog", "clear the eventlog"),
            m_pClrEventLogCmd(&controlProtocol::ClrEventLogCmd) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pClrEventLogCmd)(m_destId) )
            cout << "\nclear event log  successful" << endl;
        else
            cout << "\nclear event log failed" << endl;
    }
    
    private:
    menuClrEventLogCmd(const menuItemBase&);
    menuClrEventLogCmd& operator=(const menuItemBase&);
};


class menuGetEventLogCmd : public menuItemBase
{
    public:
    pGetEventLogCmd_t m_pGetEventLogCmd  = &controlProtocol::GetEventLogCmd;

    menuGetEventLogCmd()
        :   menuItemBase("get eventlog", "get the eventlog"),
            m_pGetEventLogCmd(&controlProtocol::GetEventLogCmd) {}

    void execute(controlProtocol* pCP)
    {
      memset(&eventlog, '\0', sizeof(eventlog));

      if( (pCP->*m_pGetEventLogCmd)(m_destId, &eventlog[0]) )
      {
        //
        // simple decode of the event log entries
        //
        for(int i = 0; i < MAX_ELOG_ENTRY; i++)
        {
          // get the time stamp
          ltime.tm_sec  = eventlog[i].ts.sec;
          ltime.tm_min  = eventlog[i].ts.min;
          ltime.tm_hour = eventlog[i].ts.hour + 1;
          ltime.tm_mon  = eventlog[i].ts.mon;
          ltime.tm_year = eventlog[i].ts.year + 101;
          ltime.tm_wday = eventlog[i].ts.wday;
          ltime.tm_mday = eventlog[i].ts.mday;

          // get the id and the instance
          id  = eventlog[i].id & 0x0000ffff;
          inst  = (eventlog[i].id  >> 16) & 0x0000ffff;
          memset(time_buff, '\0', sizeof(time_buff));
          asctime_r(&ltime, time_buff);
          time_buff[strlen(time_buff) - 1] = 0; // rid of the \n at the end
          
          switch(id)
          {
            case TECNotOnLine:
            {
              printf("%-26s : %-18s TCU %u not on line\n",
                time_buff, "TCUNotOnLine", inst);
              break;
            }
            case TECNotRunning:
            {
              printf("%-26s : %-18s TCU %u not running\n",
                time_buff, "TCUNotRunning", inst);
              break;
            }
            case TECIsMismatch:
            {
              printf("%-26s : %-18s TCU %u state mismatch\n",
                time_buff, "TCUIsMismatch", inst);

              break;
            }
            case TECErrorInfo:
            {
              printf("%-26s : %-18s TCU %u serial: %zu status: %zu errNum: %zu errInst: %zu errParam: %zu\n",
                time_buff, "TCUErrorInfo", inst, htons(eventlog[i].data[0]),
                htons(eventlog[i].data[1]), htons(eventlog[i].data[2]),
                htons(eventlog[i].data[3]), htons(eventlog[i].data[4]));

              break;
            }
            case HumiditySensorOffline:
            {
              printf("%-26s : %-18s humidity sensor offline\n",
                time_buff, "HumidityOffline");

              break;
            }
            case HumidityHigh:
            {
              printf("%-26s : %-18s humidity high\n",
                time_buff, "HumidityHigh");

              break;
            }
            case ChillerOffline:
            {
              printf("%-26s : %-18s chiller offline\n",
                time_buff, "ChillerOffline");

              break;
            }
            case ChillerNotRunning:
            {
              printf("%-26s : %-18s chiller not running\n",
                time_buff, "ChillerNotRunning");

              break;
            }
            default:
            {
              // show the bytes
              printf("%-26s : %-18s Id: %zu data[0] %zu data[1] %zu data[2] %zu data[3] %zu data[4] %zu\n",
                time_buff, "no event", inst, htons(eventlog[i].data[0]),
                htons(eventlog[i].data[1]), htons(eventlog[i].data[2]),
                htons(eventlog[i].data[3]), htons(eventlog[i].data[4]));
              break;
            }
          }
        }

      }
      else
        cout << "\nfailed to get event log" << endl;
    }
    
    private:
    elogentry eventlog[MAX_ELOG_ENTRY];
    timeind*  pTimeStamp;
    struct tm ltime;
    uint16_t  id;       // chiller, TCU, humidity sensor
    uint16_t  inst;     // in case of TCU is the TCU number
    char      time_buff[30];



    menuGetEventLogCmd(const menuItemBase&);
    menuGetEventLogCmd& operator=(const menuItemBase&);
};

// bool    GetRTCCmd(uint16_t);
class menuGetTempCmd : public menuItemBase
{
    public:
    pGetTempCmd_t m_pGetTempCmd;

    menuGetTempCmd()
        :   menuItemBase("get ambient temp", "get the ambient temperature"),
            m_pGetTempCmd(&controlProtocol::GetTempCmd) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pGetTempCmd)(m_destId, &temp) )
        {
            base      = temp & 0x00ff;
            fraction  = (temp & 0xff00) >> 8;
            ftemp     = base + (((float)(fraction))/100);
            printf("temperature (celcius) : %.2f\n", ftemp);
        }
        else
        {
            printf("get temperature failed\n");
        }
    }
    
    private:
    uint16_t  temp      = 0;
    uint16_t  base      = 0;
    uint16_t  fraction  = 0;
    float     ftemp     = 0;


    menuGetTempCmd(const menuItemBase&);
    menuGetTempCmd& operator=(const menuItemBase&);
};

#endif

