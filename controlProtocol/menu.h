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
typedef bool (controlProtocol::*pGetTECInfo_t)(uint16_t, uint16_t, uint32_t*, uint32_t*, uint32_t*, uint32_t*);


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
        :   menuItemBase("GetTECObjTemperature", "get a TCU object temperature"),
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
    pStartChiller_t m_pStartChiller;

    menuStartChiller()
        :   menuItemBase("start chiller", "start the chiller"),
            m_pStartChiller(controlProtocol::StartChiller) {}

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
                                &fwVersion, &serialNum) )
        {
            cout << "\ndeviceType: " << deviceType <<
                    "hwVersion: " << hwVersion <<
                    "fwVersion: " << fwVersion <<
                    "serialNum: " << serialNum << endl;
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
    
    private:
    menuGetTECInfo(const menuItemBase&);
    menuGetTECInfo& operator=(const menuItemBase&);
};
#endif

