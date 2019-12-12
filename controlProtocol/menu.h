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
typedef bool (controlProtocol::*pStartChiller_t)(uint16_t);
typedef bool (controlProtocol::*pStopChiller_t)(uint16_t);
typedef bool (controlProtocol::*pGetChillerInfo_t)(uint16_t, char*, uint8_t);
typedef bool (controlProtocol::*pSetChillerTemperature_t)(uint16_t, float);
typedef bool (controlProtocol::*pGetChillerTemperature_t)(uint16_t, float*);
typedef bool (controlProtocol::*pEnableTECs_t)(uint16_t);
typedef bool (controlProtocol::*pDisableTECs_t)(uint16_t);
typedef bool (controlProtocol::*pGetTECInfo_t)(uint16_t, uint16_t, uint32_t*, uint32_t*, uint32_t*, uint32_t*);


class menuItemBase
{
    public:
    string  m_name;			            // name for the command to list in a menu
    string  m_description;	            // breif description of the command
    uint16_t m_destId;

    menuItemBase(const string& name, const string& description, const uint16_t m_destId = 1)
        : m_name(name), m_description(description) {};
    virtual ~menuItemBase() {};
    virtual void getParameters(void) {};    // function pointer to get parameters for cmd
    void executeTest(controlProtocol* pCP) {cout.flush(); cout << "bla bla bla\n"; cout.flush(); };
    virtual void execute(controlProtocol*) = 0;
    friend ostream& operator<<(ostream& str, const menuItemBase& item)
    {
        str << setw(25) << item.m_name << ":" << item.m_description;
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
    void getParameters(void) { cout << "enter dest address: "; cin >> m_destId; }

    menuStartUpCmd()
        : menuItemBase("StartUpCmd", "start TCUs and chiller")
    {
        m_pStartUpCmd = &controlProtocol::StartUpCmd;
    }

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pStartUpCmd)(m_destId) )
            cout << "\nStartUpCmd successful" << endl;
        else
            cout << "\nStartUpCmd failed" << endl;
    }
    
    private:
    menuStartUpCmd(const menuItemBase&);
    menuStartUpCmd& operator=(const menuItemBase&);
};


// bool    ShutDownCmd(uint16_t);
class menuShutDownCmd : public menuItemBase
{
    public:
    pShutDownCmd_t m_pShutDownCmd  = &controlProtocol::ShutDownCmd;
    void getParameters(void) { cout << "enter dest address: "; cin >> m_destId; }

    menuShutDownCmd()
        : menuItemBase("ShutDownCmd", "stop TCUs and chiller*")
    {
        m_pShutDownCmd = &controlProtocol::ShutDownCmd;
    }

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pShutDownCmd)(m_destId) )
            cout << "\nShutDownCmd successful" << endl;
        else
            cout << "\nShutDownCmd failed" << endl;
    }
    
    private:
    menuShutDownCmd(const menuItemBase&);
    menuShutDownCmd& operator=(const menuItemBase&);
};


// bool    GetStatus(uint16_t, uint16_t*, uint16_t*, uint16_t*);
class menuGetStatus : public menuItemBase
{
    public:
    pGetStatus_t m_pGetStatus  = &controlProtocol::GetStatus;
    void getParameters(void) { cout << "enter dest address: "; cin >> m_destId; }

    menuGetStatus()
        : menuItemBase("GetStatus", "check humidity alert, TECs state, chiller state")
    {
        m_pGetStatus = &controlProtocol::GetStatus;
    }

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pGetStatus)(m_destId, &humidityAlert, &TECsRunning, &chillerRunning) )
        {
            cout << "\nhumidityAlert: " << humidityAlert <<
                    "TECsRunning: " << TECsRunning <<
                    "chillerRunning: " << chillerRunning << endl;
        } else
        {
            cout << "\nunable to GetStatus" << endl;
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
    pGetHumidity_t m_pGetHumidity  = &controlProtocol::GetHumidity;
    void getParameters(void) { cout << "enter dest address: "; cin >> m_destId; }

    menuGetHumidity()
        : menuItemBase("GetHumidity", "get current humidity")
    {
        m_pGetHumidity = &controlProtocol::GetHumidity;
    }

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pGetHumidity)(m_destId, &humidity) )
        {
            cout << "\nhumidity: " << humidity << endl;
        } else
        {
            cout << "\nunable to GetHumidity" << endl;
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
    pSetHumidityThreshold_t m_pSetHumidityThreshold  = &controlProtocol::SetHumidityThreshold;
    void getParameters(void)
    {
        cout << "enter dest address:        "; cin >> m_destId;
        cout << "enter humidity threshold:  "; cin >> humidityThreshold;
    }

    menuSetHumidityThreshold()
        : menuItemBase("SetHumidityThreshold", "set humidity threshold")
    {
        m_pSetHumidityThreshold = &controlProtocol::SetHumidityThreshold;
    }

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pSetHumidityThreshold)(m_destId, humidityThreshold) )
            cout << "\nsetHumidityThreshold success" << endl;
        else
            cout << "\nsetHumidityThreshold failed" << endl;
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
    pGetHumidityThreshold_t m_pGetHumidityThreshold  = &controlProtocol::GetHumidityThreshold;
    void getParameters(void)
    {
        cout << "enter dest address: "; cin >> m_destId;
    }

    menuGetHumidityThreshold()
        : menuItemBase("GetHumidityThreshold", "get humidity threshold")
    {
        m_pGetHumidityThreshold = &controlProtocol::GetHumidityThreshold;
    }

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pGetHumidityThreshold)(m_destId, &humidityThreshold) )
        {
            cout << "\nhumidityThreshold: " << humidityThreshold << endl;
        } else
        {
            cout << "\nunable to GetHumidityThreshold" << endl;
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
    pSetTECTemperature_t m_pSetTECTemperature  = &controlProtocol::SetTECTemperature;
    void getParameters(void)
    {
        cout << "enter dest address: "; cin >> m_destId;
        cout << "enter TEC address:  "; cin >> TECAddress;
        cout << "enter temperature:  "; cin >> temperature;
    }

    menuSetTECTemperature()
        : menuItemBase("SetTECTemperature", "set a TCU's temperature")
    {
        m_pSetTECTemperature = &controlProtocol::SetTECTemperature;
    }

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pSetTECTemperature)(m_destId, TECAddress, temperature) )
            cout << "\nSetTECTemperature succesful" << endl;
          else
            cout << "\nSetTECTemperature failed" << endl;
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
    pGetTECTemperature_t m_pGetTECTemperature  = &controlProtocol::GetTECTemperature;
    void getParameters(void)
    {
        cout << "enter dest address: "; cin >> m_destId;
        cout << "enter TEC address:  "; cin >> TECAddress;
    }

    menuGetTECTemperature()
        : menuItemBase("GetTECTemperature", "get a TCU's temperature")
    {
        m_pGetTECTemperature = &controlProtocol::GetTECTemperature;
    }

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pGetTECTemperature)(m_destId, TECAddress, &result, &temperature) )
        {
            if( (result) )
                cout << "\nGetTECTemperature: " << temperature << endl;
            else
                cout << "\nGetTECTemperature failed to return temp" << endl;
        } else
            cout << "\nGetTECTemperature failed" << endl;
    }

    uint16_t TECAddress;
    uint16_t result;
    float temperature; 
    
    private:
    menuGetTECTemperature(const menuItemBase&);
    menuGetTECTemperature& operator=(const menuItemBase&);
};


//    bool    StartChiller(uint16_t);
class menuStartChiller : public menuItemBase
{
    public:
    pStartChiller_t m_pStartChiller  = &controlProtocol::StartChiller;
    void getParameters(void) { cout << "enter dest address: "; cin >> m_destId; }

    menuStartChiller()
        : menuItemBase("StartChiller", "start the chiller")
    {
        m_pStartChiller = &controlProtocol::StartChiller;
    }

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pStartChiller)(m_destId) )
            cout << "\nStartChiller successful" << endl;
        else
            cout << "\nStartChiller failed" << endl;
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
    void getParameters(void) { cout << "enter dest address: "; cin >> m_destId; }

    menuStopChiller()
        : menuItemBase("StopChiller", "stop the chiller")
    {
        m_pStopChiller = &controlProtocol::StopChiller;
    }

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pStopChiller)(m_destId) )
            cout << "\nStopChiller successful" << endl;
        else
            cout << "\nStopChiller failed" << endl;
    }
    
    private:
    menuStopChiller(const menuItemBase&);
    menuStopChiller& operator=(const menuItemBase&);
};


//    bool    GetChillerInfo(uint16_t, char*, uint8_t);
class menuGetChillerInfo : public menuItemBase
{
    public:
    pGetChillerInfo_t m_pGetChillerInfo  = &controlProtocol::GetChillerInfo;
    void getParameters(void) { cout << "enter dest address: "; cin >> m_destId; }

    menuGetChillerInfo()
        : menuItemBase("GetChillerInfo", "get the chiller name")
    {
        m_pGetChillerInfo = &controlProtocol::GetChillerInfo;
    }

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pGetChillerInfo)(m_destId, chillerInfo, 64) )
            cout << "\nGetChillerInfo: " << chillerInfo << endl;
        else
            cout << "\nGetChillerInfo failed" << endl;
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
        cout << "enter dest address: "; cin >> m_destId;
        cout << "enter temperature:  "; cin >> temperature;
    }

    menuSetChillerTemperature()
        : menuItemBase("SetChillerTemperature", "set the chiller set point temp")
    {
        m_pSetChillerTemperature = &controlProtocol::SetChillerTemperature;
    }

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pSetChillerTemperature)(m_destId, temperature) )
            cout << "\nSetChillerTemperature successful" << endl;
        else
            cout << "\nSetChillerTemperature failed" << endl;
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
    pGetChillerTemperature_t m_pGetChillerTemperature  = &controlProtocol::GetChillerTemperature;
    void getParameters(void)
    {
        cout << "enter dest address: "; cin >> m_destId;
    }

    menuGetChillerTemperature()
        : menuItemBase("GetChillerTemperature", "get the chiller running temp")
    {
        m_pGetChillerTemperature = &controlProtocol::GetChillerTemperature;
    }

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pGetChillerTemperature)(m_destId, &temperature) )
            cout << "\nGetChillerTemperature: " << temperature << endl;
        else
            cout << "\nGetChillerTemperature failed" << endl;
    }

    float   temperature;

    private:
    menuGetChillerTemperature(const menuItemBase&);
    menuGetChillerTemperature& operator=(const menuItemBase&);
};


//    bool    EnableTECs(uint16_t);
class menuEnableTECs : public menuItemBase
{
    public:
    pEnableTECs_t m_pEnableTECs  = &controlProtocol::EnableTECs;
    void getParameters(void) { cout << "enter dest address: "; cin >> m_destId; }

    menuEnableTECs()
        : menuItemBase("EnableTECs", "start all TECs")
    {
        m_pEnableTECs = &controlProtocol::EnableTECs;
    }

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pEnableTECs)(m_destId) )
            cout << "\nEnableTECs successful" << endl;
        else
            cout << "\nEnableTECs failed" << endl;
    }
    
    private:
    menuEnableTECs(const menuItemBase&);
    menuEnableTECs& operator=(const menuItemBase&);
};


//    bool    DisableTECs(uint16_t);
class menuDisableTECs : public menuItemBase
{
    public:
    pDisableTECs_t m_pDisableTECs  = &controlProtocol::DisableTECs;
    void getParameters(void) { cout << "enter dest address: "; cin >> m_destId; }

    menuDisableTECs()
        : menuItemBase("DisableTECs", "stop all TECs")
    {
        m_pDisableTECs = &controlProtocol::DisableTECs;
    }

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pDisableTECs)(m_destId) )
            cout << "\nDisableTECs successful" << endl;
        else
            cout << "\nDisableTECs failed" << endl;
    }
    
    private:
    menuDisableTECs(const menuItemBase&);
    menuDisableTECs& operator=(const menuItemBase&);
};


//    bool    GetTECInfo(uint16_t, uint16_t, uint32_t*, uint32_t*, uint32_t*, uint32_t*);
class menuGetTECInfo : public menuItemBase
{
    public:
    pGetTECInfo_t m_pGetTECInfo  = &controlProtocol::GetTECInfo;
    void getParameters(void)
    {
        cout << "enter dest address: "; cin >> m_destId;
        cout << "enter TEC address:  "; cin >> tec_address;
    }

    menuGetTECInfo()
        : menuItemBase("GetTECInfo", "get a TEC's h/w & s/w version, model, and serial number'")
    {
        m_pGetTECInfo = &controlProtocol::GetTECInfo;
    }

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
            cout << "\nunable to GetTECInfo" << endl;
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

