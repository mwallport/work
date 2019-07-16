// file ctrlInterface.h
// this is the interface facing the controlling PC
// and, is the interface that will mux/de-mux messages from the controll PC to the chiller(s) and the TEC(s)
// the main difference here is, this has TECInterface(s) and huberInterface(s)
// .. starting with just one of each though ..
//
// and is a singleton ..
//
#ifndef __ctrlInterface__
#define __ctrlInterface__

#include "interface.h"
#include "UDPCtrlIface.h"

class ctrlInterface : public interface
{
    public:
    ctrlInterface(interface& _huber, interface& _tec)
        : interface("ctrlIface"),  m_huberInterface(_huber), m_tecInterface(_tec), m_comm() {};
    virtual ~ctrlInterface() {};
    bool TxFrame(frame&);     // can be defined in a derived class
    bool RxFrame(frame&);     // can be defined in a derived class
    interface& getTECInterface() { return m_tecInterface; };
    interface& getHuberInterface() { return m_huberInterface; };

    protected:
    ctrlInterface();
    ctrlInterface(const ctrlInterface&);
    ctrlInterface& operator=(const ctrlInterface&);

    bool handleInBoundCtrlFrame(frame&);

    interface& m_huberInterface;
    interface& m_tecInterface;
    UDPCtrlIface m_comm;  // the UDP interface
};

#endif

