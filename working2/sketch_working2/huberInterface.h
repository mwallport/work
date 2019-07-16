// file huberInterface.h

#ifndef __huberInterface__
#define __huberInterface__

#include "interface.h"
#include "UDPHuberIface.h"

class huberInterface : public interface
{
    public:
    huberInterface();
    virtual ~huberInterface();
    bool setCtrlInterface(interface*);
    virtual bool TxFrame(frame&);     // can be defined in a derived class
    virtual bool RxFrame(frame&);     // can be defined in a derived class

    protected:
    UDPHuberIface m_comm;                   // testing pkt flow with UDP
    interface*  m_pCtrl;                    // THE control interface

    huberInterface(const huberInterface&);  // no copy
};

#endif

