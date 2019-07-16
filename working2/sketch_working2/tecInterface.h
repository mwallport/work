// file tecInterface.h

#ifndef __tecInterface__
#define __tecInterface__

#include "interface.h"
#include "UDPTECIface.h"

class tecInterface : public interface
{
    public:
    tecInterface();
    virtual ~tecInterface();
    bool setCtrlInterface(interface*);
    virtual bool TxFrame(frame&);     // can be defined in a derived class
    virtual bool RxFrame(frame&);     // can be defined in a derived class

    protected:
    UDPTECIface m_comm;
    interface*  m_pCtrl;                    // THE control interface

    tecInterface(const tecInterface&);      // no copy
};

#endif

