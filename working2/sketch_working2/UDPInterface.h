// file UDPInterface.h
#ifndef __UDPInterface__
#define __UDPInterface__

#include "frame.h"

class UDPInterface
{
    public:
    UDPInterface(int _myPort, int _dstPort);
    virtual ~UDPInterface();
    bool TxFrame(frame&);
    bool RxFrame(frame&);

    protected:
    int m_myPort;
    int m_dstPort;
    int m_sock;
};

#endif

