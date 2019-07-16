// file UDPTECIface.h

#ifndef __UDPTECIface__
#define __UDPTECIface__

#include "UDPInterface.h"

#define MYPORT_t 8000
#define DSTPORT_t 8001

class UDPTECIface : public UDPInterface
{
    public:
    UDPTECIface() : UDPInterface(MYPORT_t, DSTPORT_t) {};
    virtual ~UDPTECIface() {};
};

#endif

