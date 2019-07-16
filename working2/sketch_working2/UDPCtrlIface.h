// file UDPCtrlIface.h

#ifndef __UDPCtrlIface__
#define __UDPCtrlIface__

#include "UDPInterface.h"

#define MYPORT 7000
#define DSTPORT 7001

class UDPCtrlIface : 
public UDPInterface
{
public:
  UDPCtrlIface();
  virtual ~UDPCtrlIface();
};

#endif


