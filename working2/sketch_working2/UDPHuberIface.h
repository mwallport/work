// file UDPHuberIface.h

#ifndef __UDPHuberCtrlIface__
#define __UDPHuberCtrlIface__

#include "UDPInterface.h"

#define MYPORT_h 9000
#define DSTPORT_h 9001

class UDPHuberIface : 
public UDPInterface
{
public:
  UDPHuberIface() : 
  UDPInterface(MYPORT_h, DSTPORT_h) {
  };
  virtual ~UDPHuberIface() {
  };
};

#endif


