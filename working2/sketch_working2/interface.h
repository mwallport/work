// file interface.h
// base class for the various interfaces
// this project may use

#ifndef __interface__
#define __interface__

#include <string.h>
#include "stats.h"
#include "frame.h"

using namespace std;

#define MAX_NAME_LENGTH 10

class interface
{
    public:
    interface(const char* name);
    virtual ~interface() {};
    
    virtual bool TxFrame(frame&) = 0;     // can be defined in a derived class
    virtual bool RxFrame(frame&) = 0;     // can be defined in a derived class
    static bool HandleFrame(frame&);  // state machines for each frame of interest
    stats GetStats() { return m_stats; };
    
    protected:
    interface();                                    // no default constructor
    interface(const interface&);                    // no copy constructor

    char m_name[MAX_NAME_LENGTH + 1];
    stats m_stats;  // not static, each interface will have stats, should all be the same
};

#endif

