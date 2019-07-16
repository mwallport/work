// file Frame.h
#ifndef __frame__
#define __frame__

#include <stdint.h>


const int MAX_FRAME_SIZE    = 64;   // too big, too small ?  Arduino only has 32K for everything
class frame
{
    public:
    frame(uint8_t number) : m_number(number) {};
    virtual ~frame() {};
    uint8_t*    getFrame() { return(buf); }
    const int   getMAXFrameSize() { return(MAX_FRAME_SIZE); };
    uint8_t     getFrameNumber() { return(m_number); }

    protected:
    frame();
    frame(const frame&);
    frame& operator=(const frame&);

    uint8_t     buf[MAX_FRAME_SIZE];
    uint8_t     m_number;
};

#endif

