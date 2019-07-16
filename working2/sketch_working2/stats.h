//file stats.h
#ifndef __stats__
#define __stats__

#include <stdint.h>

class huberStats
{
    public:
    huberStats();
    virtual ~huberStats();
    inline void incrVerifyRx();
    inline void incrVerifyTx();
    inline uint16_t getVerifyRx();
    inline uint16_t getVerifyTx();

    protected:
    uint16_t    m_verifyRX;
    uint16_t    m_verifyTX;
};    


class tecStats
{
    public:
    tecStats();
    virtual ~tecStats();
    inline void incrResetRx();
    inline void incrResetTx();
    inline uint16_t getResetRx();
    inline uint16_t getResetTx();

    protected:
    uint16_t    m_resetRX;
    uint16_t    m_resetTX;
};    

class stats
{
    public:
    stats();
    virtual ~stats();
    huberStats  m_huberStats;
    tecStats    m_tecStats;
};

#endif

