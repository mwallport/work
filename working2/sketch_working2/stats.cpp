//file stats.cpp

#include "stats.h"

huberStats::huberStats()
: m_verifyRX(0), m_verifyTX(0) {};

huberStats::~huberStats() {};
void huberStats::incrVerifyRx() { ++m_verifyRX; }
void huberStats::incrVerifyTx() { ++m_verifyTX; }
uint16_t huberStats::getVerifyRx() { return(m_verifyRX); }
uint16_t huberStats::getVerifyTx() { return(m_verifyTX); }


tecStats::tecStats() : m_resetRX(0), m_resetTX(0) {}
tecStats::~tecStats() {}
void tecStats::incrResetRx() { ++m_resetRX; }
void tecStats::incrResetTx() { ++m_resetTX; }
uint16_t tecStats::getResetRx() { return(m_resetRX); }
uint16_t tecStats::getResetTx() { return(m_resetTX); }


stats::stats() {};
stats::~stats() {};


