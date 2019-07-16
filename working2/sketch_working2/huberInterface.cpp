// file huberInterface.cpp
#include "huberInterface.h"

huberInterface::huberInterface()
: interface("huberIface"), m_pCtrl(0) {};   // throw if pCtrl is 0 .. bla bla


huberInterface::~huberInterface() {}; 


bool huberInterface::setCtrlInterface(interface* pCtrl)
{
    m_pCtrl = pCtrl;
    return(true);
}


bool huberInterface::TxFrame(frame& pkt)
{
    bool    retVal  = true;


    // just write the pkt for now
    if(false == (retVal = m_comm.TxFrame(pkt)))
    {
        //printf("\n\nERROR ERROR huber unable to Tx frame\n\n");
    }

    return(retVal);
}

     // can be defined in a derived class
bool huberInterface::RxFrame(frame& pkt)     // can be defined in a derived class
{
    bool    retVal  = true;


    if(false == (retVal = m_comm.RxFrame(pkt)))
    {
        // this is non-blocking I/O for the UDP, may get these
        //printf("\n\nERROR ERROR huber unable to Rx frame\n\n");
    } else
    {
        // these always go out the Ctrl interface
        //printf("\tRx huber frame, sending to ctrl\n");
        m_pCtrl->TxFrame(pkt);
    }

    return(retVal);
}


