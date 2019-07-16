// file tecInterface.cpp
#include "tecInterface.h"

tecInterface::tecInterface()
: interface("tecIface"), m_pCtrl(0) {};   // throw if pCtrl is 0 .. bla bla {}; 


tecInterface::~tecInterface() {}; 

bool tecInterface::setCtrlInterface(interface* pCtrl)
{
    m_pCtrl = pCtrl;
    return(true);
}

bool tecInterface::TxFrame(frame& pkt)
{
    bool    retVal  = true;


    // just write the pkt for now
    if(false == (retVal = m_comm.TxFrame(pkt)))
    {
        //printf("\n\nERROR ERROR TEC unable to Tx frame\n\n");
    }

    return(retVal);
}

     // can be defined in a derived class
bool tecInterface::RxFrame(frame& pkt)     // can be defined in a derived class
{
    bool    retVal  = true;


    if(false == (retVal = m_comm.RxFrame(pkt)))
    {
        // this is non-blocking I/O for the UDP, may get these
        //printf("\n\nERROR ERROR TEC unable to Rx frame\n\n");
    } else
    {
        // these always go out the Ctrl interface
        //printf("\tRx tec frame, sending to ctrl\n");
        m_pCtrl->TxFrame(pkt);
    }

    return(retVal);
}


