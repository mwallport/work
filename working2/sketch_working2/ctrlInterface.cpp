#include "ctrlInterface.h"

// read a frame from the controlling PC software
// identify whether the frame is bound for TEC or Huber
// send the frame there
bool ctrlInterface::RxFrame(frame& pkt)
{
    bool    retVal  = true;


    if(!m_comm.RxFrame(pkt))
    {
        // TODO: increment some statistic ?
        retVal = false;
    } else if(!handleInBoundCtrlFrame(pkt))
    {
        // TODO: increment some statistic ?
        retVal = false;
    }

    return(retVal);
}


bool ctrlInterface::handleInBoundCtrlFrame(frame& pkt)
{
    uint8_t*    pByte;
    bool        retVal  = true;


    pByte = pkt.getFrame();   // pointing at the 1st byte in the frame

    // for now just send the packet out the correct interface
    if(('[' == pByte[0]) && ('M' == pByte[1]))
    {
        // huber
        //printf("ctrl found a huber command, sending to huber\n");
        if(!m_huberInterface.TxFrame(pkt))
        {
            //printf("\nERROR was unable to send huber command\n");
            retVal = false;
        }
    } else if(0x23 == pByte[0])
    {
        // TEC
        //printf("ctrl found a tec command, sending to tec\n");
        if(!m_tecInterface.TxFrame(pkt))
        {
            //printf("\nERROR was unable to send tec command\n");
            retVal = false;
        }
    } else
    {
        //printf("\n\nERROR ERROR unrecognized packet\n\n");
    }

    return(retVal);
}


bool ctrlInterface::TxFrame(frame& pkt)
{
    bool    retVal  = true;


    // just write the pkt for now
    if(false == (retVal = m_comm.TxFrame(pkt)))
    {   
        //printf("\n\nERROR ERROR Ctrl unable to Tx frame\n\n");
    }   

    return(retVal);
}
