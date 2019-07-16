#include <unistd.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
//#include <arpa/inet.h>
//#include <sys/socket.h>
#include "UDPInterface.h"


/*
 * setting non-blocking for testing
 */ 



UDPInterface::UDPInterface(int _myPort, int _dstPort)
    : m_myPort(_myPort), m_dstPort(_dstPort)
{
    /*
    struct sockaddr_in  si_me;
    struct sockaddr_in  servaddr;


    if ((m_sock=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        throw("socket");
    }

    // zero out the structure
    memset((char *) &si_me, 0, sizeof(si_me));

    si_me.sin_family = AF_INET;
    si_me.sin_port = htons(m_myPort);
    si_me.sin_addr.s_addr = htonl(INADDR_ANY);

    //bind socket to port
    if( bind(m_sock, (struct sockaddr*)&si_me, sizeof(si_me) ) == -1)
    {
        throw("bind");
    }

    // make it non-blocking
    fcntl(m_sock, F_SETFL, O_NONBLOCK); 

    // connect to server 
    bzero(&servaddr, sizeof(servaddr)); 
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY); 
    servaddr.sin_port = htons(m_dstPort); 
    servaddr.sin_family = AF_INET; 
      
    if(connect(m_sock, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) 
    { 
        throw("\n Error : Connect Failed \n"); 
    }     
    */
}

UDPInterface::~UDPInterface()
{
    /*
    close(m_sock);
    */
} 


bool UDPInterface::TxFrame(frame& pkt)
{
/*
    struct sockaddr_in si_other;
    socklen_t slen = sizeof(si_other);
    bool retVal = true;


    // send it
    memset(&si_other, '\0', sizeof(si_other));
    si_other.sin_family = AF_INET;
    si_other.sin_port = htons(m_dstPort);
    si_other.sin_addr.s_addr = htonl(INADDR_ANY);

    if (sendto(m_sock, pkt.getFrame(),
        sizeof(pkt.getMAXFrameSize()), 0,
        (struct sockaddr*) &si_other, slen) == -1)
    {
        retVal  = false;
    }

    return(retVal);
*/
    return(true);
}


bool UDPInterface::RxFrame(frame& pkt)
{
/*
    bool retVal     = true;
    int recv_len;
    fd_set readfds; 


    FD_ZERO(&readfds);
    FD_SET(m_sock, &readfds);

    { 
        if(-1 == (recv_len = recv(m_sock, pkt.getFrame(), pkt.getMAXFrameSize(), 0)))
        {
            retVal  = false;
        } else if(recv_len == pkt.getMAXFrameSize())
        {
            // TODO: handle this better
            //printf("\n\nWARNING WARNING received pkt may be too big\n\n");
            retVal  = true;
        }
    }

    return(retVal);
*/
    return(true);
}

