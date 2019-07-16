#include <unistd.h>
#include "RingBuffer.h"
#include "ctrlInterface.h"
#include "tecInterface.h"
#include "huberInterface.h"

using namespace std;

#define MAX_FRAMES_POOL_SIZE 3

// global Ctrl, TEC, and Huber interfaces
interface*      pCtrl;      // controlling PC software
tecInterface*   pTEC;       // interface to TECs
huberInterface* pHuber;     // interface to Huber
RingBuffer<frame*>*  pFrames;    // RingBuffer of frames


// main functions
bool initialize(ctrlInterface*, tecInterface*, huberInterface*, RingBuffer<frame*>**);
bool ctrlIfaceInput();
bool tecIfaceInput();
bool huberIfaceInput();
bool pushPoolFrame(frame* pFrame);
frame* popPoolFrame();


// read a frame and send to the appropriate interface
// TODO:return real  retVal
bool ctrlIfaceInput()
{
  frame*  pFrame  = popPoolFrame();


  Serial.println("in ctrlIfaceInput...");
  
  if(0 == pFrame)
  {
    //printf("ctrlIfaceInput failed to get a frame\n");
    return(false);
  }

  //read a frame
  if(false == pCtrl->RxFrame(*pFrame))
  {
    // TODO: do something ?
  }

  // return the frame
  pushPoolFrame(pFrame);

  return(true);
}


// TODO:return real  retVal
bool tecIfaceInput()
{
  frame*  pFrame  = popPoolFrame();


  Serial.println("in tecIfaceInput...");

  if(0 == pFrame)
  {
    //printf("tecIfaceInput failed to get a frame\n");
    return(false);
  }

  //read a frame
  if(false == pTEC->RxFrame(*pFrame))
  {
    // TODO: do something ?
  }

  // return the frame
  pushPoolFrame(pFrame);

  return(true);
}


// TODO:return real  retVal
bool huberIfaceInput()
{
  frame*  pFrame  = popPoolFrame();


  Serial.println("in huberIfaceInput...");

  if(0 == pFrame)
  {
    //printf("huberIfaceInput failed to get a frame\n");
    return(false);
  }

  //read a frame
  if(false == pHuber->RxFrame(*pFrame))
  {
    // TODO: do something ?
  }

  // return the frame
  pushPoolFrame(pFrame);

  return(true);
}


// make the interfaces, set the global pointers to them
// TODO: return false if anything fails!
bool initialize(ctrlInterface* pctrl, tecInterface* ptec,
          huberInterface* phuber, RingBuffer<frame*>** ppPool)
{
  RingBuffer<frame*>*   pPool   = *ppPool;
  frame*          pFrame;


  // assign the global pointers
  pCtrl   = pctrl;
  pTEC    = ptec;
  pHuber  = phuber;

  // set the pCtrl in the tec and huber interfaces
  pTEC->setCtrlInterface(reinterpret_cast<interface*>(pctrl));
  pHuber->setCtrlInterface(reinterpret_cast<interface*>(pctrl));

  //setup the frame pool
  pPool  = new RingBuffer<frame*>();
  for(int i = 0; i < MAX_FRAMES_POOL_SIZE; i++)
  {
    // TODO: get rid of these frames on shutdown, no need, arduino will reboot
    pFrame  = 0;
    pFrame  = new frame(i);
    pPool->push_back(pFrame);
    if(0 == pFrame)
    {
      //printf("failure to make frame\n");
      //throw("failure to make frame\n");
    }
  }

  *ppPool = pPool;
  return(true);
}


frame* popPoolFrame()
{
  frame* pFrame;


  if(!pFrames->empty())
  {
    pFrame = pFrames->front();
    pFrames->pop();
    return(pFrame);
  } 
  else
  {
    return(0);
  }
}


bool pushPoolFrame(frame* pFrame)
{
  pFrames->push_back(pFrame);
  return(true);
}


void setup()
{
  Serial.begin(9600);

  // startup section of arduino
  tecInterface    tec;
  huberInterface  huber;
  ctrlInterface   ctrl(reinterpret_cast<interface&>(huber),
  reinterpret_cast<interface&>(tec));

  // assign globals and create the Ctrl interface
  initialize(&ctrl, &tec, &huber, &pFrames);
}

void loop()
{
  //Serial.println("looping...");
  ctrlIfaceInput();
  tecIfaceInput();
  huberIfaceInput();
  delay(1000);
}



