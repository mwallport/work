#include "eventlog.h"


//
// eventlog implementation
//
static uint8_t elogindex = 0; // this is the current event log index - points to
                              // last entry made

static elogentry  eventlog[MAX_ELOG_ENTRY];


// local functions not visible outside the library
void clrEventLogEntry(elogentry* pevent);

//
// must be called 1st by using application to initialize the eventlog
// clear all events in the eventlog
//
void clrEventLog(void)
{

  for(uint8_t i = 0; i < MAX_ELOG_ENTRY; i++)
  {
    clrEventLogEntry(&eventlog[i]);
  }
}


void clrEventLogEntry(elogentry* pevent)
{

  pevent->ts.sec  = 0;
  pevent->ts.min  = 0;
  pevent->ts.hour = 0;
  pevent->ts.mday = 0;
  pevent->ts.mon  = 0;
  pevent->ts.year = 0;
  pevent->ts.wday = 0;
  pevent->ts.fill = 0;
    
  pevent->id  = 0;

  for(uint8_t j = 0; j < MAX_DATA; j++)
    pevent->data[j] = 0;
}



// APIs to work on the event log
uint8_t addEventLogEntry(elogentry* pNewEntry) // returns 0 if no wrap around, 1 if wrap around
{
  uint8_t retVal    = 0;
  uint8_t newindex  = 0;


  //
  // set index to the next location
  //
  newindex  = (elogindex + 1) % MAX_ELOG_ENTRY;

  if( ( newindex < elogindex) )
    retVal  = 1;  // wrapped, let caller know


  //
  // set the elogindex to the new index
  //
  elogindex = newindex;


  //
  // clr the new entry index
  //
  clrEventLogEntry(&eventlog[elogindex]);

  //
  // set the new entry
  //
  eventlog[elogindex].ts.sec  = pNewEntry->ts.sec;
  eventlog[elogindex].ts.min  = pNewEntry->ts.min;
  eventlog[elogindex].ts.hour = pNewEntry->ts.hour;
  eventlog[elogindex].ts.mday = pNewEntry->ts.mday;
  eventlog[elogindex].ts.mon  = pNewEntry->ts.mon;
  eventlog[elogindex].ts.year = pNewEntry->ts.year;
  eventlog[elogindex].ts.wday = pNewEntry->ts.wday;
  eventlog[elogindex].ts.fill = pNewEntry->ts.fill;

  eventlog[elogindex].id  = pNewEntry->id;

  for(uint8_t j = 0; j < MAX_DATA; j++)
    eventlog[elogindex].data[j] = pNewEntry->data[j];
  
  return(retVal);
}


elogentry getEventLogEntry(uint8_t index) // return a pointer to the elogentry at given index
{

  index = index % MAX_ELOG_ENTRY; // just for safety-sake !

 
  return(eventlog[index]);
}

const elogentry* getEventLog(void)
{

  return(eventlog);
}

