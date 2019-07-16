//file interface.cpp

#include "interface.h"

interface::interface(const char* name)
{ 
  memset(m_name, '\0', MAX_NAME_LENGTH + 1);
  strncpy(m_name, name, strlen(name) < MAX_NAME_LENGTH ? strlen(name) : MAX_NAME_LENGTH);
}

bool interface::HandleFrame(frame&)
{
    // to be implemented
    return(true);
}


