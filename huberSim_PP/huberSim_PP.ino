#include <string.h>


const uint8_t MAX_BUFF_LENGTH               = 100; 
const uint8_t MAX_SLAVE_ID_LENGTH           = 2;
const uint8_t MAX_ONOFF_LENGTH              = 1;
const uint8_t MAX_SLAVE_NAME_LENGTH         = 30; 
const uint8_t MAX_LIMIT_LENGTH              = 4;
const uint8_t COMMAND_QUALIFIER_INDEX       = 4;
const uint8_t ADDRESS_INDEX                 = 2;
const uint8_t LENGTH_INDEX                  = 5;
const uint8_t DATA_START_INDEX              = 7;
const uint8_t CHKSUM_PLUS_ETX_LENGTH        = 3;    // check sum + '\r'
const uint8_t MIN_VERIFY_RESPONSE_LENGTH    = 11; 
const uint8_t MIN_LIMIT_RESPONSE_LENGTH     = 20; 
const uint8_t MIN_GENERAL_RESPONSE_LENGTH   = 22; 
const uint8_t TEMP_CTRL_MODE_INDEX          = 7;
const uint8_t MAX_TEMP_CTRL_MODE_LENGTH     = 1;
const uint8_t ALARM_STATUS_INDEX            = 8;
const uint8_t MAX_ALARM_STATUS_LENGTH       = 1;
const uint8_t SETPOINT_STATUS_INDEX         = 9;
const uint8_t MAX_SET_POINT_LENGTH          = 20;
const uint8_t INTERNAL_TEMP_INDEX           = 13; 
const uint8_t MAX_INTERNAL_TEMP_LENGTH      = 4;
const uint8_t EXTERNAL_TEMP_INDEX           = 17; 
const uint8_t MAX_EXTERNAL_TEMP_LENGTH      = 4;
const uint8_t BUFF_LENGTH                   = 20;


char Buff[BUFF_LENGTH + 1];
char setpoint[BUFF_LENGTH + 1];
char internalActualValue[BUFF_LENGTH + 1];
char tempControlMode[BUFF_LENGTH + 1];

uint8_t count = 0;


void setup()
{
    Serial.begin(9600);
    Serial2.begin(9600);
    strcpy(setpoint, "+02400\r\n");
    strcpy(tempControlMode, "00000\r\n");
}

void loop()
{
    Serial.println("");
    Serial.flush();
    Serial.println("waiting for data");
    Serial.flush();
    while(!Serial2.available()) {}


    Serial.println("");
    Serial.flush();
    Serial.println("reading data");
    Serial.flush();

    //
    // read a packet and reply with mostly pre-canned responses
    //
    count = 0;
    memset(Buff, '\0', BUFF_LENGTH + 1);
    while(Serial2.available())
    {
        Buff[count++] = Serial2.read();
    }

    Buff[count] = '\0';

    Serial.print("sim rx: -->");
    Serial.flush();
    Serial.print(Buff);
    Serial.flush();
    Serial.println("<--");
    Serial.flush();

    delay(150);  // simulate some processing time
    
    if(0 == (strncmp(Buff, "SP@ ", 4)) )
    {
        memset(reinterpret_cast<void*>(setpoint), '\0', BUFF_LENGTH + 1);
        strcpy(setpoint, &Buff[4]);

        memset(reinterpret_cast<void*>(Buff), '\0', BUFF_LENGTH + 1);
        strcpy(Buff, "SP ");
        strcpy(&Buff[3], setpoint);

    } else if(0 == strncmp(Buff, "CA@ ", 4))
    {
        memset(reinterpret_cast<void*>(tempControlMode), '\0', BUFF_LENGTH + 1);
        strcpy(tempControlMode, &Buff[4]);

        memset(reinterpret_cast<void*>(Buff), '\0', BUFF_LENGTH + 1);
        strcpy(Buff, "CA +");
        strcpy(&Buff[4], tempControlMode);


    } else if(0 == strncmp(Buff, "SP?", 3))
    {
        memset(reinterpret_cast<void*>(Buff), '\0', BUFF_LENGTH + 1);
        strcpy(Buff, "SP ");
        strcpy(&Buff[3], setpoint);

    } else if(0 == strncmp(Buff, "TI?", 3))
    {
        memset(reinterpret_cast<void*>(Buff), '\0', BUFF_LENGTH + 1);
        strcpy(Buff, "TI +02499\r\n");

    } else if(0 == strncmp(Buff, "CA?", 3))
    {
        memset(reinterpret_cast<void*>(Buff), '\0', BUFF_LENGTH + 1);
        strcpy(Buff, "CA +");
        strcpy(&Buff[4], tempControlMode);
    }

    Serial.print("sim tx: -->");
    Serial.flush();
    Serial.print(Buff);
    Serial.flush();
    Serial.println("<--");
    Serial.flush();

    Serial2.write(Buff);
}
