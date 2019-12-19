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
const uint8_t MAX_SET_POINT_LENGTH          = 4;
const uint8_t INTERNAL_TEMP_INDEX           = 13; 
const uint8_t MAX_INTERNAL_TEMP_LENGTH      = 4;
const uint8_t EXTERNAL_TEMP_INDEX           = 17; 
const uint8_t MAX_EXTERNAL_TEMP_LENGTH      = 4;


char Buff[100];
char v_response[100];
char l_response[100];
char g_response[100];
char setpoint[MAX_SET_POINT_LENGTH + 1];
char OnOff[MAX_ONOFF_LENGTH + 1];
uint8_t count = 0;
bool newSetPoint    = false;
bool newOnOff       = false;


void setup()
{
    Serial.begin(9600);
    Serial2.begin(9600);

    // set up the return strings
    count = strlen("[S01V14Huber ControlC1");
    strcpy(v_response, "[S01V14Huber ControlC1");
    v_response[count++] = '\r';
    v_response[count] = '\0';

    count = strlen("[S01L17F4484E20F4484E2045");
    strcpy(l_response, "[S01L17F4484E20F4484E2045");
    l_response[count++] = '\r';
    l_response[count] = '\0';

    count = strlen("[S01G15O0FE7009A4C504E7");
    strcpy(g_response, "[S01G15O0FE7009A4C504E7");
    g_response[count++] = '\r';
    g_response[count] = '\0';
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
    memset(Buff, '\0', 100);
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
    
    switch(Buff[4])
    {
        case 'V':
        {
            Serial.println("responding to Verify command");
            Serial.flush();
            Serial.print("sending: ");
            Serial.flush();
            Serial.println(v_response);
            Serial2.write(v_response);
            Serial2.flush();
            // TODO: change the slave ID in return - see if library picks it up
            break;
        }

        case 'L':
        {
            Serial.println("responding to Limit command");
            Serial.flush();
            Serial.print("sending: ");
            Serial.flush();
            Serial.println(l_response);
            Serial2.write(l_response);
            Serial2.flush();
            break;
        }

        case 'G':
        {
            newSetPoint    = false;
            newOnOff       = false;

            Serial.println("responding to General command");
            Serial.flush();

            // pick up a new set-point if there
            if( ('*' != Buff[SETPOINT_STATUS_INDEX]) )
            {
                // this is a set point temp, reply properly
                memset(setpoint, '\0', MAX_SET_POINT_LENGTH + 1);
                strncpy(setpoint, &Buff[SETPOINT_STATUS_INDEX], MAX_SET_POINT_LENGTH);
                Serial.print("picked up 0x");
                Serial.print(setpoint);
                Serial.println(" as new setpoint value");
                Serial.flush();
                newSetPoint = true;
            }

            // pick up a new temperature mode
            if( ('*' != Buff[TEMP_CTRL_MODE_INDEX]) )
            {
                memset(OnOff, '\0', MAX_ONOFF_LENGTH + 1);
                strncpy(OnOff, &Buff[TEMP_CTRL_MODE_INDEX], MAX_ONOFF_LENGTH);
                Serial.print("picked up 0x");
                Serial.print(OnOff);
                Serial.println(" as new temperature mode value");
                Serial.flush();
                newOnOff = true;
            }

            //
            // build and write the response
            //
            // copy over the generic G response into Buff
            //
            memset(reinterpret_cast<void*>(Buff), '\0', 100);
            strcpy(Buff, g_response);

            //
            // set the setpoint and the OnOff values
            //
            if( (newSetPoint) )
                strncpy(&Buff[SETPOINT_STATUS_INDEX], setpoint, MAX_SET_POINT_LENGTH);

            if( (newOnOff) )
                strncpy(&Buff[TEMP_CTRL_MODE_INDEX], OnOff, MAX_ONOFF_LENGTH);

            //
            // set the length and the checksum in the G response
            //
            setLengthAndCheckSum();

            Serial.print("sending: ");
            Serial.println(Buff);
            Serial.flush();

            Serial2.write(Buff);
            Serial2.flush();

            //
            // update the g_response
            //
            memset(reinterpret_cast<void*>(g_response), '\0', 100);
            strcpy(g_response, Buff);

            Serial.print("huberSim updated g_response w/ : ");
            Serial.flush();
            Serial.println(g_response);
            Serial.flush();

            break;
        }

        default:
        {
            Serial.println("got unexpected command");
            Serial.flush();
            break;
        }
    }
}



void setLengthAndCheckSum()
{
    char        intStr[5]   = {0, 0, 0, 0, 0};
    int16_t     chkSum      = 0;
    int16_t     chkSumIndex;

    //
    // convert integer to char[], chkSumIndex is BuffLen - chkSum len - '\r'
    //
    chkSumIndex = (strlen(Buff) > 3 ? strlen(Buff) - 3 : 0);
    sprintf(intStr, "%02X", chkSumIndex);


    //
    // set the length  - always bytes 5 and 6 - zero based count
    //
    Buff[LENGTH_INDEX]      = intStr[0];
    Buff[LENGTH_INDEX + 1]  = intStr[1];


    //
    // now calculate the checkSum including the length bytes just put in
    //
    for(int i = 0; i < chkSumIndex; i++)
        chkSum += static_cast<uint8_t>(Buff[i]);

    //
    // convert chkSum to a string
    //
    memset(reinterpret_cast<void*>(intStr), '\0', 5);
    sprintf(intStr, "%04X", chkSum);  // a hex number you numb-skull !

    //
    // depending on the magnitude of the cheksum, take the
    // lower byte
    //
    if( (chkSum < 0x0100) )
    {
        Buff[chkSumIndex++] = intStr[0];
        Buff[chkSumIndex++] = intStr[1];
    } else
    {
        Buff[chkSumIndex++] = intStr[2];
        Buff[chkSumIndex++] = intStr[3];
    }
}
