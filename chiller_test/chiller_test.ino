#include <RS232LSSeriesChiller.h>
#include <SoftwareSerial.h>



// chiller communication object this is library RS232LSSeriesChiller
RS232LSSeriesChiller chiller(10, 11, 9600, SERIAL_8N1);  // RX ping, TX ping, speed, config
                                                         // BUT! the config don't work, will
                                                         // default to 8N1

// buffer for chiller API calls
char buff[10];
char* pBuff = buff;


void setup()
{
    // no setup() content
}


void loop()
{
    while(true)
    {
        // turn off echo - send a 0
        if(chiller.SetCommandEcho('0'))
            Serial.println("SetCommandEcho success");
        else
            Serial.println("SetCommandEcho fail");
        Serial.println("");

        // turn off continuous output - send a 0
        if(chiller.OutputContinuousDataStream('0'))
            Serial.println("OutputContinuousDataStream success");
        else
            Serial.println("OutputContinuousDataStream fail");
        Serial.println("");

      
        // turn on the chiller
        if(chiller.SetOnOff('1'))
            Serial.println("SetOnOff success");
        else
            Serial.println("SetOnOff fail");
        Serial.println("");

      
        memset(buff, '\0', 10);
        strcpy(buff, "25.7");
        if(chiller.SetSetPoint(buff))
            Serial.println("SetSetPoint success");
        else
            Serial.println("SetSetPoint fail");
        Serial.println("");

      
        memset(buff, '\0', 10);
        if(chiller.ReadSetPointTemperature(&pBuff))
        {
            Serial.println("ReadSetPointTemperature success");
            Serial.print("read: ");
            Serial.println(buff);
        }
        else
            Serial.println("ReadSetPointTemperature fail");
        Serial.println("");

      
        memset(buff, '\0', 10);
        if(chiller.ReadTemperature(&pBuff))
        {
            Serial.println("ReadTemperature success");
            Serial.print("read: ");
            Serial.println(buff);
        }
        else
            Serial.println("ReadTemperature fail");
        Serial.println("");

        
        memset(buff, '\0', 10);
        if(chiller.ReadTemperatureUnits(&pBuff))
        {
            Serial.println("ReadTemperatureUnits success");
            Serial.print("read: ");
            Serial.println(buff);
        }
        else
            Serial.println("ReadTemperatureUnits fail");
        Serial.println("");

        
        memset(buff, '\0', 10);
        if(chiller.ReadStatus(&pBuff))
        {
            Serial.println("ReadStatus success");
            Serial.print("read: ");
            Serial.println(buff);
        }
        else
            Serial.println("ReadStatus fail");
        Serial.println("");

        
        
        memset(buff, '\0', 10);
        if(chiller.ReadCompressorDischargeTemperature(&pBuff))
        {
            Serial.println("ReadCompressorDischargeTemperature success");
            Serial.print("read: ");
            Serial.println(buff);
        }
        else
            Serial.println("ReadCompressorDischargeTemperature fail");
        Serial.println("");

        
        memset(buff, '\0', 10);
        if(chiller.ReadFaultStatus(&pBuff))
        {
            Serial.println("ReadFaultStatus success");
            Serial.print("read: ");
            Serial.println(buff);
        }
        else
            Serial.println("ReadFaultStatus fail");
        Serial.println("");

        
        memset(buff, '\0', 10);
        if(chiller.ReadEvaporatorInletTemperature(&pBuff))
        {
            Serial.println("ReadEvaporatorInletTemperature success");
            Serial.print("read: ");
            Serial.println(buff);
        }
        else
            Serial.println("ReadEvaporatorInletTemperature fail");
        Serial.println("");

        
        memset(buff, '\0', 10);
        if(chiller.ReadEvaporatorOutletTemperature(&pBuff))
        {
            Serial.println("ReadEvaporatorOutletTemperature success");
            Serial.print("read: ");
            Serial.println(buff);
        }
        else
            Serial.println("ReadEvaporatorOutletTemperature fail");
        Serial.println("");

        // sleep 5 seconds
        Serial.println("sleeping 5 seconds...");
        delay(5000);
    }
}

