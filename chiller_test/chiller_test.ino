#include <RS232LSSeriesChiller.h>
#include <SoftwareSerial.h>



// chiller communication object this is library RS232LSSeriesChiller
// chiler is using Serial2 on the Arduino MAXI board, only the speed
// parameter is used
RS232LSSeriesChiller chiller(16, 17, 9600, SERIAL_8N1);  // RX pin, TX pin, speed, config
                                                         // BUT! the config don't work, will
                                                         // default to 8N1

// buffer for chiller API calls
char buff[10];
char* pBuff = buff;
char* setPointTemps[] = {"-10", "-05", "+15", "+20"};  // -20 to +60C is valid


void setup()
{
    // no setup() content
    Serial.begin(9600); 

/*

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
*/
}


void loop()
{
    Serial.println("sleeping 10 seconds");
    delay(10000);
    
    Serial.println("Turning the chiller on...");

    if(chiller.StartChiller())
    {
        Serial.println("chiller is started");
    } else
    {
        Serial.println("chiller may not be started ... ");
    }


    if(chiller.ChillerRunning())
    {
        Serial.println("chiller reports as running...");
    } else
    {
        Serial.println("chiller reports as not running...");
    }
/*
    
    // 
    // guessing the Set set point using celcius .. not clear in the manual
    // 
    for(uint8_t i = 0; i < 4; i++)
    {
        memset(buff, '\0', 10);
        strcpy(buff, setPointTemps[i]);
        Serial.print("SetSetPoint to ");
        Serial.print(setPointTemps[i]);
        if(chiller.SetSetPoint(buff))
            Serial.println(" success");
        else
            Serial.println(" fail");
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

        delay(2000);
    }

      
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


    // turn off the chiller
    if( (chiller.StopChiller()) )
    {
        Serial.println("the chiller should be off");
    } else
    {
        Serial.println("the chiller may not be off");
    }
    
    Serial.println("sleeping 10 seconds...");
    delay(10000);
    */
}
