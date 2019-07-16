#include <SoftwareSerial.h>    // need to include this due to Arduino IDE strangeness..
#include <meerstetterRS485.h>

meerstetterRS485 ms(10, 11);  // the meerstetter class

float   temp=24.5;
float   newTemp;
uint8_t Address  = 2;

        
void setup()
{
  Serial.begin(9600);
}


void loop() {

    Serial.println("looping...");
    for(uint8_t i = 0; i < 3; i++)
    {
      newTemp = temp + i;
      setTECSetPoint(Address, newTemp);
      delay(1000);
      getTECSetPoint(Address);
      delay(1000);
    }
    
    getStatus(Address);
    
    delay(1000);
}


// MeCom_TEC_Tem_TargetObjectTemp
void setTECSetPoint(uint8_t Address, float temperature)
{
    int32_t ParId   = 3000;
    int32_t Inst    = 1;
    MeParFloatFields Fields;

    if(ms.MeCom_ParValuef(Address, ParId, Inst, &Fields, MeGetLimits))
    {    
        Fields.Value = temperature;  
        if(ms.MeCom_ParValuef(Address, ParId, Inst, &Fields, MeSet))
        {
            Serial.println("setTECSetPoint success");
            printFloatFields(Fields);
        } else
        {
            Serial.println("setTECSetPoint fail");
            printFloatFields(Fields);
        }
    }    
}


//MeCom_TEC_Mon_TargetObjectTemperature
void getTECSetPoint(uint8_t Address)
{
    int32_t ParId   = 1010;
    int32_t Inst    = 1;
    MeParFloatFields Fields;

    if(ms.MeCom_ParValuef(Address, ParId, Inst, &Fields, MeGetLimits))
    {    
        if(ms.MeCom_ParValuef(Address, ParId, Inst, &Fields, MeGet))
        {
            Serial.println("getTECSetPoint success");
            printFloatFields(Fields);
        } else
        {
            Serial.println("getTECSetPoint fail");
            printFloatFields(Fields);
        }
    }    
}


//MeCom_COM_DeviceStatus
void getStatus(uint8_t Address)
{
    int32_t ParId   = 104;
    int32_t Inst    = 1;
    MeParFloatFields Fields;

    if(ms.MeCom_ParValuef(Address, ParId, Inst, &Fields, MeGetLimits))
    {    
        if(ms.MeCom_ParValuef(Address, ParId, Inst, &Fields, MeGet))
        {
            Serial.println("getStatus success");
            printFloatFields(Fields);
        } else
        {
            Serial.println("getStatus fail");
            printFloatFields(Fields);
        }
    }    
}


void printFloatFields(MeParFloatFields fields)
{
  Serial.print("Value: ");
  Serial.println(fields.Value, 3);
  Serial.print("Min:");
  Serial.println(fields.Min, 3);
  Serial.print("Max: ");
  Serial.println(fields.Max, 3);
  Serial.println("");
}

void printLongFields(MeParLongFields fields)
{
  Serial.print("Value: ");
  Serial.println(fields.Value, DEC);
  Serial.print("Min: ");
  Serial.println(fields.Min, DEC);
  Serial.print("Max: ");
  Serial.println(fields.Max, DEC);
  Serial.println("");
}

