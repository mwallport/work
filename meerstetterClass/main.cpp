#include <stdio.h>
#include "meerstetterRS485.h"

void printFloatFields(MeParFloatFields);
void printLongFields(MeParLongFields);
void SetTargetObjectTemp(meerstetterRS485&, uint8_t);

/*==============================================================================*/
/** @file       main.c
    @brief      This file is not part of the MeComAPI. Just simple Demo Application.
    @author     Meerstetter Engineering GmbH: Marc Luethi / Thomas Braun
    @version    v0.42

    This is the TOP file of the MeComAPI Demo Application.
    The Demo Application is being used to test the MeComAPI and
    should show the user how the MeComAPI shoud be used.

    The Demo Application has been compiled an the following Product:
    "Microsoft Visual Studio 2010"

    The Demo Application might uses some C++ functions.
    The MeComAPI is written in C (ANSI C99).

    Please refer to the Document 5170-MeComAPI for more information.

*/




//void loop()   /****** LOOP: RUNS CONSTANTLY ******/
int main(int argc, char** argv)
{
    int32_t Address = 2;    // this is 0 - 255, default is 0
    int8_t Buf[25];
    meerstetterRS485 ms(10, 11); // instantiate a meersetter protocol over RS485 bus on pins 10 and 11


     if(ms.MeCom_ResetDevice(Address))
     {
         //Serial.println("Device Reset OK.\n");
     }

     //delay(5000);
     

    while(1)
    {
      ////Serial.println("looping ...");

      if(ms.MeCom_GetIdentString(Address, Buf)) 
      {
         ////Serial.println("Adress 2 Ident String: ");
         for(int x = 0; x < 25 && Buf[x]; x++)
         {
           ////Serial.print(Buf[x]);
         }
         
         ////Serial.println("");
      }          

      SetTargetObjectTemp(ms, Address);

      // delay 10 seconds, let devie come out of reset .. ?
//      delay(5000);
/*      
      TestAllCommonGetFunctions(Address);
      TestAllTECGetFunctions(Address);
*/
      //delay(3000);
       //delay(5000);
    }
}


void setup()   /****** SETUP: RUNS ONCE ******/
{
    // Start the built-in serial port, probably to //Serial.Monitor
  //Serial.begin(9600);

  //pinMode(Pin13LED, OUTPUT);

  // Start the software serial port, to another device
  //RS485Serial.begin(9600);   // set the data rate 
}


/*==============================================================================*/
/** @brief      Test function for all Common Get Functions
 *
*/
static void TestAllCommonGetFunctions(meerstetterRS485& ms, uint8_t Address)
{
    MeParLongFields  lFields;


    if(ms.MeCom_COM_DeviceType(Address, &lFields, MeGet)) 
    {
        //Serial.println("MeCom_COM_DeviceType");
        printLongFields(lFields);
    }

    if(ms.MeCom_COM_HardwareVersion(Address, &lFields, MeGet)) 
    {
        //Serial.println("MeCom_COM_HardwareVersion");
        printLongFields(lFields);
    }

    if(ms.MeCom_COM_SerialNumber(Address, &lFields, MeGet)) 
    {
        //Serial.println("MeCom_COM_Serial.umber");
        printLongFields(lFields);
    }

    if(ms.MeCom_COM_FirmwareVersion(Address, &lFields, MeGet)) 
    {
        //Serial.println("MeCom_COM_FirmwareVersion");
        printLongFields(lFields);
    }

    if(ms.MeCom_COM_DeviceStatus(Address, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_COM_DeviceStatus");
        printLongFields(lFields);
    }

    if(ms.MeCom_COM_ErrorNumber(Address, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_COM_ErrorNumber");
        printLongFields(lFields);
    }

    if(ms.MeCom_COM_ErrorInstance(Address, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_COM_ErrorInstance");
        printLongFields(lFields);
    }

    if(ms.MeCom_COM_ErrorParameter(Address, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_COM_ErrorParameter");
        printLongFields(lFields);
    }
    if(ms.MeCom_COM_ParameterSystemFlashSaveOff(Address, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_COM_ParameterSystemFlashSaveOff");
        printLongFields(lFields);
    }
    if(ms.MeCom_COM_ParameterSystemFlashStatus(Address, &lFields, MeGet)) 
    {
        //Serial.println("MeCom_COM_ParameterSystemFlashStatus");
        printLongFields(lFields);
    }
}


/*==============================================================================*/
/** @brief      Test function for all TEC Get Functions
 *
*/
static void TestAllTECGetFunctions(meerstetterRS485& ms, uint8_t Address)
{
    MeParLongFields  lFields;
    MeParFloatFields fFields;


    //Tab Monitor
    if(ms.MeCom_TEC_Mon_ObjectTemperature(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Mon_ObjectTemperature");
        printFloatFields(fFields);
    }
    
    if(ms.MeCom_TEC_Mon_SinkTemperature(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Mon_SinkTemperature");
        printFloatFields(fFields);
    }

    if(ms.MeCom_TEC_Mon_TargetObjectTemperature(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Mon_TargetObjectTemperature");
        printFloatFields(fFields);
    }

    if(ms.MeCom_TEC_Mon_RampNominalObjectTemperature(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Mon_RampNominalObjectTemperature");
        printFloatFields(fFields);
    }

    if(ms.MeCom_TEC_Mon_ThermalPowerModelCurrent(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Mon_ThermalPowerModelCurrent");
        printFloatFields(fFields);
    }

    if(ms.MeCom_TEC_Mon_ActualOutputCurrent(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Mon_ActualOutputCurrent");
        printFloatFields(fFields);
    }

    if(ms.MeCom_TEC_Mon_ActualOutputVoltage(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Mon_ActualOutputVoltage");
        printFloatFields(fFields);
    }

    if(ms.MeCom_TEC_Mon_PIDLowerLimitation(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Mon_PIDLowerLimitation");
        printFloatFields(fFields);
    }

    if(ms.MeCom_TEC_Mon_PIDUpperLimitation(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Mon_PIDUpperLimitation");
        printFloatFields(fFields);
    }

    if(ms.MeCom_TEC_Mon_PIDControlVariable(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Mon_PIDControlVariable");
        printFloatFields(fFields);
    }

    if(ms.MeCom_TEC_Mon_ObjectSensorRawADCValue(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Mon_ObjectSensorRawADCValue");
        printFloatFields(fFields);
    }

    if(ms.MeCom_TEC_Mon_SinkSensorRawADCValue(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Mon_SinkSensorRawADCValue");
        printFloatFields(fFields);
    }

    if(ms.MeCom_TEC_Mon_ObjectSensorResistance(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Mon_ObjectSensorResistance");
        printFloatFields(fFields);
    }

    if(ms.MeCom_TEC_Mon_SinkSensorResitance(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Mon_SinkSensorResitance");
        printFloatFields(fFields);
    }
        
    if(ms.MeCom_TEC_Mon_SinkSensorTemperature(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Mon_SinkSensorTemperature");
        printFloatFields(fFields);
    }
    
    if(ms.MeCom_TEC_Mon_FirmwareVersion(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Mon_FirmwareVersion");
        printFloatFields(fFields);
    }

    if(ms.MeCom_TEC_Mon_FirmwareBuildNumber(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Mon_FirmwareBuildNumber");
        printFloatFields(fFields);
    }

    if(ms.MeCom_TEC_Mon_HardwareVersion(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Mon_HardwareVersion");
        printFloatFields(fFields);
    }

    if(ms.MeCom_TEC_Mon_SerialNumber(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Mon_//Serial.umber");
        printFloatFields(fFields);
    }

    if(ms.MeCom_TEC_Mon_DriverInputVoltage(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Mon_DriverInputVoltage");
        printFloatFields(fFields);
    }

    if(ms.MeCom_TEC_Mon_MedVInternalSupply(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Mon_MedVInternalSupply");
        printFloatFields(fFields);
    }

    if(ms.MeCom_TEC_Mon_3_3VInternalSupply(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Mon_3_3VInternalSupply");
        printFloatFields(fFields);
    }

    if(ms.MeCom_TEC_Mon_BasePlateTemperature(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Mon_BasePlateTemperature");
        printFloatFields(fFields);
    }

    if(ms.MeCom_TEC_Mon_ErrorNumber(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Mon_ErrorNumber");
        printFloatFields(fFields);
    }

    if(ms.MeCom_TEC_Mon_ErrorInstance(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Mon_ErrorInstance");
        printFloatFields(fFields);
    }

    if(ms.MeCom_TEC_Mon_ErrorParameter(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Mon_ErrorParameter");
        printFloatFields(fFields);
    }

    if(ms.MeCom_TEC_Mon_ParallelActualOutputCurrent(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Mon_ParallelActualOutputCurrent");
        printFloatFields(fFields);
    }

    if(ms.MeCom_TEC_Mon_DriverStatus(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Mon_DriverStatus");
        printLongFields(lFields);
    }

    if(ms.MeCom_TEC_Mon_FanRelativeCoolingPower(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Mon_FanRelativeCoolingPower");
        printFloatFields(fFields);
    }
    
    if(ms.MeCom_TEC_Mon_FanNominalFanSpeed(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Mon_FanNominalFanSpeed");
        printFloatFields(fFields);
    }
    
    if(ms.MeCom_TEC_Mon_FanActualFanSpeed(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Mon_FanActualFanSpeed");
        printFloatFields(fFields);
    }
    
    if(ms.MeCom_TEC_Mon_FanActualPwmLevel(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Mon_FanActualPwmLevel");
        printFloatFields(fFields);
    }

    if(ms.MeCom_TEC_Mon_BasePlateTemperature(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Mon_BasePlateTemperature");
        printFloatFields(fFields);
    }

    if(ms.MeCom_TEC_Mon_TemperatureIsStable(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Mon_TemperatureIsStable");
        printFloatFields(fFields);
    }
    
    //Tab Operation
    if(ms.MeCom_TEC_Ope_OutputStageInputSelection(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Ope_OutputStageInputSelection");
        printFloatFields(fFields);
    }

    if(ms.MeCom_TEC_Ope_OutputStageEnable(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Ope_OutputStageEnable");
        printLongFields(lFields);
    }

    if(ms.MeCom_TEC_Ope_SetStaticCurrent(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Ope_SetStaticCurrent");
        printFloatFields(fFields);
    }

    if(ms.MeCom_TEC_Ope_SetStaticVoltage(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Ope_SetStaticVoltage");
        printFloatFields(fFields);
    }

    if(ms.MeCom_TEC_Ope_CurrentLimitation(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Ope_CurrentLimitation");
        printFloatFields(fFields);
    }

    if(ms.MeCom_TEC_Ope_VoltageLimitation(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Ope_VoltageLimitation");
        printFloatFields(fFields);
    }

    if(ms.MeCom_TEC_Ope_CurrentErrorThreshold(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Ope_CurrentErrorThreshold");
        printFloatFields(fFields);
    }

    if(ms.MeCom_TEC_Ope_VoltageErrorThreshold(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Ope_VoltageErrorThreshold");
        printFloatFields(fFields);
    }

    if(ms.MeCom_TEC_Ope_GeneralOperatingMode(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Ope_GeneralOperatingMode");
        printLongFields(lFields);
    }

    if(ms.MeCom_TEC_Ope_DeviceAddress(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Ope_DeviceAddress");
        printLongFields(lFields);
    }

    if(ms.MeCom_TEC_Ope_RS485CH1BaudRate(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Ope_RS485CH1BaudRate");
        printLongFields(lFields);
    }

    if(ms.MeCom_TEC_Ope_RS485CH1ResponseDelay(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Ope_RS485CH1ResponseDelay");
        printLongFields(lFields);
    }

    if(ms.MeCom_TEC_Ope_ComWatchDogTimeout(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Ope_ComWatchDogTimeout");
        printFloatFields(fFields);
    }

    //Tab Temperature Control
    
    if(ms.MeCom_TEC_Tem_TargetObjectTemp(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Tem_TargetObjectTemp");
        printFloatFields(fFields);
    }

    if(ms.MeCom_TEC_Tem_CoarseTempRamp(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Tem_CoarseTempRamp");
        printFloatFields(fFields);
    }

    if(ms.MeCom_TEC_Tem_ProximityWidth(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Tem_ProximityWidth");
        printFloatFields(fFields);
    }

    if(ms.MeCom_TEC_Tem_Kp(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Tem_Kp");
        printFloatFields(fFields);
    }

    if(ms.MeCom_TEC_Tem_Ti(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Tem_Ti");
        printFloatFields(fFields);
    }

    if(ms.MeCom_TEC_Tem_Td(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Tem_Td");
        printFloatFields(fFields);
    }

    if(ms.MeCom_TEC_Tem_DPartDampPT1(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Tem_DPartDampPT1");
        printFloatFields(fFields);
    }

    if(ms.MeCom_TEC_Tem_ModelizationMode(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Tem_ModelizationMode");
        printLongFields(lFields);
    }

    if(ms.MeCom_TEC_Tem_PeltierMaxCurrent(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Tem_PeltierMaxCurrent");
        printFloatFields(fFields);
    }

    if(ms.MeCom_TEC_Tem_PeltierMaxVoltage(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Tem_PeltierMaxVoltage");
        printFloatFields(fFields);
    }

    if(ms.MeCom_TEC_Tem_PeltierCoolingCapacity(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Tem_PeltierCoolingCapacity");
        printFloatFields(fFields);
    }

    if(ms.MeCom_TEC_Tem_PeltierDeltaTemperature(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Tem_PeltierDeltaTemperature");
        printFloatFields(fFields);
    }

    if(ms.MeCom_TEC_Tem_PeltierPositiveCurrentIs(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Tem_PeltierPositiveCurrentIs");
        printLongFields(lFields);
    }

    if(ms.MeCom_TEC_Tem_ResistorResistance(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Tem_ResistorResistance");
    }

    if(ms.MeCom_TEC_Tem_ResistorMaxCurrent(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Tem_ResistorMaxCurrent");
    }
    
    //Tab Object Temperature
    if(ms.MeCom_TEC_Obj_TemperatureOffset(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Obj_TemperatureOffset");
    }

    if(ms.MeCom_TEC_Obj_TemperatureGain(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Obj_TemperatureGain");
    }

    if(ms.MeCom_TEC_Obj_LowerErrorThreshold(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Obj_LowerErrorThreshold");
    }

    if(ms.MeCom_TEC_Obj_UpperErrorThreshold(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Obj_UpperErrorThreshold");
    }

    if(ms.MeCom_TEC_Obj_MaxTempChange(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Obj_MaxTempChange");
    }

    if(ms.MeCom_TEC_Obj_NTCLowerPointTemperature(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Obj_NTCLowerPointTemperature");
    }

    if(ms.MeCom_TEC_Obj_NTCLowerPointResistance(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Obj_NTCLowerPointResistance");
    }

    if(ms.MeCom_TEC_Obj_NTCMiddlePointTemperature(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Obj_NTCMiddlePointTemperature");
    }

    if(ms.MeCom_TEC_Obj_NTCMiddlePointResistance(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Obj_NTCMiddlePointResistance");
    }

    if(ms.MeCom_TEC_Obj_NTCUpperPointTemperature(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Obj_NTCUpperPointTemperature");
    }

    if(ms.MeCom_TEC_Obj_NTCUpperPointResistance(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Obj_NTCUpperPointResistance");
    }

    if(ms.MeCom_TEC_Obj_StabilityTemperatureWindow(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Obj_StabilityTemperatureWindow");
    }

    if(ms.MeCom_TEC_Obj_StabilityMinTimeInWindow(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Obj_StabilityMinTimeInWindow");
    }

    if(ms.MeCom_TEC_Obj_StabilityMaxStabiTime(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Obj_StabilityMaxStabiTime");
    }
    
    if(ms.MeCom_TEC_Obj_MeasLowestResistance(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Obj_MeasLowestResistance");
    }

    if(ms.MeCom_TEC_Obj_MeasHighestResistance(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Obj_MeasHighestResistance");
    }

    if(ms.MeCom_TEC_Obj_MeasTempAtLowestResistance(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Obj_MeasTempAtLowestResistance");
    }

    if(ms.MeCom_TEC_Obj_MeasTempAtHighestResistance(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Obj_MeasTempAtHighestResistance");
    }
    
    //Tab Sink Temperature
    
    if(ms.MeCom_TEC_Sin_TemperatureOffset(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Sin_TemperatureOffset");
    }

    if(ms.MeCom_TEC_Sin_TemperatureGain(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Sin_TemperatureGain");
    }

    if(ms.MeCom_TEC_Sin_LowerErrorThreshold(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Sin_LowerErrorThreshold");
    }

    if(ms.MeCom_TEC_Sin_UpperErrorThreshold(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Sin_UpperErrorThreshold");
    }

    if(ms.MeCom_TEC_Sin_MaxTempChange(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Sin_MaxTempChange");
    }

    if(ms.MeCom_TEC_Sin_NTCLowerPointTemperature(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Sin_NTCLowerPointTemperature");
    }

    if(ms.MeCom_TEC_Sin_NTCLowerPointResistance(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Sin_NTCLowerPointResistance");
    }

    if(ms.MeCom_TEC_Sin_NTCMiddlePointTemperature(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Sin_NTCMiddlePointTemperature");
    }

    if(ms.MeCom_TEC_Sin_NTCMiddlePointResistance(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Sin_NTCMiddlePointResistance");
    }

    if(ms.MeCom_TEC_Sin_NTCUpperPointTemperature(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Sin_NTCUpperPointTemperature");
    }

    if(ms.MeCom_TEC_Sin_NTCUpperPointResistance(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Sin_NTCUpperPointResistance");
    }

    if(ms.MeCom_TEC_Sin_SinkTemperatureSelection(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Sin_SinkTemperatureSelection");
    }

    if(ms.MeCom_TEC_Sin_FixedTemperature(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Sin_FixedTemperature");
    }

    if(ms.MeCom_TEC_Sin_MeasLowestResistance(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Sin_MeasLowestResistance");
    }

    if(ms.MeCom_TEC_Sin_MeasHighestResistance(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Sin_MeasHighestResistance");
    }

    if(ms.MeCom_TEC_Sin_MeasTempAtLowestResistance(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Sin_MeasTempAtLowestResistance");
    }

    if(ms.MeCom_TEC_Sin_MeasTempAtHighestResistance(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Sin_MeasTempAtHighestResistance");
    }

    //Tab Expert");
    if(ms.MeCom_TEC_Exp_ObjMeasPGAGain(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Exp_ObjMeasPGAGain");
    }

    if(ms.MeCom_TEC_Exp_ObjMeasCurrentSource(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Exp_ObjMeasCurrentSource");
    }

    if(ms.MeCom_TEC_Exp_ObjMeasADCRs(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Exp_ObjMeasADCRs");
    }

    if(ms.MeCom_TEC_Exp_ObjMeasADCCalibOffset(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Exp_ObjMeasADCCalibOffset");
    }

    if(ms.MeCom_TEC_Exp_ObjMeasADCCalibGain(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Exp_ObjMeasADCCalibGain");
    }

    if(ms.MeCom_TEC_Exp_ObjMeasSensorTypeSelection(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Exp_ObjMeasSensorTypeSelection");
    }

    if(ms.MeCom_TEC_Exp_SinMeasADCRv(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Exp_SinMeasADCRv");
    }

    if(ms.MeCom_TEC_Exp_SinMeasADCVps(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Exp_SinMeasADCVps");
    }

    if(ms.MeCom_TEC_Exp_SinMeasADCCalibOffset(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Exp_SinMeasADCCalibOffset");
    }

    if(ms.MeCom_TEC_Exp_SinMeasADCCalibGain(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Exp_SinMeasADCCalibGain");
    }

    //Tab Expert");
    if(ms.MeCom_TEC_Exp_DisplayType(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Exp_DisplayType");
    }
    if(ms.MeCom_TEC_Exp_DisplayLineDefText(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Exp_DisplayLineDefText");
    }
    if(ms.MeCom_TEC_Exp_DisplayLineAltText(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Exp_DisplayLineAltText");
    }
    if(ms.MeCom_TEC_Exp_DisplayLineAltMode(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Exp_DisplayLineAltMode");
    }
    //Tab Expert: Sub Tab PBC
    for(int32_t Inst=1; Inst<=8; Inst++)
    {
        if(ms.MeCom_TEC_Exp_PbcFunction(Address, Inst, &lFields, MeGet)) 
        {
            //Serial.println("ms.MeCom_TEC_Exp_PbcFunction%d");
        }
    }
    for(int32_t Inst=1; Inst<=2; Inst++)
    {
        if(ms.MeCom_TEC_Exp_ChangeButtonLowTemperature(Address, Inst, &fFields, MeGet)) 
        {
            //Serial.println("ms.MeCom_TEC_Exp_ChangeButtonLowTemperature%d");
        }
        if(ms.MeCom_TEC_Exp_ChangeButtonHighTemperature(Address, Inst, &fFields, MeGet)) 
        {
            //Serial.println("ms.MeCom_TEC_Exp_ChangeButtonHighTemperature%d");
        }
        if(ms.MeCom_TEC_Exp_ChangeButtonStepSize(Address, Inst, &fFields, MeGet)) 
        {
            //Serial.println("ms.MeCom_TEC_Exp_ChangeButtonStepSize%d");
        }
    }

    //Tab Expert: Sub Tab Fan
    for(int32_t Inst=1; Inst<=2; Inst++)
    {
        if(ms.MeCom_TEC_Exp_FanControlEnable(Address, Inst, &lFields, MeGet)) 
        {
            //Serial.println("ms.MeCom_TEC_Exp_FanControlEnable%d");
        }
        if(ms.MeCom_TEC_Exp_FanActualTempSource(Address, Inst, &lFields, MeGet)) 
        {
            //Serial.println("ms.MeCom_TEC_Exp_FanActualTempSource%d");
        }

        if(ms.MeCom_TEC_Exp_FanTargetTemp(Address, Inst, &fFields, MeGet)) 
        {
            //Serial.println("ms.MeCom_TEC_Exp_FanTargetTemp%d");
        }
        if(ms.MeCom_TEC_Exp_FanTempKp(Address, Inst, &fFields, MeGet)) 
        {
            //Serial.println("ms.MeCom_TEC_Exp_FanTempKp%d");
        }
        if(ms.MeCom_TEC_Exp_FanTempTi(Address, Inst, &fFields, MeGet)) 
        {
            //Serial.println("ms.MeCom_TEC_Exp_FanTempTi%d");
        }
        if(ms.MeCom_TEC_Exp_FanTempTd(Address, Inst, &fFields, MeGet)) 
        {
            //Serial.println("ms.MeCom_TEC_Exp_FanTempTd%d");
        }
        if(ms.MeCom_TEC_Exp_FanSpeedMin(Address, Inst, &fFields, MeGet)) 
        {
            //Serial.println("ms.MeCom_TEC_Exp_FanSpeedMin%d");
        }
        if(ms.MeCom_TEC_Exp_FanSpeedMax(Address, Inst, &fFields, MeGet)) 
        {
            //Serial.println("ms.MeCom_TEC_Exp_FanSpeedMax%d");
        }
        if(ms.MeCom_TEC_Exp_FanSpeedKp(Address, Inst, &fFields, MeGet)) 
        {
            //Serial.println("ms.MeCom_TEC_Exp_FanSpeedKp%d");
        }
        if(ms.MeCom_TEC_Exp_FanSpeedTi(Address, Inst, &fFields, MeGet)) 
        {
            //Serial.println("ms.MeCom_TEC_Exp_FanSpeedTi%d");
        }
        if(ms.MeCom_TEC_Exp_FanSpeedTd(Address, Inst, &fFields, MeGet)) 
        {
            //Serial.println("ms.MeCom_TEC_Exp_FanSpeedTd%d");
        }
        if(ms.MeCom_TEC_Exp_FanSpeedBypass(Address, Inst, &lFields, MeGet)) 
        {
            //Serial.println("ms.MeCom_TEC_Exp_FanSpeedBypass%d");
        }
    }
    if(ms.MeCom_TEC_Exp_PwmFrequency(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Exp_PwmFrequency");
    }
    
    //Tab Expert");
    if(ms.MeCom_TEC_Exp_MiscActObjectTempSource(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Exp_MiscActObjectTempSource");
    }

    if(ms.MeCom_TEC_Exp_MiscDelayTillReset(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Exp_MiscDelayTillReset");
    }
    
    if(ms.MeCom_TEC_Exp_MiscError108Delay(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Exp_MiscError108Delay");
    }

    //Other Parameters (Not directly displayed in the Service Software)
    if(ms.MeCom_TEC_Oth_LiveEnable(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Oth_LiveEnable");
    }

    if(ms.MeCom_TEC_Oth_LiveSetCurrent(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Oth_LiveSetCurrent");
    }

    if(ms.MeCom_TEC_Oth_LiveSetVoltage(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Oth_LiveSetVoltage");
    }

    if(ms.MeCom_TEC_Oth_SineRampStartPoint(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Oth_SineRampStartPoint");
    }

    if(ms.MeCom_TEC_Oth_ObjectTargetTempSourceSelection(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Oth_ObjectTargetTempSourceSelection");
    }

    if(ms.MeCom_TEC_Oth_ObjectTargetTemperature(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Oth_ObjectTargetTemperature");
    }

    if(ms.MeCom_TEC_Oth_AtmAutoTuningStart(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Oth_AtmAutoTuningStart");
    }

    if(ms.MeCom_TEC_Oth_AtmAutoTuningCancel(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Oth_AtmAutoTuningCancel");
    }

    if(ms.MeCom_TEC_Oth_AtmThermalModelSpeed(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Oth_AtmThermalModelSpeed");
    }

    if(ms.MeCom_TEC_Oth_AtmTuningParameter2A(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Oth_AtmTuningParameter2A");
    }

    if(ms.MeCom_TEC_Oth_AtmTuningParameter2D(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Oth_AtmTuningParameter2D");
    }

    if(ms.MeCom_TEC_Oth_AtmTuningParameterKu(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Oth_AtmTuningParameterKu");
    }

    if(ms.MeCom_TEC_Oth_AtmTuningParameterTu(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Oth_AtmTuningParameterTu");
    }

    if(ms.MeCom_TEC_Oth_AtmPIDParameterKp(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Oth_AtmPIDParameterKp");
    }

    if(ms.MeCom_TEC_Oth_AtmPIDParameterTi(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Oth_AtmPIDParameterTi");
    }

    if(ms.MeCom_TEC_Oth_AtmPIDParameterTd(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Oth_AtmPIDParameterTd");
    }

    if(ms.MeCom_TEC_Oth_AtmSlowPIParameterKp(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Oth_AtmSlowPIParameterKp");
    }

    if(ms.MeCom_TEC_Oth_AtmSlowPIParameterTi(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Oth_AtmSlowPIParameterTi");
    }

    if(ms.MeCom_TEC_Oth_AtmPIDDPartDamping(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Oth_AtmPIDDPartDamping");
    }

    if(ms.MeCom_TEC_Oth_AtmCoarseTempRamp(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Oth_AtmCoarseTempRamp");
    }

    if(ms.MeCom_TEC_Oth_AtmProximityWidth(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Oth_AtmProximityWidth");
    }

    if(ms.MeCom_TEC_Oth_AtmTuningStatus(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Oth_AtmTuningStatus");
    }

    if(ms.MeCom_TEC_Oth_AtmTuningProgress(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Oth_AtmTuningProgress");
    }

    if(ms.MeCom_TEC_Oth_LutTableStart(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Oth_LutTableStart");
    }

    if(ms.MeCom_TEC_Oth_LutTableStop(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Oth_LutTableStop");
    }

    if(ms.MeCom_TEC_Oth_LutTableStatus(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Oth_LutTableStatus");
    }

    if(ms.MeCom_TEC_Oth_LutCurrentTableLine(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Oth_LutCurrentTableLine");
    }
    
    if(ms.MeCom_TEC_Oth_LutTableIDSelection(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Oth_LutTableIDSelection");
    }

    if(ms.MeCom_TEC_Oth_LutNrOfRepetitions(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Oth_LutNrOfRepetitions");
    }

    if(ms.MeCom_TEC_Oth_PbcEnableFunction(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Oth_PbcEnableFunction");
    }

    if(ms.MeCom_TEC_Oth_PbcSetOutputToPushPull(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Oth_PbcSetOutputToPushPull");
    }

    if(ms.MeCom_TEC_Oth_PbcSetOutputStates(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Oth_PbcSetOutputStates");
    }

    if(ms.MeCom_TEC_Oth_PbcReadInputStates(Address, 1, &lFields, MeGet)) 
    {
        //Serial.println("ms.MeCom_TEC_Oth_PbcReadInputStates");
    }

    if(ms.MeCom_TEC_Oth_ExternalActualObjectTemperature(Address, 1, &fFields, MeGet)) 
    {
        //Serial.println("MeCom_TEC_Oth_ExternalActualObjectTemperature");
    }
}


void printFloatFields(MeParFloatFields fields)
{
  //Serial.print("Value: ");
  //Serial.println(fields.Value, 3);
  //Serial.println("");
  //Serial.print("Min:");
  //Serial.println(fields.Min, 3);
  //Serial.print("Max: ");
  //Serial.println(fields.Max, 3);
}


void printLongFields(MeParLongFields fields)
{
  //Serial.print("Value: 0x");
  //Serial.println(fields.Value, HEX);
  //Serial.println("");
  /*//Serial.print("Min: ");
  //Serial.println(fields.Min, 3);
  //Serial.print("Max: ");
  //Serial.println(fields.Max, 3);*/
}


void SetTargetObjectTemp(meerstetterRS485& ms, uint8_t Address)
{
    int32_t ParId   = 3000; //ConsoleIO_IntInput("Please Enter Parameter ID", 0, 65535, 0);
    int32_t Inst    = 1; //ConsoleIO_IntInput("Please Enter Instance", 1, 255, 1);
    MeParFloatFields Fields;

    if(ms.MeCom_ParValuef(Address, ParId, Inst, &Fields, MeGetLimits))
    {
        //ConsoleIO_SetColor(ConsoleIO_Reset);
        //Serial.println("SetTargetObjectTemp max min");
        printFloatFields(Fields);
        //Fields.Value = ConsoleIO_FloatInput("Please Enter the new float Value", Fields.Min, Fields.Max, 0);
        Fields.Value = 26.75;  
        //ConsoleIO_SetColor(ConsoleIO_Red);
        if(ms.MeCom_ParValuef(Address, ParId, Inst, &Fields, MeSet))
        {
            //ConsoleIO_SetColor(ConsoleIO_Green);
            printf("Parameter ID: %d; Instance: %d; New Value: %f\n", ParId, Inst, Fields.Value);
        }
    }
}
