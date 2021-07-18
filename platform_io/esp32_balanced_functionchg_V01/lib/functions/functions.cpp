/*--------------------------------------------------*/
/*------------------ Librerías ---------------------*/
/*--------------------------------------------------*/
#include <Wire.h>
/*-------------------SSD1306-------------------*/
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_INA219.h>
/*-------------------WIFI-------------------*/
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
/*------------------ANDRES---------------------*/
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <L3G4200D.h>
#include <Adafruit_10DOF.h>
/*------------------MCPWM---------------------*/
#include "driver/mcpwm.h"

/*------------------ OWN LIBRARIES ---------------------*/
#include "../defines.h"
//-------VARIABLES------
#include "../variables/variables.h"

/*!
    @brief  Battery measurement system designed with a voltage divider.
            A series resistance with a low pass filter (R, R//C) allow usto determine
            the remain voltage of the battery, thereby knowing how much
            battery we have left before causing a deep discharge of the battery.  
*/
void BatteryMeasure(void* parameters)
{
  while(1)
  {
    bvm.batt_raw = analogRead(BAT_PIN);  //max value 4095
    bvm.sum_bat = 0;  
    for (uint8_t i = 0; i < bvm.bat_smpl; ++i)
    {
      bvm.sum_bat += bvm.batt_raw; //max sum value 4095*100 = 409500
    }
    bvm.bat_norm = 3.818181 * (3.3/4095) * (bvm.sum_bat/bvm.bat_smpl);   // real voltage battery value.
    bvm.bat_perc = ((12.6- bvm.bat_norm)/3) * 100; bvm.bat_perc_str = String(bvm.bat_perc);
    bvm.bat_timer = millis();  //get the current "time"
    if (bvm.bat_timer - bvm.bat_start >= bvm.bat_period)
    {
      #ifdef DEBUG
      Serial.print(F("Battery Voltage: ")); Serial.print(bvm.bat_norm); Serial.print(F(" % ; "));
      Serial.print(F(" Battery Voltage Percentage: ")); Serial.print(bvm.bat_perc, 2); Serial.println(F(" % "));
      #endif
      bvm.bat_start = bvm.bat_timer;  // Save the start time of the current state.
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

/**************************************************************************/
/*!
    @brief  INA219 program for Bus Voltage, Current and Load data. 
*/
/**************************************************************************/

void INA_Measure(void* parameters)
{
  while(1)
  {
    //voltage_shunt = ina219.getShuntVoltage_mV();  
      //It is saved on the variable voltage shunt the tension (in mA) running through R6 (shunt power resistor 100 ohms)
    ina.voltage_bus = ina219.getBusVoltage_V(); 
      //It is saved on the variable voltage bus the voltage difference between VIN and VOUT of the INA219 module
    ina.current_mA = ina219.getCurrent_mA();  
      //It is saved on the variable current_mA the current that runs thrugh the INA219 module
    //voltage_load = voltage_bus + (voltage_shunt / 1000);  
      //Total tension needed for this module. Is the sum of the shunt and the bus voltage (parallel)
    ina.load_power = ina219.getPower_mW();
    //String variables for HTML interface communication  
    ina.volt_str = String(ina.voltage_bus); ina.curr_str = String(ina.current_mA); ina.pow_str = String(ina.load_power);
    #ifdef DEBUG
    //Printing on the terminal the results of the measurements whith its units
    Serial.println("Shunt Voltage: " + (String) ina.voltage_shunt + " mV"); 
    Serial.println("Bus Voltage: " + (String) ina.voltage_bus + " V");
    Serial.println("Current: " + (String) ina.current_mA + " mA");  
    Serial.println("Load Voltage: " + (String) ina.voltage_load + " V"); 
    Serial.println(); //leaving one line empty to separate from the next measurement
    #endif 
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}



