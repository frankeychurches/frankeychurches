//****************************************LIBRARIES********************************************//
#include "Arduino.h"

#include "../defines.h"
#include "../variables/variables.h"

#include "eng_mode.h"

//----------------------------------------------TASK------------------------------------------------------------//

void eng_mode(void *parameters) 
{
  while(1) //Loop for repeating the task forever
  {

  vTaskDelay(600/portTICK_PERIOD_MS); //Delaying 100 ms
  
  S4_S5_val_get = analogRead(ENG_MODE); //Getting the value of each button at this moment
  // Serial.println((String) S4_S5_val_get);
  S4_S5_voltage_get = (S4_S5_val_get * 3.3)/ESP32_12_BIT_RESOLUTION;

  //IF LOOP to compare the values taken with the value they should have in case the buttons were pressed
  if(S4_S5_val_get >= 2500 && S4_S5_val_get <= 2600)    //UPPER BUTTON pressed values are due to the analog read of the pin. This value comes from measuring the voltage
                                     //on that pin with 12-bit resolution.   (f.e 3.3 V--4095 bit on the ESP32--> 2224 => 1,8 V)
  {                                  //The voltage value comes from V_IFS4 = [3.3 * R19 * (R15 + R16)]/[R13R19 + (R15 + R16) * R13 + (R15 + R16) * R19] 
    #ifdef SERIAL_PRINT_DEBUG
        Serial.println("UPPER BUTTON IS PRESSED");
        Serial.println("UPPER BUTTON ADC VALUE = " + (String) S4_S5_val_get + " bit");
        Serial.println("Voltage on pin when UPPER BUTTON pressed = " + (String) S4_S5_voltage_get + " V");
        Serial.println();
    #endif
  }
  else if(S4_S5_val_get >= 1200 && S4_S5_val_get <= 1240) //LOWER BUTTON pressed values are due to the analog read of the pin. This value comes from measuring the voltage
                                 //on that pin with 12-bit resolution.   (f.e 3.3 V--4095 bit on the ESP32--> 2224 => 1,8 V)
  {                              //The voltage value comes from V_IFS5 = [3.3 * (R16||R19)]/[(R13 + R15) + (R16||R19)]
    #ifdef SERIAL_PRINT_DEBUG
        Serial.println("LOWER BUTTON IS PRESSED");
        Serial.println("LOWER BUTTON ADC VALUE = " + (String) S4_S5_val_get + " bit");
        Serial.println("Voltage on pin when LOWER BUTTON pressed = "+ (String) S4_S5_voltage_get + " V");
        Serial.println();
    #endif
  }
  else if(S4_S5_val_get >= 1890 && S4_S5_val_get <= 1920) //S4 and S5 (both pressed) value are due to the analog read of the pin. This value comes from measuring the voltage
                                  //on that pin with 12-bit resolution.   (f.e 3.3 V--4095 bit on the ESP32--> 1807 => 1,45 V)
  {                               //The voltage value comes from V_IFS4&&S5 = [3.3 * R19]/[(R13 + R19) + (R19 * R16)]
    #ifdef SERIAL_PRINT_DEBUG
        Serial.println("S4 AND S5 BUTTONS ARE PRESSED");
        Serial.println("S4 and S5 pressed ADC VALUE = "+ (String) S4_S5_val_get + " bit");
        Serial.println("Voltage on pin when S4 and S5 pressed = "+ (String) S4_S5_voltage_get + " V");
        Serial.println();
    #endif
  }

  vTaskDelay(500/portTICK_PERIOD_MS);   //Delaying 500 ms before taking another value

  }
}