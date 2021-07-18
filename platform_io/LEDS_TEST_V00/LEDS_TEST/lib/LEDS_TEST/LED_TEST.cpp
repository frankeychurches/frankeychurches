//**********************************LIBRARIES************************************************//

//----------------------------------------ARDUINO LIBRARIES--------------------------------------//

#include <Arduino.h>
//---------------------------------------DEFINITIONS LIBRARY-------------------------------------//

#include "../defines.h"


void leds_test(void *parameters) 
{
  while(1) //Loop for repeating the task forever
  {
    digitalWrite(HOLD_BATTERY_ON, HIGH);
    digitalWrite(SPI_SCLK,HIGH);
    //digitalWrite(LED_UNIBOARD, HIGH);
    //digitalWrite(LED_GPIO39, HIGH);
    //digitalWrite(LED_TX_RX, HIGH);
    //digitalWrite(LED_GPIO7, HIGH);

    #ifdef SERIAL_PRINT_DEBUG
        Serial.println("UP");
    #endif
    vTaskDelay(1000/portTICK_PERIOD_MS);  //Delaying 200 ms before setting led pin low
    
    digitalWrite(HOLD_BATTERY_ON, LOW);
    digitalWrite(SPI_SCLK,LOW);
    //digitalWrite(LED_UNIBOARD, LOW);
    //digitalWrite(LED_GPIO39, LOW);
    //digitalWrite(LED_TX_RX, LOW);
    //digitalWrite(LED_GPIO7, LOW);
    
    #ifdef SERIAL_PRINT_DEBUG
        Serial.println("DOWN");
    #endif
    vTaskDelay(1000/portTICK_PERIOD_MS);  //Delaying 200 ms before setting led pin high
    #ifdef SERIAL_PRINT_DEBUG
        Serial.println();
    #endif

  }
}

