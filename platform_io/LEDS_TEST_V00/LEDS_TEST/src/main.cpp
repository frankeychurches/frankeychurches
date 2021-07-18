/*
 ________  ________  ________  ________   ________  ________  ________  _________
|\   ____\|\   __  \|\   __  \|\   ___  \|\   __  \|\   ____\|\   __  \|\___   ___\
\ \  \___|\ \  \|\  \ \  \|\  \ \  \\ \  \ \  \|\  \ \  \___|\ \  \|\  \|___ \  \_|
 \ \  \  __\ \   _  _\ \   __  \ \  \\ \  \ \   __  \ \_____  \ \   __  \   \ \  \
  \ \  \|\  \ \  \\  \\ \  \ \  \ \  \\ \  \ \  \ \  \|____|\  \ \  \ \  \   \ \  \
   \ \_______\ \__\\ _\\ \__\ \__\ \__\\ \__\ \__\ \__\____\_\  \ \__\ \__\   \ \__\
    \|_______|\|__|\|__|\|__|\|__|\|__| \|__|\|__|\|__|\_________\|__|\|__|    \|__|
                                                      \|_________|              
 * Typing help from: http://patorjk.com/software/taag/#p=display&f=Graffiti&t=Type%20Something%20 
 * 
 * https://granasat.ugr.es
 * 
 * Programmers:
 *  Prof. AndrÃ©s RoldÃ¡n       1/08/2019 (amroldan@ugr.es)
 *  Irene Gil Martín        28/05/2021  (irene.gima3@gmail.com)
 *  
 *  Use:
 * 
 *  This code is aimed to program the CANSAT leds
 *    
 * Versions:
 *
 * Modifications regarding the version of the program have been explained on the README.md file located on the project's root
 * 
 * 
 *  NOTES:
 */

//**********************************LIBRARIES************************************************//

//----------------------------------------ARDUINO LIBRARIES--------------------------------------//

#include <Arduino.h>

//----------------------------------------DEFINES LIBRARIES--------------------------------------//

#include "../lib/defines.h"

//-------------------------------------------TASK LIBRARY--------------------------------------------//

#include "../lib/LEDS_TEST/LED_TEST.h"

void setup() {
  
  pinMode(HOLD_BATTERY_ON,OUTPUT);
  //pinMode(LED_UNIBOARD,OUTPUT);
  //pinMode(LED_GPIO39,OUTPUT);
  //pinMode(LED_TX_RX,OUTPUT);
  //pinMode(LED_GPIO7,OUTPUT);  
  
  Serial.begin(PC_BAUDRATE);
  
  vTaskDelay(200 / portTICK_PERIOD_MS); //Waiting 0.2 second before starting again

  #ifdef SERIAL_PRINT_DEBUG //Printing the values on the monitor if the serial comunication is up
    Serial.println("**********************************************************************************************************************");
    Serial.println();
    Serial.println("                                               INITIATING LED TEST");
    Serial.println("---------------------Program that makes each led blink once every 2 seconds----------------------------------");
    Serial.println();
    Serial.println("**********************************************************************************************************************");
  #endif

  xTaskCreatePinnedToCore(leds_test,   //Calling the task to be executed
                          "LEDs TEST", //Task name
                          1524,             //Stack size
                          NULL,             //Parameter to pass to function
                          20,                //Task Priority (top priority = 24)
                          NULL,             //Task_handle
                          1);               //Core that will exute the task (0 or 1)
}


void loop() {
  // put your main code here, to run repeatedly:
}