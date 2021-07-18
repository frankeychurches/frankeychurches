#include <Arduino.h>

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
 *  This code is aimed to program the CANSAT buzzer developed in GRANASAT. 
 * 
 *  Versions:
 *
 * Modifications regarding the version of the program have been explained on the README.md file located on the project's root
 * 
 * 
 *  NOTES:
 */

//**********************************LIBRARIES************************************************//

//----------------------------------------ARDUINO LIBRARIES--------------------------------------//
//#include <Arduino.h>

//---------------------------------------DEFINITIONS LIBRARY-------------------------------------//
#include "../lib/defines.h"
#include "../lib/variables/variables.h"

//****************************************TASKS******************************************//

#include "../lib/eng_mode/eng_mode.h"
#include "../lib/on_off_lecture/on_off_lecture.h"



void setup()
{
  Serial.begin(PC_BAUDRATE);
  
  while(!Serial);

  pinMode(ON_OFF_LECTURE, INPUT); //Setting the ON_OFF_LECTURE PIN AS INPUT--THE OTHER PIN IS ALREADY AN ONLY INPUT SO IT DOES NOT NEED TO BE SET AS INPUT


#ifdef SERIAL_PRINT_DEBUG //Printing the initialitation message on the monitor if the serial comunication is up
  Serial.println("**********************************************************************************************************************");
  Serial.println();
  Serial.println("                                               INITIATING BUTTON LECTURE TEST");
  Serial.println("-------------------------------When a button is pressed, it will appear its name on the monitor----------------------------------");
  Serial.println();
  Serial.println("**********************************************************************************************************************");
#endif

/*xTaskCreatePinnedToCore(on_off_lecture,   //Calling the task to be executed
                          "on_off_lecture", //Task name
                          1024,             //Stack size
                          NULL,             //Parameter to pass to function
                          7,                //Task Priority (top priority = 24)
                          NULL,             //Task_handle
                          1);               //Core that will exute the task (0 or 1)
                          */

  xTaskCreatePinnedToCore(eng_mode,   //Calling the task to be executed
                          "eng_mode buttons lecture", //Task name
                          2024,             //Stack size large as it needs some variables
                          NULL,             //Parameter to pass to function
                          10,               //Task Priority (top priority = 24)
                          NULL,             //Task_handle
                          1);               //Core that will exute the task (0 or 1)
}

void loop() {
  // put your main code here, to run repeatedly:
}