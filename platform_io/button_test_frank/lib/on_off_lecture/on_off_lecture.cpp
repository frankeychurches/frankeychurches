//****************************************LIBRARIES********************************************//

#include "Arduino.h"

#include "../defines.h"
#include "../variables/variables.h"

#include "on_off_lecture.h"

//----------------------------------------------TASK------------------------------------------------------------//
/*
int buttonPressed(uint8_t button) {
  static uint16_t lastStates = 0;
  uint8_t state = digitalRead(button);
  if (state != ((lastStates >> button) & 1)) {
    lastStates ^= 1 << button;
    return state == LOW;
  }
  return false;
}
*/
void on_off_lecture(void *parameters) 
{
  while(1) //Loop for repeating the task forever
  {
    vTaskDelay(600/portTICK_PERIOD_MS);
    state = digitalRead (ON_OFF_LECTURE); //Reading the state of the button (HIGH OR LOW)
    if(state!=lastBtnState) //If the value taken is different that the value taken on the last measure
    {
      lastBtnState = state; //Changing the value of the state
      if (state == LOW) //If the value taken is that the Button is pressed (LOW EFFECTIVELY)
      {
        Serial.println("ON OFF LECTURE BUTTON IS PRESSED!"); //Displaying that the button is pressed
        Serial.println();
      }

    }
  vTaskDelay(500/portTICK_PERIOD_MS);  //Delaying 500 ms before taking another measure

  }
}