/*------------------ OWN LIBRARIES ---------------------*/
#include "../defines.h"
//-------VARIABLES------
#include "../variables/variables.h"

/*!
    @brief  Tachometer function for the PITCH. With the CNY70 and this program below we are
            able to read the analog signal received as an RPM signal.

            Since there is only one rising edge per rotation on this
            tachometer, and only the rising edges are being used in the RPM
            calculation, the elapsed time since the last reading is equal to the
            revolutions per microsecond.
*/
void Tachometer_P(void* parameter)
{
  while(1)
  {
    cny70.thisReading_p = analogRead(PITCH_PIN);    // Take a reading check to see
    if (cny70.thisReading_p > highLevel)            // if above  the high level 
    {                                         // threshold, if so, it was low on
      if (cny70.fallen_p == true)                   // previous pass which means a
      {                                       // rising edge was detected
        cny70.thisTime_p = micros();
        // 1 rev/ usec * 1e6 usec/1 sec *60 sec/1 min == [revs/min] == RPM
        // RPM = Revolutions per minute
        cny70.RPM_p = 60000000 / (cny70.thisTime_p - cny70.lastTime_p);    
        // rad/s because we need angular acceleration for our torque design response
        // 1 rev/ min * 1 min/60 sec * 2pi rad/1 rev == [rad/sec] == w
        cny70.rads_p = (cny70.RPM_p*2*PI) / 60;  
        // Angular acceleration is dw/dt which can be discretised into w2-w1/t2-t1:
          cny70.ang_accel_p = (cny70.rads_p - cny70.rads_p_f) / (cny70.thisTime_p - cny70.lastTime_p);
          cny70.rpm_p_str = String(cny70.RPM_p); cny70.ang_acc_p_str = String(cny70.ang_accel_p); 
          cny70.rads_p_f = cny70.rads_p;
        cny70.lastTime_p = cny70.thisTime_p;         // Set the time to "last time" and
        cny70.fallen_p = false;                      // the edge to false so we won't count this rise again
      }
    }
    if (cny70.thisReading_p < lowLevel)        // When the edge first falls below the 
    {                                          // low level threshold, make fallen true                          
      if (cny70.fallen_p == false)                        
      {
        cny70.fallen_p = true;
      }
    }
  }
}

/*!
    @brief  Tachometer function for the YAW. With the CNY70 and this program below we are
            able to read the analog signal received as an RPM signal.
*/
void Tachometer_Y(void* parameter)
{
  while(1)
  {
    cny70.thisReading_y = analogRead(YAW_PIN);   
    if (cny70.thisReading_y > highLevel)            
    {                                          
      if (cny70.fallen_y == true)                     
      {                                               
        cny70.thisTime_y = micros();
        cny70.RPM_y = 60000000 / (cny70.thisTime_y - cny70.lastTime_y); cny70.rads_y = (cny70.RPM_y*2*PI) / 60;
          cny70.ang_accel_y = (cny70.rads_y - cny70.rads_y_f) / (cny70.thisTime_y - cny70.lastTime_y);
          cny70.rpm_y_str = String(cny70.RPM_y); cny70.ang_acc_y_str = String(cny70.ang_accel_y); 
          cny70.rads_y_f = cny70.rads_y;
        cny70.lastTime_y = cny70.thisTime_y;                 
        cny70.fallen_y = false;                                                          
      }
    }
    if (cny70.thisReading_y < lowLevel)                
    {                                                             
      if (cny70.fallen_y == false)                        
      {
        cny70.fallen_y = true;
      }
    }
  }
}

/*!
    @brief  Tachometer function for the ROLL. With the CNY70 and this program below we are
            able to read the analog signal received as an RPM signal.
*/
void Tachometer_R(void* parameter)
{
  while(1)
  {
    cny70.thisReading_r = analogRead(ROLL_PIN);   
    if (cny70.thisReading_r > highLevel )            
    {                                      
      if (cny70.fallen_r == true)                 
      {                                                                       
        cny70.thisTime_r = micros();
        cny70.RPM_r = 60000000 / (cny70.thisTime_r - cny70.lastTime_r); cny70.rads_r = (cny70.RPM_r*2*PI) / 60;
          cny70.ang_accel_r = (cny70.rads_r - cny70.rads_r_f) / (cny70.thisTime_r - cny70.lastTime_r);
          cny70.rpm_r_str = String(cny70.RPM_r); cny70.ang_acc_r_str = String(cny70.ang_accel_r); 
          cny70.rads_r_f = cny70.rads_r;
        cny70.lastTime_r = cny70.thisTime_r;                    
        cny70.fallen_r = false;                                                        
      }
    }
    if (cny70.thisReading_r < lowLevel)                
    {                                                                
      if (cny70.fallen_r == false)                        
      {
        cny70.fallen_r = true;
      }
    }
  }
}

/*!
    @brief  Data surveillance through serial monitor. 
            ONLY FOR TESTING, HEAVY PROCESSING MAY ALTER THE ALGORITHM.
*/
static void RPM_data(void* parameters)
{
  while(1)
  {
    cny70.currentMillis = millis();  //get the current "time"
    if (cny70.currentMillis - cny70.startMillis >= cny70.period)
    {
      #ifdef DEBUG
      Serial.print(F("RPM Pitch: ")); Serial.print(cny70.RPM_p); Serial.print(F("  ,"));
      Serial.print(F("rad/s: ")); Serial.print(cny70.ang_accel_p,2); Serial.print(F("  ;"));
      Serial.print(F("RPM Yaw: ")); Serial.print(cny70.RPM_y); Serial.print(F("  ,"));
      Serial.print(F("rad/s: ")); Serial.print(cny70.ang_accel_y,2); Serial.print(F("  ;"));
      Serial.print(F("RPM Roll: ")); Serial.print(cny70.RPM_r); Serial.print(F("  ,")); 
      Serial.print(F("rad/s: ")); Serial.print(cny70.ang_accel_r); Serial.print(F("  ;"));
      Serial.println(F(" "));
      #endif
      cny70.startMillis = cny70.currentMillis;  // Save the start time of the current state.
    }
  }
}