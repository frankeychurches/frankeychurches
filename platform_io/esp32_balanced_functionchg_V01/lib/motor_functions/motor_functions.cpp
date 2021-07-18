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
    @brief  MCPWM setup configuration for the motor velocity management.
*/
void MCPWM_setup(){
  //mcpwm_gpio_init(Unit PWM 0-1, saida (Timer 0-2, Pair A-B), GPIO)     
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
  //------------- Adding for two motors
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, GPIO_PWM1A_OUT); 
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, GPIO_PWM2A_OUT); 
  // configuration of the pwm signal characteristics
  mcpwm_config_t pwm_config;
  pwm_config.frequency = 50;                          //frequency = 50Hz,
  pwm_config.cmpr_a = 0.0;                                // (duty cycle) do PWMxA = 0
  pwm_config.cmpr_b = 0.0;                                // (duty cycle) do PWMxb = 0
  pwm_config.counter_mode = MCPWM_UP_COUNTER;           //Para MCPWM assimetric
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;             //duty cycle high
  //Inicia(Unit , Timer, Config PWM)
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config); //Define PWM0A & PWM0B with above configurations
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config); //Define PWM0A & PWM0B with above configurations
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config); //Define PWM0A & PWM0B with above configurations
}

/*!
    @brief  Function which configures the operator's A MCPWM (Unit, Timer, Percentage)
            Duty Cycle.
*/
void brushed_motor_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
  mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
}

/*!
    @brief  Function which manages the PWM sending to the motors
            Duty Cycle.
*/
void MotorMovement(void* parameters)
{
  while(1)
  {
    // vTaskDelay(2000 / portTICK_PERIOD_MS);
    if(activateMotors)
    {
      digitalWrite(LED_INDICATORS, HIGH);
      // Acceleration from initial velocity to medium velocity
      // Min == 5.346%    Max == 9.516%    Values from RC Controller
      for(float i=5.0 ; i<=7.0 ; i+= 0.1){
        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, i);
        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_1, i);
        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_2, i);
        //Serial.print(F("ACCEL: ")); Serial.print(i); Serial.println(F(""));
        Serial.println(F("SPEEDING"));
        vTaskDelay(300 / portTICK_PERIOD_MS);
      }
      // Delay between accel and decel not to overimpose one onto the other
      vTaskDelay(300 / portTICK_PERIOD_MS);
      // Deceleration from maximum velocity to initial velocity 
      for(float j=7.0 ; j>=5.0 ; j-= 0.1){
        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, j);
        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_1, j);
        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_2, j);
        //Serial.print(F(" DECEL: ")); Serial.print(j); Serial.println(F(""));
        Serial.println(F("BRAKING")); 
        vTaskDelay(300 / portTICK_PERIOD_MS);
      }
      digitalWrite(LED_INDICATORS, LOW);
      // Change the state variable 
      portENTER_CRITICAL_ISR(&MotorMux);
      activateMotors = false;
      // LED_motor = LOW;
      portEXIT_CRITICAL_ISR(&MotorMux);
    }
    else
    {
      brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 5.0);
      brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_1, 5.0);
      brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_2, 5.0);
      vTaskDelay(300 / portTICK_PERIOD_MS);
    }
  }
}