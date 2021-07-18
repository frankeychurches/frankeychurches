/*--------------------------------------------------*/
/*------------------ Librerías ---------------------*/
/*--------------------------------------------------*/
#include <Wire.h>
/*-------------------SSD1306-------------------*/
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
/*------------------ANDRES---------------------*/
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <L3G4200D.h>
#include <Adafruit_10DOF.h>
/*------------------MCPWM---------------------*/
#include "driver/mcpwm.h"

//#define CABLES_GUARRETAS TRUE

#ifdef CABLES_GUARRETAS 
  #define ledPin_4 12   //Declara GPIO D12 como PWM PITCH
  #define ledPin_5 26   //Declara GPIO D26 como PWM ROLL
  #define ledPin_18 25   //Declara GPIO D25 como PWM YAW
  #define PITCH_PIN 32
#else
  #define ledPin_4 4   //Declara GPIO D4 como PWM PITCH
  #define ledPin_5 5   //Declara GPIO D5 como PWM ROLL
  #define ledPin_18 18  //Declara GPIO D18 como PWM YAW
  #define PITCH_PIN 25
#endif

// setting PWM properties
const int freq = 50;
const int ledChannel_1 = 0;
const int ledChannel_2 = 1;
const int ledChannel_3 = 2;
const int resolution = 16;
 
void setup(){
  Serial.begin(115200); Wire.begin();
  // configure LED PWM functionalitites
  ledcSetup(ledChannel_1, freq, resolution);
  ledcSetup(ledChannel_2, freq, resolution);
  ledcSetup(ledChannel_3, freq, resolution);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(ledPin_4, ledChannel_1); 
  ledcAttachPin(ledPin_5, ledChannel_2);
  ledcAttachPin(ledPin_18, ledChannel_3);
  // ledcWrite(ledChannel, 0);
  // delay(4000);
}
 
void loop(){
 ledcWrite(ledChannel_1, 65500);
 ledcWrite(ledChannel_2, 65500);
 ledcWrite(ledChannel_3, 65500);
 delay(50);
}