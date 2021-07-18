/*
 * 

 
  Created by Frank Milburn, 1 June 2015
  Released into the public domain
 ______    ________  ________  ________   ________  ________  ________  _________
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
 *  Prof. Andrés Roldán      01/03/2019 (amroldan@ugr.es)
 *  Francisco Llave Iglesias 11/02/2021 (francisco.llaveiglesias@gmail.com)
 *  
 *  Uso:
 *  
 *   Esta versión inicial de uso del IMU 10 DOF  https://www.adafruit.com/product/1604 
 *   que tiene disponible:
 *       L3GD20H 3-axis gyroscope: ±250, ±500, or ±2000 degree-per-second scale
 *       LSM303 3-axis compass: ±1.3 to ±8.1 gauss magnetic field scale
 *       LSM303 3-axis accelerometer: ±2g/±4g/±8g/±16g selectable scale
 *       BMP180 barometric pressure/temperature: -40 to 85 °C, 300 - 1100hPa range, 0.17m resolution
 *   
 * Versions:
 * 
 *  V02:  
 *    I2C communication with the Adafruit sensors to show data in the SSD1306 OLED screen. 
 *    Added buzzer system and "balanced" string sign in the oled when positioned  x=0, y=0, z=10.
 *
 *  V01:  
 *    Metido la cabecera y la descripción de emails.
 *    Versión portable del código para ver los valores de los sensores incluidos en el IMU, 
 *    preparada por el prof. Andrés Roldán
 * 
 * Scanning... I2C devices:  Estos son los dispositivos encontrados.
 *  I2C device found at address 0x19  !     #define LSM303_ADDRESS_ACCEL          (0x32 >> 1)         // 0011001x
 *  I2C device found at address 0x1E  !     #define LSM303_ADDRESS_MAG            (0x3C >> 1)         // 0011110x
 *  I2C device found at address 0x69  !     
 *  I2C device found at address 0x77  ! BMP085_ADDRESS 
 *  I2C device found at address 0x3C  ! SSD1306_ADDRESS 
 *  done
 */

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

// Specify the pins where the analog signal will be read
#define PITCH_PIN 25
#define YAW_PIN 26
#define ROLL_PIN 27
         

bool fallen = false;             // Flag for falling/rising edge detection
int lowLevel = 2000;              // Low level threshold for falling edge
int highLevel = 3500;            // High level threshold for rising edge
long lastTime = 0;               // Last time a rising edge was detected
long thisTime = 0;               // Current time

void setup() 
{
  Serial.begin(115200);          // Highest transmission the board allows
  Serial.print("Starting tachometer...");
  pinMode(PITCH_PIN, INPUT_PULLDOWN);
  lastTime = micros();           // Using micros for higher resolution
  
}

void loop() 
{
  int thisReading = analogRead(PITCH_PIN);    // Take a reading
  // Serial.print("Reading: ");
  // Serial.println(thisReading);
  if (thisReading > highLevel)                // check to see if above
  {                                           // the high level threshold,
    if (fallen == true)                       // if so, it was low on
    {                                         // previous pass which means a
                                              // rising edge was detected
                                              
      // Since there is only one rising edge per rotation on this
      // tachometer, and only the rising edges are being used in the RPM
      // calculation, the elapsed time since the last reading is equal to the
      // revolutions per microsecond.  To get RPM, divide (1,000,000
      // micros/second x 60 seconds/minute) by the elapsed time which
      // is equivalent to dividing 60000000 by the elasped time.
      
      thisTime = micros();
      long RPM = 60000000 / (thisTime - lastTime);
      Serial.print("   RPM: ");
      Serial.println(RPM);
      lastTime = thisTime;                      // Set the time to "last time" and
      fallen = false;                           // the edge to false so we won't count
                                                // this rise again
    }
  }
  if (thisReading < lowLevel)                   // When the edge first falls below the 
  {                                             // low level threshold, make fallen true                          
    if (fallen == false)                        
    {
      fallen = true;
    }
  }  
}