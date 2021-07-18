/*
 * 
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

/*
    PITCH -- PIN 25
    YAW   -- PIN 26
    ROLL  -- PIN 27
*/

#define PITCH_PIN 25
#define YAW_PIN 26
#define ROLL_PIN 27

void setup() {
  Serial.begin(115200); //Configuracion de la velocidad serial
  pinMode(ROLL_PIN, INPUT_PULLDOWN);

}

void loop() {
  Serial.print("Lectura: ");
  Serial.println(analogRead(ROLL_PIN)); //Imprime el valor de la lectura del canal D25
  delay(2000); //retardo de 2 segundos entre lectura
}