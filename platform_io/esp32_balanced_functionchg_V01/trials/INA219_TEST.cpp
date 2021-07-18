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
 *  Irene Gil Martín
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
#include <Adafruit_INA219.h>
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

// EN: Official USB serial console for Arduino SAMD core
// CZ: Oficiální USB serial port pro Arduino SAMD

//#define Serial SerialUSB //Defining the communication between the device and the PC through the USB port

Adafruit_INA219 ina219(0x40);  //Address of the current register (04h)


int ms_delay = 500; //Setting a varible of 500 ms 


float voltage_shunt = 0; //Defining the variables for measurement or calculation
float voltage_bus = 0;
float current_mA = 0;
float voltage_load = 0;
  

void setup(void)  //Setup function runs once
{ 
  Serial.begin(115200);//Initiating the serial communication whith a rate of 115200 bits per second (baudies)
  
  Serial.println("CANSAT kit INA219 test."); //Printing this sentence when the serial communication is ready
  if(!ina219.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print("Ooops, no INA219 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

}


void loop(void)  //Initiating a loop that will end when we shut it throguh the terminal
{
  voltage_shunt = ina219.getShuntVoltage_mV();  //It is save on the variable voltage shunt the tension (in mA) running through R6 (shunt power resistor 100 ohms)
  voltage_bus = ina219.getBusVoltage_V(); //It is save on the variable voltage bus the voltage difference between VIN and VOUT of the INA219 module
  current_mA = ina219.getCurrent_mA();  //It is save on the variable current_mA the current that runs thrugh the INA219 module
  voltage_load = voltage_bus + (voltage_shunt / 1000);  //Total tension needed for this module. Is the sum of the shunt and the bus voltage (parallel)

//Printing on the terminal the results of the measurements whith its units
  Serial.println("Shunt Voltage: " + (String) voltage_shunt + " mV"); 
  Serial.println("Bus Voltage: " + (String) voltage_bus + " V");
  Serial.println("Current: " + (String) current_mA + " mA");  
  Serial.println("Load Voltage: " + (String) voltage_load + " V"); 
  Serial.println(); //leaving one line empty to separate from the next measurement

  //EN: Wait on set ms_delay
  //CZ: Počkej nastavený počet milisekund
  delay(ms_delay); //Delaying 500 ms before iniziating the next measurement
}

