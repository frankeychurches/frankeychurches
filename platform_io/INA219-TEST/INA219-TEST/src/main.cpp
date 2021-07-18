#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>



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


  while(!Serial); //Waiting for the serial communication to be ready
  
  Serial.println("CANSAT kit INA219 test."); //Printing this sentence when the serial communication is ready

  ina219.begin(); //Setting the ina219 to start
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
