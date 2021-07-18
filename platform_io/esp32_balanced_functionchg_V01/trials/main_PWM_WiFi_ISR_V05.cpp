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
 *  Scanning... I2C devices:  Estos son los dispositivos encontrados.
 *  I2C device found at address 0x19  !     #define LSM303_ADDRESS_ACCEL   (0x32 >> 1)  // 0011001x
 *  I2C device found at address 0x1E  !     #define LSM303_ADDRESS_MAG     (0x3C >> 1)  // 0011110x
 *  I2C device found at address 0x69  !     
 *  I2C device found at address 0x77  ! BMP085_ADDRESS 
 *  I2C device found at address 0x3C  ! SSD1306_ADDRESS 
 *  done
 *   
 * Versions:
 *  V05:  7/07/2021
 *       
 * 
 *  V04:  13/06/2021
 *    ISR implementation for Button control.
 * 
 *  V03:  
 *        03/06/2021
 *    HTML & CSS Interface with Button control over de PWM motor activation and INA219 implementation.
 *    Problem with CNY70 flag and signal detection.
 *        20/05/2021
 *    Including the MCPWM commands, altogether with the RTOS configuration xTasks, which allow us to 
 *    compute all of the tasks simultanously. 
 *    Also the entrance of the tachometer configuration reading, as well as the battery voltage measure.
 *        02/05/2021
 *    Fixed problem with readings from the Adafruit IMU 10dof. The accelerometer and 
 *    magnetometer are fusioned to deploy a better reading. 
 *    Also, I found a library for the L3G4200D gyroscope which gives us a more accurate and precise
 *    measurement
 * 
 *  V02:  16/03/2021
 *    I2C communication with the Adafruit sensors to show data in the SSD1306 OLED screen. 
 *    Added buzzer system and "balanced" string sign in the oled when positioned  x=0, y=0, z=10.
 *
 *  V01:  11/02/2021
 *    Metido la cabecera y la descripción de emails.
 *    Versión portable del código para ver los valores de los sensores incluidos en el IMU, 
 *    preparada por el prof. Andrés Roldán
 * 
 */

/*--------------------------------------------------*/
/*------------------ LIBRARIES ---------------------*/
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
//-------VARIABLES------
#include "../lib/variables/variables.h"
//-------FUNCTIONS------
#include "../lib/functions/functions.h"
//-------SENSORS--------
#include "../lib/sensors/sensors.h"
//-------WIFI--------
#include "../lib/wifi/wifi.h"
//-------MOTORS--------
#include "../lib/motor_functions/motor_functions.h"
//-------TACHOMETER--------
#include "../lib/tacho/tacho.h"
//-------DEFINES--------
#include "../defines.h"

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

void IRAM_ATTR isr() {
  portENTER_CRITICAL_ISR(&MotorMux);
  //LED_motor = HIGH;
  activateMotors = true;
  portEXIT_CRITICAL_ISR(&MotorMux);
}


/********************************/
/*setup*/
/********************************/
void setup(void)
{
  Serial.begin(115200); Wire.begin();

  // LED indication for motors' startup 
  pinMode(integrated_LED, OUTPUT); digitalWrite(integrated_LED, LOW);
  pinMode(LED_INDICATORS, OUTPUT); digitalWrite(LED_INDICATORS, LOW);

  // Button configuration setup
  pinMode(buttonPin, INPUT);
  attachInterrupt(buttonPin, isr, FALLING);
  // Buzzer config:
  ledcSetup(channel_buzzer, freq_buzzer, resolution_buzzer);
  ledcAttachPin(13, channel_buzzer);
  // WiFi setup and initialization
  WiFi_HTML_setup();
  // Tachometer config:
  pinMode(PITCH_PIN, INPUT_PULLDOWN);
  pinMode(YAW_PIN, INPUT_PULLDOWN);
  pinMode(ROLL_PIN, INPUT_PULLDOWN);
  // MCPWM setup configuration function:
    MCPWM_setup(); initCompo();
  // Gyro setup code:
    gyro.init(-120.1, -563.64724, -0.48845); 
    imu.pT = 0;
    imu.xi = imu.yi = imu.zi = 0;
    delay(1500);   //Time to wait before calibration to open the serial
    gyro.printCalibrationValues(150);

  // Initialize the OLED display:
  display.clearDisplay();
  display.setTextColor(WHITE);

  /* // Start serial read task
  xTaskCreatePinnedToCore(  // Use xTaskCreate() in vanilla FreeRTOS
            readSerial,     // Function to be called
            "Read Serial",  // Name of task
            1024,           // Stack size (bytes in ESP32, words in FreeRTOS)
            NULL,           // Parameter to pass
            1,              // Task priority (must be same to prevent lockup)
            NULL,           // Task handle
            app_cpu);       // Run on one core for demo purposes (ESP32 only) */

  // Start IMU data reading task:
  xTaskCreatePinnedToCore( SensorResults,"IMU data reading",102400, NULL,1, NULL,app_cpu);       
  // Start Tachometer for Pitch function
  xTaskCreatePinnedToCore( Tachometer_P,"Tachometer Pitch", 10240, NULL,1, NULL,app_cpu);       
  // Start Tachometer for Yaw function
  xTaskCreatePinnedToCore( Tachometer_Y, "Tachometer Yaw",  10240, NULL,1, NULL,app_cpu);       
  // Start Tachometer for Roll function
  xTaskCreatePinnedToCore( Tachometer_R, "Tachometer Roll", 10240, NULL,1, NULL,app_cpu);       
  // Timer for serial data RPM management function
  // xTaskCreatePinnedToCore( RPM_data,"RPM serial data",      1024, NULL,1, NULL,app_cpu);   
  // Read Battery Voltage:
  xTaskCreatePinnedToCore( BatteryMeasure,"Voltage battery measure",10240, NULL,1, NULL,app_cpu); 
  // Data Receiving from INA219:
  xTaskCreatePinnedToCore( INA_Measure,"INA219 Data",102400, NULL,1, NULL,app_cpu); 
  // Start Motor PWM signal creation task with LED built in indication:
  xTaskCreatePinnedToCore( MotorMovement,"Motor PWM signal",102400, NULL,1, NULL,app_cpu); 
  // Delete "setup and loop" task
  vTaskDelete(NULL);
}

void loop()
{
  // nothing here
}