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

/*-----  COMPILACIÓN CONDICIONAL  ------*/
//#define DEBUG TRUE    // Si esa línea no está comentada, se imprimen los mensajes por el puerto serie.

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

/* Assign a unique ID to the sensors */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
Adafruit_10DOF                dof   = Adafruit_10DOF();
L3G4200D gyro;

/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

#define freq_buzzer 2400   // maxima frecuencia para maximo volumen
const int channel_buzzer = 0;     // porque va por PWM 
const int resolution_buzzer = 8;  // no necesitamos mucha resolución porque va al máximo
const int duty_c_buzzer_on = 125; // definimos el duty cycle al 50%
const int duty_c_buzzer_off = 0;  // apagar el buzzer


// EL PWM SE AJUSTA AQUI:
const float pulse_width = 20.0;                 // ancho de pulso de la señal de control in ms
const float pwwork = 2.0;                       // Anchod e pulso de la señal de trabajo in ms
const float startpow = 0.6;                     // Ancho de pulso de inicio in ms
const float pwm_1 = (pwwork - startpow)/200.0;  // Ancho de pulso para 1% de potencia
const float max_pow = 75.0;                     // Limite de potencia global in % (leistung=rendimiento)
const float min_pow = 10.0;                     // Potencia minima necesaria del motor in %

  /* Read values FROM 10DOF CALCULATIONS: */
  sensors_event_t accel_event;      // event ask for accelerometer data
  sensors_event_t mag_event;        // event ask for magnetometer data
  sensors_event_t bmp_event;        // event ask for barometer data
  sensors_event_t event;            // event ask for gyroscope data
  sensors_vec_t   orientation;
  /*TAKEN FROM ADAFRUIT 10DOF LIBRARY*/

/////////////////////////////////////////////////////////////////////////
//funcion para el calculo del valor de actuacion para una PWM determinada
// void calcPowerMotor(float power_motor){
//   return (startpow + ((power_motor + 100)* pwm_1)) / pulse_width;
// }

/**************************************************************************/
/*!
    @brief  Initialises all the sensors and components used by the system
*/
/**************************************************************************/

void initCompo()
{
  /* Initialise the display */
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  /* Initialise the sensors BMP085 and LSM303*/
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  } 
  
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }  

  if(!gyro.begin(L3G4200D_SCALE_2000DPS, L3G4200D_DATARATE_100HZ_12_5))
  {
    /* There was a problem detecting the L3G4200D ... check your connections */
    Serial.print("Ooops, no L3G4200D gyroscope detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  // Calibrate gyro. This must be made at rest.

  gyro.calibrate();
  // Threshold sens. 
  gyro.setThreshold(3);
 
}

/**************************************************************************/
/*setup*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(115200);
  Serial.println(F("Adafruit 10DOF Tester")); Serial.println("");
  // Buzzer config:
  ledcSetup(channel_buzzer, freq_buzzer, resolution_buzzer);
  ledcAttachPin(13, channel_buzzer);

  /* Initialise the sensors and the display */
  initCompo();
  display.clearDisplay();
  display.setTextColor(WHITE);
}

/**************************************************************************/
/*!
    @brief  Function which reads the gravity, pitch, roll, heading and altitude.
            It shows the data on the display (due to size restrictions only
            gravity on three axis and pitch, roll and heading) and in the
            Serial monitor. 
            Baud rate is 115200.
*/
/**************************************************************************/

void SensorResults(void){

  Vector norm = gyro.readNormalize();

  /*Function which shows the data from the accelerometer on the SSD1306 screen.*/
  display.clearDisplay(); 

  /* Display the accelerometer results (acceleration is measured in m/s^2) */
  accel.getEvent(&accel_event);  // get event from accelerometer
  display.setFont(); display.setTextSize(2); display.setCursor(0,0);
  display.print("ACCEL");
  display.setTextSize(1); display.setCursor(0, 16); display.print("X: ");
  display.print(String(accel_event.acceleration.x));
  display.setTextSize(1); display.setCursor(0,26); display.print("Y: ");
  display.print(String(accel_event.acceleration.y));
  display.setTextSize(1); display.setCursor(0,36); display.print("Z: ");
  display.print(String(accel_event.acceleration.z));
  display.setTextSize(1); display.setCursor(0,46); display.print("m/s2");
  /*Serial print of the prior sensor, to see clearly the behaviour*/
#ifdef DEBUG
  Serial.print(F("X grav: "));
    Serial.print(accel_event.acceleration.x); Serial.print(F("; "));
  Serial.print(F("Y grav: "));
    Serial.print(accel_event.acceleration.y); Serial.print(F("; "));
  Serial.print(F("Z grav: "));
    Serial.print(accel_event.acceleration.z); 
    Serial.print(F(" m/s2; "));
#endif
  /* Display the pitch, roll and yaw results ( is measured in deg/s) */
  mag.getEvent(&mag_event);   // get event from magnetometer
  if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation))
  {
    display.setFont(); display.setTextSize(2); display.setCursor(70,0);
    display.print("GYRO");
    display.setTextSize(1); display.setCursor(70, 16); display.print("P: ");
    display.print(String(orientation.pitch));
    display.setTextSize(1); display.setCursor(70, 26); display.print("R: ");
    display.print(String(orientation.roll));
    display.setTextSize(1); display.setCursor(70, 36); display.print("Y: ");
    display.print(String(orientation.heading));
    display.setTextSize(1); display.setCursor(70, 46);display.print("deg");
    #ifdef DEBUG
    // Serial display of data
    Serial.print(F("Roll: "));
    Serial.print(orientation.roll); Serial.print(F("; "));
    Serial.print(F("Pitch: "));
    Serial.print(orientation.pitch); Serial.print(F("; "));
    Serial.print(F("Heading: "));
    Serial.print(orientation.heading); 
    Serial.print(F(" deg; "));
    #endif
  }

  /* Calculate the altitude using the barometric pressure sensor */
  bmp.getEvent(&bmp_event);
  if (bmp_event.pressure)
  {
    /* Get ambient temperature in C */
    float temperature;
    bmp.getTemperature(&temperature);
    /* Convert atmospheric pressure, SLP and temp to altitude    */
    #ifdef DEBUG
    Serial.print(F("Alt: "));
    Serial.print(bmp.pressureToAltitude(seaLevelPressure,
                                        bmp_event.pressure,
                                        temperature)); 
    Serial.print(F(" m; "));
    #endif
  }
  #ifdef DEBUG
  // Display the results (speed is measured in deg/s)
  Serial.print(F("Roll: "));
    Serial.print(norm.XAxis); Serial.print(F("; "));
    Serial.print(F("Pitch: "));
    Serial.print(norm.YAxis); Serial.print(F("; "));
    Serial.print(F("Heading: "));
    Serial.print(norm.ZAxis); 
    Serial.print(F(" deg/s; "));
  #endif

  // Acoustic sign of having reached the balanced position with BUZZER
  if ((accel_event.acceleration.z > 9.2) && ( abs(accel_event.acceleration.x) < 0.3) && (abs(accel_event.acceleration.y) < 0.3)) 
  {
    display.setTextSize(1); display.setCursor(0,56); display.print("BALANCED");
    ledcWriteTone(channel_buzzer, freq_buzzer);
    ledcWrite(channel_buzzer, duty_c_buzzer_on);
  } else{
          display.setTextSize(1); display.setCursor(0,56); display.print("UNBALANCED");
          ledcWrite(channel_buzzer, duty_c_buzzer_off);
        }
}

/**************************************************************************/
/*!
    @brief  Constantly check the gravity/roll/pitch/heading/altitude
*/
/**************************************************************************/
void loop()
{
  SensorResults();
  Serial.println(F(""));
  display.display(); 
  delay(200); 
}