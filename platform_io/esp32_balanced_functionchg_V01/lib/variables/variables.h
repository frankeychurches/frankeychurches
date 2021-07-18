/*--------------------------------------------------*/
/*------------------ Librerías ---------------------*/
/*--------------------------------------------------*/
#include <Arduino.h>
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

// Deep Sleep variables



/*------------------IMU VARIABLES---------------------*/
    // Update this with the correct SLP for accurate altitude measurements 
    extern float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
    // Addresses
    extern Adafruit_LSM303_Accel_Unified accel ;
    extern Adafruit_LSM303_Mag_Unified   mag   ;
    extern Adafruit_BMP085_Unified       bmp   ;
    extern Adafruit_10DOF                dof   ;
    extern L3G4200D gyro;
    // INA219 : Address of the current register (04h)
    extern Adafruit_INA219 ina219;  
    // OLED SCREEN ADDRESS
    Adafruit_SSD1306 display;

    // Read values FROM 10DOF CALCULATIONS
    extern sensors_event_t accel_event;      // event ask for accelerometer data
    extern sensors_event_t mag_event;        // event ask for magnetometer data
    extern sensors_event_t bmp_event;        // event ask for barometer data
    extern sensors_event_t event;            // event ask for gyroscope data
    extern sensors_vec_t   orientation;      // orientation vector pointer
    extern GyroDPS gDPS;                     // gyro request for degrees per second
    
    // Struct with the necessary variables
    struct IMU_MEMS {
    //ACCELEROMETER & MAGNETOMETER
      // Variable string preparation for sending data through WiFi
      String accel_x, accel_y, accel_z;
      String fusion_pitch, fusion_roll, fusion_heading;
    //BAROMETER
    float temperature, altitude;
      // Variable string preparation for sending data through WiFi
      String temp_str, alt_str;
    //GYROSCOPE
    double xi,yi,zi;    // Variables for integration
    unsigned long cT;   // Current time
    unsigned long pT;   // Previous time
    unsigned long dT;   // Discrete time difference 
    double angleF_roll; double angleF_pitch;   //Fusioned angles
      // Variable string preparation for sending data through WiFi:
      String gyro_x, gyro_y, gyro_z; 
      String filter_roll, filter_pitch;
    };
    IMU_MEMS imu;
/*------------------CNY70-TACHOMETER---------------------*/
  struct CNY70_SENSOR{
    bool fallen_y, fallen_p, fallen_r;  // Flag for falling/rising edge detection
    unsigned long lastTime_y, lastTime_p, lastTime_r;  // Last time a rising edge was detected
    unsigned long thisTime_y, thisTime_p, thisTime_r;  // Current time
    uint16_t thisReading_y, thisReading_p, thisReading_r;  // Analog reading, maximum 4095
    uint16_t RPM_p, RPM_y, RPM_r, rads_p, rads_y, rads_r, rads_p_f, rads_r_f, rads_y_f;
    int16_t ang_accel_r, ang_accel_p, ang_accel_y;


    //Variables needed for WiFi communication string transformation
    String rpm_p_str, rpm_r_str, rpm_y_str;
    String ang_acc_p_str, ang_acc_r_str, ang_acc_y_str;

    // Timer for cny70 rpm data reading
    unsigned long startMillis, currentMillis;  
    unsigned long period;  //the value is a number of milliseconds
  };
  CNY70_SENSOR cny70;
/*------------------INA219---------------------*/
//Defining the variables for measurement or calculation
  struct INA219 {
    float voltage_bus, current_mA, load_power;   
    String volt_str, curr_str, pow_str;
  };
  INA219 ina;
/*-------------BATTERY MEASURE VOLTAGE DIVIDER---------------*/
    struct BATTERY_VOLTAGE_MEASURE {
        uint8_t bat_smpl;
        uint16_t batt_raw;
        uint32_t sum_bat;
        float bat_norm, bat_perc;
        String bat_perc_str;      // Variable string preparation for sending data through WiFi
        // timer for battery voltage data reading
        unsigned long bat_timer, bat_start; 
        unsigned long bat_period;  //the value is a number of milliseconds
    };
    BATTERY_VOLTAGE_MEASURE bvm;

/*-----------BUTTON CONFIG-----------*/
// Creation of the ISR needed for critical actions 
extern portMUX_TYPE MotorMux = portMUX_INITIALIZER_UNLOCKED;
extern volatile bool activateMotors;

/*-----------WiFi-----------*/
// Create AsyncWebServer object on port 80
extern AsyncWebServer server(80);
// Definition, design and communication configuration between server
extern const char index_html[];