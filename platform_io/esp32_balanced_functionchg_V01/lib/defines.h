/*--------------------------------------------------*/
/*------------------ Librerías ---------------------*/
/*--------------------------------------------------*/
#include <Wire.h>

// For the deepsleep function
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  5        /* Time ESP32 will go to sleep (in seconds) */

/*-----  COMPILACIÓN CONDICIONAL  ------*/
//#define DEBUG TRUE    // Si esa línea no está comentada, se imprimen los mensajes por el puerto serie.
//#define CABLES_GUARRETAS TRUE  // Comment this line if you're using the PCB_GUARRETA

#ifdef CABLES_GUARRETAS 
  #define GPIO_PWM0A_OUT 12   //Declara GPIO D12 como PWM PITCH
  #define GPIO_PWM1A_OUT 26   //Declara GPIO D26 como PWM ROLL
  #define GPIO_PWM2A_OUT 25   //Declara GPIO D25 como PWM YAW
#else
  #define GPIO_PWM0A_OUT 23   //Declara GPIO D4 como PWM PITCH
  #define GPIO_PWM1A_OUT 5   //Declara GPIO D5 como PWM ROLL
  #define GPIO_PWM2A_OUT 18  //Declara GPIO D18 como PWM YAW
#endif

// LEDs
#define LED_INDICATORS 16

/*------------------SCREEN CONFIG---------------------*/
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

/*------------------CORE SELECTION FROM ESP32---------------------*/
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

/*------------------CNY70-TACHOMETER---------------------*/
#define lowLevel  2000          // Low level threshold for falling edge
#define lowLevel_R 2300
#define highLevel 3700          // High level threshold for rising edge

// Specify the pins where the analog signal from CNY70 will be read from
#define PITCH_PIN 36
#define YAW_PIN 39
#define ROLL_PIN 34

/*------------------BUZZER---------------------*/
#define freq_buzzer       2400  // maxima frecuencia para maximo volumen
#define channel_buzzer    0     // porque va por PWM 
#define resolution_buzzer 8     // no necesitamos mucha resolución porque va al máximo
#define duty_c_buzzer_on  125   // definimos el duty cycle al 50%
#define duty_c_buzzer_off 0     // apagar el buzzer

/*------------------INA219---------------------*/
#define ms_delay  500           //Setting a varible of 500 ms 

/*-------------BATTERY MEASURE VOLTAGE DIVIDER---------------*/
#define BAT_PIN 32

/*-----------BUTTON CONFIG-----------*/
// For the button communication
#define buttonPin 33
#define integrated_LED 2

/*-------------WIFI CONFIG---------------*/
// Replace with your network credentials
//#define MIWIFI TRUE
#ifdef MIWIFI  
const char* ssid = "MOVISTAR_88CD";
const char* password = "qj889fyys3B322L5jszv";
#else
const char* ssid = "IMUDS_E";
const char* password = "IMUDS_E2019*";
#endif