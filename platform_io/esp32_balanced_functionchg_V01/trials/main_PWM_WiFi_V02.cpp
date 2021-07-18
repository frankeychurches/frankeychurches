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
 *   
 *  V05:  03/06/2021
 *    
 *  
 *  V04:  20/05/2021
 *    Including the MCPWM commands, altogether with the RTOS configuration xTasks, which allow us to 
 *    compute all of the tasks simultanously. 
 *    Also the entrance of the tachometer configuration reading, as well as the battery voltage measure.
 * 
 *  V03:  02/05/2021
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
/*------------------ Librerías ---------------------*/
/*--------------------------------------------------*/
#include <Wire.h>
/*-------------------SSD1306-------------------*/
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
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

/*-----  COMPILACIÓN CONDICIONAL  ------*/
//#define DEBUG TRUE    // Si esa línea no está comentada, se imprimen los mensajes por el puerto serie.
//#define CABLES_GUARRETAS TRUE  // Comment this line if you're using the PCB_GUARRETA

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#ifdef CABLES_GUARRETAS 
  #define GPIO_PWM0A_OUT 12   //Declara GPIO D12 como PWM PITCH
  #define GPIO_PWM1A_OUT 26   //Declara GPIO D26 como PWM ROLL
  #define GPIO_PWM2A_OUT 25   //Declara GPIO D25 como PWM YAW
#else
  #define GPIO_PWM0A_OUT 23   //Declara GPIO D4 como PWM PITCH
  #define GPIO_PWM1A_OUT 5   //Declara GPIO D5 como PWM ROLL
  #define GPIO_PWM2A_OUT 18  //Declara GPIO D18 como PWM YAW
#endif

/*------------------CORE SELECTION FROM ESP32---------------------*/
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

/*------------------SENSORS ID---------------------*/
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
Adafruit_10DOF                dof   = Adafruit_10DOF();
L3G4200D gyro;

  // Update this with the correct SLP for accurate altitude measurements 
    float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

  // Read values FROM 10DOF CALCULATIONS
    sensors_event_t accel_event;      // event ask for accelerometer data
    sensors_event_t mag_event;        // event ask for magnetometer data
    sensors_event_t bmp_event;        // event ask for barometer data
    sensors_event_t event;            // event ask for gyroscope data
    sensors_vec_t   orientation;      // orientation vector pointer
    GyroDPS gDPS;                     // gyro request for degrees per second
  // TAKEN FROM ADAFRUIT 10DOF LIBRARY
 
  // Variables definition:
    //ACCELEROMETER & MAGNETOMETER
      // Variable string preparation for sending data through WiFi
      static String accel_x, accel_y, accel_z;
      static String fusion_pitch, fusion_roll, fusion_heading;
    //BAROMETER
    static float temperature; static float altitude;
      // Variable string preparation for sending data through WiFi
      static String temp_str, alt_str;
    //GYROSCOPE
    static double xi,yi,zi;    // Variables for integration
    static unsigned long cT;   // Current time
    static unsigned long pT;   // Previous time
    static unsigned long dT;   // Discrete time difference 
    double angleF_roll; double angleF_pitch;   //Fusioned angles
      // Variable string preparation for sending data through WiFi:
      static String gyro_x, gyro_y, gyro_z; 
      static String filter_roll, filter_pitch;

/*------------------CNY70-TACHOMETER---------------------*/
#define lowLevel  2000          // Low level threshold for falling edge
#define lowLevel_R 2300
#define highLevel 3700          // High level threshold for rising edge

static bool fallen_y, fallen_p, fallen_r = false;  // Flag for falling/rising edge detection
static unsigned long lastTime_y, lastTime_p, lastTime_r = 0;  // Last time a rising edge was detected
static unsigned long thisTime_y, thisTime_p, thisTime_r = 0;  // Current time
static uint16_t thisReading_y, thisReading_p, thisReading_r = 0;  // Analog reading, maximum 4095
static uint16_t RPM_p, RPM_y, RPM_r, rads_p, rads_y, rads_r, rads_p_f, rads_r_f, rads_y_f = 0;
static int16_t ang_accel_r, ang_accel_p, ang_accel_y = 0;

//Variables needed for WiFi communication string transformation
static String rpm_p_str, rpm_r_str, rpm_y_str;
static String ang_acc_p_str, ang_acc_r_str, ang_acc_y_str;

// Specify the pins where the analog signal from CNY70 will be read from
#define PITCH_PIN 36
#define YAW_PIN 39
#define ROLL_PIN 34

// timer for cny70 rpm data reading
static unsigned long startMillis, currentMillis;  
static const unsigned long period = 50;  //the value is a number of milliseconds

/*------------------BUZZER---------------------*/
#define freq_buzzer       2400  // maxima frecuencia para maximo volumen
#define channel_buzzer    0     // porque va por PWM 
#define resolution_buzzer 8     // no necesitamos mucha resolución porque va al máximo
#define duty_c_buzzer_on  125   // definimos el duty cycle al 50%
#define duty_c_buzzer_off 0     // apagar el buzzer

/*-------------BATTERY MEASURE VOLTAGE DIVIDER---------------*/
#define BAT_PIN 32

static uint8_t bat_smpl = 20;
static uint16_t batt_raw = 0;
static uint32_t sum_bat;
static float bat_norm, bat_perc;
static String bat_perc_str;      // Variable string preparation for sending data through WiFi

// timer for battery voltage data reading
static unsigned long bat_timer, bat_start; 
static const unsigned long bat_period = 2000;  //the value is a number of milliseconds

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
// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// For the button communication
#define buttonPin 33
#define integrated_LED 2
const char* BUTTON_INPUT = "state";

static bool activateMotors = false;
static int button_read, button_state;
static int LED_motor, lastButtonState = LOW;
// In order to avoid overflow, unsigned long is the best option
static unsigned long lastDebounceTime = 0;
static unsigned long debounceDelay = 70;


// Definition, design and communication configuration between server
// and ESP32 data reading
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <link rel="stylesheet" href="https://use.fontawesome.com/releases/v5.7.2/css/all.css" integrity="sha384-fnmOCqbTlWIlj8LyTjo7mOUStjsKC4pOpQbqyi7RrhN7udi9RwhKkMHpvLbHG9Sr" crossorigin="anonymous">
  <style>
    html {
     font-family: Arial;
     display: inline-block;
     margin: 0px auto;
     text-align: center;
    }
    h2 { font-size: 2.0rem; }
    p { font-size: 2.0rem; }


    temp { position:absolute; left:100px; top:100px;}
    bat { position:absolute; left:800px; top:100px;}

    imu_fusion_x { position:absolute; left:100px; top:175px;}
    imu_fusion_y { position:absolute; left:550px; top:175px;}
    imu_fusion_z { position:absolute; left:800px; top:175px;}

    imu_gyro_x { position:absolute; left:100px; top:225px;}
    imu_gyro_y { position:absolute; left:550px; top:225px;}
    imu_gyro_z { position:absolute; left:800px; top:225px;}

    filter_imu_r { position:absolute; left:100px; top:275px;}
    filter_imu_p { position:absolute; left:600px; top:275px;}

    cny_rpm_r { position:absolute; left:100px; top:325px;}
    cny_rpm_p { position:absolute; left:550px; top:325px;}
    cny_rpm_y { position:absolute; left:850px; top:325px;}

    cny_ang_r { position:absolute; left:100px; top:375px;}
    cny_ang_p { position:absolute; left:550px; top:375px;}
    cny_ang_y { position:absolute; left:850px; top:375px;}

    button { position:absolute; left:100px; top:425px;}

    .switch {position: relative; display: inline-block; width: 120px; height: 68px} 
    .switch input {display: none}
    .slider {position: absolute; top: 0; left: 0; right: 0; bottom: 0; background-color: #ccc; border-radius: 34px}
    .slider:before {position: absolute; content: ""; height: 52px; width: 52px; left: 8px; bottom: 8px; background-color: #fff; -webkit-transition: .4s; transition: .4s; border-radius: 68px}
    input:checked+.slider {background-color: #2196F3}
    input:checked+.slider:before {-webkit-transform: translateX(52px); -ms-transform: translateX(52px); transform: translateX(52px)}

    .units { font-size: 1.2rem; }
    .dht-labels{
      font-size: 1.5rem;
      vertical-align:middle;
      padding-bottom: 15px;
    }
  </style>
</head>
<body>
  <h2>REACTION WHEEL ESP32 MANAGEMENT INTERFACE</h2>

  <p><temp>
    <i class="fas fa-thermometer-half" style="color:#059e8a;"></i> 
    <span class="dht-labels">Temperature: </span> 
    <span id="temperature">%TEMPERATURE%</span>
    <sup class="units">&deg;C</sup>
  </temp></p>
  <p>
    <i class="fas fa-mountain" style="color:#ff0000;"></i>
    <span class="dht-labels">Altitude: </span>
    <span id="altitude">%ALTITUDE%</span>
    <span class="dht-labels">meters</span>
  </p>
  <p><bat>
    <i class="fas fa-battery-full" style="color:#000000;"></i> 
    <span class="dht-labels">Battery (V)</span>
    <span id="battery">%BATTERY%</span>
    <sup class="units">&percnt;</sup>
  </bat></p>

  <p><imu_fusion_x>
    <i class="fas fa-compass" style="color:#0000ff;"></i> 
    <span class="dht-labels">ORIENTATION X: </span>
    <span id="fusion_x">%FUSION_X%</span>
    <sup class="units">rad</sup>
  </imu_fusion_x></p>
  <p><imu_fusion_y>
    <span class="dht-labels"> Y: </span>
    <span id="fusion_y">%FUSION_Y%</span>
    <sup class="units">rad</sup>
  </imu_fusion_y></p>
  <p><imu_fusion_z>
    <span class="dht-labels"> Z: </span>
    <span id="fusion_z">%FUSION_Z%</span>
    <sup class="units">rad</sup>
  </imu_fusion_z></p>

  <p><imu_gyro_x>
    <i class="fas fa-compass" style="color:#00ff00;"></i> 
    <span class="dht-labels">GYROSCOPE ROLL: </span>
    <span id="gyro_x">%GYRO_X%</span>
    <sup class="units">rad</sup>
  </imu_gyro_x></p>
  <p><imu_gyro_y>
    <span class="dht-labels"> PITCH: </span>
    <span id="gyro_y">%GYRO_Y%</span>
    <sup class="units">rad</sup>
  </imu_gyro_y></p>
  <p><imu_gyro_z>
    <span class="dht-labels"> YAW: </span>
    <span id="gyro_z">%GYRO_Z%</span>
    <sup class="units">rad</sup>
  </imu_gyro_z></p>

  <p><filter_imu_r>
    <i class="fas fa-exclamation-triangle" style="color:#ff0000;"></i>
    <span class="dht-labels">ACCU. ERR. ROLL:  </span>
    <span id="filter_roll">%FILTER_ROLL%</span>
    <sup class="units">rad</sup>
  </filter_imu_r></p>
  <p><filter_imu_p>
    <span class="dht-labels"> PITCH: </span>
    <span id="filter_pitch">%FILTER_PITCH%</span>
    <sup class="units">rad</sup>
  </filter_imu_p></p>

  <p><cny_rpm_r>
    <i class="fas fa-tachometer-alt" style="color:#00ff00;"></i>
    <span class="dht-labels">SPEED ROLL: </span>
    <span id="rpm_roll">%RPM_R%</span>
    <sup class="units">rpm</sup>
  </cny_rpm_r></p>
  <p><cny_rpm_p>
    <span class="dht-labels"> PITCH: </span>
    <span id="rpm_pitch">%RPM_P%</span>
    <sup class="units">rpm</sup>
  </cny_rpm_p></p>
  <p><cny_rpm_y>
    <span class="dht-labels"> YAW: </span>
    <span id="rpm_yaw">%RPM_Y%</span>
    <sup class="units">rpm</sup>
  </cny_rpm_y></p>

  <p><cny_ang_r>
    <i class="fas fa-tachometer-alt" style="color:#800080;"></i>
    <span class="dht-labels">SPEED ROLL: </span>
    <span id="ang_acc_r">%ANGACC_R%</span>
    <sup class="units">rad/s2</sup>
  </cny_ang_r></p>
  <p><cny_ang_p>
    <span class="dht-labels"> PITCH: </span>
    <span id="ang_acc_p">%ANGACC_P%</span>
    <sup class="units">rad/s2</sup>
  </cny_ang_p></p>
  <p><cny_ang_y>
    <span class="dht-labels"> YAW: </span>
    <span id="ang_acc_y">%ANGACC_Y%</span>
    <sup class="units">rad/s2</sup>
  </cny_ang_y></p>
    
    <p><button>
    %BUTTONPLACEHOLDER%
    </button></p>
</body>


<script>
setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("temperature").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/temperature", true);
  xhttp.send();
}, 5000 ) ;

setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("altitude").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/altitude", true);
  xhttp.send();
}, 8000 ) ;

setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("battery").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/battery", true);
  xhttp.send();
}, 2000 ) ;

setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("fusion_x").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/fusion_x", true);
  xhttp.send();
}, 100 ) ;
setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("fusion_y").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/fusion_y", true);
  xhttp.send();
}, 100 ) ;
setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("fusion_z").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/fusion_z", true);
  xhttp.send();
}, 100 ) ;

setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("gyro_x").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/gyro_x", true);
  xhttp.send();
}, 100 ) ;
setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("gyro_y").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/gyro_y", true);
  xhttp.send();
}, 100 ) ;
setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("gyro_z").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/gyro_z", true);
  xhttp.send();
}, 100 ) ;

setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("filter_roll").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/filter_roll", true);
  xhttp.send();
}, 100 ) ;
setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("filter_pitch").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/filter_pitch", true);
  xhttp.send();
}, 100 ) ;

setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("rpm_roll").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/rpm_roll", true);
  xhttp.send();
}, 100 ) ;
setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("rpm_pitch").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/rpm_pitch", true);
  xhttp.send();
}, 100 ) ;
setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("rpm_yaw").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/rpm_yaw", true);
  xhttp.send();
}, 100 ) ;

setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("ang_acc_r").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/ang_acc_r", true);
  xhttp.send();
}, 100 ) ;
setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("ang_acc_p").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/ang_acc_p", true);
  xhttp.send();
}, 100 ) ;
setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("ang_acc_y").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/ang_acc_y", true);
  xhttp.send();
}, 100 ) ;

function toggleCheckbox(element) {
  var xhr = new XMLHttpRequest();
  if(element.checked){ xhr.open("GET", "/update?state=1", true); }
  else { xhr.open("GET", "/update?state=0", true); }
  xhr.send();
}
setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      var inputChecked;
      var outputStateM;
      if( this.responseText == 1){ 
        inputChecked = true;
        outputStateM = "On";
      }
      else { 
        inputChecked = false;
        outputStateM = "Off";
      }
      document.getElementById("integrated_LED").checked = inputChecked;
      document.getElementById("outputState").innerHTML = outputStateM;
    }
  };
  xhttp.open("GET", "/state", true);
  xhttp.send();
}, 1000 ) ;

</script>
</html>)rawliteral";

/**************************************************************************/
/*!
    @brief  Initialises all the sensors and the OLED used by the system.
*/
/**************************************************************************/

static void initCompo()
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
}

/**************************************************************************/
/*!
    @brief  MCPWM setup configuration for the motor velocity management.
*/
/**************************************************************************/

static void MCPWM_setup(){
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

/**************************************************************************/
/*!
    @brief  Function which configures the operator's A MCPWM (Unit, Timer, Percentage)
            Duty Cycle.
*/
/**************************************************************************/

static void brushed_motor_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
  mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
}

/**************************************************************************/
/*!
    @brief  Battery measurement system designed with a voltage divider.
            A series resistance with a low pass filter (R, R//C) allow usto determine
            the remain voltage of the battery, thereby knowing how much
            battery we have left before causing a deep discharge of the battery.  
*/
/**************************************************************************/

void BatteryMeasure(void* parameters)
{
  while(1)
  {
    batt_raw = analogRead(BAT_PIN);  //max value 4095
    sum_bat = 0;  
    for (uint8_t i = 0; i < bat_smpl; ++i)
    {
      sum_bat += batt_raw; //max sum value 4095*100 = 409500
    }
    bat_norm = 3.818181 * (3.3/4095) * (sum_bat/bat_smpl);   // real voltage battery value.
    bat_perc = ((12.6-bat_norm)/3) * 100; bat_perc_str = String(bat_perc);
    bat_timer = millis();  //get the current "time"
    if (bat_timer - bat_start >= bat_period)
    {
      #ifdef DEBUG
      Serial.print(F("Battery Voltage: ")); Serial.print(bat_norm); Serial.print(F(" % ; "));
      Serial.print(F(" Battery Voltage Percentage: ")); Serial.print(bat_perc, 2); Serial.println(F(" % "));
      #endif
      bat_start = bat_timer;  // Save the start time of the current state.
    }
  }
}

/**************************************************************************/
/*!
    @brief  Function which reads the value from the button to see if we can
            activate the motors.
            This value will be communicated to our WiFi interface.
*/
/**************************************************************************/

void ButtonMotor(void* parameters)
{
  while(1)
  {
    // Check to see the value of the button
    button_read = digitalRead(buttonPin);
    // Depending on the change, it can be LOW to HIGH o HIGH to LOW, that's why we use change:
    if (button_read != lastButtonState) {
        lastDebounceTime = millis();   // update the value of debounce from button
    }
    // Once the debounce delay has passed (consider 50 ms)
    if ((millis() - lastDebounceTime) > debounceDelay) {
      if (button_read != button_state) {    // if the value read is different from the 
        button_state = button_read;         // former state value of the button, update it.
        if (button_state == HIGH) {         // If it's HIGH, change the state of the variable
            activateMotors == true;   // For the motors, TRUE or FALSE
            LED_motor = !LED_motor;             // For the LED indicator, HIGH or LOW
        }
      }
    }
  // set the LED:
  digitalWrite(integrated_LED, LED_motor);
  lastButtonState = button_read;  // in order to compare if it has changed in the next iteration 
  vTaskDelay(100/ portTICK_PERIOD_MS);
  }
}

/**************************************************************************/
/*!
    @brief  Function which manages the PWM sending to the motors
            Duty Cycle.
*/
/**************************************************************************/

void MotorMovement(void* parameters)
{
  while(1)
  {
    // vTaskDelay(2000 / portTICK_PERIOD_MS);
    if(activateMotors == true){
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
      // Change the state variable so it does not repeat until told by the button
      activateMotors == false;
    }
  }
}

/**************************************************************************/
/*!
    @brief  Tachometer function. With the CNY70 and this program below we are
            able to read the analog signal received as an RPM signal.

            Since there is only one rising edge per rotation on this
            tachometer, and only the rising edges are being used in the RPM
            calculation, the elapsed time since the last reading is equal to the
            revolutions per microsecond.
*/
/**************************************************************************/

static void Tachometer_P(void* parameter)
{
  while(1)
  {
    thisReading_p = analogRead(PITCH_PIN);    // Take a reading check to see
    if (thisReading_p > highLevel)            // if above  the high level 
    {                                         // threshold, if so, it was low on
      if (fallen_p == true)                   // previous pass which means a
      {                                       // rising edge was detected
        thisTime_p = micros();
        // 1 rev/ usec * 1e6 usec/1 sec *60 sec/1 min == [revs/min] == RPM
        RPM_p = 60000000 / (thisTime_p - lastTime_p);       // RPM = Revolutions per minute
        // rad/s because we need angular acceleration for our torque design response
        // 1 rev/ min * 1 min/60 sec * 2pi rad/1 rev == [rad/sec] == w
        rads_p = (RPM_p*2*PI) / 60;  
        // Angular acceleration is dw/dt which can be discretised into w2-w1/t2-t1:
          ang_accel_p = (rads_p - rads_p_f) / (thisTime_p - lastTime_p);
          rpm_p_str = String(RPM_p); ang_acc_p_str = String(ang_accel_p); 
          rads_p_f = rads_p;
        lastTime_p = thisTime_p;               // Set the time to "last time" and
        fallen_p = false;                      // the edge to false so we won't count this rise again
      }
    }
    if (thisReading_p < lowLevel)              // When the edge first falls below the 
    {                                          // low level threshold, make fallen true                          
      if (fallen_p == false)                        
      {
        fallen_p = true;
      }
    }
  }
}

static void Tachometer_Y(void* parameter)
{
  while(1)
  {
    thisReading_y = analogRead(YAW_PIN);   
    if (thisReading_y > highLevel)            
    {                                          
      if (fallen_y == true)                     
      {                                               
        thisTime_y = micros();
        RPM_y = 60000000 / (thisTime_y - lastTime_y); rads_y = (RPM_y*2*PI) / 60;
          ang_accel_y = (rads_y - rads_y_f) / (thisTime_y - lastTime_y);
          rpm_y_str = String(RPM_y); ang_acc_y_str = String(ang_accel_y); 
          rads_y_f = rads_y;
        lastTime_y = thisTime_y;                 
        fallen_y = false;                                                          
      }
    }
    if (thisReading_y < lowLevel)                
    {                                                             
      if (fallen_y == false)                        
      {
        fallen_y = true;
      }
    }
  }
}

static void Tachometer_R(void* parameter)
{
  while(1)
  {
    thisReading_r = analogRead(ROLL_PIN);   
    if (thisReading_r > highLevel )            
    {                                      
      if (fallen_r == true)                 
      {                                                                       
        thisTime_r = micros();
        RPM_r = 60000000 / (thisTime_r - lastTime_r); rads_r = (RPM_r*2*PI) / 60;
          ang_accel_r = (rads_r - rads_r_f) / (thisTime_r - lastTime_r);
          rpm_r_str = String(RPM_r); ang_acc_r_str = String(ang_accel_r); 
          rads_r_f = rads_r;
        lastTime_r = thisTime_r;                    
        fallen_r = false;                                                        
      }
    }
    if (thisReading_r < lowLevel)                
    {                                                                
      if (fallen_r == false)                        
      {
        fallen_r = true;
      }
    }
  }
}

/* static void RPM_data(void* parameters)
{
  while(1)
  {
    currentMillis = millis();  //get the current "time"
    if (currentMillis - startMillis >= period)
    {
      #ifdef DEBUG
      Serial.print(F("RPM Pitch: ")); Serial.print(RPM_p); Serial.print(F("  ,"));
      Serial.print(F("rad/s: ")); Serial.print(ang_accel_p,2); Serial.print(F("  ;"));
      Serial.print(F("RPM Yaw: ")); Serial.print(RPM_y); Serial.print(F("  ,"));
      Serial.print(F("rad/s: ")); Serial.print(ang_accel_y,2); Serial.print(F("  ;"));
      Serial.print(F("RPM Roll: ")); Serial.print(RPM_r); Serial.print(F("  ,")); 
      Serial.print(F("rad/s: ")); Serial.print(ang_accel_r); Serial.print(F("  ;"));
      Serial.println(F(" "));
      #endif
      startMillis = currentMillis;  // Save the start time of the current state.
    }
  }
} */

/**************************************************************************/
/*!
    @brief  Function which reads the gravity, pitch, roll, heading and altitude.
            It shows the data on the display (due to size restrictions only
            gravity on three axis and pitch, roll and heading) and in the
            Serial monitor. 
            Baud rate is 115200.
*/
/**************************************************************************/

static void SensorResults(void * parameters)
{
  while(1)
  {
    display.clearDisplay();        // Clear former data shown in OLED
    accel.getEvent(&accel_event);  // Get event from accelerometer
    mag.getEvent(&mag_event);      // Get event from magnetometer
    accel_x = String(accel_event.acceleration.x);
    accel_y  = String(accel_event.acceleration.y);
    accel_z = String(accel_event.acceleration.z);
    /*------------------ACCELEROMETER---------------------*/
    // Display the accelerometer results (acceleration is measured in m/s^2)
      display.setFont(); display.setTextSize(2); display.setCursor(0,0);
      display.print("ACCEL");
      display.setTextSize(1); display.setCursor(0, 16); display.print("X: ");
      display.print(accel_x);
      display.setTextSize(1); display.setCursor(0,26); display.print("Y: ");
      display.print(accel_y);
      display.setTextSize(1); display.setCursor(0,36); display.print("Z: ");
      display.print(accel_z);
      display.setTextSize(1); display.setCursor(0,46); display.print("m/s2");
    /*Serial print of the prior sensor, to see clearly the behaviour*/
  #ifdef DEBUG
    Serial.print(F("X grav: ")); 
    Serial.print(accel_event.acceleration.x); Serial.print(F("; "));
    Serial.print(F("Y grav: ")); 
    Serial.print(accel_event.acceleration.y); Serial.print(F("; "));
    Serial.print(F("Z grav: "));
    Serial.print(accel_event.acceleration.z); Serial.print(F(" m/s2; "));
  #endif

    /*------------------GYROSCOPE---------------------*/
    // Measurements from gyroscope in DPS (degrees per second)
    gDPS = gyro.readGyroDPS();
    gyro_x = String(gDPS.x); gyro_y = String(gDPS.y); gyro_z = String(gDPS.z);
    /* 
      Measurements from gyroscope integrating dps -> distance
      This distance is the sum (or substract, depending on the 
      twist direction) of the angles tilted 
      from the original position. 
      dT represents the discrete time differentiation.
    */
    cT = micros(); dT = cT - pT; pT = cT;
    xi = xi + gDPS.x*(dT/1000000.0);
    yi = yi + gDPS.y*(dT/1000000.0);
    zi = zi + gDPS.z*(dT/1000000.0);
  #ifdef DEBUG
    // Print gyroscope degrees per second values
    Serial.print(F("X dps: ")); Serial.print(gyro_x); Serial.print(F("  ; "));
    Serial.print(F("Y dps: ")); Serial.print(gyro_y); Serial.print(F("  ; "));
    Serial.print(F("Z dps: ")); Serial.print(gyro_z); Serial.print(F("  deg/sec; "));
    // Print gyroscope integral of previous values, to see if there's a lot of noise or not
    Serial.print(F("Xi: ")); Serial.print(xi); Serial.print(F("; "));
    Serial.print(F("Yi: ")); Serial.print(yi); Serial.print(F("; "));
    Serial.print(F("Zi: ")); Serial.print(zi); Serial.print(F(" sum; "));
  #endif

    /*------------------ORIENTATION---------------------*/
    if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation))
      { 
      fusion_pitch = String(orientation.pitch);
      fusion_roll  = String(orientation.roll);
      fusion_heading = String(orientation.heading);
      // Display the orientation, which is the pitch, roll and yaw in degrees
      display.setFont(); display.setTextSize(2); display.setCursor(70,0);
      display.print("ORTN");
      display.setTextSize(1); display.setCursor(70, 16); display.print("P: ");
      display.print(fusion_pitch);
      display.setTextSize(1); display.setCursor(70, 26); display.print("R: ");
      display.print(fusion_roll);
      display.setTextSize(1); display.setCursor(70, 36); display.print("Y: ");
      display.print(fusion_heading);
      display.setTextSize(1); display.setCursor(70, 46);display.print("deg");
      /*
        This next expression computes the difference between the accelerometer 
        and the gyroscope, showing how much error has been accumulating
        since the beginning of the data collection.
      */
      angleF_roll = 0.95*(angleF_roll + gDPS.x*(dT/1000000.0)) + 0.05*orientation.roll; 
      angleF_pitch = 0.95*(angleF_pitch + gDPS.y*(dT/1000000.0)) + 0.05*orientation.pitch;
      filter_roll = String(angleF_roll); filter_pitch = String(angleF_pitch);

    #ifdef DEBUG
      // Serial display of data
      Serial.print(F("Roll: ")); Serial.print(orientation.roll); Serial.print(F("; "));
      Serial.print(F("Pitch: ")); Serial.print(orientation.pitch); Serial.print(F("; "));
      Serial.print(F("Heading: ")); Serial.print(orientation.heading); 
      Serial.print(F(" deg; "));
      Serial.print(F("Roll fus: ")); Serial.print(angleF_roll); Serial.print(F("; "));
      Serial.print(F("Pitch fus: ")); Serial.print(angleF_pitch); Serial.print(F("; "));
    #endif
      }

    /*----------------ALTITUDE & TEMPERATURE-----------------------*/
    /* Calculate the altitude using the barometric pressure sensor */
    bmp.getEvent(&bmp_event);
    if (bmp_event.pressure)
    {
      /* Get ambient temperature in C */
      bmp.getTemperature(&temperature);
      temp_str = String(temperature);
      /* Convert atmospheric pressure, SLP and temp to altitude    */
      altitude = bmp.pressureToAltitude(seaLevelPressure, bmp_event.pressure, temperature);
      alt_str = String(altitude);
      display.setTextSize(1); display.setCursor(70,56); display.print("ALT:");
      display.print(altitude);

      #ifdef DEBUG
      Serial.print(F("Alt: ")); Serial.print(altitude); Serial.print(F(" m; "));
      Serial.print(F("Temp: ")); Serial.print(temperature); Serial.print(F(" m; "));
      #endif
    }

    // Acoustic sign of having reached the balanced position with BUZZER
    /* if ((accel_event.acceleration.z > 9.2) && ( abs(accel_event.acceleration.x) < 0.3) && (abs(accel_event.acceleration.y) < 0.3)) 
    {
      display.setTextSize(1); display.setCursor(0,56); display.print("BALANCED");
      ledcWriteTone(channel_buzzer, freq_buzzer);
      ledcWrite(channel_buzzer, duty_c_buzzer_on);
    } else{
            display.setTextSize(1); display.setCursor(0,56); display.print("UNBALANCED");
            ledcWrite(channel_buzzer, duty_c_buzzer_off);
          } */
    
    display.display(); 
    vTaskDelay(300 / portTICK_PERIOD_MS);
  }
}

String outputState(){
  if(digitalRead(integrated_LED)){
    return "checked";
  }
  else {
    return "";
  }
  return "";
}

/**************************************************************************/
/*!
    @brief  Processor function creates the gateway between our main program
            data and the HTML variable where it will be saved and afterwards
            sent to our server.

*/
/**************************************************************************/
String processor(const String& var){
  //Serial.println(var);
  if(var == "BUTTONPLACEHOLDER"){
    String buttons ="";
    String outputStateValue = outputState();
    buttons+= "<h4>Output - GPIO 2 - State <span id=\"outputState\"></span></h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"toggleCheckbox(this)\" id=\"output\" " + outputStateValue + "><span class=\"slider\"></span></label>";
    return buttons;
  }
  else if(var == "TEMPERATURE"){ return temp_str;}
  else if(var == "ALTITUDE"){ return alt_str;}
  else if(var == "BATTERY"){ return bat_perc_str;}
  
  else if(var == "FUSION_X"){ return fusion_roll;}
  else if(var == "FUSION_Y"){ return fusion_pitch ;}
  else if(var == "FUSION_Z"){ return fusion_heading;}
  else if(var == "GYRO_X"){ return gyro_x;}
  else if(var == "GYRO_Y"){ return gyro_y; }
  else if(var == "GYRO_Z"){ return gyro_z;}
  else if(var == "FILTER_ROLL"){ return filter_roll;}
  else if(var == "FILTER_PITCH"){ return filter_pitch;}

  else if(var == "RPM_R"){ return rpm_r_str;}
  else if(var == "RPM_P"){ return rpm_p_str;}
  else if(var == "RPM_Y"){ return rpm_y_str;}

  else if(var == "ANGACC_R"){ return ang_acc_r_str;}
  else if(var == "ANGACC_P"){ return ang_acc_p_str;}
  else if(var == "ANGACC_Y"){ return ang_acc_y_str;}

  return String();
}

void WiFi_HTML_setup()
{
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  // Print ESP32 Local IP Address
  Serial.println(WiFi.localIP());

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });
  // Request to get temperature and altitude from the barometer
  server.on("/temperature", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", temp_str.c_str());
  });
  server.on("/altitude", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", alt_str.c_str());
  });
  // Request to get the battery voltage level in percentage
  server.on("/battery", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", bat_perc_str.c_str());
  });
  // Request to get the fusion orientation from the IMU
  server.on("/fusion_x", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", fusion_roll.c_str());
  });
  server.on("/fusion_y", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", fusion_pitch.c_str());
  });
  server.on("/fusion_z", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", fusion_heading.c_str());
  });
  // Request to get the gyro information
  server.on("/gyro_x", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", gyro_x.c_str());
  });
  server.on("/gyro_y", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", gyro_y.c_str());
  });
  server.on("/gyro_z", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", gyro_z.c_str());
  });
  // Request to get the filtered signal obtained with the accelerometer and the gyroscope
  server.on("/filter_roll", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", filter_roll.c_str());
  });
  server.on("/filter_pitch", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", filter_pitch.c_str());
  });
  // Request to get the RPM value from the inertial disks
  server.on("/rpm_roll", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", rpm_r_str.c_str());
  });
  server.on("/rpm_pitch", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", rpm_p_str.c_str());
  });
  server.on("/rpm_yaw", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", rpm_y_str.c_str());
  });
  // Request to get the angular acceleration from the inertial disks
  server.on("/ang_acc_r", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", ang_acc_r_str.c_str());
  });
  server.on("/ang_acc_p", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", ang_acc_p_str.c_str());
  });
  server.on("/ang_acc_y", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", ang_acc_y_str.c_str());
  });

  /*-----FOR THE BUTTON (PHYSYCAL & HTML INTERFACE) COMMUNICATION------*/
  // Send a GET request to <ESP_IP>/update?state=<inputMessage>
  server.on("/update", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    String inputParam;
    // GET input1 value on <ESP_IP>/update?state=<inputMessage>
    if (request->hasParam(BUTTON_INPUT)) {
      inputMessage = request->getParam(BUTTON_INPUT)->value();
      inputParam = BUTTON_INPUT;
      digitalWrite(integrated_LED, inputMessage.toInt());
      LED_motor = !LED_motor; 
    }
    else {
      inputMessage = "No message sent";
      inputParam = "none";
    }
    Serial.println(inputMessage);
    request->send(200, "text/plain", "OK");
  });

  // Send a GET request to <ESP_IP>/state
  server.on("/state", HTTP_GET, [] (AsyncWebServerRequest *request) {
    request->send(200, "text/plain", String(digitalRead(integrated_LED)).c_str());
  });

  // Start server
  server.begin();
}

/**************************************************************************/
/*setup*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(115200); Wire.begin();
  // LED indication for motors' startup 
  pinMode(integrated_LED, OUTPUT); digitalWrite(integrated_LED, LOW);
  // Button configuration setup
  pinMode(buttonPin, INPUT);
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
    MCPWM_setup();

  // Initialise the sensors and the display function:
    initCompo();
  // Gyro setup code:
    gyro.init(-120.1, -563.64724, -0.48845); 
    pT = 0;
    xi = yi = zi = 0;
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

  // Start Motor PWM signal creation task:
  xTaskCreatePinnedToCore( MotorMovement,"Motor PWM signal",102400, NULL,1, NULL,app_cpu); 
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
  // LED and Motor control WIfi:
  xTaskCreatePinnedToCore( ButtonMotor,"LED and Motor Controll through WIFI",10240, NULL,1, NULL,app_cpu); 

  // Delete "setup and loop" task
  vTaskDelete(NULL);
}

void loop()
{
  // nothing here
}