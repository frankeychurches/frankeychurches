/*--------------------------------------------------*/
/*------------------ Librerías ---------------------*/
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
//-------DEFINES------
#include "../defines.h"

/*------------------IMU VARIABLES---------------------*/
  // Update this with the correct SLP for accurate altitude measurements 
    static float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
/*------------------ADDRESSES---------------------*/
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
Adafruit_10DOF                dof   = Adafruit_10DOF();
L3G4200D gyro;
// IMU ADDRESS
Adafruit_INA219 ina219(0x40);  //Address of the current register (04h)
// OLED SCREEN ADDRESS
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

  // Read values FROM 10DOF CALCULATIONS
    sensors_event_t accel_event;      // event ask for accelerometer data
    sensors_event_t mag_event;        // event ask for magnetometer data
    sensors_event_t bmp_event;        // event ask for barometer data
    sensors_event_t event;            // event ask for gyroscope data
    sensors_vec_t   orientation;      // orientation vector pointer
    GyroDPS gDPS;                     // gyro request for degrees per second

    //ACCELEROMETER & MAGNETOMETER
      // Variable string preparation for sending data through WiFi
      String accel_x, accel_y, accel_z;
      String fusion_pitch, fusion_roll, fusion_heading;
    //BAROMETER
    float temperature; float altitude;
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
    
    
/*------------------CNY70-TACHOMETER---------------------*/

    bool fallen_y, fallen_p, fallen_r = false;  // Flag for falling/rising edge detection
    unsigned long lastTime_y, lastTime_p, lastTime_r = 0;  // Last time a rising edge was detected
    unsigned long thisTime_y, thisTime_p, thisTime_r = 0;  // Current time
    uint16_t thisReading_y, thisReading_p, thisReading_r = 0;  // Analog reading, maximum 4095
    uint16_t RPM_p, RPM_y, RPM_r, rads_p, rads_y, rads_r, rads_p_f, rads_r_f, rads_y_f = 0;
    int16_t ang_accel_r, ang_accel_p, ang_accel_y = 0;

    //Variables needed for WiFi communication string transformation
    String rpm_p_str, rpm_r_str, rpm_y_str;
    String ang_acc_p_str, ang_acc_r_str, ang_acc_y_str;

    // timer for cny70 rpm data reading
    unsigned long startMillis, currentMillis;  
    const unsigned long period = 50;  //the value is a number of milliseconds


/*------------------INA219---------------------*/

    //Defining the variables for measurement or calculation
    float voltage_bus, current_mA, load_power  = 0;   
    String volt_str, curr_str, pow_str;


/*-------------BATTERY MEASURE VOLTAGE DIVIDER---------------*/

    uint8_t bat_smpl = 20;
    uint16_t batt_raw = 0;
    uint32_t sum_bat;
    float bat_norm, bat_perc;
    String bat_perc_str;      // Variable string preparation for sending data through WiFi
    // timer for battery voltage data reading
    unsigned long bat_timer, bat_start; 
    const unsigned long bat_period = 2000;  //the value is a number of milliseconds


/*-----------BUTTON CONFIG-----------*/
// Creation of the ISR needed for critical actions 
portMUX_TYPE MotorMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool activateMotors;
//static volatile int LED_motor;

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

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

    volt_ina { position:absolute; left:100px; top:425px;}
    curr_ina { position:absolute; left:550px; top:425px;}
    pow_ina { position:absolute; left:850px; top:425px;}

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

  <p><volt_ina>
    <i class="fas fa-tachometer-alt" style="color:#800080;"></i>
    <span class="dht-labels">BUS VOLTAGE: </span>
    <span id="ina_volt">%INA_VOLT%</span>
    <sup class="units">V</sup>
  </volt_ina></p>
  <p><curr_ina>
    <span class="dht-labels"> BUS CURRENT: </span>
    <span id="ina_curr">%INA_CURR%</span>
    <sup class="units">A</sup>
  </curr_ina></p>
  <p><pow_ina>
    <span class="dht-labels"> BUS POWER: </span>
    <span id="ina_pow">%INA_POW%</span>
    <sup class="units">W</sup>
  </pow_ina></p>
    
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
}, 5000 ) ;

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

setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("ina_volt").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/ina_volt", true);
  xhttp.send();
}, 200 ) ;
setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("ina_curr").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/ina_curr", true);
  xhttp.send();
}, 200 ) ;
setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("ina_pow").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/ina_pow", true);
  xhttp.send();
}, 200 ) ;

</script>
</html>)rawliteral";
