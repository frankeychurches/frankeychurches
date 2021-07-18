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

#ifdef CABLES_GUARRETAS 
  #define GPIO_PWM0A_OUT 12   //Declara GPIO D12 como PWM PITCH
  #define GPIO_PWM1A_OUT 26   //Declara GPIO D26 como PWM ROLL
  #define GPIO_PWM2A_OUT 25   //Declara GPIO D25 como PWM YAW
  #define PITCH_PIN 32
#else
  #define GPIO_PWM0A_OUT 4   //Declara GPIO D4 como PWM PITCH
  #define GPIO_PWM1A_OUT 5   //Declara GPIO D5 como PWM ROLL
  #define GPIO_PWM2A_OUT 18  //Declara GPIO D18 como PWM YAW
  // #define PITCH_PIN 25
#endif

/*------------------CORE SELECTION FROM ESP32---------------------*/
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

/*------------------OLED DISPLAY CONFIGURATION---------------------*/
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3C for 128x64, 0x3D for 128x32


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
    sensors_vec_t   orientation;
  // TAKEN FROM ADAFRUIT 10DOF LIBRARY

  // Variables definition:
    //BAROMETER
    static float temperature; static float altitude;
    static String temp_str, alt_str;
    //GYROSCOPE
    static double xi,yi,zi;    // Variables for integration
    static unsigned long cT;   // Current time
    static unsigned long pT;   // Previous time
    static unsigned long dT;   // Discrete time difference 
    double angleF_roll; double angleF_pitch;   //Fusioned angles


// Specify the pins where the analog signal from CNY70 will be read from
// #define YAW_PIN 26
// #define ROLL_PIN 27

// timer for cny70 rpm data reading
static unsigned long startMillis, currentMillis;  
static const unsigned long cny_period = 50;  //the value is a number of milliseconds

/*------------------BUZZER---------------------*/
#define freq_buzzer       2400  // maxima frecuencia para maximo volumen
#define channel_buzzer    0     // porque va por PWM 
#define resolution_buzzer 8     // no necesitamos mucha resolución porque va al máximo
#define duty_c_buzzer_on  125   // definimos el duty cycle al 50%
#define duty_c_buzzer_off 0     // apagar el buzzer

/*-------------BATTERY MEASURE VOLTAGE DIVIDER---------------*/
#define BAT_PIN 14

static uint8_t bat_smpl = 20;
static uint16_t batt_raw = 0;
static uint32_t sum_bat;
static float bat_norm, bat_perc;

// timer for cny70 rpm data reading
static unsigned long bat_timer, bat_start; 
static const unsigned long bat_period = 5000;  //the value is a number of milliseconds



/*-------------WIFI CONFIG---------------*/
// Replace with your network credentials
const char* ssid = "MOVISTAR_88CD";
const char* password = "qj889fyys3B322L5jszv";
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
    h2 { font-size: 3.0rem; }
    p { font-size: 3.0rem; }
    .units { font-size: 1.2rem; }
    .dht-labels{
      font-size: 1.5rem;
      vertical-align:middle;
      padding-bottom: 15px;
    }
  </style>
</head>
<body>
  <h2>ESP32 DHT Server</h2>
  <p>
    <i class="fas fa-thermometer-half" style="color:#059e8a;"></i> 
    <span class="dht-labels">Temperature</span> 
    <span id="temperature">%TEMPERATURE%</span>
    <sup class="units">&deg;C</sup>
  </p>
  <p>
    <i class="fas fa-tint" style="color:#00add6;"></i> 
    <span class="dht-labels">Humidity</span>
    <span id="humidity">%HUMIDITY%</span>
    <sup class="units">&percnt;</sup>
  </p>
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
}, 1000 ) ;

setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("humidity").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/humidity", true);
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
    bat_perc = ((12.6-bat_norm)/3) * 100;
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

    /*------------------ACCELEROMETER---------------------*/
    // Display the accelerometer results (acceleration is measured in m/s^2)
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
    Serial.print(accel_event.acceleration.z); Serial.print(F(" m/s2; "));
  #endif

    /*------------------GYROSCOPE---------------------*/
    // Measurements from gyroscope in DPS (degrees per second)
    GyroDPS gDPS; gDPS = gyro.readGyroDPS();
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
    Serial.print(F("X dps: ")); Serial.print(gDPS.x); Serial.print(F("  ; "));
    Serial.print(F("Y dps: ")); Serial.print(gDPS.y); Serial.print(F("  ; "));
    Serial.print(F("Z dps: ")); Serial.print(gDPS.z); Serial.print(F("  deg/sec; "));
    // Print gyroscope integral of previous values, to see if there's a lot of noise or not
    Serial.print(F("Xi: ")); Serial.print(xi); Serial.print(F("; "));
    Serial.print(F("Yi: ")); Serial.print(yi); Serial.print(F("; "));
    Serial.print(F("Zi: ")); Serial.print(zi); Serial.print(F(" sum; "));
  #endif

    /*------------------ORIENTATION---------------------*/
    if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation))
      { 
      // Display the orientation, which is the pitch, roll and yaw in degrees
      display.setFont(); display.setTextSize(2); display.setCursor(70,0);
      display.print("ORTN");
      display.setTextSize(1); display.setCursor(70, 16); display.print("P: ");
      display.print(String(orientation.pitch));
      display.setTextSize(1); display.setCursor(70, 26); display.print("R: ");
      display.print(String(orientation.roll));
      display.setTextSize(1); display.setCursor(70, 36); display.print("Y: ");
      display.print(String(orientation.heading));
      display.setTextSize(1); display.setCursor(70, 46);display.print("deg");
      /*
        This next expression computes the difference between the accelerometer 
        and the gyroscope, showing how much error has been accumulating
        since the beginning of the data collection.
      */
      angleF_roll = 0.95*(angleF_roll + gDPS.x*(dT/1000000.0)) + 0.05*orientation.roll; 
      angleF_pitch = 0.95*(angleF_pitch + gDPS.y*(dT/1000000.0)) + 0.05*orientation.pitch;
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
      display.setTextSize(1); display.setCursor(70,56); display.print("ALT:");
      display.print(altitude);

      #ifdef DEBUG
      Serial.print(F("Alt: ")); Serial.print(altitude); Serial.print(F(" m; "));
      Serial.print(F("Temp: ")); Serial.print(temperature); Serial.print(F(" m; "));
      #endif
    }
    
    display.display(); 
    vTaskDelay(300 / portTICK_PERIOD_MS);
  }
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
    @brief  Processor function creates the gateway between our main program
            data and the HTML variable where it will be saved and afterwards
            sent to our server.

*/
/**************************************************************************/
String processor(const String& var){
  //Serial.println(var);
  if(var == "TEMPERATURE"){
    return temp_str;
  }
  else if(var == "HUMIDITY"){
    return alt_str;
  }
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
  server.on("/temperature", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", temp_str.c_str());
  });
  server.on("/humidity", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", alt_str.c_str());
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
  // Buzzer config:
  ledcSetup(channel_buzzer, freq_buzzer, resolution_buzzer);
  ledcAttachPin(13, channel_buzzer);
  
  // Initialise the sensors and the display function:
    initCompo();
  // Gyro setup code:
    gyro.init(-463.33378, -563.64724, -0.48845); 
    pT = 0;
    xi = yi = zi = 0;
    delay(1500);   //Time to wait before calibration to open the serial
    gyro.printCalibrationValues(150);

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
  // Timer for serial data RPM management function
  xTaskCreatePinnedToCore( RPM_data,"RPM serial data",      102400, NULL,1, NULL,app_cpu);   
  
  // Delete "setup and loop" task
  vTaskDelete(NULL);
}

void loop()
{
  // nothing here
}