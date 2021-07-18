/*
 * 
 ________  ________  ________  ________   ________  ________  ________  _________
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
 *  I2C device found at address 0x69  !             L3G4_ADDRESS                  (0xD2)
 *  I2C device found at address 0x77  ! BMP085_ADDRESS 
 *  I2C device found at address 0x3C  ! SSD1306_ADDRESS 
 *  done
 */

/*--------------------------------------------------*/
/*------------------ Librerías ---------------------*/
/*--------------------------------------------------*/
#include <Wire.h>
/*-------------------WIFI----------------------*/
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
/*-------------------SSD1306-------------------*/
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
/*------------------ANDRES---------------------*/
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <L3G4200D.h>
#include <Adafruit_10DOF.h>

/*-------------------WIFI----------------------*/
// REPLACE WITH YOUR NETWORK CREDENTIALS
const char* ssid = "vodafone27E0";
const char* password = "4ZSJCMFB8B2NEF";

String web_accel_x, web_accel_y, web_accel_z;
String web_pitch, web_roll, web_yaw;
String angletw_syst = "45";
String balance_syst, unbalance_syst, clktw_syst, anticlktw_syst = "check";
String input_check_bal = "true", input_check_unbal = "false", input_check_clk = "false", input_check_aclk = "false";


// HTML web page to handle 2 input fields (threshold_input, enable_arm_input)
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>ESP32 INERTIAL DISK MANAGEMENT</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}
  table {width:100%;} 
  table, th, td {border: 1px solid black; border-collapse: collapse;}
  th, td {padding: 15px; text-align: left;} 
  #t01 tr:nth-child(even) {background-color: #eee;}
  #t01 tr:nth-child(odd) {background-color: #fff;} 
  #t01 th: {background-color: black; color: white;}
  </style></head><body><h1>INERTIAL SYSTEM INFORMATION</h1>
  <p><table id="t01"><tr><th>ACCELEROMETER</th><th> X Axis: + %accel_x% + </th><th> Y Axis: + %accel_y% + </th><th> Z Axis: + %accel_z% + </th></tr>
  <tr><th>GYROSCOPE</th><th> Pitch: + %gyro_pitch% + </th><th> Roll: + %gyro_roll% + </th><th> Yaw: + %gyro_yaw% + </th></tr></table></p>
    
  <p>DISK MANAGEMENT</p>
  <form action="/get">
    SYSTEM TWIST ANGLE (1 to 360 step) <input type="number" step="1" name="angle_input" value="%angle_twist%" required><br>
    BALANCED <input type="checkbox" name="balanced_TW" value="true" %balanced_twist%><br>
    UNBALANCED <input type="checkbox" name="unbalanced_TW" value="true" %unbalanced_twist%><br>
    CLOCKWISE TWIST <input type="checkbox" name="clkwise_TW" value="true" %clkwise_twist%><br>
    ANTICLOCKWISE TWIST <input type="checkbox" name="anticlkwise_TW" value="true" %anticlkwise_twist%><br><br>
    <input type="submit" value="Submit">
  </form>
</body></html>)rawliteral";

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

AsyncWebServer server(80);

String processor(const String& var){
  //Serial.println(var);
  if(var == "accel_x"){
    return web_accel_x;
  }
  else if(var == "accel_y"){
    return web_accel_y;
  }
  else if(var == "accel_z"){
    return web_accel_z;
  }
  else if(var == "gyro_pitch"){
    return web_pitch;
  }
  else if(var == "gyro_roll"){
    return web_roll;
  }
  else if(var == "gyro_yaw"){
    return web_yaw;
  }
  else if(var == "angle_twist"){
    return angletw_syst;
  }
  else if(var == "balanced_twist"){
    return balance_syst;
  }
  else if(var == "unbalanced_twist"){
    return unbalance_syst;
  }
  else if(var == "clkwise_twist"){
    return clktw_syst;
  }
  else if(var == "anticlkwise_twist"){
    return anticlktw_syst;
  }
  return String();
}

// Flag variable to keep track if triggers were activated or not
bool triggerActive = false;

const char* PARAM_INPUT_ANG = "angle_input";
const char* PARAM_INPUT_BAL = "balanced_TW";
const char* PARAM_INPUT_UNBAL = "unbalanced_TW";
const char* PARAM_INPUT_CLKT_TW = "clkwise_TW";
const char* PARAM_INPUT_ACLK_TW = "anticlkwise_TW";

// Interval between sensor readings. Learn more about ESP32 timers: https://RandomNerdTutorials.com/esp32-pir-motion-sensor-interrupts-timers/
unsigned long previousMillis = 0;     
const long interval = 1000;    

// LED built in usage to see everything works fine
const int output = 2;

/*------------------------------------------*/

// OLED
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

/* Assign a unique ID to the sensors */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
L3G4200D gyro;

// TIMERS
unsigned long timer = 0;
float timeStep = 0.01;

// BUZZER
const int freq_buzzer = 2400;     // Max freq for max volume
const int channel_buzzer = 0;     // PWM controlled 
const int resolution_buzzer = 8;  // No need for much resolution, it is at max vol.
const int duty_c_buzzer_on = 125; // Defining duty cycle at 50%
const int duty_c_buzzer_off = 0;  // Turn off buzzer

// IMU
float deg_pitch = 0; float deg_roll = 0; float deg_yaw = 0;

void setup()
{
  Serial.begin(115200);
  Serial.println(F("Adafruit 10DOF Tester")); Serial.println("");
  // Buzzer config:
  ledcSetup(channel_buzzer, freq_buzzer, resolution_buzzer);
  ledcAttachPin(13, channel_buzzer);

  ////////////////////////////////////////
  // Connect to Wi-Fi network 
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed!");
    return;
  }
  Serial.println();
  Serial.print("ESP IP Address: http://");
  Serial.println(WiFi.localIP());

  pinMode(output, OUTPUT);
  digitalWrite(output, LOW);

   // Send web page to client
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });

  ////////////////////////////////////////
  // Starting up the sensors:
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  /* Initialise the sensors */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  } 
  // Initialize L3G4200D
  Serial.println("Initialize L3G4200D");
  // Set scale 2000 dps and 400HZ Output data rate (cut-off 50)
  if(!gyro.begin(L3G4200D_SCALE_2000DPS, L3G4200D_DATARATE_400HZ_50))
  {
    /* There was a problem detecting the L3G4200D ... check your connections */
    Serial.print("Ooops, no L3G4200D gyroscope detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  // Calibrate gyroscope. The calibration must be at rest.
  gyro.calibrate(100); // If you don't want to calibrate, comment this line.

  // Receive an HTTP GET request at <ESP_IP>/get?threshold_input=<inputMessage>&enable_arm_input=<inputMessage2>
  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
    // GET angle_input value on <ESP_IP>/get?angle_input=<angletw_syst>
    if (request->hasParam(PARAM_INPUT_ANG)) {
      angletw_syst = request->getParam(PARAM_INPUT_ANG)->value();
      // GET enable_arm_input value on <ESP_IP>/get?enable_arm_input=<inputMessage2>
      if (request->hasParam(PARAM_INPUT_BAL)) {
        input_check_bal = request->getParam(PARAM_INPUT_BAL)->value();
        balance_syst = "checked";
      }
      if (request->hasParam(PARAM_INPUT_UNBAL)){
        input_check_unbal = request->getParam(PARAM_INPUT_UNBAL)->value();
        unbalance_syst = "checked";
      }
      if (request->hasParam(PARAM_INPUT_CLKT_TW)){
        input_check_clk = request->getParam(PARAM_INPUT_CLKT_TW)->value();
        clktw_syst = "checked";
      }
      if (request->hasParam(PARAM_INPUT_ACLK_TW)){
        input_check_aclk = request->getParam(PARAM_INPUT_ACLK_TW)->value();
        anticlktw_syst = "checked";
      }
      else {
        input_check_bal = "false";input_check_unbal = "false";input_check_clk = "false";input_check_aclk = "false";
        balance_syst, unbalance_syst, clktw_syst, anticlktw_syst = "";
      }
    }
    Serial.println(angletw_syst);
    // Serial.println(input_check);
    request->send(200, "text/html", "HTTP GET request sent to your ESP.<br><a href=\"/\">Return to Home Page</a>");
  });
  server.onNotFound(notFound);
  server.begin();

  display.clearDisplay();
  display.setTextColor(WHITE);
}

void Wifi_Connection (){
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) { 
    previousMillis = currentMillis;
    // Request values of accelerometer 
      sensors_event_t event;
      /* Clears display in each loop so it updates the data shwon with each new iteration: */
      display.clearDisplay();
      /* Display the results on the OLED (acceleration is measured in m/s^2) */
      accel.getEvent(&event);
      // Data string transformation which is sent to the web page:
        web_accel_x = String(event.acceleration.x); web_accel_y = String(event.acceleration.y);
        web_accel_z = String(event.acceleration.z);
      display.setFont(); display.setTextSize(2); display.setCursor(0,0); display.print("ACCEL");
      display.setTextSize(1); display.setCursor(0,16); display.print("X: "); display.print(web_accel_x);
      display.setTextSize(1); display.setCursor(0,26); display.print("Y: "); display.print(web_accel_y);
      display.setTextSize(1); display.setCursor(0,36); display.print("Z: "); display.print(web_accel_z);
      display.setTextSize(1); display.setCursor(0,46); display.print("m/s^2");

    // Read gyroscope normalized values: 
      Vector norm = gyro.readNormalize();
      /*Pitch, Roll and Yaw values: */
      float pitch = 0; float roll = 0; float yaw = 0;
      /* Calculate Pitch, Roll and Yaw: */
      pitch = pitch + norm.YAxis * timeStep; deg_pitch = pitch*(180/3.1416);
      roll  = roll  + norm.XAxis * timeStep; deg_roll  = roll*(180/3.1416);
      yaw   = yaw   + norm.ZAxis * timeStep; deg_yaw   = yaw*(180/3.1416);
      // Data string transformation which is sent to the web page:
        web_pitch = String(deg_pitch); web_roll = String(deg_roll); web_yaw = String(deg_yaw);
      /* Display the results on the OLED (gyroscope is measured in deg/s) */
      display.setFont(); display.setTextSize(2); display.setCursor(70,0); display.print("GYRO");
      display.setTextSize(1); display.setCursor(70, 16); display.print("P: "); display.print(web_pitch);
      display.setTextSize(1); display.setCursor(70, 26); display.print("R: "); display.print(web_roll);
      display.setTextSize(1); display.setCursor(70, 36); display.print("Y: "); display.print(web_yaw);
      display.setTextSize(1); display.setCursor(70, 46); display.print("deg/s ");

    // Check if system is balanced and check the trigger:
    if((event.acceleration.z > 9.2) && ( abs(event.acceleration.x) < 0.3) && (abs(event.acceleration.y) < 0.3) && input_check_bal == "true" && !triggerActive){
      String message = String("BALANCED -> Zgrav: ")+web_accel_z+String(" Xgrav: ")+web_accel_x+String(" Ygrav: ")+web_accel_y;
      Serial.println(message);
      triggerActive = true;
      digitalWrite(output, HIGH); //LED indicator
      display.setTextSize(1); display.setCursor(0,56); display.print("BALANCED"); //OLED display
      // Buzzer indicator of balanced system:
      ledcWriteTone(channel_buzzer, freq_buzzer);
      ledcWrite(channel_buzzer, duty_c_buzzer_on);
    }
    // Check if temperature is below threshold and if it needs to trigger output
    else if((event.acceleration.z < 9.2) && ( abs(event.acceleration.x) > 0.3) && (abs(event.acceleration.y) > 0.3) && input_check_unbal == "true" && triggerActive) {
      String message = String("UNBALANCED -> Zgrav: ")+web_accel_z+String(" Xgrav: ")+web_accel_x+String(" Ygrav: ")+web_accel_y;
      Serial.println(message);
      triggerActive = false;
      digitalWrite(output, LOW);
      display.setTextSize(1); display.setCursor(0,56); display.print("UNBALANCED"); //OLED display
      // Buzzer off until is balanced:
      ledcWrite(channel_buzzer, duty_c_buzzer_off);
    }
  }
}

void loop()
{
  Wifi_Connection();
  display.display();
}