/*-------------------WIFI-------------------*/
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <Wire.h>

/*------------------ OWN LIBRARIES ---------------------*/
#include "../defines.h"
//-------VARIABLES------
#include "../variables/variables.h"

/*!
    @brief  Processor function creates the gateway between our main program
            data and the HTML variable where it will be saved and afterwards
            sent to our server.
*/
String processor(const String& var){
    // Values from the BMP180
  if(var == "TEMPERATURE"){  return imu.temp_str;}
  else if(var == "ALTITUDE"){ return imu.alt_str;}
    // Value from the Battery Voltage Measure filter
  else if(var == "BATTERY"){ return bvm.bat_perc_str;}
    // Values from the IMU MEMS unit
  else if(var == "FUSION_X"){ return imu.fusion_roll;}
  else if(var == "FUSION_Y"){ return imu.fusion_pitch;}
  else if(var == "FUSION_Z"){ return imu.fusion_heading;}
  else if(var == "GYRO_X"){ return imu.gyro_x;}
  else if(var == "GYRO_Y"){ return imu.gyro_y;}
  else if(var == "GYRO_Z"){ return imu.gyro_z;}
  else if(var == "FILTER_ROLL"){  return imu.filter_roll;}
  else if(var == "FILTER_PITCH"){ return imu.filter_pitch;}
    // RPM values from the tachometer
  else if(var == "RPM_R"){ return cny70.rpm_r_str;}
  else if(var == "RPM_P"){ return cny70.rpm_p_str;}
  else if(var == "RPM_Y"){ return cny70.rpm_y_str;}
    // Angular acceleration calculation from RPM values
  else if(var == "ANGACC_R"){ return cny70.ang_acc_r_str;}
  else if(var == "ANGACC_P"){ return cny70.ang_acc_p_str;}
  else if(var == "ANGACC_Y"){ return cny70.ang_acc_y_str;}
    // INA values
  else if(var == "INA_VOLT"){ return ina.volt_str;}
  else if(var == "INA_CURR"){ return ina.curr_str;}
  else if(var == "INA_POW"){  return ina.pow_str;}

  return String();
}

/*!
    @brief  WiFi Configuration function which sets up the server and the 
            communication with it. 
*/
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
    request->send_P(200, "text/plain", imu.temp_str.c_str());
  });
  server.on("/altitude", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", imu.alt_str.c_str());
  });
  // Request to get the battery voltage level in percentage
  server.on("/battery", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", bvm.bat_perc_str.c_str());
  });
  // Request to get the fusion orientation from the IMU
  server.on("/fusion_x", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", imu.fusion_roll.c_str());
  });
  server.on("/fusion_y", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", imu.fusion_pitch.c_str());
  });
  server.on("/fusion_z", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", imu.fusion_heading.c_str());
  });
  // Request to get the gyro information
  server.on("/gyro_x", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", imu.gyro_x.c_str());
  });
  server.on("/gyro_y", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", imu.gyro_y.c_str());
  });
  server.on("/gyro_z", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", imu.gyro_z.c_str());
  });
  // Request to get the filtered signal obtained with the accelerometer and the gyroscope
  server.on("/filter_roll", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", imu.filter_roll.c_str());
  });
  server.on("/filter_pitch", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", imu.filter_pitch.c_str());
  });
  // Request to get the RPM value from the inertial disks
  server.on("/rpm_roll", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", cny70.rpm_r_str.c_str());
  });
  server.on("/rpm_pitch", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", cny70.rpm_p_str.c_str());
  });
  server.on("/rpm_yaw", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", cny70.rpm_y_str.c_str());
  });
  // Request to get the angular acceleration from the inertial disks
  server.on("/ang_acc_r", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", cny70.ang_acc_r_str.c_str());
  });
  server.on("/ang_acc_p", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", cny70.ang_acc_p_str.c_str());
  });
  server.on("/ang_acc_y", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", cny70.ang_acc_y_str.c_str());
  });
  // Request to get the INA219 information
  server.on("/ina_volt", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", ina.volt_str.c_str());
  });
  server.on("/ina_curr", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", ina.curr_str.c_str());
  });
  server.on("/ina_pow", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", ina.pow_str.c_str());
  });

  // Start server
  server.begin();
}