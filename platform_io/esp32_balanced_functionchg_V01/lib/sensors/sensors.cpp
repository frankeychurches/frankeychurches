
/*------------------ OWN LIBRARIES ---------------------*/
//-------VARIABLES------
#include "../lib/variables/variables.h"
//-------DEFINES------
#include "../defines.h"


/*!
    @brief  Initialises all the sensors and the OLED used by the system.
*/
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

  /* Initialise INA219 */
  if(!ina219.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print("Ooops, no INA219 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
}

/*!
  *   @brief  Function which reads the gravity, pitch, roll, heading and altitude.
  *           It shows the data on the display (due to size restrictions only
  *           gravity on three axis and pitch, roll and heading) and in the
  *           Serial monitor. 
  *           Baud rate is 115200.
*/
void SensorResults(void * parameters)
{
  while(1)
  {
    display.clearDisplay();        // Clear former data shown in OLED
    accel.getEvent(&accel_event);  // Get event from accelerometer
    mag.getEvent(&mag_event);      // Get event from magnetometer
    imu.accel_x = String(accel_event.acceleration.x);
    imu.accel_y = String(accel_event.acceleration.y);
    imu.accel_z = String(accel_event.acceleration.z);
    /*------------------ACCELEROMETER---------------------*/
    // Display the accelerometer results (acceleration is measured in m/s^2)
      display.setFont(); display.setTextSize(2); display.setCursor(0,0);
      display.print("ACCEL");
      display.setTextSize(1); display.setCursor(0, 16); display.print("X: ");
      display.print(imu.accel_x);
      display.setTextSize(1); display.setCursor(0,26); display.print("Y: ");
      display.print(imu.accel_y);
      display.setTextSize(1); display.setCursor(0,36); display.print("Z: ");
      display.print(imu.accel_z);
      display.setTextSize(1); display.setCursor(0,46); display.print("m/s2");
    /*Serial print of the prior sensor, to see clearly the behaviour*/
  #ifdef DEBUG
    Serial.print(F("X grav: ")); 
    Serial.print(imu.accel_x); Serial.print(F("; "));
    Serial.print(F("Y grav: ")); 
    Serial.print(imu.accel_y); Serial.print(F("; "));
    Serial.print(F("Z grav: "));
    Serial.print(imu.accel_z); Serial.print(F(" m/s2; "));
  #endif

    /*------------------GYROSCOPE---------------------*/
    // Measurements from gyroscope in DPS (degrees per second)
    gDPS = gyro.readGyroDPS();
    imu.gyro_x = String(gDPS.x); imu.gyro_y = String(gDPS.y); imu.gyro_z = String(gDPS.z);
    /* 
      Measurements from gyroscope integrating dps -> distance
      This distance is the sum (or substract, depending on the 
      twist direction) of the angles tilted 
      from the original position. 
      dT represents the discrete time differentiation.
    */
    imu.cT = micros(); imu.dT = imu.cT - imu.pT; imu.pT = imu.cT;
    imu.xi = imu.xi + gDPS.x*(imu.dT/1000000.0);
    imu.yi = imu.yi + gDPS.y*(imu.dT/1000000.0);
    imu.zi = imu.zi + gDPS.z*(imu.dT/1000000.0);
  #ifdef DEBUG
    // Print gyroscope degrees per second values
    Serial.print(F("X dps: ")); Serial.print(imu.gyro_x); Serial.print(F("  ; "));
    Serial.print(F("Y dps: ")); Serial.print(imu.gyro_y); Serial.print(F("  ; "));
    Serial.print(F("Z dps: ")); Serial.print(imu.gyro_z); Serial.print(F("  deg/sec; "));
    // Print gyroscope integral of previous values, to see if there's a lot of noise or not
    Serial.print(F("Xi: ")); Serial.print(imu.xi); Serial.print(F("; "));
    Serial.print(F("Yi: ")); Serial.print(imu.yi); Serial.print(F("; "));
    Serial.print(F("Zi: ")); Serial.print(imu.zi); Serial.print(F(" sum; "));
  #endif

    /*------------------ORIENTATION---------------------*/
    if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation))
      { 
      imu.fusion_pitch = String(orientation.pitch);
      imu.fusion_roll  = String(orientation.roll);
      imu.fusion_heading = String(orientation.heading);
      // Display the orientation, which is the pitch, roll and yaw in degrees
      display.setFont(); display.setTextSize(2); display.setCursor(70,0);
      display.print("ORTN");
      display.setTextSize(1); display.setCursor(70, 16); display.print("P: ");
      display.print(imu.fusion_pitch);
      display.setTextSize(1); display.setCursor(70, 26); display.print("R: ");
      display.print(imu.fusion_roll);
      display.setTextSize(1); display.setCursor(70, 36); display.print("Y: ");
      display.print(imu.fusion_heading);
      display.setTextSize(1); display.setCursor(70, 46);display.print("deg");
      /*
        This next expression computes the difference between the accelerometer 
        and the gyroscope, showing how much error has been accumulating
        since the beginning of the data collection.
      */
      imu.angleF_roll = 0.95*(imu.angleF_roll + gDPS.x*(imu.dT/1000000.0)) + 0.05*orientation.roll; 
      imu.angleF_pitch = 0.95*(imu.angleF_pitch + gDPS.y*(imu.dT/1000000.0)) + 0.05*orientation.pitch;
      imu.filter_roll = String(imu.angleF_roll); imu.filter_pitch = String(imu.angleF_pitch);

    #ifdef DEBUG
      // Serial display of data
      Serial.print(F("Roll: ")); Serial.print(imu.fusion_roll); Serial.print(F("; "));
      Serial.print(F("Pitch: ")); Serial.print(imu.fusion_pitch); Serial.print(F("; "));
      Serial.print(F("Heading: ")); Serial.print(imu.fusion_heading); Serial.print(F(" deg; "));
      Serial.print(F("Roll fus: ")); Serial.print(imu.angleF_roll); Serial.print(F("; "));
      Serial.print(F("Pitch fus: ")); Serial.print(imu.angleF_pitch); Serial.print(F("; "));
    #endif
      }

    /*----------------ALTITUDE & TEMPERATURE-----------------------*/
    /* Calculate the altitude using the barometric pressure sensor */
    bmp.getEvent(&bmp_event);
    if (bmp_event.pressure)
    {
      /* Get ambient temperature in C */
      bmp.getTemperature(&imu.temperature);
      imu.temp_str = String(imu.temperature);
      /* Convert atmospheric pressure, SLP and temp to altitude    */
      imu.altitude = bmp.pressureToAltitude(seaLevelPressure, bmp_event.pressure, imu.temperature);
      imu.alt_str = String(imu.altitude);
      display.setTextSize(1); display.setCursor(70,56); display.print("ALT:");
      display.print(imu.altitude);

      #ifdef DEBUG
      Serial.print(F("Alt: ")); Serial.print(imu.altitude); Serial.print(F(" m; "));
      Serial.print(F("Temp: ")); Serial.print(imu.temperature); Serial.print(F(" m; "));
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

