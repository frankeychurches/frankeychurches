
/**************************************************************************/
/*!
    @brief  Battery measurement system designed with a voltage divider.
            A series resistance with a low pass filter (R, R//C) allow usto determine
            the remain voltage of the battery, thereby knowing how much
            battery we have left before causing a deep discharge of the battery.  
*/
/**************************************************************************/

void BatteryMeasure(void* parameters);

/**************************************************************************/
/*!
    @brief  INA219 program for Bus Voltage, Current and Load data. 
*/
/**************************************************************************/

void INA_Measure(void* parameters);
