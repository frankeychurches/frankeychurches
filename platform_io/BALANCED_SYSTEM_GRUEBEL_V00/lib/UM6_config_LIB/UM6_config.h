/* ------------------------------------------------------------------------------
  File: UM6_config.h
  Author: CH Robotics
  Version: 1.0

  Description: Preprocessor definitions and function declarations for UM6 configuration
------------------------------------------------------------------------------ */
#ifndef __UM6_CONFIG_H
#define __UM6_CONFIG_H
#include <UM6_usart.h>


MODSERIAL um6_uart(p9, p10);    // UM6 SERIAL OVER UART Pin 9 & Pin 10








// CONFIG_ARRAY_SIZE and DATA_ARRAY_SIZE specify the number of 32 bit configuration and data registers used by the firmware
// (Note: The term "register" is used loosely here.  These "registers" are not actually registers in the same sense of a
// microcontroller register.  They are simply index locations into arrays stored in global memory.  Data and configuration
// parameters are stored in arrays because it allows a common communication protocol to be used to access all data and
// configuration.  The software communicating with the sensor needs only specify the register address, and the communication
// software running on the sensor knows exactly where to find it - it needn't know what the data is.  The software communicatin
// with the sensor, on the other hand, needs to know what it is asking for (naturally...)
// This setup makes it easy to make more data immediately available when needed - simply increase the array size, add code in
// the firmware that writes data to the new array location, and then make updates to the firmware definition on the PC side.
#define    CONFIG_ARRAY_SIZE        44
#define    DATA_ARRAY_SIZE          33
#define    COMMAND_COUNT             9

//
#define    CONFIG_REG_START_ADDRESS       0
#define    DATA_REG_START_ADDRESS        85
#define    COMMAND_START_ADDRESS        170

// These preprocessor definitions make it easier to access specific configuration parameters in code
// They specify array locations associated with each register name.  Note that in the comments below, many of the values are
// said to be 32-bit IEEE floating point.  Obviously this isn't directly the case, since the arrays are actually 32-bit unsigned
// integer arrays.  Bit for bit, the data does correspond to the correct floating point value.  Since you can't cast ints as floats,
// special conversion has to happen to copy the float data to and from the array.
// Starting with configuration register locations...


// Now for data register locations.
// In the communication protocol, data registers are labeled with number ranging from 128 to 255.  The value of 128 will be subtracted from these numbers
// to produce the actual array index labeled below
#define    UM6_STATUS                DATA_REG_START_ADDRESS                // Status register defines error codes with individual bits
#define    UM6_GYRO_RAW_XY        (DATA_REG_START_ADDRESS    + 1)        // Raw gyro data is stored in 16-bit signed integers
#define    UM6_GYRO_RAW_Z            (DATA_REG_START_ADDRESS + 2)
#define    UM6_ACCEL_RAW_XY        (DATA_REG_START_ADDRESS + 3)        // Raw accel data is stored in 16-bit signed integers
#define    UM6_ACCEL_RAW_Z        (DATA_REG_START_ADDRESS + 4)
#define    UM6_MAG_RAW_XY            (DATA_REG_START_ADDRESS + 5)        // Raw mag data is stored in 16-bit signed integers
#define    UM6_MAG_RAW_Z            (DATA_REG_START_ADDRESS + 6)
#define    UM6_GYRO_PROC_XY        (DATA_REG_START_ADDRESS + 7)        // Processed gyro data has scale factors applied and alignment correction performed.  Data is 16-bit signed integer.
#define    UM6_GYRO_PROC_Z        (DATA_REG_START_ADDRESS + 8)
#define    UM6_ACCEL_PROC_XY        (DATA_REG_START_ADDRESS + 9)        // Processed accel data has scale factors applied and alignment correction performed.  Data is 16-bit signed integer.
#define    UM6_ACCEL_PROC_Z        (DATA_REG_START_ADDRESS + 10)
#define    UM6_MAG_PROC_XY        (DATA_REG_START_ADDRESS + 11)        // Processed mag data has scale factors applied and alignment correction performed.  Data is 16-bit signed integer.
#define    UM6_MAG_PROC_Z            (DATA_REG_START_ADDRESS + 12)
#define    UM6_EULER_PHI_THETA    (DATA_REG_START_ADDRESS + 13)        // Euler angles are 32-bit IEEE floating point
#define    UM6_EULER_PSI            (DATA_REG_START_ADDRESS + 14)
#define    UM6_QUAT_AB                (DATA_REG_START_ADDRESS + 15)        // Quaternions are 16-bit signed integers.
#define    UM6_QUAT_CD                (DATA_REG_START_ADDRESS + 16)
#define    UM6_ERROR_COV_00        (DATA_REG_START_ADDRESS + 17)        // Error covariance is a 4x4 matrix of 32-bit IEEE floating point values
#define    UM6_ERROR_COV_01        (DATA_REG_START_ADDRESS + 18)
#define    UM6_ERROR_COV_02        (DATA_REG_START_ADDRESS + 19)
#define    UM6_ERROR_COV_03        (DATA_REG_START_ADDRESS + 20)
#define    UM6_ERROR_COV_10        (DATA_REG_START_ADDRESS + 21)
#define    UM6_ERROR_COV_11        (DATA_REG_START_ADDRESS + 22)
#define    UM6_ERROR_COV_12        (DATA_REG_START_ADDRESS + 23)
#define    UM6_ERROR_COV_13        (DATA_REG_START_ADDRESS + 24)
#define    UM6_ERROR_COV_20        (DATA_REG_START_ADDRESS + 25)
#define    UM6_ERROR_COV_21        (DATA_REG_START_ADDRESS + 26)
#define    UM6_ERROR_COV_22        (DATA_REG_START_ADDRESS + 27)
#define    UM6_ERROR_COV_23        (DATA_REG_START_ADDRESS + 28)
#define    UM6_ERROR_COV_30        (DATA_REG_START_ADDRESS + 29)
#define    UM6_ERROR_COV_31        (DATA_REG_START_ADDRESS + 30)
#define    UM6_ERROR_COV_32        (DATA_REG_START_ADDRESS + 31)
#define    UM6_ERROR_COV_33        (DATA_REG_START_ADDRESS + 32)



#define    UM6_GET_FW_VERSION    COMMAND_START_ADDRESS            // Causes the UM6 to report the firmware revision
#define    UM6_FLASH_COMMIT        (COMMAND_START_ADDRESS + 1)    // Causes the UM6 to write all configuration values to FLASH
#define    UM6_ZERO_GYROS            (COMMAND_START_ADDRESS + 2)    // Causes the UM6 to start a zero gyros command
#define    UM6_RESET_EKF             (COMMAND_START_ADDRESS + 3)    // Causes the UM6 to reset the EKF
#define    UM6_GET_DATA             (COMMAND_START_ADDRESS + 4)    // Causes the UM6 to transmit a data packet containing data from all enabled channels
#define    UM6_SET_ACCEL_REF        (COMMAND_START_ADDRESS + 5)    // Causes the UM6 to set the current measured accel data to the reference vector
#define    UM6_SET_MAG_REF        (COMMAND_START_ADDRESS + 6)    // Causes the UM6 to set the current measured magnetometer data to the reference vector
#define    UM6_RESET_TO_FACTORY    (COMMAND_START_ADDRESS + 7)    // Causes the UM6 to load default factory settings
 
#define    UM6_SAVE_FACTORY        (COMMAND_START_ADDRESS + 8)    // Causes the UM6 to save the current settings to the factory flash location
 
#define    UM6_USE_CONFIG_ADDRESS        0
#define    UM6_USE_FACTORY_ADDRESS        1
 
#define    UM6_BAD_CHECKSUM            253                                // Sent if the UM6 receives a packet with a bad checksum
#define    UM6_UNKNOWN_ADDRESS        254                                // Sent if the UM6 receives a packet with an unknown address
#define    UM6_INVALID_BATCH_SIZE    255                                // Sent if a requested batch read or write operation would go beyond the bounds of the config or data array






/*******************************************************************************
* Function Name  : ComputeChecksum
* Input          : USARTPacket* new_packet
* Output         : None
* Return         : uint16_t
* Description    : Returns the two byte sum of all the individual bytes in the
                         given packet.
*******************************************************************************/
uint16_t ComputeChecksum( USARTPacket* new_packet ) {
    int32_t index;
    uint16_t checksum = 0x73 + 0x6E + 0x70 + new_packet->PT + new_packet->address;

    for ( index = 0; index < new_packet->data_length; index++ ) {
        checksum += new_packet->packet_data[index];
    }
    return checksum;
}





static USARTPacket new_packet;

// Flag for storing the current USART state
uint8_t gUSART_State = USART_STATE_WAIT;


struct UM6{
float Gyro_Proc_X;
float Gyro_Proc_Y;
float Gyro_Proc_Z;
float Accel_Proc_X;
float Accel_Proc_Y;
float Accel_Proc_Z;
float Mag_Proc_X;
float Mag_Proc_Y;
float Mag_Proc_Z;
float Roll;
float Pitch;
float Yaw;
};
UM6 data;




void Process_um6_packet() {

int16_t MY_DATA_GYRO_PROC_X;
int16_t MY_DATA_GYRO_PROC_Y;
int16_t MY_DATA_GYRO_PROC_Z;
int16_t MY_DATA_ACCEL_PROC_X;
int16_t MY_DATA_ACCEL_PROC_Y;
int16_t MY_DATA_ACCEL_PROC_Z;
int16_t MY_DATA_MAG_PROC_X;
int16_t MY_DATA_MAG_PROC_Y;
int16_t MY_DATA_MAG_PROC_Z;
int16_t MY_DATA_EULER_PHI;
int16_t MY_DATA_EULER_THETA;
int16_t MY_DATA_EULER_PSI;



static uint8_t data_counter = 0;



            // The next action should depend on the USART state.
            switch ( gUSART_State ) {
                    // USART in the WAIT state.  In this state, the USART is waiting to see the sequence of bytes
                    // that signals a new incoming packet.
                case USART_STATE_WAIT:
                    if ( data_counter == 0 ) {     // Waiting on 's' character
                        if ( um6_uart.getc() == 's' ) {

                            data_counter++;
                        } else {
                            data_counter = 0;
                        }
                    } else if ( data_counter == 1 ) {    // Waiting on 'n' character
                        if ( um6_uart.getc() == 'n' ) {
                            data_counter++;

                        } else {
                            data_counter = 0;
                        }
                    } else if ( data_counter == 2 ) {    // Waiting on 'p' character
                        if ( um6_uart.getc() == 'p' ) {
                            // The full 'snp' sequence was received.  Reset data_counter (it will be used again
                            // later) and transition to the next state.
                            data_counter = 0;
                            gUSART_State = USART_STATE_TYPE;

                        } else {
                            data_counter = 0;
                        }
                    }
                    break;

                    // USART in the TYPE state.  In this state, the USART has just received the sequence of bytes that
                    // indicates a new packet is about to arrive.  Now, the USART expects to see the packet type.
                case USART_STATE_TYPE:

                    new_packet.PT = um6_uart.getc();

                    gUSART_State = USART_STATE_ADDRESS;

                    break;

                    // USART in the ADDRESS state.  In this state, the USART expects to receive a single byte indicating
                    // the address that the packet applies to
                case USART_STATE_ADDRESS:
                    new_packet.address = um6_uart.getc();
                    
                    // For convenience, identify the type of packet this is and copy to the packet structure
                    // (this will be used by the packet handler later)
                    if ( (new_packet.address >= CONFIG_REG_START_ADDRESS) && (new_packet.address < DATA_REG_START_ADDRESS) ) {
                        new_packet.address_type = ADDRESS_TYPE_CONFIG;
                    } else if ( (new_packet.address >= DATA_REG_START_ADDRESS) && (new_packet.address < COMMAND_START_ADDRESS) ) {
                        new_packet.address_type = ADDRESS_TYPE_DATA;
                    } else {
                        new_packet.address_type = ADDRESS_TYPE_COMMAND;
                    }

                    // Identify the type of communication this is (whether reading or writing to a data or configuration register, or sending a command)
                    // If this is a read operation, jump directly to the USART_STATE_CHECKSUM state - there is no more data in the packet
                    if ( (new_packet.PT & PACKET_HAS_DATA) == 0 ) {
                        gUSART_State = USART_STATE_CHECKSUM;
                    }

                    // If this is a write operation, go to the USART_STATE_DATA state to read in the relevant data
                    else {
                        gUSART_State = USART_STATE_DATA;
                        // Determine the expected number of bytes in this data packet based on the packet type.  A write operation
                        // consists of 4 bytes unless it is a batch operation, in which case the number of bytes equals 4*batch_size,
                        // where the batch size is also given in the packet type byte.
                        if ( new_packet.PT & PACKET_IS_BATCH ) {
                            new_packet.data_length = 4*((new_packet.PT >> 2) & PACKET_BATCH_LENGTH_MASK);

                        } else {
                            new_packet.data_length = 4;
                        }
                    }
                    break;

                    // USART in the DATA state.  In this state, the USART expects to receive new_packet.length bytes of
                    // data.
                case USART_STATE_DATA:
                    new_packet.packet_data[data_counter] =  um6_uart.getc();
                    data_counter++;

                    // If the expected number of bytes has been received, transition to the CHECKSUM state.
                    if ( data_counter == new_packet.data_length ) {
                        // Reset data_counter, since it will be used in the CHECKSUM state.
                        data_counter = 0;
                        gUSART_State = USART_STATE_CHECKSUM;
                    }
                    break;



                    // USART in CHECKSUM state.  In this state, the entire packet has been received, with the exception
                    // of the 16-bit checksum.
                case USART_STATE_CHECKSUM:
                    // Get the highest-order byte
                    if ( data_counter == 0 ) {
                        new_packet.checksum = ((uint16_t)um6_uart.getc() << 8);
                        data_counter++;
                    } else { // ( data_counter == 1 )
                        // Get lower-order byte
                        new_packet.checksum = new_packet.checksum | ((uint16_t)um6_uart.getc() & 0x0FF);

                        // Both checksum bytes have been received.  Make sure that the checksum is valid.
                        uint16_t checksum = ComputeChecksum( &new_packet );



                        // If checksum does not match, exit function
                        if ( checksum != new_packet.checksum ) {
                            return;
                        }   // end if(checksum check)



                        else

                        {

                            //  Packet was received correctly.

                            //-----------------------------------------------------------------------------------------------
                            //-----------------------------------------------------------------------------------------------
                            //
                            // CHECKSUM WAS GOOD SO GET ARE DATA!!!!!!!!!!!!


                            // IF DATA ADDRESS
                            if  (new_packet.address_type == ADDRESS_TYPE_DATA) {

                                //------------------------------------------------------------
                                // UM6_GYRO_PROC_XY (0x5C)
                                // To convert the register data from 16-bit 2's complement integers to actual angular rates in degrees
                                // per second, the data should be multiplied by the scale factor 0.0610352 as shown below
                                // angular_rate = register_data*0.0610352

                                if (new_packet.address == UM6_GYRO_PROC_XY) {

                                    // GYRO_PROC_X
                                    MY_DATA_GYRO_PROC_X = (int16_t)new_packet.packet_data[0]<<8; //bitshift it
                                    MY_DATA_GYRO_PROC_X |= new_packet.packet_data[1];
                                    data.Gyro_Proc_X = MY_DATA_GYRO_PROC_X*0.0610352;

                                    MY_DATA_GYRO_PROC_Y = (int16_t)new_packet.packet_data[2]<<8; //bitshift it
                                    MY_DATA_GYRO_PROC_Y |= new_packet.packet_data[3];
                                    data.Gyro_Proc_Y = MY_DATA_GYRO_PROC_Y*0.0610352;


                                    //------------------------------------------------------------


                                    //------------------------------------------------------------
                                    // UM6_GYRO_PROC_Z (0x5D)
                                    // To convert the register data from a 16-bit 2's complement integer to the actual angular rate in
                                    // degrees per second, the data should be multiplied by the scale factor 0.0610352 as shown below.


                                    // GYRO_PROC_Z
                                    MY_DATA_GYRO_PROC_Z = (int16_t)new_packet.packet_data[4]<<8; //bitshift it
                                    MY_DATA_GYRO_PROC_Z |= new_packet.packet_data[5];;
                                    data.Gyro_Proc_Z = MY_DATA_GYRO_PROC_Z*0.0610352;
                                    
                                    
                                }   // end if(MY_DATA_GYRO_PROC)
                                //------------------------------------------------------------


                                //------------------------------------------------------------
                                // UM6_ACCEL_PROC_XY (0x5E)
                                // To convert the register data from 16-bit 2's complement integers to actual acceleration in gravities,
                                // the data should be multiplied by the scale factor 0.000183105 as shown below.
                                // acceleration = register_data* 0.000183105
                                if (new_packet.address == UM6_ACCEL_PROC_XY) {

                                    // ACCEL_PROC_X
                                    MY_DATA_ACCEL_PROC_X = (int16_t)new_packet.packet_data[0]<<8; //bitshift it
                                    MY_DATA_ACCEL_PROC_X |= new_packet.packet_data[1];
                                    data.Accel_Proc_X = MY_DATA_ACCEL_PROC_X*0.000183105;

                                    // ACCEL_PROC_Y
                                    MY_DATA_ACCEL_PROC_Y = (int16_t)new_packet.packet_data[2]<<8; //bitshift it
                                    MY_DATA_ACCEL_PROC_Y |= new_packet.packet_data[3];
                                    data.Accel_Proc_Y = MY_DATA_ACCEL_PROC_Y*0.000183105;

                                    //------------------------------------------------------------


                                    //------------------------------------------------------------
                                    // UM6_ACCEL_PROC_Z (0x5F)
                                    // To convert the register data from a 16-bit 2's complement integer to the actual acceleration in
                                    // gravities, the data should be multiplied by the scale factor 0.000183105 as shown below.

                                    // ACCEL_PROC_Z
                                    MY_DATA_ACCEL_PROC_Z = (int16_t)new_packet.packet_data[4]<<8; //bitshift it
                                    MY_DATA_ACCEL_PROC_Z |= new_packet.packet_data[5];
                                    data.Accel_Proc_Z = MY_DATA_ACCEL_PROC_Z*0.000183105;
                                    
                                }   // end if(MY_DATA_ACCEL_PROC)

                                //------------------------------------------------------------


                                //------------------------------------------------------------
                                // UM6_MAG_PROC_XY (0x60)
                                // To convert the register data from 16-bit 2's complement integers to a unit-norm (assuming proper
                                // calibration)  magnetic-field vector, the data should be multiplied by the scale factor 0.000305176 as
                                // shown below.
                                // magnetic field = register_data* 0.000305176
                                if (new_packet.address == UM6_MAG_PROC_XY) {

                                    // MAG_PROC_X
                                    MY_DATA_MAG_PROC_X = (int16_t)new_packet.packet_data[0]<<8; //bitshift it
                                    MY_DATA_MAG_PROC_X |= new_packet.packet_data[1];
                                    data.Mag_Proc_X = MY_DATA_MAG_PROC_X*0.000305176;
                                    

                                    // MAG_PROC_Y
                                    MY_DATA_MAG_PROC_Y = (int16_t)new_packet.packet_data[2]<<8; //bitshift it
                                    MY_DATA_MAG_PROC_Y |= new_packet.packet_data[3];
                                    data.Mag_Proc_Y = MY_DATA_MAG_PROC_Y*0.000305176;

                                    //------------------------------------------------------------


                                    //------------------------------------------------------------
                                    // UM6_MAG_PROC_Z (0x61)
                                    // To convert the register data from 16-bit 2's complement integers to a unit-norm (assuming proper
                                    // calibration)  magnetic-field vector, the data should be multiplied by the scale factor 0.000305176 as
                                    // shown below.
                                    // magnetic field = register_data*0.000305176

                                    // MAG_PROC_Z
                                    MY_DATA_MAG_PROC_Z = (int16_t)new_packet.packet_data[4]<<8; //bitshift it
                                    MY_DATA_MAG_PROC_Z |= new_packet.packet_data[5];
                                    data.Mag_Proc_Z = MY_DATA_MAG_PROC_Z*0.000305176;

                                }   // end if(UM6_MAG_PROC)
                                //------------------------------------------------------------



                                //------------------------------------------------------------
                                // UM6_EULER_PHI_THETA (0x62)
                                // Stores the most recently computed roll (phi) and pitch (theta) angle estimates.  The angle
                                // estimates are stored as 16-bit 2's complement integers.  To obtain the actual angle estimate in
                                // degrees, the register data should be multiplied by the scale factor 0.0109863 as shown below
                                // angle estimate = register_data* 0.0109863
                                if (new_packet.address == UM6_EULER_PHI_THETA) {
                                    // EULER_PHI (ROLL)
                                   MY_DATA_EULER_PHI = (int16_t)new_packet.packet_data[0]<<8; //bitshift it
                                   MY_DATA_EULER_PHI |= new_packet.packet_data[1];
                                   data.Roll = MY_DATA_EULER_PHI*0.0109863;
                                    
         
         
         

                                    // EULER_THETA (PITCH)
                                    MY_DATA_EULER_THETA = (int16_t)new_packet.packet_data[2]<<8; //bitshift it
                                    MY_DATA_EULER_THETA |= new_packet.packet_data[3];
                                    data.Pitch = MY_DATA_EULER_THETA*0.0109863;

                                    //------------------------------------------------------------

                                    //------------------------------------------------------------
                                    // UM6_EULER_PSI (0x63) (YAW)
                                    // Stores the most recently computed yaw (psi) angle estimate.  The angle estimate is stored as a 16-
                                    // bit 2's complement integer.  To obtain the actual angle estimate in degrees, the register data
                                    // should be multiplied by the scale factor 0.0109863 as shown below

                                    MY_DATA_EULER_PSI = (int16_t)new_packet.packet_data[4]<<8; //bitshift it
                                    MY_DATA_EULER_PSI |= new_packet.packet_data[5];
                                    data.Yaw = MY_DATA_EULER_PSI*0.0109863;
          

                                }    // end if(UM6_EULER_PHI_THETA)
                                //------------------------------------------------------------

                            }    // end if(ADDRESS_TYPE_DATA)


                            // A full packet has been received.
                            // Put the USART back into the WAIT state and reset
                            // the data_counter variable so that it can be used to receive the next packet.
                            data_counter = 0;
                            gUSART_State = USART_STATE_WAIT;


                        }      // end else(GET_DATA)
 
                    }
                    break;
            
            }   //  end switch ( gUSART_State )
        
return;
   
 }       // end get_gyro_x()

#endif