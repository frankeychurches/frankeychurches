/*--------------------------------------FILE FOR HEADERS AND PIN DEFINITION-------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------*/

//*********************************CONFIGURING CORES TO BE USED***********************************//

#if CONFIG_FREETOS_UNICORE // CONFIGURING ONLY 1 CPU
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

/*******************************DEFINING IMPORTANT HEADERS***********************************************/

#define Serial Serial    //Defining the communication between the device and the computer through the USB port
                            //the pins for this communication are RX,TX
                            //The addres is extracted from the datasheet

#define PC_BAUDRATE 115200 //defining the baud rate at the maximum admitted
//#define SERIAL_PRINT_DEBUG 1 //SAVING RESOURCES WHEN THE DEVICE IS NOT CONNECTED TO THE COMPUTER

//************************************PIN DEFINITION**********************************************//

#define HOLD_BATTERY_ON 16 //Changed for ENG BUTTONS ON PCB
#define LED_UNIBOARD 33
#define LED_GPIO39 34
#define LED_TX_RX 27
#define LED_GPIO2 2
#define SPI_SCLK 14
