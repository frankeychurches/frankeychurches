//**************************************PIN DEFINITION**********************************************//
#define ENG_MODE 33
#define ON_OFF_LECTURE 35

//***********************************SERIAL DEFINITIONS***********************************************//
#define Serial Serial 
#define PC_BAUDRATE 9600
#define SERIAL_PRINT_DEBUG 1 //SAVING RESOURCES WHEN THE DEVICE IS NOT CONNECTED TO THE COMPUTER

//*********************************CONFIGURING CORES TO BE USED***********************************//
//UNCOMMENT "IF" BELOW IF USING ONLY ONE CORE//

#if CONFIG_FREETOS_UNICORE // CONFIGURING ONLY 1 CPU
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

#define ESP32_12_BIT_RESOLUTION 4095