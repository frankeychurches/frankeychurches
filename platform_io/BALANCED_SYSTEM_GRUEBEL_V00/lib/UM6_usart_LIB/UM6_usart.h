/* ______________________________________________________________________________________
 File: UM6_usart.h
 Author: CH Robotics, adapted for mbed by lhiggs
 Version: 1.0
 
 Description: Function declarations for USART communucation
 -------------------------------------------------------------------------------------- */
 
 #ifndef _CHR_USART_H
 #define _CHR_USART_H

 #define  MAX_PACKET_DATA      40
 
 // Definitions of states for USART receiver state machine (for receiving packets)
 #define USART_STATE_WAIT       1
 #define USART_STATE_TYPE       2
 #define USART_STATE_ADDRESS    3
 #define USART_STATE_DATA       4
 #define USART_STATE_CHECKSUM   5

 // Flags for interpreting the packet type byte in communication packets
 #define PACKET_HAS_DATA            (1 << 7)
 #define PACKET_IS_BATCH            (1 << 6)
 #define PACKET_BATCH_LENGTH_MASK   ( 0x0F )
 
 #define PACKET_BATCH_LENGTH_OFFSET     2
 
 #define BATCH_SIZE_2                   2
 #define BATCH_SIZE_3                   3
 
 #define PACKET_NO_DATA                 0
 #define PACKET_COMMAND_FAILED      (1 << 0)
 
 
 // Define flags for identifying the type of packet address received
 #define ADDRESS_TYPE_CONFIG            0
 #define ADDRESS_TYPE_DATA              1
 #define ADDRESS_TYPE_COMMAND           2
 
 
extern uint8_t gUSART_State;

 // Structure for storing TX and RX packet data
 typedef struct _USARTPacket
 {
        uint8_t PT;         // Packet type
        uint8_t address;    // Packet address
        uint16_t checksum;  // Checksum
        
        // Data included for convenience, but that isn't stored in the packet itself
        uint8_t data_length;  // Number of bytes in data section
        uint8_t address_type; // Specified the address type (DATA, CONFIG, OR COMMAND)
        
        uint8_t packet_data[MAX_PACKET_DATA];
        
 } USARTPacket;
 
uint16_t ComputeChecksum( USARTPacket* new_packet );

#endif