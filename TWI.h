/*
 * TWI.h
 *
 *  Created on: 2018/11/20
 *      Author: Ehab Bellkasy
 */

#ifndef TWI_H_
#define TWI_H_
#include "hw_types.h"
#include "AVR_TWI_Reg.h"


#define TWI_BUFFER_SIZE  50



typedef enum twiStatus_t {
	M_TX_START_CONDITION					= 0x08, // A START condition has been transmitted
	M_TX_REPEATED_START						= 0x10, // A repeated START condition has been transmitted
	M_TX_SLA_W__ACK 						= 0x18, // SLA+W has been transmitted; ACK has been received
	M_TX_SLA_W__NOT_ACK	 					= 0x20, // SLA+W has been transmitted; NOT ACK has been received
	M_TX_DATA__ACK	 						= 0x28, // Data byte has been transmitted; ACK has been received
	M_TX_DATA__NOT_ACK	 					= 0x30, // Data byte has been transmitted; NOT ACK has been received
	M_ARBITRATION_LOST	 					= 0x38, // Arbitration lost in SLA+W/R or data bytes
	M_TX_SLA_R__ACK 						= 0x40, // SLA+R has been transmitted; ACK has been received
	M_TX_SLA_R__NOT_ACK	 					= 0x48, // SLA+R has been transmitted; NOT ACK has been received
	M_RX_DATA__ACK	 						= 0x50, // Data byte has been received; ACK has been received
	M_RX_DATA__NOT_ACK	 					= 0x58, // Data byte has been received; NOT ACK has been received
	S_RX_SLA_W__ACK	 						= 0x60, // Own SLA+W has been received;ACK has been returned.
	S_ARBITRATION_LOST__OWN_SLA_W__ACK		= 0x68, // Arbitration lost in SLA+R/W as master; own SLA+W has been received; ACK has been returned.
	S_GENERAL_CALL__ACK	 					= 0x70, // General call address has been received; ACK has been returned.
	S_ARBITRATION_LOST__GENERAL_CALL__ACK	= 0x78, // Arbitration lost in SLA+R/W as master; General call address has been received; ACK has been returned.
	S_RX_DATA__SLA_W__ACK	 				= 0x80, // Previously addressed with own SLA+W; data has been received; ACK has been returned .
	S_RX_DATA__SLA_W__NOT_ACK				= 0x88, // Previously addressed with own SLA+W; data has been received; NOT ACK has been returned .
	S_RX_DATA__GENERAL_CALL__ACK	 		= 0x90, // Previously addressed with general call; data has been received; ACK has been returned.
	S_RX_DATA__GENERAL_CALL__NOT_ACK		= 0x98, // Previously addressed with general call; data has been received; NOT ACK has been returned.
	S_RX_STOP_OR_REPEATED_START_CONDITION	= 0xA0, // A STOP condition or repeated START condition has been received while still addressed as slave.
	S_RX_SLA_R__ACK	 						= 0xA8, // Own SLA+R has been received; ACK has been returned.
	S_ARBITRATION_LOST__OWN_SLA_R__ACK		= 0xB0, // Arbitration lost in SLA+R/W as master; own SLA+R has been received; ACK has been returned.  
	S_TX_DATA__ACK	 						= 0xB8, // Data byte in TWDR has been transmitted; ACK has been received.
	S_TX_DATA__NOT_ACK	 					= 0xC0, // Data byte in TWDR has been transmitted; NOT ACK has been  received.
	S_TX_LAST_DATA	 						= 0xC8  // Last data byte in TWDR has been transmitted (TWEA = “0”); ACK has been received.
	
	
} gTWIStatus_t;




typedef enum twiTXErr {
	NO_ERRORS									= 0,
	TWI_IS_DISABLE_ERR	 						= 1,
	TWI_TAKEN			 						= 2,
	TWI_BUSY									= 3
	
} gTWITXErr_t;

typedef enum twiTaskStatus {
	IDEAL										= 0,
	START										= 1,
	TRANSFER									= 2,
	FINISH										= 3,
	NO_RESPONSE									= 4,
	DISCONNECTION								= 5
	
} gTWItaskStatus_t; 




typedef enum twiEnable {
	DISABLE			= 0, //	
	ENABLE 			= 1 //	
} gTWIEnable_t;


typedef enum twiClockPrescaler {
	DIVISION_FACTOR_1			= 0b00000000, // Division Factor 1
	DIVISION_FACTOR_4			= 0b00000001, // Division Factor 4
	DIVISION_FACTOR_16 			= 0b00000010, // Division Factor 16
	DIVISION_FACTOR_64	 		= 0b00000011  // Division Factor 64
	
	
} gTWIClockPrescaler_t;


typedef struct {
	uint8_t						bitRate;
	TWIEnable					enableAcknowledgeBit;
	TWIEnable					enableInterrupt;
	TWIEnable					enableTWI;
	gTWIClockPrescaler_t		clockPrescaler;
	uint8_t						slaveAddress;
	TWIEnable					enableGeneralCallRecognition;
	

} gIntTWICfg_t;


typedef enum twiStdErr {
	NO_ERRORS						= 0b00000000,
	TWI_ENABLE_ERR 					= 0b00000001,
	CLOCK_PRESCALER_ERR 			= 0b00000010,
	SLAVE_ADDRESS_ERR				= 0b00000100
	
	
} gTWIStdErr_t;

/* 
===========================================
Global Variables
===========================================
*/
extern uint8_t 			TWIbufferArry [TWI_BUFFER_SIZE];
extern uint16_t 		TWIbufferCurrentIndex = 0 ;
extern uint16_t 		TWIbufferPreviousIndex = 0 ;
extern uint16_t 		TWIframeSize = 0 ; // zero for strings
extern uint16_t 		TWIframeIndex = 0 ; // It can tell the length of the last string
extern uint16_t 		TWIsendIndex = 0 ; // It can tell the last byte send from frame
extern uint16_t 		TWIreceiveIndex = 0 ; // It can tell the last byte Receive from frame
extern uint16_t 		TWIresponseIndex = 0 ; // It can tell the last byte response from frame
extern uint8_t 			TWIreTXtrialsMax = 3 ; 
extern uint8_t 			TWIreTXtrialsCount = 0 ; 
extern enum 			twibufferStats {EMPTY,FULL,READY,BUSY_RX,BUSY_READ,OVERFLOW} TWIbufferStats;
extern gTWIStatus_t 	TWIStatus =0 ;



extern uint8_t			TWIslvWAddress = 0 ;
extern uint8_t*			TWIuserDataPtrMTX = 0 ;
extern uint8_t			TWIslvRAddress = 0 ;
extern uint8_t*			TWIuserDataPtrMRX = 0 ;
extern uint8_t*			TWIuserDataPtrSTX = 0 ;

extern gTWItaskStatus_t TWImasterTXstatus = IDEAL ;
extern gTWItaskStatus_t TWImasterRXstatus = IDEAL ;
extern gTWItaskStatus_t TWIslaveTXstatus  = IDEAL ;
extern gTWItaskStatus_t TWIslaveRXstatus  = IDEAL ;

/*
================================================================
Functions Declaration
================================================================
*/
 
gTWIStdErr_t 	TWIInitCfg(gIntTWICfg_t * obj) ;
gTWIStdErr_t 	TWIReadCfg(gIntTWICfg_t * obj) ;

gTWITXErr_t TWIMasterTransmitRequest( uint8_t slaveAddress , uint8_t* userData );
gTWITXErr_t TWIMasterReceiveRequest( uint8_t slaveAddress , uint8_t* userData );
gTWITXErr_t TWIslaveTransmitResponse( uint8_t* userData );

void 			TWIBufferEmpty (void) ;
void 			TWIBufferIgnoreLastRX (void) ;

uint16_t 	TWIBufferReadFirstFrame(uint8_t * userFrame []) ;
uint16_t 	TWIBufferReadLastFrame(uint8_t * userFrame []) ;

uint16_t 	TWIBufferReadFirstString(uint8_t * userFrame []) ;
uint16_t 	TWIBufferReadLastString(uint8_t * userFrame []) ;


#endif /* TWI_H_ */
