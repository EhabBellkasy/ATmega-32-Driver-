/*
 * USART.h
 *
 *  Created on: 2018/06/14
 *      Author: Ehab Bellkasy
 */

#ifndef USART_H_
#define USART_H_
#include "hw_types.h"
#include "AVR_USART_Reg.h"


#define FCLK_SYSTEM     8000000UL
#define BUFFER_SIZE  50


typedef enum usartEnable {
	DISABLE			= 0, //	
	Enable 			= 1 //	
} gUSARTEnable_t;

typedef enum usartFlags {
	NO_FLAGS							= 0b00000000,
	MULTI_PROCESSOR_COMMUNICATION_MODE 	= 0b00000001,
	DOUBLE_TRANSMISSION_SPEED 			= 0b00000010,
	PARITY_ERROR						= 0b00000100,
	DATA_OVERRUN 						= 0b00001000,
	FRAME_ERROR 						= 0b00010000,
	DATA_REGISTER_EMPTY 				= 0b00100000,
	TRANSMIT_COMPLETE 					= 0b01000000,
	RECEIVE_COMPLETE					= 0b10000000
	
} gUSARTFlags_t;

typedef enum usartModeSelect {
	ASYNCHRONOUS			= 0b00000000, //	
	SYNCHRONOUS 			= 0b01000000 //	
} gUSARTModeSelect_t;


typedef enum usartParityMode {
	DISABLED		= 0b00000000,
	EVEN_PARITY 	= 0b00100000,
	ODD_PARITY 		= 0b00110000
	
	
} gUSARTParityMode_t;

typedef enum usartStopBitSelect {
	ONE_BIT			= 0b00000000, //	
	TWO_BIT 		= 0b00001000 //	
} gUSARTStopBitSelect_t;


typedef enum usartCharacterSize {
	BIT_5			= 0b00000000, //	
	BIT_6 			= 0b00000010, //	
	BIT_7			= 0b00000100, //	
	BIT_8 			= 0b00000110, //	
	BIT_9			= 0b00001110  //	
} gUSARTCharacterSize_t;


typedef enum usartClockPolarity {
	FALLING_EDGE		= 0b00000000, //	
	RISING_EDGE 		= 0b00000001 //	
} gUSARTClockPolarity_t;



typedef struct usartCfg {
	gUSARTEnable_t				doubleTransmission;
	gUSARTEnable_t				multiProcessor;
	
	gUSARTEnable_t				interruptRXEnable;
	gUSARTEnable_t				interruptTXEnable;
	gUSARTEnable_t				interruptDataEmptyEnable;
	
	gUSARTEnable_t				receiverEnable;
	gUSARTEnable_t				transmitterEnable;
	
	gUSARTModeSelect_t			selectMode;
	gUSARTParityMode_t			parityMode;
	gUSARTStopBitSelect_t		selectStopBit;
	gUSARTClockPolarity_t		clockPolarity;
	gUSARTCharacterSize_t		characterSize;
	
	uint16_t					baudrate;
	
	void (*intFun)(void);
} gUSARTCfg_t;
 
	
 typedef enum usartStdErr {
	NO_ERRORS							= 0b00000000,
	USART_ENABLE_ERR					= 0b00000001,
	
	USART_RX_INTERRUPT_ERR 				= 0b00000010,
	USART_TX_INTERRUPT_ERR				= 0b00000100,
	USART_DATA_EMPTY_INTERRUPT_ERR 		= 0b00001000,
	
	SELECT_MODE_ERR 					= 0b00010000,
	PARITY_MODE_ERR 					= 0b00100000,
	SELECT_STOP_BIT_ERR 				= 0b01000000,
	CHARACTER_SIZE_ERR 					= 0b10000000
} gUSARTStdErr;


extern uint8_t 	USARTbufferArry [BUFFER_SIZE];
extern uint16_t USARTbufferCurrentIndex= 0 ;
extern uint16_t USARTbufferPreviousIndex= 0 ;
extern uint16_t USARTframeSize= 0 ; // zero for strings
extern uint16_t USARTframeIndex= 0 ; // It can tell the length of the last string
extern enum 	usartbufferStats {EMPTY,FULL,READY,BUSY_RX,BUSY_READ,OVERFLOW} USARTbufferStats;



typedef struct usartBuffer {
	uint8_t bufferArry [BUFFER_SIZE];
	uint16_t bufferCount=0;
} gUSARTBuffer_t;
 
gUSARTStdErr 	USARTInitCfg(gUSARTCfg_t * obj) ;
gUSARTStdErr 	USARTReadCfg(gUSARTCfg_t * obj) ;

gUSARTStdErr 	USARTPutChr( uint8_t data ) ;
gUSARTStdErr 	USARTPutStr( uint8_t * str ) ;
gUSARTStdErr 	USARTPutFrame( uint8_t * frame[] , uint16_t frameSize ) ;

void 			USARTBufferEmpty (void) ;
void 			USARTBufferIgnoreLastRX (void) ;

uint16_t 	USARTBufferReadFirstFrame(uint8_t * userFrame []) ;
uint16_t 	USARTBufferReadLastFrame(uint8_t * userFrame []) ;

uint16_t 	USARTBufferReadFirstString(uint8_t * userFrame []) ;
uint16_t 	USARTBufferReadLastString(uint8_t * userFrame []) ;

#endif /* USART_H_ */