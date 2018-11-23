/*
 * SPI.h
 *
 *  Created on: 2018/11/10
 *      Author: Ehab Bellkasy
 */

#ifndef SPI_H_
#define SPI_H_
#include "hw_types.h"
#include "AVR_SPI_Reg.h"




#define SPI_BUFFER_SIZE  50



typedef enum spiEnable {
	DISABLE			= 0, //	
	ENABLE 			= 1 //	
} gSPIEnable_t;



typedef enum spiDataOrder {
	LSB				= 0b00000000, // the LSB of the data word is transmitted first.	
	MSB 			= 0b00100000  // the MSB of the data word is transmitted first.	
} gSPIDataOrder_t;

typedef enum spiMasterSlaveSelect {
	SLAVE			= 0b00000000, // Selects Master SPI mode	
	MASTER 			= 0b00010000  // Selects Slave  SPI mode
} gSPIMasterSlaveSelect_t;


typedef enum spiClockPolarity {
	IDLE_LOW		= 0b00000000, // SCK is high when idle.	
	IDLE_HIGH 		= 0b00001000  // SCK is low  when idle.
} gSPIClockPolarity_t;


typedef enum spiClockPhase {
	LEADING_EDGE			= 0b00000000, // data is sampled on the leading (first) edge	
	TRAILING_EDGE 			= 0b00000100  // data is sampled on the trailing (last) edge
} gSPIClockPhase_t;



typedef enum spiClockRateSelect {
	DIVISION_FACTOR_4			= 0b00000000, // Division Factor 4
	DIVISION_FACTOR_16			= 0b00000001, // Division Factor 16
	DIVISION_FACTOR_64 			= 0b00000010, // Division Factor 64
	DIVISION_FACTOR_128 		= 0b00000011, // Division Factor 128
	DIVISION_FACTOR_2			= 0b00000100, // Division Factor 2
	DIVISION_FACTOR_8 			= 0b00000101, // Division Factor 8
	DIVISION_FACTOR_32 			= 0b00000110, // Division Factor 32
	DIVISION_FACTOR_D64 		= 0b00000111  // Division Factor 64	
	
} gSPIClockRateSelect_t;


typedef struct {
	spiEnable					enableInterrupt;
	spiEnable					enableSPI;
	gSPIDataOrder_t				dataOrder;
	gSPIMasterSlaveSelect_t		selectMasterSlave;
	gSPIClockPolarity_t			clockPolarity;
	gSPIClockPhase_t			clockPhase;
	gSPIClockRateSelect_t		selectClockRate;

} gIntSPICfg_t;

typedef enum spiStdErr {
	NO_ERRORS						= 0b00000000,
	SPI_ENABLE_ERR 					= 0b00000001,
	DATA_ORDER_ERR 					= 0b00000010,
	MASTER_SLAVE_SELECT_ERR			= 0b00000100,
	CLOCK_POLARITY_ERR 				= 0b00001000,
	CLOCK_PHASE_ERR 				= 0b00010000,
	CLOCK_RATE_SELECT_ERR	 		= 0b00100000
	
} gSPIStdErr_t;

typedef enum spiTXErr {
	NO_ERRORS									= 0,
	BUFFER_FULL_ERR 							= 1,
	NO_FREE_SPACE_ENOUGH_FOR_TX_DATA_ERR 		= 2,
	SPI_BUFFER_CURRENT_INDEX_OVERFLOW_ERR		= 3,
	SPI_BUFFER_PREVIOUS_INDEX_OVERFLOW_ERR 		= 4,
	SPI_BUFFER_SEND_INDEX_OVERFLOW_ERR 			= 5,
	SPI_IS_DISABLE_ERR	 						= 6,
	
	
} gSPITXErr_t;

extern uint8_t 	SPIbufferArry [SPI_BUFFER_SIZE];
extern uint16_t SPIbufferCurrentIndex = 0 ;
extern uint16_t SPIbufferPreviousIndex = 0 ;
extern uint16_t SPIbufferSendIndex = 0 ;
extern uint16_t SPIframeSize = 0 ; // zero for strings
extern uint16_t SPIframeIndex = 0 ; // It can tell the length of the last string
extern enum 	spitbufferStats {EMPTY,FULL,READY,BUSY_RX,BUSY_READ,OVERFLOW} SPIbufferStats;

 
gSPIStdErr_t 	SPIInitCfg(gIntSPICfg_t * obj) ;
gSPIStdErr_t 	SPIReadCfg(gIntSPICfg_t * obj) ;

gSPITXErr_t 	SPIPutChr( uint8_t data ) ;
gSPITXErr_t 	SPIPutStr( uint8_t * str ) ;
gSPITXErr_t 	SPIPutFrame( uint8_t * frame[] , uint16_t frameSize ) ;

void 			SPIBufferEmpty (void) ;
void 			SPIBufferIgnoreLastRX (void) ;

uint16_t 	SPIBufferReadFirstFrame(uint8_t * userFrame []) ;
uint16_t 	SPIBufferReadLastFrame(uint8_t * userFrame []) ;

uint16_t 	SPIBufferReadFirstString(uint8_t * userFrame []) ;
uint16_t 	SPIBufferReadLastString(uint8_t * userFrame []) ;


#endif /* SPI_H_ */

