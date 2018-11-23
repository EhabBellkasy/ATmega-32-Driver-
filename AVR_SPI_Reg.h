/*
 * AVR_ADC_Reg.h
 *
 *  Created on: 2018/11/04
 *      Author: Ehab Bellkasy
 */
 
#ifndef AVR_SPI_REG_H_
#define AVR_SPI_REG_H_


//SPI		registers 	addresses
#define 	SPDR		0x2F	// The SPI Data Register .
#define 	SPCR		0x2D	// SPI Control Register .
								#define SPR0	0	// 	SPI Clock Rate Select 0   
								#define SPR1	1	// 	SPI Clock Rate Select 1   
								#define CPHA	2	// 	settings of the Clock determine if data is sampled on the leading (first) ortrailing (last) edge of SCK.   
								#define CPOL	3	// 	When this bit is written to one,SCK is high when idle. When CPOL is written to zero, SCK is low when idle.   
								#define MSTR	4	// 	This bit selects Master SPI mode when written to one, and Slave SPI mode when written zero.   
								#define DORD	5	// 	When the DORD bit is written to one, the LSB of the data word is transmitted first & When the DORD bit is written to zero, the MSB of the data word is transmitted first.     
								#define SPE		6	// 	When the SPE bit is written to one, the SPI is enabled.   
								#define SPIE	7	// 	This bit causes the SPI interrupt to be executed if SPIF bit in the SPSR Register is set .   
#define 	SPSR		0x2E	// SPI Status Register  .
								#define SPI2X	0	// 	When this bit is written logic one the SPI speed (SCK Frequency) will be doubled .
							//	#define Bit1	1	// 	These bits are reserved bits.
							//	#define Bit2	2	// 	These bits are reserved bits.
							//	#define Bit3	3	// 	These bits are reserved bits.
							//	#define Bit4	4	// 	These bits are reserved bits.
							//	#define Bit5	5	// 	These bits are reserved bits.
								#define WCOL	6	// 	Analog Channel and Gain Selection BitsThe WCOL bit is set if the SPI Data Register (SPDR) is written during a data transfer.
								#define SPIF	7	// 	SPI Interrupt Flag.


#endif /* AVR_SPI_REG_H_ */