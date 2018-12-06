/*
 * AVR_TWI_Reg.h
 *
 *  Created on: 2018/11/19
 *      Author: Ehab Bellkasy
 */

  
#ifndef AVR_TWI_REG_H_
#define AVR_TWI_REG_H_



//TWI		registers 	addresses
#define 	TWBR		0x20	// The TWI Bit Rate Register .
#define 	TWDR		0x23	// The TWI Data Register .
#define 	TWCR		0x56	// SPI Control Register .
								#define TWIE	0	// 	TWI Interrupt Enable .  
							//	#define Bit1	1	// 	These bits are reserved bits .   
								#define TWEN	2	// 	TWI Enable Bit .   
								#define TWWC	3	// 	TWI Write Collision Flag :  .   
								#define TWSTO	4	// 	TWI STOP Condition Bit : generate a STOP condition .   
								#define TWSTA	5	// 	TWI START Condition Bit become a master , checks if the bus is available, and generates a START condition .     
								#define TWEA	6	// 	TWI Enable Acknowledge Bit .   
								#define TWINT	7	// 	TWI Interrupt Flag .   
#define 	TWSR		0x21	// TWI Status Register  .
								#define TWPS0	0	// 	TWI Prescaler Bits .
								#define TWPS1	1	// 	TWI Prescaler Bits.
							//	#define Bit2	2	// 	These bits are reserved bits.
								#define TWS3	3	// 	TWI Status: reflect the status of the TWI logic and the Two-wire Serial Bus.
								#define TWS4	4	// 	TWI Status: reflect the status of the TWI logic and the Two-wire Serial Bus.
								#define TWS5	5	// 	TWI Status: reflect the status of the TWI logic and the Two-wire Serial Bus.
								#define TWS6	6	// 	TWI Status: reflect the status of the TWI logic and the Two-wire Serial Bus.
								#define TWS7	7	// 	TWI Status: reflect the status of the TWI logic and the Two-wire Serial Bus.
#define 	TWAR		0x22	// TWI (Slave) Address Register  .
								#define TWGCE	0	// 	TWI General Call Recognition Enable Bit .
								#define TWA0	1	// 	TWI (Slave) Address: These seven bits constitute the slave address of the TWI unit.
								#define TWA1	2	// 	TWI (Slave) Address: These seven bits constitute the slave address of the TWI unit.
								#define TWA2	3	// 	TWI (Slave) Address: These seven bits constitute the slave address of the TWI unit.
								#define TWA3	4	// 	TWI (Slave) Address: These seven bits constitute the slave address of the TWI unit.
								#define TWA4	5	// 	TWI (Slave) Address: These seven bits constitute the slave address of the TWI unit.
								#define TWA5	6	// 	TWI (Slave) Address: These seven bits constitute the slave address of the TWI unit.
								#define TWA6	7	// 	TWI (Slave) Address: These seven bits constitute the slave address of the TWI unit.

								




#endif /* AVR_TWI_REG_H_ */