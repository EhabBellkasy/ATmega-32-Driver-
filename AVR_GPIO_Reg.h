/*
 * AVR_GPIO.h
 *
 *  Created on: 2018/06/07
 *      Author: Ehab Bellkasy
 */

#ifndef AVR_GPIO_REG_H_
#define AVR_GPIO_REG_H_

#define MAX_PINS 7

#define BASE_A 0x39
#define BASE_B 0x36
#define BASE_C 0x33
#define BASE_D 0x30

#define DIR_OFFSET 1
#define OUTPUT_OFFSET 2
#define INPUT_OFFSET 0


#define INPUT 	0
#define OUTPUT 	1

//Interrupt registers 	addresses
#define 	MCUSR		0x55	//The MCU Control Register contains control bits for interrupt sense control and general MCU functions.
								#define ISC00	0	// 	Interrupt Sense Control 0 Bit 0 	| ISC00 = 0 & ISC01 = 0:The low level of INT0 generates an interrupt request. 		| ISC00 = 1 & ISC01 = 0: The falling edge of INT0 generates an interrupt request.	|
								#define ISC01	1	// 	Interrupt Sense Control 0 Bit 1 	| ISC00 = 0 & ISC01 = 1:Any logical change on INT0 generates an interrupt request.	| ISC00 = 1 & ISC01 = 1: The rising edge of INT0 generates an interrupt request.	|
								#define ISC10	2	// 	Interrupt Sense Control 1 Bit 0 	| ISC10 = 0 & ISC11 = 0:The low level of INT1 generates an interrupt request. 		| ISC10 = 1 & ISC11 = 0: The falling edge of INT1 generates an interrupt request.	|
								#define ISC11	3	// 	Interrupt Sense Control 1 Bit 1 	| ISC10 = 0 & ISC11 = 1:Any logical change on INT1 generates an interrupt request.	| ISC10 = 1 & ISC11 = 1: The rising edge of INT1 generates an interrupt request.	|
#define 	MCUCSR		0x54	//MCU Control and Status Register
								#define ISC2	6	// 	Interrupt Sense Control 2 bit 0	| ISC2	= 0 :A falling edge on INT2 activates the interrupt. | ISC2	= 1 :  A rising edge on INT2 activates the interrupt.
#define		GICR		0x5B	//General Interrupt Control Register
								#define INT0	6	//	External Interrupt Request 0 Enable
								#define INT1	7	//  External Interrupt Request 1 Enable
								#define INT2	5	//	External Interrupt Request 2 Enable
#define 	GIFR		0x5A	//General Interrupt Flag Register
								#define INTF0	6	//	External Interrupt Flag 0
								#define INTF1	7	//  External Interrupt Flag 1
								#define INTF2	5	//	External Interrupt Flag 2
#define 	SREG		0x5F	//Status Register
								#define GI		7	//	Global Interrupt Enable

#endif /* AVR_GPIO_REG_H_ */

