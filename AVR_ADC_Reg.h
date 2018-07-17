/*
 * AVR_ADC_Reg.h
 *
 *  Created on: 2018/06/14
 *      Author: Ehab Bellkasy
 */
 
#ifndef AVR_ADC_REG_H_
#define AVR_ADC_REG_H_


//ADC		registers 	addresses
#define 	ADCH		0x25	// The ADC Data Register .
#define 	ADCL		0x24	// The ADC Data Register .
#define 	ADMUX		0x27	// Multiplexer Selection Register .
								#define MUX0	0	// 	Analog Channel and Gain Selection Bits
								#define MUX1	1	// 	Analog Channel and Gain Selection Bits
								#define MUX2	2	// 	Analog Channel and Gain Selection Bits
								#define MUX3	3	// 	Analog Channel and Gain Selection Bits
								#define MUX4	4	// 	Analog Channel and Gain Selection Bits
								#define ADLAR	5	// 	ADC Left Adjust Result
								#define REFS0	6	// 	Reference Selection Bits
								#define REFS0	7	// 	Reference Selection Bits
#define 	ADCSRA		0x26	// Control and Status Register A .
								#define ADPS0	0	// 	ADC Prescaler Select Bits.
								#define ADPS1	1	// 	ADC Prescaler Select Bits.
								#define ADPS2	2	// 	ADC Prescaler Select Bits.
								#define ADIE	3	// 	ADC Interrupt Enable.
								#define ADIF	4	// 	ADC Interrupt Flag.
								#define ADATE	5	// 	ADC Auto Trigger Enable.
								#define ADSC	6	// 	ADSC: ADC Start Conversion.
								#define ADEN	7	// 	ADEN: ADC Enable.
#define 	SFIOR		0x50	// Special FunctionIO Register .
								#define PSR10	0	// 	
								#define PSR2	1	// 	
								#define PUD		2	// 	
								#define ACME	3	// 	
							  //#define Reserved_Bit	4	// 	
								#define ADTS0	5	// 	ADC Auto Trigger Source.
								#define ADTS1	6	// 	ADC Auto Trigger Source.
								#define ADTS2	7	// 	ADC Auto Trigger Source.





#endif /* AVR_ADC_REG_H_ */