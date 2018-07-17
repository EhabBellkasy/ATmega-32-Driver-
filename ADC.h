/*
 * ADC.h
 *
 *  Created on: 2018/06/14
 *      Author: Ehab Bellkasy
 */

#ifndef ADC_H_
#define ADC_H_
#include "hw_types.h"
#include "AVR_ADC_Reg.h"

typedef enum adcReferenceSelection {
	EXTERNAL_VREF			= 0b00000000, // AREF, Internal Vref turned off
	AVCC 					= 0b01000000, // AVCC with external capacitor at AREF pin
	RESERVED 				= 0b10000000, // Reserved
	INTERNAL_2_56V 			= 0b11000000, // Internal 2.56V Voltage Reference with external capacitor at AREF pin
	
} gADCReferenceSelection_t;


typedef enum adcAnalogChannelGainSelection {
//	NAME					VALUE				Single Ended Input	Positive Differential Input		Negative Differential Input		Gain
	ADC0					= 0b00000000, //	ADC0 				N/A								N/A								N/A
	ADC1 					= 0b00000001, // 	ADC1				N/A								N/A								N/A
	ADC2	 				= 0b00000010, // 	ADC2				N/A								N/A								N/A
	ADC3 					= 0b00000011, // 	ADC3				N/A								N/A								N/A
	ADC4					= 0b00000100, // 	ADC4				N/A								N/A								N/A
	ADC5 					= 0b00000101, // 	ADC5				N/A								N/A								N/A
	ADC6	 				= 0b00000110, // 	ADC6				N/A								N/A								N/A
	ADC7 					= 0b00000111, // 	ADC7				N/A								N/A								N/A
	ADC0_10X				= 0b00001000, // 	N/A					ADC0							ADC0							10X
	ADC1_ADC0_10X 			= 0b00001001, // 	N/A					ADC1							ADC0							10X
	ADC0_200X	 			= 0b00001010, // 	N/A					ADC0							ADC0							200X
	ADC1_ADC0_200X 			= 0b00001011, // 	N/A					ADC1							ADC0							200X
	ADC2_10X				= 0b00001100, // 	N/A					ADC2							ADC2							10X
	ADC3_ADC2_10X 			= 0b00001101, // 	N/A					ADC3							ADC2							10X
	ADC2_200X	 			= 0b00001110, // 	N/A					ADC2							ADC2							200X
	ADC3_ADC2_200X 			= 0b00001111, // 	N/A					ADC3							ADC2							200X
	ADC0_ADC1_1X			= 0b00010000, // 	N/A					ADC0							ADC1							1X
	ADC1_1X 				= 0b00010001, // 	N/A					ADC1							ADC1							1X
	ADC2_ADC1_1X			= 0b00010010, // 	N/A					ADC2							ADC1							1X
	ADC3_ADC1_1X 			= 0b00010011, // 	N/A					ADC3							ADC1							1X
	ADC4_ADC1_1X			= 0b00010100, // 	N/A					ADC4							ADC1							1X
	ADC5_ADC1_1X 			= 0b00010101, // 	N/A					ADC5							ADC1							1X
	ADC6_ADC1_1X			= 0b00010110, // 	N/A					ADC6							ADC1							1X
	ADC7_ADC1_1X 			= 0b00010111, // 	N/A					ADC7							ADC1							1X		
	ADC0_ADC2_1X			= 0b00011000, // 	N/A					ADC0							ADC2							1X
	ADC1_ADC2_1X 			= 0b00011001, // 	N/A					ADC1							ADC2							1X
	ADC2_1X	 				= 0b00011010, // 	N/A					ADC2							ADC2							1X
	ADC3_ADC2_1X 			= 0b00011011, // 	N/A					ADC3							ADC2							1X
	ADC4_ADC2_1X			= 0b00011100, // 	N/A					ADC4							ADC2							1X
	ADC5_ADC2_1X			= 0b00011101, // 	N/A					ADC5							ADC2							1X
	VBG_1_22V	 			= 0b00011110, // 	1.22V (VBG)			N/A								N/A								N/A
	GND 					= 0b00011111, // 	0V (GND)			N/A								N/A								N/A
	
} gADCAnalogChannelGainSelection_t;


typedef enum adcLeftAdjustResult {
	RIGHT_ADJUSTED			= 0b00000000, //	affects the presentation of the ADC conversion result in the ADC Data Register. Write Zero to ADLAR to right adjust the result.
	LEFT_ADJUST 			= 0b00100000, //	affects the presentation of the ADC conversion result in the ADC Data Register. Write one to ADLAR to left adjust the result.	
} gADCLeftAdjustResult_t;



typedef enum adcPrescalerSelect {
	DIVISION_FACTOR2			= 0b00000000, // Division Factor 2
	DIVISION_FACTOR_2 			= 0b00000001, // Division Factor 2
	DIVISION_FACTOR_4 			= 0b00000010, // Division Factor 4
	DIVISION_FACTOR_8 			= 0b00000011, // Division Factor 8
	DIVISION_FACTOR_16			= 0b00000100, // Division Factor 16
	DIVISION_FACTOR_32 			= 0b00000101, // Division Factor 32
	DIVISION_FACTOR_64 			= 0b00000110, // Division Factor 64
	DIVISION_FACTOR_128 		= 0b00000111, // Division Factor 128
	
} gADCPrescalerSelect_t;



typedef enum adcEnable {
	DISABLE			= 0, //	
	Enable 			= 1 //	
} gADCEnable_t;

/*
typedef enum adcStartConversion {
	DISABLE			= 0b00000000, //	
	Enable 			= 0b01000000, //	
} gADCStartConversion_t;


typedef enum adcAutoTrigger {
	DISABLE			= 0b00000000, //	
	Enable 			= 0b00100000, //	
} gADCAutoTrigger_t;



typedef enum adcInterruptFlag {
	DISABLE			= 0b00000000, //	
	Enable 			= 0b00010000, //	
} gADCInterruptFlag_t;

*/

typedef enum adcAutoTriggerSource {
	FREE_RUN_MODE			= 0b00000000, //	Free Running mode
	ANALOG_COMP 			= 0b00100000, // 	Analog Comparator
	EXT_INTE_REQ_0 			= 0b01000000, //	External Interrupt Request 0	
	TIMER0_CM 				= 0b01100000, // 	Timer/Counter0 Compare Match
	TIMER0_OF				= 0b10000000, // 	Timer/Counter0 Overflow
	TIMER1_CMB 				= 0b10100000, // 	Timer/Counter1 Compare Match B
	TIMER1_OF 				= 0b11000000, // 	Timer/Counter1 Overflow
	TIMER1_CE 				= 0b11100000, // 	Timer/Counter1 Capture Event
	
} gADCAutoTriggerSource_t;


typedef struct adc16BitReg {
   uint8_t 						high_Reg;
   uint8_t 						low_Reg;
   
}gADC16BitReg_t;

typedef struct adc16BitRegPtr {
   uint8_t *						high_Reg_Ptr;
   uint8_t *						low_Reg_Ptr;
   
}gADC16BitReg_ptr;

typedef struct {
	gADCAnalogChannelGainSelection_t 		channelADC;
	gADCReferenceSelection_t 				referenceVolt;
	gADCLeftAdjustResult_t 					adjustResult;
	
	gADCEnable_t							enableADC;
	gADCEnable_t							startConversion; 
	gADCEnable_t							interruptFlag;
	gADCEnable_t							interruptEnable;
	
	gADCEnable_t							autoTrigger;
	gADCAutoTriggerSource_t					autoTriggerSource;
	gADCPrescalerSelect_t					prescalerSelections;
	
	gADC16BitReg_t							dataADC;
	gADC16BitReg_ptr						dataPtrADC;
	void (*intFun)(void);
} gIntPinCfg_t;



 typedef enum adcStdErr {
	NO_ERRORS					= 0b00000000,
	ANALOG_CHANNEL_ERR 			= 0b00000001,
	REFERENCE_SELECTION_ERR 	= 0b00000010,
	LEFT_ADJUST_RESULT_ERR		= 0b00000100,
	AUTO_TRIGGER_ERR 			= 0b00001000,
	AUTO_TRIGGER_SOURCE_ERR 	= 0b00010000,
	PRESCALER_SELECTIONS_ERR 	= 0b00100000,
	INTERRUPT_ERR 				= 0b01000000,
	ENABLE_ADC_ERR				= 0b10000000
	
} gADCStdErr_t;


 
gTimerStdErr ADCInitCfg(gTimerCfg_t * obj) ;
gTimerStdErr ADCReadCfg(gTimerCfg_t * obj) ;
gTimerStdErr ADCFastRead(gADC16BitReg_t * obj) ;

#endif /* ADC_H_ */