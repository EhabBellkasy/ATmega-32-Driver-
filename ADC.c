/*
 * ADC.h
 *
 *  Created on: 2018/06/14
 *      Author: Ehab Bellkasy
 */

#include "types.h"
#include "hw_types.h"
#include "AVR_ADC_Reg.h"
#include "ADC.h"
#include <avr/interrupt.h>


static void (*ptrADC)	(void);


gTimerStdErr ADCInitCfg(gTimerCfg_t * obj) {
	
	gTimerStdErr ret = NO_ERRORS;
	
			// Analog Channel and Gain Selection
			switch (obj->channelADC) {
				case ADC0  :
				case ADC1  :
				case ADC2  :
				case ADC3  :
				case ADC4  :
				case ADC5  :
				case ADC6  :
				case ADC7  :
				case ADC0_10X  :
				case ADC1_ADC0_10X  :
				case ADC0_200X  :
				case ADC1_ADC0_200X  :
				case ADC2_10X  :
				case ADC3_ADC2_10X  :
				case ADC2_200X  :
				case ADC3_ADC2_200X  :
				case ADC0_ADC1_1X  :
				case ADC1_1X  :
				case ADC2_ADC1_1X  :
				case ADC3_ADC1_1X  :
				case ADC4_ADC1_1X  :
				case ADC5_ADC1_1X  :
				case ADC6_ADC1_1X  :
				case ADC7_ADC1_1X  :
				case ADC0_ADC2_1X  :
				case ADC1_ADC2_1X  :
				case ADC2_1X  :
				case ADC3_ADC2_1X  :
				case ADC4_ADC2_1X  :
				case ADC5_ADC2_1X  :
				case VBG_1_22V  :
				case GND  :
					HW_REG(ADMUX) &= ~((1 << MUX4) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0) ); // clear all selected bits
					HW_REG(ADMUX) |= obj->channelADC &((1 << MUX4) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0)); // Set as define
					break;
				default :
					ret |= ANALOG_CHANNEL_ERR ;
			}
			
			// select the voltage reference for the ADC
			switch (obj->referenceVolt) {
				case EXTERNAL_VREF  :
				case AVCC  :
				case INTERNAL_2_56V  :
					HW_REG(ADMUX) &= ~((1 << REFS1) | (1 << REFS0)  ); // clear all selected bits
					HW_REG(ADMUX) |= obj->referenceVolt &((1 << REFS1) | (1 << REFS0) ); // Set as define
					break;
				default :
					ret |= REFERENCE_SELECTION_ERR ;
			}
			
			//	 the presentation of the ADC conversion result in the ADC Data Register.
			switch (obj->adjustResult) {
				case RIGHT_ADJUSTED  :
				case LEFT_ADJUST  :
					HW_REG(ADMUX) &= ~((1 << ADLAR)   ); // clear all selected bits
					HW_REG(ADMUX) |= obj->adjustResult &((1 << ADLAR)  ); // Set as define
					break;
				default :
					ret |= LEFT_ADJUST_RESULT_ERR ;
			}
			
			//	 Enables the ADC.
			switch (obj->enableADC) {
				case DISABLE  :
				case Enable  :
					HW_REG(ADCSRA) &= ~((1 << ADEN)   ); // clear all selected bits
					HW_REG(ADCSRA) |= (obj->enableADC << ADEN)&((1 << ADEN)  ); // Set as define
					break;
				default :
					ret |= ENABLE_ADC_ERR ;
			}
			
			//	Start each conversion. In Free Running Mode
			switch (obj->startConversion) {
				case DISABLE  :
				case Enable  :
					HW_REG(ADCSRA) &= ~((1 << ADSC)   ); // clear all selected bits
					HW_REG(ADCSRA) |= (obj->startConversion << ADSC)&((1 << ADSC)  ); // Set as define
					break;
				default :
					ret |= ENABLE_ADC_ERR ;
			}
			
			//	when an ADC conversion completes and the Data Registers are updated.
			switch (obj->interruptFlag) {
				case DISABLE  :
				case Enable  :
					HW_REG(ADCSRA) &= ~((1 << ADIF)   ); // clear all selected bits
					HW_REG(ADCSRA) |= (obj->interruptFlag << ADIF)&((1 << ADIF)  ); // Set as define
					break;
				default :
					ret |= INTERRUPT_ERR ;
			}
			
			//	the ADC Conversion Complete Interrupt is activated.
			switch (obj->interruptEnable) {
				case DISABLE  :
				case Enable  :
					HW_REG(ADCSRA) &= ~((1 << ADIE)   ); // clear all selected bits
					HW_REG(ADCSRA) |= (obj->interruptEnable << ADIE)&((1 << ADIE)  ); // Set as define
					break;
				default :
					ret |= INTERRUPT_ERR ;
			}
			
			//	Auto Triggering of the ADC isenabled
			switch (obj->autoTrigger) {
				case DISABLE  :
				case Enable  :
					HW_REG(ADCSRA) &= ~((1 << ADATE)   ); // clear all selected bits
					HW_REG(ADCSRA) |= (obj->autoTrigger << ADATE)&((1 << ADATE)  ); // Set as define
					break;
				default :
					ret |= AUTO_TRIGGER_ERR ;
			}
			
			//	 A conversion will be triggered by the rising edge of the selected Interrupt Flag.
			switch (obj->autoTriggerSource) {
				case FREE_RUN_MODE  :
				case ANALOG_COMP  :
				case EXT_INTE_REQ_0  :
				case TIMER0_CM  :
				case TIMER0_OF  :
				case TIMER1_CMB  :
				case TIMER1_OF  :
				case TIMER1_CE  :
					HW_REG(SFIOR) &= ~((1 << ADTS2) | (1 << ADTS1) | (1 << ADTS0) ); // clear all selected bits
					HW_REG(SFIOR) |= obj->autoTriggerSource &((1 << ADTS2) | (1 << ADTS1) | (1 << ADTS0) ); // Set as define
					break;
				default :
					ret |= AUTO_TRIGGER_SOURCE_ERR ;
			}
			//	determine the division factor between the XTAL frequency and the input clock to the ADC.
			switch (obj->prescalerSelections) {
				case DIVISION_FACTOR2  :
				case DIVISION_FACTOR_2  :
				case DIVISION_FACTOR_4  :
				case DIVISION_FACTOR_8  :
				case DIVISION_FACTOR_16  :
				case DIVISION_FACTOR_32  :
				case DIVISION_FACTOR_64  :
				case DIVISION_FACTOR_128  :
					HW_REG(ADCSRA) &= ~((1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) ); // clear all selected bits
					HW_REG(ADCSRA) |= obj->autoTriggerSource &((1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) ); // Set as define
					break;
				default :
					ret |= PRESCALER_SELECTIONS_ERR ;
			}
			
			while (   ( HW_REG(ADCSRA) & ((1 << ADSC) )    )  ;  // waite for read become ready
			
			// Get  ADC data AND GIVE ADDRESS OF ADC data register
			obj->dataADC.high_Reg = HW_REG(ADCH);
			obj->dataADC.low_Reg  = HW_REG(ADCL);
			
			obj->dataPtrADC.high_Reg_Ptr = (ADCH);
			obj->dataPtrADC.low_Reg_Ptr  = (ADCL);
			
			ptrADC = obj->intFun;
			
return ret ; 
}//	End of Function



gTimerStdErr ADCReadCfg(gTimerCfg_t * obj) {
	
	gTimerStdErr ret = NO_ERRORS;
	
	
	// Analog Channel and Gain Selection
	obj->channelADC &= ~((1 << MUX4) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0) ); // clear all selected bits
	obj->channelADC |= HW_REG(ADMUX) &((1 << MUX4) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0)); // Set as define
	
	// select the voltage reference for the ADC
	obj->referenceVolt &= ~((1 << REFS1) | (1 << REFS0)  ); // clear all selected bits
	obj->referenceVolt |= HW_REG(ADMUX) &((1 << REFS1) | (1 << REFS0) ); // Set as define
	
	//	 the presentation of the ADC conversion result in the ADC Data Register.
	obj->adjustResult &= ~((1 << ADLAR)   ); // clear all selected bits
	obj->adjustResult |= HW_REG(ADMUX) &((1 << ADLAR)  ); // Set as define
	
	//	 Enables the ADC.
	obj->enableADC &= ~((1 << ADEN)   ); // clear all selected bits
	obj->enableADC |= ( ( HW_REG(ADCSRA) & (1 << ADEN) ) >> ADEN ); // Set as define
	
	//	Start each conversion. In Free Running Mode
	obj->startConversion &= ~((1 << ADSC)   ); // clear all selected bits
	obj->startConversion |= ( ( HW_REG(ADCSRA) & (1 << ADSC) ) >> ADSC ); // Set as define	
	
	//	when an ADC conversion completes and the Data Registers are updated.
	obj->interruptFlag &= ~((1 << ADIF)   ); // clear all selected bits
	obj->interruptFlag |= ( ( HW_REG(ADCSRA) & (1 << ADIF) ) >> ADIF ); // Set as define
					
	//	the ADC Conversion Complete Interrupt is activated.
	obj->interruptEnable &= ~((1 << ADIE)   ); // clear all selected bits
	obj->interruptEnable |= ( ( HW_REG(ADCSRA) & (1 << ADIE) ) >> ADIE ); // Set as define

	//	Auto Triggering of the ADC isenabled
	obj->autoTrigger &= ~((1 << ADATE)   ); // clear all selected bits
	obj->autoTrigger |= ( ( HW_REG(ADCSRA) & (1 << ADATE) ) >> ADATE ); // Set as define

	//	 A conversion will be triggered by the rising edge of the selected Interrupt Flag.
	obj->autoTriggerSource &= ~((1 << ADTS2) | (1 << ADTS1) | (1 << ADTS0) ); // clear all selected bits
	obj->autoTriggerSource |= HW_REG(SFIOR) &((1 << ADTS2) | (1 << ADTS1) | (1 << ADTS0) ); // Set as define

	//	determine the division factor between the XTAL frequency and the input clock to the ADC.
	obj->autoTriggerSource &= ~((1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) ); // clear all selected bits
	obj->autoTriggerSource |= HW_REG(ADCSRA) &((1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) ); // Set as define

	// Get  ADC data AND GIVE ADDRESS OF ADC data register
	obj->dataADC.high_Reg = HW_REG(ADCH);
	obj->dataADC.low_Reg  = HW_REG(ADCL);
			
	obj->dataPtrADC.high_Reg_Ptr = (ADCH);
	obj->dataPtrADC.low_Reg_Ptr  = (ADCL);
			
	ptrADC = obj->intFun;
			
return ret ; 
}//	End of Function





gTimerStdErr ADCFastRead(gADC16BitReg_t * obj) {
	
	gTimerStdErr ret = NO_ERRORS;
	
	obj->high_Reg = HW_REG(ADCH);
	obj->low_Reg  = HW_REG(ADCL);
	
	
				
	return ret ; 
}//	End of Function



ISR(ADC_vect)
{
 
	 (*ptrADC)();
	 
	 
}
 
