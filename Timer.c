/*
 * GPIO.c
 *
 *  Created on: 2018/06/11
 *      Author: Ehab Bellkasy
 */
 
 #include "types.h"
#include "hw_types.h"
#include "AVR_Timer_Reg.h"
#include "Timer.h"
#include <avr/interrupt.h>

static void (*ptrT0TOV)	(void);
static void (*ptrT0OC)	(void);
static void (*ptrT1TOV)	(void);
static void (*ptrT1OCB)	(void);
static void (*ptrT1OCA)	(void);
static void (*ptrT1IC)	(void);
static void (*ptrT2TOV)	(void);
static void (*ptrT2OC)	(void);


gTimerStdErr TimerInitCfg(gTimerCfg_t * obj) {
	gTimerStdErr ret = NO_ERRORS;
	
	if (obj->timer == TIMER_0 ) {
		// WAVE FORMG ENERATION AND COMPARE OUTPUT MODE CHECK
		if ((obj->waveFormGeneration.timer_0 == NORMAL )|(obj->waveFormGeneration.timer_0 == CTC) ) { 
			switch (obj->compareOutputMode.timer_0.non_PWM_Mode) {
				case NORMAL  :
				case TOGGLE_OC0  :
				case CLEAR_OC0  :
				case SET_OC0  :
					break;
				default :
					ret |= COMPARE_OUTPUT_MODE_ERR ;
			}
			
		} else if (obj->waveFormGeneration.timer_0 == PHASE_CORRECT_PWM ){
			switch (obj->compareOutputMode.timer_0.phase_Correct_PWM_Mode) {
				case NORMAL  :
				case CLEAR_OC0_MATCH_UPCOUNTING__SET_OC0_MATCH_DOWNCOUNTING  :
				case SET_OC0_MATCH_UPCOUNTING__CLEAR_OC0_MATCH_DOWNCOUNTING  :
				
					break;
				default :
					ret |= COMPARE_OUTPUT_MODE_ERR ;
			}
		
		} else if (obj->waveFormGeneration.timer_0 == FAST_PWM ){
			switch (obj->compareOutputMode.timer_0.fast_PWM_Mode) {
				case NORMAL  :
				case CLEAR_OC0_MATCH__SET_OC0_BOTTOM  :
				case SET_OC0_MATCH__CLEAR_OC0_BOTTOM  :
				
					break;
				default :
					ret |= COMPARE_OUTPUT_MODE_ERR ;
			}						
				
		}	else {
			ret |= WAVE_FORM_GENERATION_ERR ;
		}	
		
		// EXTRA FEATURE CHECK
		switch (obj->extraFeature.timer_0) {
				case DISABLE_ALL  :
				case FOC0  :
					break;
				default :
					ret |= EXTRA_FEATURE_ERR ;
			}	
		// INTERRUPT MASK CHECK
		switch (obj->interruptMask.timer_0) {
				case DISABLE_ALL  :
				case TOIE0  :
				case OCIE0  :
				case OCIE0_TOIE0  :
					break;
				default :
					ret |= INTERRUPT_MASK_ERR ;
			}	
		
		// INTERRUPT FLAG CHECK
		switch (obj->interruptFlag.timer_0) {
				case DISABLE_ALL  :
				case TOV0  :
				case OCF0  :
				case OCF0_TOV0  :
					break;
				default :
					ret |= INTERRUPT_MASK_ERR ;
			}	
		
		// CLOCK SELECT CHECK
		switch (obj->clockSelect.timer_0) {
				case TIMER_STOP  :
				case NO_PRESCALING  :
				case PRESCALER_8  :
				case PRESCALER_64  :
				case PRESCALER_256  :
				case PRESCALER_1024  :
				case T0_PIN_CLOCK_FALLING_EDGE  :
				case T0_PIN_CLOCK_RISING_EDGE  :
					break;
				default :
					ret |= CLOCK_SELECT_ERR ;
			}	
		
		
		if (ret == NO_ERRORS ) {
			// SET  WAVE FORMG ENERATION
			HW_REG(TCCR0) &= ~((1 << WGM01) | (1 << WGM00)); // clear all selected bits
			HW_REG(TCCR0) |= obj->waveFormGeneration.timer_0 &((1 << WGM01) | (1 << WGM00)); // Set as define
			
			// SET COMPARE OUTPUT MODE
			HW_REG(TCCR0) &= ~((1 << COM01) | (1 << COM00)); // clear all selected bits
			HW_REG(TCCR0) |= obj->compareOutputMode.timer_0.non_PWM_Mode &((1 << COM01) | (1 << COM00)); // Set as define
			
			// SET  EXTRA FEATURE 
			HW_REG(TCCR0) &= ~((1 << FOC0) ); // clear all selected bits
			HW_REG(TCCR0) |= obj->extraFeature.timer_0 &((1 << FOC0) ); // Set as define
			
			// SET  COUNTER AND COMPARE AND GIVE ADDRESS OF COUNTER AND COMPARE
			HW_REG(TCNT0) = obj->counter.timer_0 ; 
			HW_REG(OCR0)  = obj->compare.timer_0; 
			 
			obj->counterPtr.timer_0 = TCNT0 ;
			obj->comparePtr.timer_0 = OCR0 ;
			
			// SET  INTERRUPT FUNCTION IN ISR
			ptrT0TOV = obj->intFun[0] ;
			ptrT0OC =  obj->intFun[1] ;
			
			// SET  INTERRUPT MASK 
			HW_REG(TIMSK) &= ~((1 << OCIE0) | (1 << TOIE0)); // clear all selected bits
			HW_REG(TIMSK) |= obj->interruptMask.timer_0 &((1 << OCIE0) | (1 << TOIE0)); // Set as define
			
			// SET  INTERRUPT FLAG
			HW_REG(TIFR) &= ~((1 << OCF0) | (1 << TOV0)); // clear all selected bits
			HW_REG(TIFR) |= obj->interruptFlag.timer_0 &((1 << OCF0) | (1 << TOV0)); // Set as define
			
			// SET  CLOCK SELECT
			HW_REG(TCCR0) &= ~((1 << CS02) | (1 << CS01) | (1 << CS00) ); // clear all selected bits
			HW_REG(TCCR0) |= obj->clockSelect.timer_0 &((1 << CS02) | (1 << CS01) | (1 << CS00) ); // Set as define
			
		} else {
			// Do onthing
		}
	} else if (obj->timer == TIMER_1 ) {
		
		// WAVE FORMG ENERATION AND COMPARE OUTPUT MODE CHECK
		if ( (obj->waveFormGeneration.timer_1 == NORMAL )
		    |(obj->waveFormGeneration.timer_1 == CTC_OCR1A)
			|(obj->waveFormGeneration.timer_1 == CTC_ICR1)
			
			) { 
							switch (obj->compareOutputMode.timer_1.timer1A.non_PWM_Mode) {
								case NORMAL  :
								case TOGGLE_OC1A  :
								case CLEAR_OC1A  :
								case SET_OC1A  :
									break;
								default :
									ret |= COMPARE_OUTPUT_MODE_ERR ;
							}
							switch (obj->compareOutputMode.timer_1.timer1B.non_PWM_Mode) {
								case NORMAL   :
								case TOGGLE_OC1B  :
								case CLEAR_OC1B  :
								case SET_OC1B  :
									break;
								default :
									ret |= COMPARE_OUTPUT_MODE_ERR ;
							}
			
		} else if (	 (obj->waveFormGeneration.timer_1 == PHASE_CORRECT_PWM_8BIT )
					|(obj->waveFormGeneration.timer_1 == PHASE_CORRECT_PWM_9BIT)
					|(obj->waveFormGeneration.timer_1 == PHASE_CORRECT_PWM_10BIT)
					|(obj->waveFormGeneration.timer_1 == PHASE_FREQUENCY_CORRECT_PWM_ICR1)
					|(obj->waveFormGeneration.timer_1 == PHASE_FREQUENCY_CORRECT_PWM_OCR1A)
					|(obj->waveFormGeneration.timer_1 == PHASE_CORRECT_PWM_ICR1)
					|(obj->waveFormGeneration.timer_1 == PHASE_CORRECT_PWM_OCR1A)
			
					) { 
							switch (obj->compareOutputMode.timer_1.timer1A.phase_Correct_PWM_Mode) {
								case NORMAL  :
								case TOGGLE_OC1A_MATCH  :
								case CLEAR_OC1A_MATCH_UPCOUNTING__SET_OC1A_MATCH_DOWNCOUNTING  :
								case SET_OC1A_MATCH_UPCOUNTING__CLEAR_OC1A_MATCH_DOWNCOUNTING  :
									break;
								default :
									ret |= COMPARE_OUTPUT_MODE_ERR ;
							}
							switch (obj->compareOutputMode.timer_1.timer1B.phase_Correct_PWM_Mode) {
								case NORMAL:
								case DISCONNECTED  :
								case CLEAR_OC1B_MATCH_UPCOUNTING__SET_OC1B_MATCH_DOWNCOUNTING  :
								case SET_OC1B_MATCH_UPCOUNTING__CLEAR_OC1B_MATCH_DOWNCOUNTING  :
									break;
								default :
									ret |= COMPARE_OUTPUT_MODE_ERR ;
							}
		
		} else if (  (obj->waveFormGeneration.timer_1 == FAST_PWM_8BIT )
					|(obj->waveFormGeneration.timer_1 == FAST_PWM_9BIT)
					|(obj->waveFormGeneration.timer_1 == FAST_PWM_10BIT)
					|(obj->waveFormGeneration.timer_1 == FAST_PWM_ICR1)
					|(obj->waveFormGeneration.timer_1 == FAST_PWM_OCR1A)
					
			
					) {
							switch (obj->compareOutputMode.timer_1.timer1A.fast_PWM_Mode) {
								case NORMAL  :
								case TOGGLE_OC1A_MATCH  :
								case CLEAR_OC1A_MATCH__SET_OC1A_BOTTOM  :
								case SET_OC1A_MATCH__CLEAR_OC1A_BOTTOM  :
									break;
								default :
									ret |= COMPARE_OUTPUT_MODE_ERR ;
							}
							switch (obj->compareOutputMode.timer_1.timer1B.fast_PWM_Mode) {
								case NORMAL  :
								case DISCONNECTED  :
								case CLEAR_OC1B_MATCH__SET_OC1B_BOTTOM  :
								case SET_OC1B_MATCH__CLEAR_OC1B_BOTTOM  :
									break;
								default :
									ret |= COMPARE_OUTPUT_MODE_ERR ;
							}					
				
		}	else {
			ret |= WAVE_FORM_GENERATION_ERR ;
		}	
		
		// EXTRA FEATURE CHECK
		switch (obj->extraFeature.timer_1) {
				case DISABLE_ALL  :
				case FOC1B  :
				case FOC1A  :
				case FOC1A_FOC1B  :
				case ICES1  :
				case ICES1_FOC1B  :
				case ICES1_FOC1A  :
				case ICES1_FOC1A_FOC1B  :
				case ICNC1  :
				case ICNC1_FOC1B  :
				case ICNC1_FOC1A  :
				case ICNC1_FOC1A_FOC1B  :
				case ICNC1_ICES1  :
				case ICNC1_ICES1_FOC1B  :
				case ICNC1_ICES1_FOC1A  :
				case ICNC1_ICES1_FOC1A_FOC1B  :
					break;
				default :
					ret |= EXTRA_FEATURE_ERR ;
			}	
		// INTERRUPT MASK CHECK
		switch (obj->interruptMask.timer_1) {
				case DISABLE_ALL  :
				case TOIE1  :
				case OCIE1B  :
				case OCIE1B_TOIE1  :
				case OCIE1A  :
				case OCIE1A_TOIE1  :
				case OCIE1A_OCIE1B  :
				case OCIE1A_OCIE1B_TOIE1  :
				case TICIE1  :
				case TICIE1_TOIE1  :
				case TICIE1_OCIE1B  :
				case TICIE1_OCIE1B_TOIE1  :
				case TICIE1_OCIE1A  :
				case TICIE1_OCIE1A_TOIE1  :
				case TICIE1_OCIE1A_OCIE1B  :
				case TICIE1_OCIE1A_OCIE1B_TOIE1  :
					break;
				default :
					ret |= INTERRUPT_MASK_ERR ;
			}	
		
		// INTERRUPT FLAG CHECK
		switch (obj->interruptFlag.timer_1) {
				case DISABLE_ALL  :
				case TOV1  :
				case OCF1B  :
				case OCF1B_TOV1  :
				case OCF1A  :
				case OCF1A_TOV1  :
				case OCF1A_OCF1B  :
				case OCF1A_OCF1B_TOV1  :
				case ICF1  :
				case ICF1_TOV1  :
				case ICF1_OCF1B  :
				case ICF1_OCF1B_TOV1  :
				case ICF1_OCF1A  :
				case ICF1_OCF1A_TOV1  :
				case ICF1_OCF1A_OCF1B  :
				case ICF1_OCF1A_OCF1B_TOV1  :
					break;
				default :
					ret |= INTERRUPT_MASK_ERR ;
			}	
		
		// CLOCK SELECT CHECK
		switch (obj->clockSelect.timer_1) {
				case TIMER_STOP  :
				case NO_PRESCALING  :
				case PRESCALER_8  :
				case PRESCALER_64  :
				case PRESCALER_256  :
				case PRESCALER_1024  :
				case T1_PIN_CLOCK_FALLING_EDGE  :
				case T1_PIN_CLOCK_RISING_EDGE  :
					break;
				default :
					ret |= CLOCK_SELECT_ERR ;
			}	
		
		
		if (ret == NO_ERRORS ) {
			// SET  WAVE FORMG ENERATION
			HW_REG(TCCR1A) &= ~((1 << WGM11) | (1 << WGM10)); // clear all selected bits
			HW_REG(TCCR1A) |= obj->waveFormGeneration.timer_1 &((1 << WGM11) | (1 << WGM10)); // Set as define
			
			HW_REG(TCCR1B) &= ~((1 << WGM13) | (1 << WGM12)); // clear all selected bits
			HW_REG(TCCR1B) |= obj->waveFormGeneration.timer_1 &((1 << WGM13) | (1 << WGM12)); // Set as define
			
			
			// SET COMPARE OUTPUT MODE
			HW_REG(TCCR1A) &= ~((1 << COM1A1) | (1 << COM1A0)); // clear all selected bits
			HW_REG(TCCR1A) |= obj->compareOutputMode.timer_1.non_PWM_Mode &((1 << COM1A1) | (1 << COM1A0)); // Set as define
			
			HW_REG(TCCR1A) &= ~((1 << COM1B1) | (1 << COM1B0)); // clear all selected bits
			HW_REG(TCCR1A) |= obj->compareOutputMode.timer_1.non_PWM_Mode &((1 << COM1B1) | (1 << COM1B0)); // Set as define
			
			
			// SET  EXTRA FEATURE 
			HW_REG(TCCR1A) &= ~((1 << FOC1A) | (1 << FOC1B) ); // clear all selected bits
			HW_REG(TCCR1A) |= obj->extraFeature.timer_1 &((1 << FOC1A) | (1 << FOC1B) ); // Set as define
			
			HW_REG(TCCR1B) &= ~((1 << ICNC1) | (1 << ICES1) ); // clear all selected bits
			HW_REG(TCCR1B) |= obj->extraFeature.timer_1 &((1 << ICNC1) | (1 << ICES1) ); // Set as define
			
			// SET  COUNTER AND COMPARE AND GIVE ADDRESS OF COUNTER AND COMPARE
			HW_REG(TCNT1H) = obj->counter.timer_1.high_Reg ; 
			HW_REG(TCNT1L) = obj->counter.timer_1.low_Reg ; 
			
			HW_REG(OCR1AH)  = obj->compare.timer_1.compare_A.high_Reg; 
			HW_REG(OCR1AL)  = obj->compare.timer_1.compare_A.low_Reg;
			
			HW_REG(OCR1BH)  = obj->compare.timer_1.compare_B.high_Reg; 
			HW_REG(OCR1BL)  = obj->compare.timer_1.compare_B.low_Reg;
			
			HW_REG(ICR1H) = obj->inputCapture.timer_1.high_Reg ; 
			HW_REG(ICR1L) = obj->inputCapture.timer_1.low_Reg ;
			 
			obj->counterPtr.timer_1.high_Reg = TCNT1H ;
			obj->counterPtr.timer_1.low_Reg = TCNT1L ;
			
			obj->comparePtr.timer_1.compare_A.high_Reg = OCR1AH ;
			obj->comparePtr.timer_1.compare_A.low_Reg = OCR1AL ;
			
			obj->comparePtr.timer_1.compare_B.high_Reg = OCR1BH ;
			obj->comparePtr.timer_1.compare_B.low_Reg = OCR1BL ;
			
			obj->inputCapturePtr.timer_1.high_Reg = ICR1H ;
			obj->inputCapturePtr.timer_1.low_Reg = ICR1L ;
			
			// SET  INTERRUPT FUNCTION IN ISR
			ptrT1TOV = obj->intFun[0] ;
			ptrT1OCB = obj->intFun[1] ;
			ptrT1OCA = obj->intFun[2] ;
			ptrT1IC  = obj->intFun[3] ;
			
			// SET  INTERRUPT MASK 
			HW_REG(TIMSK) &= ~((1 << TICIE1) | (1 << OCIE1A) | (1 << OCIE1B) | (1 << TOIE1) ); // clear all selected bits
			HW_REG(TIMSK) |= obj->interruptMask.timer_1 &((1 << TICIE1) | (1 << OCIE1A) | (1 << OCIE1B) | (1 << TOIE1) ); // Set as define
			
			// SET  INTERRUPT FLAG
			HW_REG(TIFR) &= ~((1 << OCF0) | (1 << TOV0) | (1 << TOV0) | (1 << TOV0) ); // clear all selected bits
			HW_REG(TIFR) |= obj->interruptFlag.timer_1 &((1 << OCF0) | (1 << TOV0) | (1 << TOV0) | (1 << TOV0) ); // Set as define
			
			// SET  CLOCK SELECT
			HW_REG(TCCR1B) &= ~((1 << CS12) | (1 << CS11) | (1 << CS10) ); // clear all selected bits
			HW_REG(TCCR1B) |= obj->clockSelect.timer_1 &((1 << CS12) | (1 << CS11) | (1 << CS10) ); // Set as define
			
		} else {
			// Do onthing
		}
	
	} else if (obj->timer == TIMER_2 ) {
		// WAVE FORMG ENERATION AND COMPARE OUTPUT MODE CHECK
		if ((obj->waveFormGeneration.timer_2 == NORMAL )|(obj->waveFormGeneration.timer_2 == CTC) ) { 
			switch (obj->compareOutputMode.timer_2.non_PWM_Mode) {
				case NORMAL  :
				case TOGGLE_OC2  :
				case CLEAR_OC2  :
				case SET_OC2  :
					break;
				default :
					ret |= COMPARE_OUTPUT_MODE_ERR ;
			}
			
		} else if (obj->waveFormGeneration.timer_2 == PHASE_CORRECT_PWM ){
			switch (obj->compareOutputMode.timer_2.phase_Correct_PWM_Mode) {
				case NORMAL  :
				case CLEAR_OC2_MATCH_UPCOUNTING__SET_OC2_MATCH_DOWNCOUNTING  :
				case SET_OC2_MATCH_UPCOUNTING__CLEAR_OC2_MATCH_DOWNCOUNTING  :
				
					break;
				default :
					ret |= COMPARE_OUTPUT_MODE_ERR ;
			}
		
		} else if (obj->waveFormGeneration.timer_2 == FAST_PWM ){
			switch (obj->compareOutputMode.timer_2.fast_PWM_Mode) {
				case NORMAL  :
				case CLEAR_OC2_MATCH__SET_OC2_BOTTOM  :
				case SET_OC2_MATCH__CLEAR_OC2_BOTTOM  :
				
					break;
				default :
					ret |= COMPARE_OUTPUT_MODE_ERR ;
			}						
				
		}	else {
			ret |= WAVE_FORM_GENERATION_ERR ;
		}	
		
		// EXTRA FEATURE CHECK
		switch (obj->extraFeature.timer_2) {
				case DISABLE_ALL  :
				case AS2  :
				case FOC2  :
				case FOC2_AS2  :
					break;
				default :
					ret |= EXTRA_FEATURE_ERR ;
			}	
		// INTERRUPT MASK CHECK
		switch (obj->interruptMask.timer_2) {
				case DISABLE_ALL  :
				case TOIE2  :
				case OCIE2  :
				case OCIE2_TOIE2  :
					break;
				default :
					ret |= INTERRUPT_MASK_ERR ;
			}	
		
		// INTERRUPT FLAG CHECK
		switch (obj->interruptFlag.timer_2) {
				case DISABLE_ALL  :
				case TOV2  :
				case OCF2  :
				case OCF2_TOV2  :
					break;
				default :
					ret |= INTERRUPT_MASK_ERR ;
			}	
		
		// CLOCK SELECT CHECK
		switch (obj->clockSelect.timer_2) {
				case TIMER_STOP  :
				case NO_PRESCALING  :
				case PRESCALER_8  :
				case PRESCALER_32  :
				case PRESCALER_64  :
				case PRESCALER_128  :
				case PRESCALER_256  :
				case PRESCALER_1024  :
					break;
				default :
					ret |= CLOCK_SELECT_ERR ;
			}	
		
		
		if (ret == NO_ERRORS ) {
			// SET  WAVE FORMG ENERATION
			HW_REG(TCCR2) &= ~((1 << WGM21) | (1 << WGM20)); // clear all selected bits
			HW_REG(TCCR2) |= obj->waveFormGeneration.timer_2 &((1 << WGM21) | (1 << WGM20)); // Set as define
			
			// SET COMPARE OUTPUT MODE
			HW_REG(TCCR2) &= ~((1 << COM21) | (1 << COM20)); // clear all selected bits
			HW_REG(TCCR2) |= obj->compareOutputMode.timer_2.non_PWM_Mode &((1 << COM21) | (1 << COM20)); // Set as define
			
			// SET  EXTRA FEATURE 
			HW_REG(TCCR2) &= ~((1 << FOC2) ); // clear all selected bits
			HW_REG(TCCR2) |= obj->extraFeature.timer_2 &((1 << FOC2) ); // Set as define
			
			HW_REG(ASSR) &= ~((1 << AS2) ); // clear all selected bits
			HW_REG(ASSR) |= obj->extraFeature.timer_2 &((1 << AS2) ); // Set as define
			
			// SET  COUNTER AND COMPARE AND GIVE ADDRESS OF COUNTER AND COMPARE
			HW_REG(TCNT2) = obj->counter.timer_2 ; 
			HW_REG(OCR2)  = obj->compare.timer_2; 
			 
			obj->counterPtr.timer_2 = TCNT2 ;
			obj->comparePtr.timer_2 = OCR2 ;
			
			// SET  INTERRUPT FUNCTION IN ISR
			ptrT2TOV = obj->intFun[0] ;
			ptrT2OC =  obj->intFun[1] ;
			
			// SET  INTERRUPT MASK 
			HW_REG(TIMSK) &= ~((1 << OCIE2) | (1 << TOIE2)); // clear all selected bits
			HW_REG(TIMSK) |= obj->interruptMask.timer_2 &((1 << OCIE2) | (1 << TOIE2)); // Set as define
			
			// SET  INTERRUPT FLAG
			HW_REG(TIFR) &= ~((1 << OCF2) | (1 << TOV2)); // clear all selected bits
			HW_REG(TIFR) |= obj->interruptFlag.timer_2 &((1 << OCF2) | (1 << TOV2)); // Set as define
			
			// SET  CLOCK SELECT
			HW_REG(TCCR2) &= ~((1 << CS22) | (1 << CS21) | (1 << CS20) ); // clear all selected bits
			HW_REG(TCCR2) |= obj->clockSelect.timer_2 &((1 << CS22) | (1 << CS21) | (1 << CS20) ); // Set as define
			
		} else {
			// Do onthing
		}
		
	} else {
		ret |= TIMER_NUMBER_ERR ;
	}

	
return ret ;	
} // End of Function 


gTimerStdErr TimerReadCfg(gTimerCfg_t * obj) {
	gTimerStdErr ret = NO_ERRORS;	
	//======================================================================================
	// Timer 0
	if (obj->timer == TIMER_0 ) {
			// SET  WAVE FORMG ENERATION
			obj->waveFormGeneration.timer_0  &= ~((1 << WGM01) | (1 << WGM00)); // clear all selected bits
			obj->waveFormGeneration.timer_0  |= HW_REG(TCCR0) &((1 << WGM01) | (1 << WGM00)); // Set as define
			
			// SET COMPARE OUTPUT MODE
			obj->compareOutputMode.timer_0.non_PWM_Mode &= ~((1 << COM01) | (1 << COM00)); // clear all selected bits
			obj->compareOutputMode.timer_0.non_PWM_Mode |= HW_REG(TCCR0) &((1 << COM01) | (1 << COM00)); // Set as define
			
			// SET  EXTRA FEATURE 
			HW_REG(TCCR0) &= ~((1 << FOC0) ); // clear all selected bits
			HW_REG(TCCR0) |= obj->extraFeature.timer_0 &((1 << FOC0) ); // Set as define
			
			// SET  COUNTER AND COMPARE AND GIVE ADDRESS OF COUNTER AND COMPARE
			 obj->counter.timer_0 = HW_REG(TCNT0) ; 
			 obj->compare.timer_0 = HW_REG(OCR0)  ; 
			 
			obj->counterPtr.timer_0 = TCNT0 ;
			obj->comparePtr.timer_0 = OCR0 ;
			
			// SET  INTERRUPT FUNCTION IN ISR
			obj->intFun[0] = ptrT0TOV ;
			obj->intFun[1] = ptrT0OC  ;
			
			// SET  INTERRUPT MASK 
			obj->interruptMask.timer_0 &= ~((1 << OCIE0) | (1 << TOIE0)); // clear all selected bits
			obj->interruptMask.timer_0 |= HW_REG(TIMSK)  &((1 << OCIE0) | (1 << TOIE0)); // Set as define
			
			// SET  INTERRUPT FLAG
			obj->interruptFlag.timer_0 &= ~((1 << OCF0) | (1 << TOV0)); // clear all selected bits
			obj->interruptFlag.timer_0 |= HW_REG(TIFR) &((1 << OCF0) | (1 << TOV0)); // Set as define
			
			// SET  CLOCK SELECT
			obj->clockSelect.timer_0 &= ~((1 << CS02) | (1 << CS01) | (1 << CS00) ); // clear all selected bits
			obj->clockSelect.timer_0 |= HW_REG(TCCR0) &((1 << CS02) | (1 << CS01) | (1 << CS00) ); // Set as define
			

	//======================================================================================
	// Timer 1
	} else if (obj->timer == TIMER_1) {
			// SET  WAVE FORMG ENERATION
			obj->waveFormGeneration.timer_1 &= ~((1 << WGM11) | (1 << WGM10)); // clear all selected bits
			obj->waveFormGeneration.timer_1 |= HW_REG(TCCR1A) &((1 << WGM11) | (1 << WGM10)); // Set as define
			
			obj->waveFormGeneration.timer_1 &= ~((1 << WGM13) | (1 << WGM12)); // clear all selected bits
			obj->waveFormGeneration.timer_1 |= HW_REG(TCCR1B) &((1 << WGM13) | (1 << WGM12)); // Set as define
			
			
			// SET COMPARE OUTPUT MODE
			obj->compareOutputMode.timer_1.non_PWM_Mode &= ~((1 << COM1A1) | (1 << COM1A0)); // clear all selected bits
			obj->compareOutputMode.timer_1.non_PWM_Mode |= HW_REG(TCCR1A) &((1 << COM1A1) | (1 << COM1A0)); // Set as define
			
			obj->compareOutputMode.timer_1.non_PWM_Mode &= ~((1 << COM1B1) | (1 << COM1B0)); // clear all selected bits
			obj->compareOutputMode.timer_1.non_PWM_Mode |= HW_REG(TCCR1A) &((1 << COM1B1) | (1 << COM1B0)); // Set as define
			
			
			// SET  EXTRA FEATURE 
			obj->extraFeature.timer_1 &= ~((1 << FOC1A) | (1 << FOC1B) ); // clear all selected bits
			obj->extraFeature.timer_1 |= HW_REG(TCCR1A) &((1 << FOC1A) | (1 << FOC1B) ); // Set as define
			
			obj->extraFeature.timer_1 &= ~((1 << ICNC1) | (1 << ICES1) ); // clear all selected bits
			obj->extraFeature.timer_1 |= HW_REG(TCCR1B) &((1 << ICNC1) | (1 << ICES1) ); // Set as define
			
			// SET  COUNTER AND COMPARE AND GIVE ADDRESS OF COUNTER AND COMPARE
			obj->counter.timer_1.high_Reg = HW_REG(TCNT1H) ; 
			obj->counter.timer_1.low_Reg  = HW_REG(TCNT1L) ; 
			
			obj->compare.timer_1.compare_A.high_Reg = HW_REG(OCR1AH)  ; 
			obj->compare.timer_1.compare_A.low_Reg  = HW_REG(OCR1AL)  ;
			
			obj->compare.timer_1.compare_B.high_Reg = HW_REG(OCR1BH)  ; 
			obj->compare.timer_1.compare_B.low_Reg  = HW_REG(OCR1BL)  ;
			
			obj->inputCapture.timer_1.high_Reg = HW_REG(ICR1H)  ; 
			obj->inputCapture.timer_1.low_Reg = HW_REG(ICR1L)   ;
			 
			obj->counterPtr.timer_1.high_Reg = TCNT1H ;
			obj->counterPtr.timer_1.low_Reg = TCNT1L ;
			
			obj->comparePtr.timer_1.compare_A.high_Reg = OCR1AH ;
			obj->comparePtr.timer_1.compare_A.low_Reg = OCR1AL ;
			
			obj->comparePtr.timer_1.compare_B.high_Reg = OCR1BH ;
			obj->comparePtr.timer_1.compare_B.low_Reg = OCR1BL ;
			
			obj->inputCapturePtr.timer_1.high_Reg = ICR1H ;
			obj->inputCapturePtr.timer_1.low_Reg = ICR1L ;
			
			// SET  INTERRUPT FUNCTION IN ISR
			obj->intFun[0] = ptrT1TOV ;
			obj->intFun[1] = ptrT1OCB ;
			obj->intFun[2] = ptrT1OCA ;
			obj->intFun[3] = ptrT1IC  ;
			
			// SET  INTERRUPT MASK 
			obj->interruptMask.timer_1 &= ~((1 << TICIE1) | (1 << OCIE1A) | (1 << OCIE1B) | (1 << TOIE1) ); // clear all selected bits
			obj->interruptMask.timer_1 |= HW_REG(TIMSK) &((1 << TICIE1) | (1 << OCIE1A) | (1 << OCIE1B) | (1 << TOIE1) ); // Set as define
			
			// SET  INTERRUPT FLAG
			obj->interruptFlag.timer_1 &= ~((1 << OCF0) | (1 << TOV0) | (1 << TOV0) | (1 << TOV0) ); // clear all selected bits
			obj->interruptFlag.timer_1 |= HW_REG(TIFR) &((1 << OCF0) | (1 << TOV0) | (1 << TOV0) | (1 << TOV0) ); // Set as define
			
			// SET  CLOCK SELECT
			obj->clockSelect.timer_1 &= ~((1 << CS12) | (1 << CS11) | (1 << CS10) ); // clear all selected bits
			obj->clockSelect.timer_1 |= HW_REG(TCCR1B) &((1 << CS12) | (1 << CS11) | (1 << CS10) ); // Set as define
			
	
	
	//======================================================================================
	// Timer 2
	}else if (obj->timer == TIMER_2) {
			// SET  WAVE FORMG ENERATION
			obj->waveFormGeneration.timer_2 &= ~((1 << WGM21) | (1 << WGM20)); // clear all selected bits
			obj->waveFormGeneration.timer_2 |= HW_REG(TCCR2) &((1 << WGM21) | (1 << WGM20)); // Set as define
			
			// SET COMPARE OUTPUT MODE
			obj->compareOutputMode.timer_2.non_PWM_Mode &= ~((1 << COM21) | (1 << COM20)); // clear all selected bits
			obj->compareOutputMode.timer_2.non_PWM_Mode |= HW_REG(TCCR2) &((1 << COM21) | (1 << COM20)); // Set as define
			
			// SET  EXTRA FEATURE 
			obj->extraFeature.timer_2 &= ~((1 << FOC2) ); // clear all selected bits
			obj->extraFeature.timer_2 |= HW_REG(TCCR2) &((1 << FOC2) ); // Set as define
			
			obj->extraFeature.timer_2 &= ~((1 << AS2) ); // clear all selected bits
			obj->extraFeature.timer_2 |= HW_REG(ASSR) &((1 << AS2) ); // Set as define
			
			// SET  COUNTER AND COMPARE AND GIVE ADDRESS OF COUNTER AND COMPARE
			obj->counter.timer_2 = HW_REG(TCNT2) ; 
			obj->compare.timer_2 = HW_REG(OCR2)  ; 
			 
			obj->counterPtr.timer_2 = TCNT2 ;
			obj->comparePtr.timer_2 = OCR2 ;
			
			// SET  INTERRUPT FUNCTION IN ISR
			obj->intFun[0] = ptrT2TOV ;
			obj->intFun[1] = ptrT2OC  ;
			
			// SET  INTERRUPT MASK 
			obj->interruptMask.timer_2 &= ~((1 << OCIE2) | (1 << TOIE2)); // clear all selected bits
			obj->interruptMask.timer_2 |= HW_REG(TIMSK) &((1 << OCIE2) | (1 << TOIE2)); // Set as define
			
			// SET  INTERRUPT FLAG
			obj->interruptFlag.timer_2 &= ~((1 << OCF2) | (1 << TOV2)); // clear all selected bits
			obj->interruptFlag.timer_2 |= HW_REG(TIFR) &((1 << OCF2) | (1 << TOV2)); // Set as define
			
			// SET  CLOCK SELECT
			obj->clockSelect.timer_2 &= ~((1 << CS22) | (1 << CS21) | (1 << CS20) ); // clear all selected bits
			obj->clockSelect.timer_2 |= HW_REG(TCCR2) &((1 << CS22) | (1 << CS21) | (1 << CS20) ); // Set as define
			




	//======================================================================================
	// Timer Error
	} else {
		ret |= TIMER_NUMBER_ERR ;
	}
	return ret ;	
} // End of Function




ISR(TIMER0_OVF_vect) {
	(*ptrT0TOV)();
}

ISR(TIMER0_COMP_vect) {
	(*ptrT0OC)();
}

ISR(TIMER1_OVF_vect) {
	(*ptrT1TOV)();
}
ISR(TIMER1_COMPB_vect) {
	(*ptrT1OCB)();
}
ISR(TIMER1_COMPA_vect) {
	(*ptrT1OCA)();
}
ISR(TIMER1_CAPT_vect) {
	(*ptrT1IC)();
}

ISR(TIMER2_OVF_vect) {
	(*ptrT2TOV)();
}

ISR(TIMER2_COMP_vect) {
	(*ptrT2OC)();
}
