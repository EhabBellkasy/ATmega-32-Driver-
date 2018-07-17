/*
 * Timer.h
 *
 * Created: 2018/06/10
 *  Author: Ehab Bellkasy
 */ 


#ifndef TIMER_H_
#define TIMER_H_

#include "hw_types.h"
#include "AVR_Timer_Reg.h"

//======================================================================================
typedef enum timer0ClockSelect {
	TIMER_STOP					= 0,
	NO_PRESCALING 				= 1,
	PRESCALER_8 				= 2,
	PRESCALER_64 				= 3,
	PRESCALER_256 				= 4,
	PRESCALER_1024 				= 5,
	T0_PIN_CLOCK_FALLING_EDGE 	= 6,
	T0_PIN_CLOCK_RISING_EDGE 	= 7
	
} gTimer0ClockSelect;

typedef enum timer1ClockSelect {
	TIMER_STOP					= 0,
	NO_PRESCALING 				= 1,
	PRESCALER_8 				= 2,
	PRESCALER_64 				= 3,
	PRESCALER_256 				= 4,
	PRESCALER_1024 				= 5,
	T1_PIN_CLOCK_FALLING_EDGE 	= 6,
	T1_PIN_CLOCK_RISING_EDGE 	= 7
	
} gTimer1ClockSelect;

typedef enum timer2ClockSelect {
	TIMER_STOP					= 0,
	NO_PRESCALING 				= 1,
	PRESCALER_8 				= 2,
	PRESCALER_32 				= 3,
	PRESCALER_64 				= 4,
	PRESCALER_128 				= 5,
	PRESCALER_256 				= 6,
	PRESCALER_1024	 			= 7
	
} gTimer2ClockSelect;

typedef union timerClockSelect {
   gTimer0ClockSelect timer_0;
   gTimer1ClockSelect timer_1;
   gTimer2ClockSelect timer_2;
}gTimerClockSelect;

//======================================================================================

typedef enum timer0WaveformGeneration {
	NORMAL					= 0b00000000,
	PHASE_CORRECT_PWM 		= 0b01000000,
	CTC 					= 0b00001000,
	FAST_PWM 				= 0b01001000
	
} gTimer0WaveformGeneration;

typedef enum timer0WaveformGeneration {
	NORMAL									= 0b00000000,
	PHASE_CORRECT_PWM_8BIT 					= 0b00000001,
	PHASE_CORRECT_PWM_9BIT 					= 0b00000010,
	PHASE_CORRECT_PWM_10BIT 				= 0b00000011,
	CTC 									= 0b00001000,
	FAST_PWM_8BIT 							= 0b00001001,
	FAST_PWM_9BIT 							= 0b00001010,
	FAST_PWM_10BIT 							= 0b00001011,
	PHASE_FREQUENCY_CORRECT_PWM_ICR1 		= 0b00010000,
	PHASE_FREQUENCY_CORRECT_PWM_OCR1A 		= 0b00010001,
	PHASE_CORRECT_PWM_ICR1 					= 0b00010010,
	PHASE_CORRECT_PWM_OCR1A 				= 0b00010011,
	CTC 									= 0b00011000,
	RESERVED								= 0b00011001,
	FAST_PWM_ICR1 							= 0b00011010,
	FAST_PWM_OCR1A 							= 0b00011011
	
} gTimer1WaveformGeneration;

typedef enum timer2WaveformGeneration {
	NORMAL					= 0b00000000,
	PHASE_CORRECT_PWM 		= 0b01000000,
	CTC 					= 0b00001000,
	FAST_PWM 				= 0b01001000
	
} gTimer2WaveformGeneration;

typedef union timerWaveformGeneration {
   gTimer0WaveformGeneration timer_0;
   gTimer1WaveformGeneration timer_1;
   gTimer2WaveformGeneration timer_2;
}gTimerWaveformGeneration;

//======================================================================================

typedef enum timer0CompareOutputModeNonPWMMode {
	NORMAL					= 0b00000000, // Normal port operation, OC0 disconnected.
	TOGGLE_OC0 				= 0b00010000, // Toggle OC0 on compare match
	CLEAR_OC0  				= 0b00100000, // Clear OC0 on compare match
	SET_OC0 				= 0b00110000  // Set OC0 on compare match
	
} gTimer0CompareOutputModeNonPWMMode;

typedef enum timer0CompareOutputModeFastPWMMode {
	NORMAL								= 0b00000000, // Normal port operation, OC0 disconnected.
	RESERVED 							= 0b00010000, // Reserved
	CLEAR_OC0_MATCH__SET_OC0_BOTTOM  	= 0b00100000, // Clear OC0 on compare match, set OC0 at BOTTOM,(nin-inverting mode)
	SET_OC0_MATCH__CLEAR_OC0_BOTTOM  	= 0b00110000  // Set OC0 on compare match, clear OC0 at BOTTOM, (inverting mode)
	
} gTimer0CompareOutputModeFastPWMMode;

typedef enum timer0CompareOutputModePhaseCorrectPWMMode {
	NORMAL													= 0b00000000, // Normal port operation, OC0 disconnected.
	RESERVED 												= 0b00010000, // Reserved
	CLEAR_OC0_MATCH_UPCOUNTING__SET_OC0_MATCH_DOWNCOUNTING  = 0b00100000, // Clear OC0 on compare match when up-counting. Set OC0 on compare match when downcounting.
	SET_OC0_MATCH_UPCOUNTING__CLEAR_OC0_MATCH_DOWNCOUNTING  = 0b00110000  // Set OC0 on compare match when up-counting. Clear OC0 on compare match when downcounting.
	
} gTimer0CompareOutputModePhaseCorrectPWMMode;

typedef union timer0CompareOutputMode {
   gTimer0CompareOutputModeNonPWMMode 			non_PWM_Mode;
   gTimer0CompareOutputModeFastPWMMode 			fast_PWM_Mode;
   gTimer0CompareOutputModePhaseCorrectPWMMode 	phase_Correct_PWM_Mode;
}gTimer0CompareOutputMode;


//--------------------------------------------------------------------------------------

typedef enum timer1ACompareOutputModeNonPWMMode {
	NORMAL					= 0b00000000, // Normal port operation, OC1A disconnected.
	TOGGLE_OC1A 			= 0b01000000, // Toggle OC1A on compare match
	CLEAR_OC1A  			= 0b10000000, // Clear OC1A on compare match (Set output to low level)
	SET_OC1A				= 0b11000000  // Set OC1A on compare match  (Set output to high level)
	
	
} gTimer1ACompareOutputModeNonPWMMode;

typedef enum timer1ACompareOutputModeFastPWMMode {
	NORMAL								= 0b00000000, // Normal port operation, OC1A disconnected.
	TOGGLE_OC1A_MATCH 					= 0b01000000, // WGM13:0 = 15: Toggle OC1A on Compare Match, For all other WGM13:0 settings, normal port operation, OC1A disconnected.
	CLEAR_OC1A_MATCH__SET_OC1A_BOTTOM  	= 0b10000000, // Clear OC1A on compare match, set OC1A at BOTTOM,(nin-inverting mode)
	SET_OC1A_MATCH__CLEAR_OC1A_BOTTOM  	= 0b11000000  // Set OC1A on compare match, clear OC1A at BOTTOM, (inverting mode)
	
} gTimer1ACompareOutputModeFastPWMMode;

typedef enum timer1ACompareOutputModePhaseCorrectPWMMode {
	NORMAL														= 0b00000000, // Normal port operation, OC1A disconnected.
	TOGGLE_OC1A_MATCH 											= 0b01000000, // WGM13:0 = 9 or 14: Toggle OC1A on Compare Match For all other WGM13:0 settings, normal port operation, OC1A disconnected.
	CLEAR_OC1A_MATCH_UPCOUNTING__SET_OC1A_MATCH_DOWNCOUNTING  	= 0b10000000, // Clear OC1A on compare match when up-counting. Set OC1A on compare match when downcounting.
	SET_OC1A_MATCH_UPCOUNTING__CLEAR_OC1A_MATCH_DOWNCOUNTING	= 0b11000000  // Set OC1A on compare match when up-counting. Clear OC1A on compare match when downcounting.
	
} gTimer1ACompareOutputModePhaseCorrectPWMMode;

typedef union timer1ACompareOutputMode {
   gTimer1ACompareOutputModeNonPWMMode 				non_PWM_Mode;
   gTimer1ACompareOutputModeFastPWMMode 			fast_PWM_Mode;
   gTimer1ACompareOutputModePhaseCorrectPWMMode 	phase_Correct_PWM_Mode;
}gTimer1ACompareOutputMode;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


typedef enum timer1BCompareOutputModeNonPWMMode {
	NORMAL					= 0b00000000, // Normal port operation, OC1B disconnected.
	TOGGLE_OC1B 			= 0b00010000, // Toggle OC1B on compare match
	CLEAR_OC1B  			= 0b00100000, // Clear OC1B on compare match (Set output to low level)
	SET_OC1B				= 0b00110000  // Set OC1B on compare match  (Set output to high level)
	
	
	
} gTimer1BCompareOutputModeNonPWMMode;

typedef enum timer1BCompareOutputModeFastPWMMode {
	NORMAL								= 0b00000000, // Normal port operation, OC1B disconnected.
	DISCONNECTED 						= 0b00010000, // OC1B disconnected (normal port operation). For all other WGM13:0 settings, normal port operation, OC1B disconnected.
	CLEAR_OC1B_MATCH__SET_OC1B_BOTTOM  	= 0b00100000, // Clear OC1B on compare match, set OC1B at BOTTOM,(nin-inverting mode)
	SET_OC1B_MATCH__CLEAR_OC1B_BOTTOM  	= 0b00110000  // Set OC1B on compare match, clear OC1B at BOTTOM, (inverting mode)
	
	
} gTimer1BCompareOutputModeFastPWMMode;

typedef enum timer1BCompareOutputModePhaseCorrectPWMMode {
	NORMAL														= 0b00000000, // Normal port operation, OC1B disconnected.
	DISCONNECTED 												= 0b00010000, // OC1B disconnected (normal port operation). For all other WGM13:0 settings, normal port operation, OC1B disconnected.
	CLEAR_OC1B_MATCH_UPCOUNTING__SET_OC1B_MATCH_DOWNCOUNTING  	= 0b00100000, // Clear OC1B on compare match when up-counting. Set OC1B on compare match when downcounting.
	SET_OC1B_MATCH_UPCOUNTING__CLEAR_OC1B_MATCH_DOWNCOUNTING	= 0b00110000  // Set OC1B on compare match when up-counting. Clear OC1B on compare match when downcounting.
	
} gTimer1BCompareOutputModePhaseCorrectPWMMode;

typedef union timer1BCompareOutputMode {
   gTimer1BCompareOutputModeNonPWMMode 				non_PWM_Mode;
   gTimer1BCompareOutputModeFastPWMMode 			fast_PWM_Mode;
   gTimer1BCompareOutputModePhaseCorrectPWMMode 	phase_Correct_PWM_Mode;
}gTimer1BCompareOutputMode;


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

typedef struct timer1CompareOutputMode {
   gTimer1ACompareOutputMode 			timer1A;
   gTimer1BCompareOutputMode 			timer1B;
   
}gTimer1CompareOutputMode;

//--------------------------------------------------------------------------------------


typedef enum timer2CompareOutputModeNonPWMMode {
	NORMAL					= 0b00000000, // Normal port operation, OC2 disconnected.
	TOGGLE_OC2 				= 0b00010000, // Toggle OC2 on compare match
	CLEAR_OC2  				= 0b00100000, // Clear OC2 on compare match
	SET_OC2 				= 0b00110000  // Set OC2 on compare match
	
} gTimer2CompareOutputModeNonPWMMode;

typedef enum timer2CompareOutputModeFastPWMMode {
	NORMAL								= 0b00000000, // Normal port operation, OC2 disconnected.
	RESERVED 							= 0b00010000, // Reserved
	CLEAR_OC2_MATCH__SET_OC2_BOTTOM  	= 0b00100000, // Clear OC2 on compare match, set OC2 at BOTTOM,(nin-inverting mode)
	SET_OC2_MATCH__CLEAR_OC2_BOTTOM  	= 0b00110000  // Set OC2 on compare match, clear OC2 at BOTTOM, (inverting mode)
	
} gTimer2CompareOutputModeFastPWMMode;

typedef enum timer2CompareOutputModePhaseCorrectPWMMode {
	NORMAL													= 0b00000000, // Normal port operation, OC2 disconnected.
	RESERVED 												= 0b00010000, // Reserved
	CLEAR_OC2_MATCH_UPCOUNTING__SET_OC2_MATCH_DOWNCOUNTING  = 0b00100000, // Clear OC2 on compare match when up-counting. Set OC2 on compare match when downcounting.
	SET_OC2_MATCH_UPCOUNTING__CLEAR_OC2_MATCH_DOWNCOUNTING  = 0b00110000  // Set OC2 on compare match when up-counting. Clear OC2 on compare match when downcounting.
	
} gTimer2CompareOutputModePhaseCorrectPWMMode;

typedef union timer2CompareOutputMode {
   gTimer2CompareOutputModeNonPWMMode 			non_PWM_Mode;
   gTimer2CompareOutputModeFastPWMMode 			fast_PWM_Mode;
   gTimer2CompareOutputModePhaseCorrectPWMMode 	phase_Correct_PWM_Mode;
}gTimer2CompareOutputMode;

//--------------------------------------------------------------------------------------

typedef union timerCompareOutputMode {
   gTimer0CompareOutputMode		timer_0;
   gTimer1CompareOutputMode		timer_1;
   gTimer2CompareOutputMode		timer_2;
}gTimerCompareOutputMode;

//======================================================================================

typedef enum timer0ExtraFeature {
	DISABLE_ALL						= 0b00000000,
	FOC0 							= 0b10000000 	//	FOC0: Force Output Compare
	
} gTimer0ExtraFeature;

typedef enum timer1ExtraFeature {
	DISABLE_ALL						= 0b00000000,
	FOC1B							= 0b00000100,	//  FOC1B: Force Output Compare for Compare unit B
	FOC1A							= 0b00001000,	//	FOC1A: Force Output Compare for Compare unit A
	FOC1A_FOC1B						= 0b00001100,
	ICES1							= 0b01000000,	//	ICES1: Input Capture Edge Select
	ICES1_FOC1B						= 0b01000100,
	ICES1_FOC1A						= 0b01001000,
	ICES1_FOC1A_FOC1B				= 0b01001100,
	ICNC1							= 0b10000000,	//	ICNC1: Input Capture Noise Canceler
	ICNC1_FOC1B						= 0b10000100,
	ICNC1_FOC1A						= 0b10001000,
	ICNC1_FOC1A_FOC1B				= 0b10001100,
	ICNC1_ICES1						= 0b11000000,
	ICNC1_ICES1_FOC1B				= 0b11000100,
	ICNC1_ICES1_FOC1A				= 0b11001000,
	ICNC1_ICES1_FOC1A_FOC1B 		= 0b11001100
	
} gTimer1ExtraFeature;

typedef enum timer2ExtraFeature {
	DISABLE_ALL						= 0b00000000,
	AS2								= 0b00001000,	//	AS2: Asynchronous Timer/Counter2
	FOC2							= 0b10000000,	//	 FOC2: Force Output Compare
	FOC2_AS2						= 0b10001000
		
} gTimer2ExtraFeature;


typedef enum timer2ExtraFeatureReadOnly {
	DISABLE_ALL						= 0b00000000,
	TCR2UB							= 0b00000001,	//	TCR2UB: Timer/Counter Control Register2 Update Busy
	OCR2UB							= 0b00000010,	//	OCR2UB: Output Compare Register2 Update Busy
	OCR2UB_TCR2UB					= 0b00000011,
	TCN2UB							= 0b00000100,	//	TCN2UB: Timer/Counter2 Update Busy
	TCN2UB_TCR2UB					= 0b00000101,
	TCN2UB_OCR2UB					= 0b00000110,
	TCN2UB_OCR2UB_TCR2UB			= 0b00000111,
	AS2								= 0b00001000,	//	AS2: Asynchronous Timer/Counter2
	AS2_TCR2UB						= 0b00001001,
	AS2_OCR2UB						= 0b00001010,
	AS2_OCR2UB_TCR2UB				= 0b00001011,
	AS2_TCN2UB						= 0b00001100,
	AS2_TCN2UB_TCR2UB				= 0b00001101,
	AS2_TCN2UB_OCR2UB				= 0b00001110,
	AS2_TCN2UB_OCR2UB_TCR2UB		= 0b00001111,
	FOC2							= 0b10000000,	//	 FOC2: Force Output Compare
	FOC2_TCR2UB						= 0b10000001,
	FOC2_OCR2UB						= 0b10000010,
	FOC2_OCR2UB_TCR2UB				= 0b10000011,
	FOC2_TCN2UB						= 0b10000100,
	FOC2_TCN2UB_TCR2UB				= 0b10000101,
	FOC2_TCN2UB_OCR2UB				= 0b10000110,
	FOC2_TCN2UB_OCR2UB_TCR2UB		= 0b10000111,
	FOC2_AS2						= 0b10001000,
	FOC2_AS2_TCR2UB					= 0b10001001,
	FOC2_AS2_OCR2UB					= 0b10001010,
	FOC2_AS2_OCR2UB_TCR2UB			= 0b10001011,
	FOC2_AS2_TCN2UB					= 0b10001100,
	FOC2_AS2_TCN2UB_TCR2UB			= 0b10001101,
	FOC2_AS2_TCN2UB_OCR2UB			= 0b10001110,
	FOC2_AS2_TCN2UB_OCR2UB_TCR2UB	= 0b10001111
		
} gTimer2ExtraFeatureReadOnly;

typedef union timerExtraFeature {
   gTimer0ExtraFeature				timer_0;
   gTimer1ExtraFeature				timer_1;
   gTimer2ExtraFeature				timer_2;
   gTimer2ExtraFeatureReadOnly		timer_2_readOnly;
}gTimerExtraFeature;


//======================================================================================

typedef enum timer0InterruptMask {
	DISABLE_ALL						= 0b00000000,
	TOIE0							= 0b00000001, // TOIE0: Timer/Counter0 Overflow Interrupt Enable
	OCIE0							= 0b00000010, // OCIE0: Timer/Counter0 Output Compare Match Interrupt Enable
	OCIE0_TOIE0						= 0b00000011
		
} gTimer0InterruptMask;
 
 
 typedef enum timer1InterruptMask {
	DISABLE_ALL						= 0b00000000,
	TOIE1							= 0b00000100, // TOIE1: Timer/Counter1, Overflow Interrupt Enable
	OCIE1B							= 0b00001000, // OCIE1B: Timer/Counter1, Output Compare B Match Interrupt Enable
	OCIE1B_TOIE1					= 0b00001100,
	OCIE1A							= 0b00010000, // OCIE1A: Timer/Counter1, Output Compare A Match Interrupt Enable
	OCIE1A_TOIE1					= 0b00010100,
	OCIE1A_OCIE1B					= 0b00011000,
	OCIE1A_OCIE1B_TOIE1				= 0b00011100,
	TICIE1							= 0b00100000, //  TICIE1: Timer/Counter1, Input Capture Interrupt Enable
	TICIE1_TOIE1					= 0b00100100,
	TICIE1_OCIE1B					= 0b00101000,
	TICIE1_OCIE1B_TOIE1				= 0b00101100,
	TICIE1_OCIE1A					= 0b00110000,
	TICIE1_OCIE1A_TOIE1				= 0b00110100,
	TICIE1_OCIE1A_OCIE1B			= 0b00111000,
	TICIE1_OCIE1A_OCIE1B_TOIE1		= 0b00111100
		
} gTimer1InterruptMask;
 
 typedef enum timer2InterruptMask {
	DISABLE_ALL						= 0b00000000,
	TOIE2							= 0b01000000, // TOIE2: Timer/Counter2 Overflow Interrupt Enable
	OCIE2							= 0b10000000, // OCIE2: Timer/Counter2 Output Compare Match Interrupt Enable
	OCIE2_TOIE2						= 0b11000000
		
} gTimer2InterruptMask;
 
 typedef union timerInterruptMask {
   gTimer0InterruptMask				timer_0;
   gTimer1InterruptMask				timer_1;
   gTimer2InterruptMask				timer_2;
}gTimerInterruptMask;


//======================================================================================

typedef enum timer0InterruptFlag {
	DISABLE_ALL						= 0b00000000,
	TOV0							= 0b00000001, // TOV0: Timer/Counter0 Overflow Flag
	OCF0							= 0b00000010, // OCF0: Output Compare Flag 0
	OCF0_TOV0						= 0b00000011
		
} gTimer0InterruptFlag;
 
 
 
 typedef enum timer1InterruptFlag {
	DISABLE_ALL						= 0b00000000,
	TOV1							= 0b00000100, // TOV1: Timer/Counter1, Overflow Flag
	OCF1B							= 0b00001000, //  OCF1B: Timer/Counter1, Output Compare B Match Flag
	OCF1B_TOV1						= 0b00001100,
	OCF1A							= 0b00010000, //  Timer/Counter1, Output Compare A Match Flag
	OCF1A_TOV1						= 0b00010100,
	OCF1A_OCF1B						= 0b00011000,
	OCF1A_OCF1B_TOV1				= 0b00011100,
	ICF1							= 0b00100000, //  ICF1: Timer/Counter1, Input Capture Flag
	ICF1_TOV1						= 0b00100100,
	ICF1_OCF1B						= 0b00101000,
	ICF1_OCF1B_TOV1					= 0b00101100,
	ICF1_OCF1A						= 0b00110000,
	ICF1_OCF1A_TOV1					= 0b00110100,
	ICF1_OCF1A_OCF1B				= 0b00111000,
	ICF1_OCF1A_OCF1B_TOV1			= 0b00111100
		
} gTimer1InterruptFlag;
 
 
 typedef enum timer2InterruptFlag {
	DISABLE_ALL						= 0b00000000,
	TOV2							= 0b01000000, // TOV2: Timer/Counter2 Overflow Flag
	OCF2							= 0b10000000, // OCF2: Output Compare Flag 2
	OCF2_TOV2						= 0b11000000
		
} gTimer2InterruptFlag;


 typedef union timerInterruptFlag {
   gTimer0InterruptFlag				timer_0;
   gTimer1InterruptFlag				timer_1;
   gTimer2InterruptFlag				timer_2;
}gTimerInterruptFlag;


//======================================================================================

typedef struct timer16BitReg {
   uint8_t 						high_Reg;
   uint8_t 						low_Reg;
   
}gTimer16BitReg;

typedef struct timer16BitRegPtr {
   uint8_t *					high_Reg;
   uint8_t *					low_Reg;
   
}gTimer16BitRegPtr;


 typedef union timerCounter {
   uint8_t						timer_0;
   gTimer16BitReg				timer_1;
   uint8_t						timer_2;
}gTimerCounter;


 typedef union timerCounterPtr {
   uint8_t *					timer_0;
   gTimer16BitRegPtr				timer_1;
   uint8_t *					timer_2;
}gTimerCounterPtr;


typedef struct timer16BitRegCompare {
   gTimer16BitReg 				compare_A;
   gTimer16BitReg 				compare_B;
   
}gTimer16BitRegCompare;


typedef struct timer16BitRegComparePtr {
   gTimer16BitRegPtr 				compare_A;
   gTimer16BitRegPtr 				compare_B;
   
}gTimer16BitRegComparePtr;


typedef union timerCompare {
   uint8_t						timer_0;
   gTimer16BitRegCompare		timer_1;
   uint8_t						timer_2;
}gTimerCompare;


typedef union timerComparePtr {
   uint8_t *					timer_0;
   gTimer16BitRegComparePtr		timer_1;
   uint8_t *					timer_2;
}gTimerComparePtr;


typedef union timerInputCapture {
   gTimer16BitReg				timer_1;
   
}gTimerInputCapture;


typedef union timerInputCapturePtr {
   gTimer16BitReg *				timer_1;
   
}gTimerInputCapturePtr;



typedef enum timerNumber {
	TIMER_0							= 0,
	TIMER_1							= 1, 
	TIMER_2							= 2 
		
} gTimerNumber;


//======================================================================================

typedef struct timerCfg {
	gTimerNumber						timer;
	gTimerClockSelect 					clockSelect;
	gTimerWaveformGeneration 			waveFormGeneration;
	gTimerCompareOutputMode 			compareOutputMode;
	gTimerExtraFeature					extraFeature;
	gTimerInterruptMask					interruptMask;
	gTimerInterruptFlag					interruptFlag;
	
	gTimerCounter						counter;
	gTimerCompare						compare;
	gTimerInputCapture					inputCapture;
	
	gTimerCounterPtr					counterPtr;
	gTimerComparePtr					comparePtr;
	gTimerInputCapturePtr				inputCapturePtr;
	
	void (*intFun[4])(void);
	
	
} gTimerCfg_t;
 
 typedef enum timerStdErr {
	NO_ERRORS					= 0b00000000,
	TIMER_NUMBER_ERR 			= 0b00000001,
	CLOCK_SELECT_ERR 			= 0b00000010,
	WAVE_FORM_GENERATION_ERR	= 0b00000100,
	COMPARE_OUTPUT_MODE_ERR 	= 0b00001000,
	EXTRA_FEATURE_ERR 			= 0b00010000,
	INTERRUPT_MASK_ERR 			= 0b00100000,
	INTERRUPT_FLAG_ERR 			= 0b01000000
	
} gTimerStdErr;


//======================================================================================
 
 
gTimerStdErr TimerInitCfg(gTimerCfg_t * obj) ;
gTimerStdErr TimerReadCfg(gTimerCfg_t * obj) ;
#endif /* TIMER_H_ */