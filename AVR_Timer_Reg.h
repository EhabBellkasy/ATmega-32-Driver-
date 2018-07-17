/*
 * AVR_Timer_Reg.h
 *
 *  Created on: 2018/06/09
 *      Author: Ehab Bellkasy
 */
 
 #ifndef AVR_TIMER_REG_H_
#define AVR_TIMER_REG_H_


//======================================================================================================================================

//Timer 0	registers 	addresses	BITS
#define 	TCNT0		0x52		//Timer/Counter Register.
#define 	OCR0		0x5C		//Output Compare Register.
#define 	TCCR0		0x53		//Timer/Counter Control Register.
									#define CS00	0	// 	Clock Select bits select the clock source to be used by the Timer/Counter.
									#define CS01	1	// 	Clock Select bits select the clock source to be used by the Timer/Counter.
									#define CS02	2	// 	Clock Select bits select the clock source to be used by the Timer/Counter.
									#define WGM01	3	// 	Waveform Generation Mode.
									#define COM00	4	// 	Compare Match Output Mode.
									#define COM01	5	// 	Compare Match Output Mode.
									#define WGM00	6	// 	Waveform Generation Mode.
									#define FOC0	7	// 	Force Output Compare.

//======================================================================================================================================
									
//Timer 1	registers 	addresses	BITS
#define 	TCNT1H		0x4D		//Timer/Counter High Register.
#define 	TCNT1L		0x4C		//Timer/Counter low Register.
#define 	OCR1AH		0x4B		//Output Compare A High Register.
#define 	OCR1AL		0x4A		//Output Compare A low Register.
#define 	OCR1BH		0x49		//Output Compare B High Register.
#define 	OCR1BL		0x48		//Output Compare B low Register.
#define 	ICR1H		0x47		//Input Capture B High Register.
#define 	ICR1L		0x46		//Input Capture B low Register.
#define 	TCCR1A		0x4F		//Timer/Counter1 Control Register A.
									#define WGM10	0	// 	Waveform Generation Mode.
									#define WGM11	1	// 	Waveform Generation Mode.
									#define FOC1B	2	// 	Force Output Compare for Compare unit B.
									#define FOC1A	3	// 	Force Output Compare for Compare unit A.
									#define COM1B0	4	// 	Compare Output Mode for Compare unit B.
									#define COM1B1	5	// 	Compare Output Mode for Compare unit B.
									#define COM1A0	6	// 	Compare Output Mode for Compare unit A.
									#define COM1A1	7	// 	Compare Output Mode for Compare unit A.
									
#define 	TCCR1B		0x4E		//Timer/Counter Control Register.
									#define CS10	0	// 	Clock Select bits select the clock source to be used by the Timer/Counter.
									#define CS11	1	// 	Clock Select bits select the clock source to be used by the Timer/Counter.
									#define CS12	2	// 	Clock Select bits select the clock source to be used by the Timer/Counter.
									#define WGM12	3	// 	Waveform Generation Mode.
									#define WGM13	4	// 	Waveform Generation Mode.
									//#define  Reserved Bit	5	// 	-.
									#define ICES1	6	// 	Input Capture Edge Select.
									#define ICNC1	7	// 	Input Capture Noise Canceler.

//======================================================================================================================================									

//Timer 2	registers 	addresses	BITS
#define 	TCNT2		0x44		//Timer/Counter Register.
#define 	OCR2		0x43		//Output Compare Register.
#define 	TCCR2		0x45		//Timer/Counter Control Register.
									#define CS20	0	// 	Clock Select bits select the clock source to be used by the Timer/Counter.
									#define CS21	1	// 	Clock Select bits select the clock source to be used by the Timer/Counter.
									#define CS22	2	// 	Clock Select bits select the clock source to be used by the Timer/Counter.
									#define WGM21	3	// 	Waveform Generation Mode.
									#define COM20	4	// 	Compare Match Output Mode.
									#define COM21	5	// 	Compare Match Output Mode.
									#define WGM20	6	// 	Waveform Generation Mode.
									#define FOC2	7	// 	Force Output Compare.
									
#define 	ASSR		0x42		//Asynchronous Status Register.
									#define TCR2UB	0	// 	Timer/Counter Control Register2 Update Busy.
									#define OCR2UB	1	// 	Output Compare Register2 Update Busy.
									#define TCN2UB	2	// 	Timer/Counter2 Update Busy.
									#define AS2		3	// 	Asynchronous Timer/Counter2.
									

//======================================================================================================================================
//Interrupt	registers 	addresses	BITS
#define 	TIMSK		0x59		//Timer/Counter Control Register.
									#define TOIE0	0	// 	Timer/Counter0 Overflow Interrupt Enable.
									#define OCIE0	1	// 	Timer/Counter0 Output Compare Match Interrupt Enable.
									#define TOIE1	2	// 	Timer/Counter1, Overflow Interrupt Enable.
									#define OCIE1B	3	// 	Timer/Counter1, Output Compare B Match Interrupt Enable.
									#define OCIE1A	4	// 	Timer/Counter1, Output Compare A Match Interrupt Enable.
									#define TICIE1	5	// 	Timer/Counter1, Input Capture Interrupt Enable.
									#define TOIE2	6	// 	Timer/Counter2 Overflow Interrupt Enable.
									#define OCIE2	7	// 	Timer/Counter2 Output Compare Match Interrupt Enable.

#define 	TIFR		0x58		//Timer/Counter Interrupt Flag Register.
									#define TOV0	0	// 	Timer/Counter0 Overflow Flag.
									#define OCF0	1	// 	Output Compare Flag 0.
									#define TOV1	2	// 	Timer/Counter1, Overflow Flag.
									#define OCF1B	3	// 	Timer/Counter1, Output Compare B Match Flag.
									#define OCF1A	4	// 	Timer/Counter1, Output Compare A Match Flag.
									#define ICF1	5	// 	Timer/Counter1, Input Capture Flag.
									#define TOV2	6	// 	Timer/Counter2 Overflow Flag.
									#define OCF2	7	// 	Output Compare Flag 2.

									
#endif /* AVR_TIMER_REG_H_ */

