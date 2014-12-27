/*
 * df4iah_fw_clkSlowCtr.c
 *
 *  Created on: 27.12.2014
 *      Author: espero
 */


/* this modules uses the T0 timer/counter/pwm-generator of the AVR controller as counter */


#include <avr/interrupt.h>

#include "chipdef.h"

#include "df4iah_fw_clkSlowCtr.h"


extern uint32_t csc_timer_s_HI;


#ifdef RELEASE
__attribute__((section(".df4iah_fw_clkslowctr"), aligned(2)))
#endif
void clkSlowCtr_fw_init()
{
	/* power up this module */
	PRR &= ~(_BV(PRTIM0));

	/* switch mode to normal mode and overflows at MAX (= 255) */
	TCCR0A = (0b00 << WGM00);
	//TCCR0B = 0;

	/* initialize counter */
	TCNT0 = 0;

	/* switch clock source to external T0 rising edge */
	TCCR0B = (0b111 << CS00);								// since now the clock runs

	/* TOV0 interrupt enable */
	TIFR0  = _BV(TOV0);										// clear any pending overflows
	TIMSK0 = _BV(TOIE0);									// enable the overflow interrupt

	/* activate PCINT20 interrupt for PIN PD4 (T0), @see page 88 */

}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_clkslowctr"), aligned(2)))
#endif
void clkSlowCtr_fw_close()
{
	/* switch clock source to halted */
	TCCR0B = 0;

	// switch off interrupts
	TIMSK0 = 0;

	/* no more power is needed for this module */
	PRR |= _BV(PRTIM0);
}

/*
 * x	Mnemonics	clocks	resulting clocks
 * ------------------------------------------------
 * 7	push		2		 14
 * 1	in			1		 1
 * 1	eor			1		 1
 * 4	lds			2		 8
 * 1	adiw		2		 2
 * 2	adc			1		 2
 * 4	sts			2		 8
 * 1	sei			1		 1
 *
 * = 35 clocks --> 1.85 µs until sei() is done
 */
#ifdef RELEASE
__attribute__((section(".df4iah_fw_clkslowctr"), aligned(2)))
#endif
//void clkSlowCtr_ISR_T0_OVL() - __vector_16
ISR(TIMER0_OVF_vect, ISR_BLOCK)
{
	// increment PPS counter
	csc_timer_s_HI++;

	sei();
}
