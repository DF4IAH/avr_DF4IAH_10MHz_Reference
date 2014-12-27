/*
 * df4iah_fw_clkFastCtr.c
 *
 *  Created on: 26.12.2014
 *      Author: espero
 */


/* this modules uses the T2 timer/counter/pwm-generator of the AVR controller as timer */


#include <avr/interrupt.h>

#include "chipdef.h"

#include "df4iah_fw_clkFastCtr.h"


#ifdef RELEASE
__attribute__((section(".df4iah_fw_clkfastctr"), aligned(2)))
#endif
void clkFastCtr_fw_init()
{
	/* power up this module */
	PRR &= ~(_BV(PRTIM2));

	/* no asynchronous clock is used */
	ASSR = 0;

	/* switch mode to CTC (counter with reset) */
	TCCR2A = (0b10 << WGM20);
	//TCCR2B = 0;

	/* TOV2 interrupt enable */
	TIMSK2 = _BV(OCIE2A);

	/* switch clock source to T2S/1 */
	TCCR2B = (0b001 << CS20);								// since now the clock runs
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_clkfastctr"), aligned(2)))
#endif
void clkFastCtr_fw_close()
{
	/* switch clock source to halted */
	TCCR2B = 0;

	// switch off interrupts
	TIMSK2 = 0;

	/* no more power is needed for this module */
	PRR |= _BV(PRTIM2);
}

/*
 * x	Mnemonics	clocks	resulting clocks
 * ------------------------------------------------
 * 4	push		2		 8
 * 1	in			1		 1
 * 1	eor			1		 1
 * 1	ldi			1		 1
 * 1	sts			2		 2
 * 1	sei			1		 1
 *
 * = 14 clocks --> 0.70 µs until sei() is done
 */
#ifdef RELEASE
__attribute__((section(".df4iah_fw_clkfastctr"), aligned(2)))
#endif
//void clkFastCtr_ISR_OVF() - __vector_7
ISR(TIMER2_COMPA_vect, ISR_BLOCK)
{
	// reset counter offset to default value
	OCR2A = DEFAULT_OCR2A_VALUE;

	sei();
}
