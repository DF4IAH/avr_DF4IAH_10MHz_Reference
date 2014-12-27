/*
 * df4iah_fw_anlgComp.c
 *
 *  Created on: 26.12.2014
 *      Author: espero
 */


/* this modules uses the Analog Comparator to generate interrupts based on GPS (10 kHz) */


#include <avr/interrupt.h>

#include "chipdef.h"

#include "df4iah_fw_anlgComp.h"


extern uint8_t  ac_TCNT2;
extern uint16_t ac_timer_10us;


#ifdef RELEASE
__attribute__((section(".df4iah_fw_anlgcomp"), aligned(2)))
#endif
void anlgComp_fw_init()
{
	/* enable power for ADC, reference voltage and analog comparator */
	PRR &= ~(_BV(PRADC));

	/* disable digital input buffers on AIN0 and AIN1 */
	DIDR1 |= (0b11 << AIN0D);

	/* enable comparator AIN1 pin */
	ADCSRB &= ~(_BV(ACME));									// disable Analog Comparator Multiplex Enable
	//ADCSRA |= _BV(ADEN);  								// AD enable

	/* enable comparator AIN0 pin */
	ACSR  = (ACSR &  ~(_BV(ACBG) | _BV(ACD)    | 		  	// disable bandgap reference voltage, switch off Analog Comparator Disable
			_BV(ACO) | _BV(ACIC) | _BV(ACIE))) | 			// no Analog Comparator Output, no Analog Comparator Input Capture for the Counter1
			(0b11 << ACIS0);								// disable ACIE for interrupt as long interrupt source is changed, interrupt on Rising Edge
	ACSR |= _BV(ACI);										// clear any pending interrupt
	ACSR |= _BV(ACIE);										// now set ACIE for interrupt
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_anlgcomp"), aligned(2)))
#endif
void anlgComp_fw_close()
{
	/* enable comparator AIN0 pin */
	ACSR &= ~(_BV(ACIE));									// disable interrupt

	/* disable power for ADC, reference voltage and analog comparator */
	PRR |= _BV(PRADC);
}

/*
 * x	Mnemonics	clocks	resulting clocks
 * ------------------------------------------------
 * 5	push		2		10
 * 1	in			1		 1
 * 1	eor			1		 1
 * 3	lds			2		 6
 * 3	sts			2		 6
 * 1	adiw		2		 2
 * 1	sei			1		 1
 *
 * = 27 clocks --> 1.35 µs until sei() is done
 */
#ifdef RELEASE
__attribute__((section(".df4iah_fw_anlgcomp"), aligned(2)))
#endif
//void anlgComp_ISR_ANALOG_COMP() - __vector_23
ISR(ANALOG_COMP_vect, ISR_BLOCK)
{
	ac_TCNT2 = TCNT2;
	ac_timer_10us++;

	sei();
}
