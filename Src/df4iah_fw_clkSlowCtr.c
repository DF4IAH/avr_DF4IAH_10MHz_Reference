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


extern uint16_t slowStampTCNT1;
extern uint16_t slowStampTCNT1_last;
extern uint32_t slowStampCtr1ms;
extern uint32_t slowStampCtr1ms_last;
#ifdef CLKSLOWCTR_STAMP_CAPTURE
extern uint32_t fastStampTCNT1;
extern uint32_t fastStampCtr1ms;
#else
extern uint32_t fastCtr1ms;
#endif

#ifdef RELEASE
__attribute__((section(".df4iah_fw_clkslowctr"), aligned(2)))
#endif
void clkSlowCtr_fw_init()
{
#if 0
	/* power up this module */
	PRR &= ~(_BV(PRTIM2));

	/* switch mode to normal mode and overflows at MAX (= 255) */
	TCCR2A = (0b00 << WGM20);
	//TCCR2B = 0;

	/* initialize counter */
	TCNT2 = 0;

	/* switch clock source to internal clock T2S/1024 */
	TCCR2B = (0b111 << CS20);								// since now the clock runs

	/* TOV2 interrupt enable */
	//TIFR2  = _BV(TOV2);									// clear any pending overflows
	//TIMSK2 = _BV(TOIE2);									// enable the overflow interrupt
#endif

	/* activate PCINT20 interrupt for PIN PD4 (T0) */
	PCMSK2 |= _BV(PCINT20);
	PCICR  |= _BV(PCIE2);
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_clkslowctr"), aligned(2)))
#endif
void clkSlowCtr_fw_close()
{
	/* deactivate PCINT20 interrupt for PIN PD4 (T0) */
	PCICR  &= ~(PCIE2);
	PCMSK2 &= ~(PCINT20);

#if 0
	/* switch clock source to halted */
	TCCR2B = 0;

	// switch off interrupts
	TIMSK2 = 0;

	/* no more power is needed for this module */
	PRR |= _BV(PRTIM2);
#endif
}

#if 0
/*
 * x	Mnemonics	clocks	resulting clocks
 * ------------------------------------------------
 * 7	push		2		 14
 * 1	in			1		  1
 * 1	eor			1		  1
 * 4	lds			2		  8
 * 1	adiw		2		  2
 * 2	adc			1		  2
 * 4	sts			2		  8
 * 1	sei			1		  1
 *
 * = 37 clocks --> 1.85 µs until sei() is done
 */
#ifdef RELEASE
__attribute__((section(".df4iah_fw_clkslowctr"), aligned(2)))
#endif
//void clkSlowCtr_ISR_T0_OVL() - __vector_16
ISR(TIMER0_OVF_vect, ISR_BLOCK)
{
//	__asm__ __volatile__ ("reti" ::: "memory");

	// increment PPS counter
	slowPPS_HI++;

	sei();
}
#endif

/*
 * x	Mnemonics	clocks	resulting clocks
 * ------------------------------------------------
 *  7	push		2		 14
 *  1	in			1		  1
 *  1	eor			1		  1
 *  1	sbis		3		  3
 *  0	rjmp		2		  0
 * 12	lds			2		 24
 *  0	ldi			1		  0
 *  0	or			1		  0
 * 12	sts			2		 24
 *  1	sei			1		  1
 *
 * = 58 clocks --> 2.40 µs until sei() is done
 */
#ifdef RELEASE
__attribute__((section(".df4iah_fw_clkslowctr"), aligned(2)))
#endif
//void clkSlowCtr_ISR_PCI2() - __vector_5
ISR(PCINT2_vect, ISR_BLOCK)
{
#ifndef CLKSLOWCTR_STAMP_CAPTURE
	/* get current timer values */
	uint8_t  localTCNT1L = TCNT1L;							// LSB first
	uint8_t  localTCNT1H = TCNT1H;
	uint32_t localCtr1ms = fastCtr1ms;
#endif

	/* stamp on rising edge only */
	if (PIND & _BV(PIND4)) {
		// remember last value
		slowStampTCNT1_last = slowStampTCNT1;
		slowStampCtr1ms_last = slowStampCtr1ms;

#ifdef CLKSLOWCTR_STAMP_CAPTURE
		/* stamp the stamp of the analog comparator / time CAPT unit */
		slowStampTCNT1  = fastStampTCNT1;
		slowStampCtr1ms = fastStampCtr1ms;
#else
		/* stamp current timer */
		slowStampTCNT1  = (localTCNT1L | (localTCNT1H << 8));
		slowStampCtr1ms = localCtr1ms;
#endif
	}

	sei();													// since here we can accept interruptions
}
