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


extern uint16_t fastStampTCNT1;
extern uint32_t fastStampCtr1ms;
extern uint32_t fastCtr1ms;


#ifdef RELEASE
__attribute__((section(".df4iah_fw_clkfastctr"), aligned(2)))
#endif
void clkFastCtr_fw_init()
{
	/* power up this module */
	PRR &= ~(_BV(PRTIM1));

	/* clear the timer */
	TCNT1H = 0;												// write high byte first
	TCNT1L = 0;

	/* set the timer top value for 20.000 clocks @ 20MHz --> 1ms */
	OCR1AH = (OCR1_TOP_VALUE >> 8);							// write high byte first
	OCR1AL = (OCR1_TOP_VALUE & 0xff);

	/* set the timer-1 mode of operation: 0x4 = CTC, counting up, TOP := OCR1A [. . WGM01 WGM00] */
	TCCR1A = (0b00<<WGM10);

	/* set the timer-1 mode of operation: 0x4 = CTC, counting up, TOP := OCR1A [WGM03 WGM02 . .] */
	/* no ICNC1 input filtering */
	/* ICES is set to trigger on the rising edge of the Comparator output --> rising edge of AIN0 */
	/* set the timer-1 clock source to 20 MHz XTAL */
	TCCR1B = (0b01<<WGM12) | (0b001<<CS10);					// since now the timer runs

	/* ICF1 and OCF1A interrupt enable */
	TIMSK1 = _BV(ICIE1) | _BV(OCIE1A);
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_clkfastctr"), aligned(2)))
#endif
void clkFastCtr_fw_close()
{
	/* switch off interrupts */
	TIMSK1 = 0;

	/* switch clock source to halted */
	TCCR1B = 0;

	/* no more power is needed for this module */
	PRR |= _BV(PRTIM1);
}

/*
 * x	Mnemonics	clocks	resulting clocks
 * ------------------------------------------------
 * 8	push		2		16
 * 1	in			1		 1
 * 1	eor			1		 1
 * 6	lds			2		12
 * 1	ldi			1		 1
 * 1	or			1		 1
 * 6	sts			2		12
 * 1	sei			1		 1
 *
 * = 45 clocks --> 2.25 µs until sei() is done
 */
#ifdef RELEASE
__attribute__((section(".df4iah_fw_clkfastctr"), aligned(2)))
#endif
//void clkFastCtr_ISR_T1_Capt() - __vector_10
ISR(TIMER1_CAPT_vect, ISR_BLOCK)
{
	/* 10 kHz signal */

	/* take the current timestamp of the free floating 20 MHz timer */
	uint8_t localICR1L = ICR1L;									// low byte first
	uint8_t localICR1H = ICR1H;

	fastStampTCNT1  = localICR1L | (localICR1H << 8);
	fastStampCtr1ms = fastCtr1ms;

	sei();														// since here we can accept interruptions
}

/*
 * x	Mnemonics	clocks	resulting clocks
 * ------------------------------------------------
 * 7	push		2		14
 * 1	in			1		 1
 * 1	eor			1		 1
 * 4	lds			2		 8
 * 1	adiw		2		 2
 * 2	adc			1		 2
 * 4	sts			2		 8
 * 1	sei			1		 1
 *
 * = 36 clocks --> 1.85 µs until sei() is done
 */
#ifdef RELEASE
__attribute__((section(".df4iah_fw_clkfastctr"), aligned(2)))
#endif
//void clkFastCtr_ISR_T1_CompA() - __vector_11
ISR(TIMER1_COMPA_vect, ISR_BLOCK)
{
	/* whenever 20.000 clocks are done, reload it with the MIN value */
	/* mind you: the reload is not synchronized with any input signals like 1PPS or 10kHz */

	/* the 32 bit timer overflows every 3 1/4 year */
	fastCtr1ms++;

	sei();														// since here we can accept interruptions
}