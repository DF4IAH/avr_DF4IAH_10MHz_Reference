/*
 * df4iah_fw_clkFastCtr.c
 *
 *  Created on: 26.12.2014
 *      Author: espero
 */


/* this modules uses the T1 timer/counter/pwm-generator of the AVR controller as timer */


#include <math.h>
#include <stdlib.h>
#include <avr/interrupt.h>

#include "chipdef.h"
#include "df4iah_fw_main.h"
#include "df4iah_fw_anlgComp.h"

#include "df4iah_fw_clkFastCtr.h"


extern uint16_t fastStampTCNT1;
extern uint32_t fastStampCtr1ms;
extern uint32_t fastCtr1ms;
extern uint8_t  fastPwmSubCnt;
extern uint8_t  fastPwmSubCmp;
extern uint8_t  pullPwmVal;


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

#if 0
	/* activate PCINT20 interrupt for PIN PD4 (T0) */
	PCMSK2 |= _BV(PCINT20);
	PCICR  |= _BV(PCIE2);
#endif
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_clkfastctr"), aligned(2)))
#endif
void clkFastCtr_fw_close()
{
#if 0
	/* deactivate PCINT20 interrupt for PIN PD4 (T0) */
	PCICR  &= ~(PCIE2);
	PCMSK2 &= ~(PCINT20);
#endif

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
 * 7	push		2		14
 * 2	in			1		 2
 * 1	eor			1		 1
 * 8	lds			2		16
 * 1	adiw		2		 2
 * 2	adc			1		 2
 * 5	sts			2		10
 * 2	out			1		 2
 * 1	cp			1		 1
 * 1	brcc		2		 2
 * 1	subi		1		 1
 * 1	sei			1		 1
 *
 * = 54 clocks --> 2.70 µs until sei() is done
 */
#ifdef RELEASE
__attribute__((section(".df4iah_fw_clkfastctr"), aligned(2)))
#endif
//void clkFastCtr_fw_ISR_T1_CompA() - __vector_11
ISR(TIMER1_COMPA_vect, ISR_BLOCK)
{
	/* this ISR is called every 20.000 clocks - repeating each ms again */

	/* minimal Sub-PWM value for its FAST_PWM_SUB_BITCNT */
	const uint8_t localSubPwmInc = (1 << FAST_PWM_SUB_BITCNT);

	/* the 32 bit timer overflows every 3 1/4 year */
	fastCtr1ms++;

	/* set the compare A register with the new integer PWM value */
	OCR0B = pullPwmVal;

	/* increment if counter is lower than the sub-compare value to get a Sub-PWM (fractional part) */
	if (fastPwmSubCnt < fastPwmSubCmp) {
		OCR0B++;
	}

	/* sub-counter increment */
	fastPwmSubCnt += localSubPwmInc;
	//fastPwmSubCnt %= localSubPwmMax;

	sei();														// since here we can accept interruptions
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
//void clkFastCtr_fw_ISR_T1_Capt() - __vector_10
ISR(TIMER1_CAPT_vect, ISR_BLOCK)
{
	/* rising edge of the PPS signal from df4iah_fw_anlgComp.c detected */

	/* take the current timestamp of the free floating 20 MHz timer */
	uint8_t localICR1L = ICR1L;									// capture timer value - low byte first
	uint8_t localICR1H = ICR1H;
	fastStampCtr1ms = fastCtr1ms;
	sei();

	fastStampTCNT1  = localICR1L | (localICR1H << 8);
}

#if 0
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
__attribute__((section(".df4iah_fw_clkfastctr"), aligned(2)))
#endif
//void clkSlowCtr_fw_ISR_PCI2() - __vector_5
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
#endif
