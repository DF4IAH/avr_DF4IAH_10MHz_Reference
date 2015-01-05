/*
 * df4iah_fw_clkFastCtr.c
 *
 *  Created on: 26.12.2014
 *      Author: espero
 */


/* this modules uses the T2 timer/counter/pwm-generator of the AVR controller as timer */


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
extern enum REFCLK_STATE_t mainRefClkState;
extern uint8_t  mainIsAPC;
extern uint16_t acAdcCh[AC_ADC_CH_COUNT + 1];
extern uint16_t fastPwmAdcNow;
extern uint16_t fastPwmAdcLast;
extern int16_t  fastPwmAdcAscendingVal;



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

	{
		/* DEBUGGING */
		DEBUG_UP_PORT &= ~(_BV(DEBUG_UP_NR));
		DEBUG_DN_PORT &= ~(_BV(DEBUG_DN_NR));
		DEBUG_UP_DDR  |=   _BV(DEBUG_UP_NR);
		DEBUG_DN_DDR  |=   _BV(DEBUG_DN_NR);
	}
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_clkfastctr"), aligned(2)))
#endif
void clkFastCtr_fw_close()
{
	{
		/* DEBUGGING */
		DEBUG_UP_PORT &= ~(_BV(DEBUG_UP_NR));
		DEBUG_DN_PORT &= ~(_BV(DEBUG_DN_NR));
		DEBUG_UP_DDR  &= ~(_BV(DEBUG_UP_NR));
		DEBUG_DN_DDR  &= ~(_BV(DEBUG_DN_NR));
	}

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
	const uint8_t localSubPwmCnt = (1 << FAST_PWM_SUB_BITCNT);
	const uint8_t localSubPwmMax = (localSubPwmCnt - 1);

	/* the 32 bit timer overflows every 3 1/4 year */
	fastCtr1ms++;

	/* preload the compare A register with its current base value */
	OCR0B = pullPwmVal;

	/* adjust by one when sub-compare value is higher than counter */
	if (fastPwmSubCnt < fastPwmSubCmp) {
		OCR0B++;
	}

#if 0
	DEBUG_UP_PORT |=   _BV(DEBUG_UP_NR);		// TODO debugging aid
	DEBUG_UP_PORT &= ~(_BV(DEBUG_UP_NR));		// TODO debugging aid
#endif
	sei();														// since here we can accept interruptions
#if 0
	DEBUG_DN_PORT |=   _BV(DEBUG_DN_NR);		// TODO debugging aid
	DEBUG_DN_PORT &= ~(_BV(DEBUG_DN_NR));		// TODO debugging aid
#endif

	/* sub-counter increment */
	fastPwmSubCnt++;
	fastPwmSubCnt &= localSubPwmMax;

	/* APC = automatic phase control */
	if (mainIsAPC && (mainRefClkState >= REFCLK_STATE_SEARCH_PHASE_CNTR_STABLIZED)) {
		static uint8_t  localSpeedCtr = 0;
		static uint16_t localSpeedSum = 0;
#ifdef ADC_PWM_REMEMBER_LAST_NULL
		static uint16_t localFastPwmAdcAscendingValLast = 0;
		static uint8_t localSubPwmStore = 0;
		static uint8_t localPwmStore = 0;
#endif

		localSpeedSum += acAdcCh[ADC_CH_PHASE];
		if (localSpeedCtr++ == ADC_PWM_SAMPLING_CNT) {
			/* subsampling data reduction */
			localSpeedCtr = 0;
			fastPwmAdcLast = fastPwmAdcNow;
			fastPwmAdcNow = localSpeedSum;
			localSpeedSum = 0;

			if ((ADC_PWM_LO < fastPwmAdcNow) && (fastPwmAdcNow < ADC_PWM_HI)) {
				/* indicator valid inside this range only */
#ifdef ADC_PWM_REMEMBER_LAST_NULL
				localFastPwmAdcAscendingValLast = fastPwmAdcAscendingVal;
#endif
				fastPwmAdcAscendingVal = ((int16_t) fastPwmAdcNow) - ((int16_t) fastPwmAdcLast);

#ifdef ADC_PWM_REMEMBER_LAST_NULL
				if (((localFastPwmAdcAscendingValLast < 0) && (fastPwmAdcAscendingVal > 0)) ||
					((localFastPwmAdcAscendingValLast < 0) && (fastPwmAdcAscendingVal > 0))) {
					/* change of direction */
					if (((fastPwmAdcNow - ADC_PWM_CENTER_NEUTRAL_DELTA) < ADC_PWM_CENTER) ||
						((fastPwmAdcNow + ADC_PWM_CENTER_NEUTRAL_DELTA) > ADC_PWM_CENTER)) {
						/* store frequency of reversing direction when outside of center phase */
						cli();
						localSubPwmStore = fastPwmSubCmp;
						localPwmStore = pullPwmVal;
						sei();
					}
				}
#endif

				if ((fastPwmAdcAscendingVal < -ADC_PWM_SWITCH_OUT) || (ADC_PWM_SWITCH_OUT < fastPwmAdcAscendingVal)) {
					fastPwmAdcAscendingVal = 0;
				}

				/* inside of measuring phase */
				if (mainRefClkState < REFCLK_STATE_LOCKING_PHASE) {
					mainRefClkState = REFCLK_STATE_LOCKING_PHASE;
				}

				{ /* keeping the phase at 2.4 volts */
					if (((fastPwmAdcNow - ADC_PWM_CENTER_NEUTRAL_DELTA) < ADC_PWM_CENTER) && (fastPwmAdcAscendingVal <= -ADC_PWM_SWITCH_SPEED)) {
#if 1
						/* try to increase the phase-detector-voltage */	// XXX Richtung OK, Spannungsschwelle falsch?
						DEBUG_UP_PORT &= ~(_BV(DEBUG_UP_NR));		// TODO debugging aid
						DEBUG_DN_PORT |=   _BV(DEBUG_DN_NR);		// TODO debugging aid
						cli();
# if 1
						if (!(fastPwmSubCmp--))						// VCO-Frequency goes lower
						{
							fastPwmSubCmp = localSubPwmMax;
							OCR0B = --pullPwmVal;
						}
# endif
							//DEBUG_DN_PORT &= ~(_BV(DEBUG_DN_NR));		// TODO debugging aid
						sei();
#endif

					} else if (((fastPwmAdcNow + ADC_PWM_CENTER_NEUTRAL_DELTA) > ADC_PWM_CENTER) && (fastPwmAdcAscendingVal >= ADC_PWM_SWITCH_SPEED)) {
#if 1
						/* try to decrease the phase-detector-voltage */  	// XXX OK
						DEBUG_UP_PORT |=   _BV(DEBUG_UP_NR);		// TODO debugging aid
						DEBUG_DN_PORT &= ~(_BV(DEBUG_DN_NR));		// TODO debugging aid
						cli();
# if 1
						if (!(++fastPwmSubCmp % localSubPwmCnt)) {	// VCO-Frequency goes higher
							fastPwmSubCmp = 0;
							OCR0B = ++pullPwmVal;
						}
# endif
						//DEBUG_UP_PORT &= ~(_BV(DEBUG_UP_NR));		// TODO debugging aid
						sei();
#endif

					} else if (((fastPwmAdcNow - ADC_PWM_CENTER_NEUTRAL_DELTA) >= ADC_PWM_CENTER) &&
							   ((fastPwmAdcNow + ADC_PWM_CENTER_NEUTRAL_DELTA) <= ADC_PWM_CENTER)) {
#ifdef ADC_PWM_REMEMBER_LAST_NULL
						/* take neutral frequency setting for current PWM */
						cli();
						fastPwmSubCmp = localSubPwmStore;
						OCR0B = pullPwmVal = localPwmStore;
						sei();
#endif

						DEBUG_UP_PORT &= ~(_BV(DEBUG_UP_NR));		// TODO debugging aid
						DEBUG_DN_PORT &= ~(_BV(DEBUG_DN_NR));		// TODO debugging aid

					} else {
						DEBUG_UP_PORT &= ~(_BV(DEBUG_UP_NR));		// TODO debugging aid
						DEBUG_DN_PORT &= ~(_BV(DEBUG_DN_NR));		// TODO debugging aid
					}
				}
				/* try to advance the phase */
				/* try to retard the phase */

			} else {
				/* got outside of locked phase */
				if (mainRefClkState >= REFCLK_STATE_LOCKING_PHASE) {
					mainRefClkState = REFCLK_STATE_SEARCH_PHASE_CNTR_STABLIZED;

					DEBUG_UP_PORT &= ~(_BV(DEBUG_UP_NR));		// TODO debugging aid
					DEBUG_DN_PORT &= ~(_BV(DEBUG_DN_NR));		// TODO debugging aid
				}
			}
		}
	}
}
