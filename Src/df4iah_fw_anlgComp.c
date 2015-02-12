/*
 * df4iah_fw_anlgComp.c
 *
 *  Created on: 26.12.2014
 *      Author: espero
 */


/* this modules uses the Analog Comparator to generate interrupts based on GPS-PPS (1 Hz) */


#include <avr/interrupt.h>
#include <avr/sleep.h>

#include "chipdef.h"

#include "df4iah_fw_anlgComp.h"


extern uint8_t  acAdcConvertNowState;
extern uint8_t  acAdcConvertNowCntr;
extern uint16_t acAdcCh[AC_ADC_CH_COUNT + 1];				// plus one for the temperature sensor


#ifdef RELEASE
__attribute__((section(".df4iah_fw_anlgcomp"), aligned(2)))
#endif
void anlgComp_fw_init()
{
	/* enable power for ADC, reference voltage and analog comparator */
	PRR &= ~(_BV(PRADC));

	/* init the ADC counter */
	acAdcConvertNowCntr = 0;

	/* disable digital input buffers on AIN0, AIN1, ADC0 and ADC1 */
	DIDR1 |= (0b11 << AIN0D);
	DIDR0  = (0b000011 << ADC0D);

	/* enable comparator AIN1 pin */
	ADCSRB &= ~(_BV(ACME));									// disable Analog Comparator Multiplex Enable
	ADCSRA |= _BV(ADEN) | (0b111 << ADPS0);					// AD enable, use 128/1 prescaler for ADC clock = 156250 Hz

	/* enable comparator AIN0 pin */
	ACSR  = (ACSR &  ~(_BV(ACBG) | _BV(ACD)	|	 		  	// disable bandgap reference voltage, switch off Analog Comparator Disable
			_BV(ACO) | _BV(ACIE)))			| 				// no Analog Comparator Output, disable Interrupt
			_BV(ACIC)						|				// Analog Comparator Input Capture for the Counter1
			(0b11 << ACIS0);								// disable ACIE for interrupt as long interrupt source is changed, interrupt on Rising Edge
	ACSR |= _BV(ACI);										// clear any pending interrupt
	ACSR |= _BV(ACIE);										// now set ACIE for interrupt
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_anlgcomp"), aligned(2)))
#endif
void anlgComp_fw_close()
{
	/* disable interrupt, disable analog comparator */
	ACSR = (ACSR & ~(_BV(ACIE))) | _BV(ACD);

	/* turn off ADC and Analog Comparator */
	ADCSRA = (0b111 << ADPS0);								// disable but keep the prescaler output at the lowest frequency
	ADCSRB = 0;												// disable Analog Comparator Multiplex Enable

	/* turn off reference voltage at selection */
	ADMUX = 0;

	/* disable power for ADC, reference voltage and analog comparator */
	PRR |= _BV(PRADC);

	/* ADC reference set to AREF */
	acAdcConvertNowState = 0x11;								// set FSM address to "discard next conversion"
	ADMUX = (0b01 << REFS0) | 1;							// keep ADLAR off, switch to channel ADC1 (phase input)

	/* start the initial conversion */
	ADCSRA |= _BV(ADIF);									// clear any pending ADC interrupt flag
	ADCSRA |= _BV(ADSC) | _BV(ADIE);						// start first conversion of the conversion train and activate the interrupt handler
}

void startAdcConvertion() {
#if 0
	set_sleep_mode(SLEEP_MODE_ADC);							// do not use SLEEP_MODE_ADC due to the fact that the timers stop
	sleep_enable();
	sleep_cpu();
#else
	ADCSRA |= _BV(ADSC);									// start conversion
#endif
}

/*
 * x	Mnemonics	clocks	resulting clocks
 * ------------------------------------------------
 * 6	push		2		12
 * 1	in			1		 1
 * 1	eor			1		 1
 * 2	ldi			1		 2
 * 2	ld(Z)		2		 4
 * 2	ori			1		 2
 * 2	st(Z)		2		 4
 * 1	sei			1		 1
 *
 * = 27 clocks --> 1.35 µs until sei() is done
 */
#ifdef RELEASE
__attribute__((section(".df4iah_fw_anlgcomp"), aligned(2)))
#endif
//void anlgComp_fw_ISR_ANALOG_COMP() - __vector_23
ISR(ANALOG_COMP_vect, ISR_BLOCK)
{
	/* timer strobe is sent to df4iah_fw_clkFastCtr.c */
	/* this compare interrupt handles the ADC only */

	/* start the conversion train - channel 1 set, already */
	ADCSRA |= _BV(ADIF);									// clear any pending ADC interrupt flag
	ADCSRA |= _BV(ADIE);									// activate the interrupt handler
	sei();

	startAdcConvertion();
}

/*
 * x	Mnemonics	clocks	resulting clocks
 * ------------------------------------------------
 * 6	push		2		12
 * 1	in			1		 1
 * 1	eor			1		 1
 * 2	lds			2		 4
 * 1	sei			1		 1
 *
 * = 19 clocks --> 0.95 µs until sei() is done
 */
#ifdef RELEASE
__attribute__((section(".df4iah_fw_anlgcomp"), aligned(2)))
#endif
//void anlgComp_fw_ISR_ADC() - __vector_21
ISR(ADC_vect, ISR_BLOCK)
{
	//sleep_disable();

	/* read the ADC value */
	uint8_t localADCL = ADCL;								// read LSB first
	uint8_t localADCH = ADCH;

	sei();

	uint16_t adVal  =  localADCL | (localADCH << 8);

	switch (acAdcConvertNowState)
	{
	case 0x01:
		/* store PHASE value */
		acAdcCh[1] = adVal;

		/* switch to ADC input channel 0 - PWM analog value */
		acAdcConvertNowState = 0x10;
		ADMUX = 0b01000000;  								// = (0b01 << REFS0) | ((ac_adc_convertNowCh & 0x07) << MUX0);

		/* start next ADC conversion and reset ADIF flag */
		startAdcConvertion();
		break;

	case 0x10:
		/* sample after switching MUX to be discarded */
		acAdcConvertNowState = 0x00;

		/* start next ADC conversion and reset ADIF flag */
		startAdcConvertion();
		break;

	case 0x00:
		/* store PWM analog value */
		acAdcCh[0] = adVal;

		/* switch to ADC input channel for temperature */
		acAdcConvertNowState = 0x18;

		/* switch over to temperature conversion */
		ADMUX = 0b11001000;  								// = (0b11 << REFS0) | (0x08 << MUX0);

		/* start next ADC conversion and reset ADIF flag */
		startAdcConvertion();
		break;

	case 0x18:
		/* sample after switching MUX to be discarded */
		acAdcConvertNowState = 0x08;

		/* start next ADC conversion and reset ADIF flag */
		startAdcConvertion();
		break;

	case 0x08:
		acAdcCh[2] = adVal;
		// no break
	default:
		/* switch to ADC input channel 1 - PHASE value */
		acAdcConvertNowState = 0x11;
		ADMUX = 0b01000000 | 1;  							// = (0b01 << REFS0) | ((ac_adc_convertNowCh & 0x07) << MUX0);

		/* start next ADC conversion and reset ADIF flag */
		startAdcConvertion();
		break;

	case 0x11:
		/* sample after switching MUX to be discarded */
		acAdcConvertNowState = 0x01;

		/* update ADC counter to inform about a new conversion train is ready to be read */
		acAdcConvertNowCntr++;

		/* end of conversion train - no more ADSC. Woken up by the next rising edge of PPS in ISR(ANALOG_COMP_vect, ISR_BLOCK) */
		break;
	}
}
