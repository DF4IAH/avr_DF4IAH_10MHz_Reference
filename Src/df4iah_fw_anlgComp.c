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


extern uint8_t  acAdcConvertNowCh;
extern uint8_t  acAdcConvertNowTempStep;
extern uint16_t acAdcCh[AC_ADC_CH_COUNT + 1];				// plus one for the temperature sensor


#ifdef RELEASE
__attribute__((section(".df4iah_fw_anlgcomp"), aligned(2)))
#endif
void anlgComp_fw_init()
{
	/* enable power for ADC, reference voltage and analog comparator */
	PRR &= ~(_BV(PRADC));

	/* disable digital input buffers on AIN0, AIN1, ADC0 and ADC1 */
	DIDR1 |= (0b11 << AIN0D);
	DIDR0  = (0b000011 << ADC0D);

	/* enable comparator AIN1 pin */
	ADCSRB &= ~(_BV(ACME));									// disable Analog Comparator Multiplex Enable
	ADCSRA |= _BV(ADEN) | (0b111 << ADPS0);					// AD enable, use 128/1 prescaler for ADC clock = 156250 Hz

	/* enable comparator AIN0 pin */
	ACSR  = (ACSR &  ~(_BV(ACBG) | _BV(ACD)	|	 		  	// disable bandgap reference voltage, switch off Analog Comparator Disable
			_BV(ACO) | _BV(ACIE)))			| 				// no Analog Comparator Output
			_BV(ACIC)						|				// Analog Comparator Input Capture for the Counter1
			(0b11 << ACIS0);								// disable ACIE for interrupt as long interrupt source is changed, interrupt on Rising Edge
	ACSR |= _BV(ACI);										// clear any pending interrupt
	ACSR |= _BV(ACIE);										// now set ACIE for interrupt


	/* ADC reference set to AREF */
	ADMUX = (0b01 << REFS0);								// keep ADLAR off

	/* start the initial conversion */
	ADCSRA |= _BV(ADSC);
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
}

/*
 * x	Mnemonics	clocks	resulting clocks
 * ------------------------------------------------
 * 9	push		2		18
 * 1	in			1		 1
 * 1	eor			1		 1
 * 2	lds			2		 2
 * 1	sei			1		 1
 *
 * = 23 clocks --> 1.15 µs until sei() is done
 */
#ifdef RELEASE
__attribute__((section(".df4iah_fw_anlgcomp"), aligned(2)))
#endif
//void anlgComp_ISR_ANALOG_COMP() - __vector_23
ISR(ANALOG_COMP_vect, ISR_BLOCK)
{
	/* timer strobe is taken from df4iah_fw_clkFastCtr.h */
	/* use compare interrupt to handle the ADC */

	/* read the ADC value */
	uint8_t localADCL = ADCL;								// read LSB first
	uint8_t localADCH = ADCH;

	sei();

	/* when ADC has finished with previous conversion (it should be), store the value and restart new job */
	if (!(ADCSRA & _BV(ADSC))) {
		uint16_t adVal  =  localADCL | (localADCH << 8);

		switch (acAdcConvertNowTempStep)
		{
		case 0:
			// store last measurement
			acAdcCh[acAdcConvertNowCh] = adVal;

			// switch to next ADC input channel (ADMUX)
			acAdcConvertNowCh++;
			acAdcConvertNowCh = acAdcConvertNowCh % AC_ADC_CH_COUNT;
			ADMUX = 0b01000000 | (acAdcConvertNowCh & 0x07);  // = (0b01 << REFS0) | ((ac_adc_convertNowCh & 0x07) << MUX0);
			break;

		case 1:
			// store last measurement
			acAdcCh[acAdcConvertNowCh] = adVal;

			// calculate next ADC input channel (ADMUX)
			acAdcConvertNowCh++;
			acAdcConvertNowCh = acAdcConvertNowCh % AC_ADC_CH_COUNT;

			// switch over to temperature conversion
			ADMUX = 0b11001000;  // = (0b11 << REFS0) | (0x08 << MUX0);
			acAdcConvertNowTempStep = 2;
			break;

		case 2:
			// first sample to be discarded
			acAdcConvertNowTempStep = 3;
			break;

		case 3:
			acAdcCh[AC_ADC_CH_COUNT] = adVal;
			// no break
		default:
			// switch to next ADC input channel (ADMUX)
			acAdcConvertNowCh++;
			acAdcConvertNowCh = acAdcConvertNowCh % AC_ADC_CH_COUNT;
			ADMUX = 0b01000000 | (acAdcConvertNowCh & 0x07);  // = (0b01 << REFS0) | ((ac_adc_convertNowCh & 0x07) << MUX0);
			acAdcConvertNowTempStep = 4;
			break;

		case 4:
			// first sample to be discarded
			acAdcConvertNowTempStep = 0;
			break;
		}

		// start next ADC conversion and reset ADIF flag
		ADCSRA |= _BV(ADSC) | _BV(ADIF);
	}
}
