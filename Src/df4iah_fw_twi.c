/*
 * df4iah_fw_twi.c
 *
 *  Created on: 01.03.2015
 *      Author: DF4IAH, Ulrich Habel
 */
// tabsize: 4

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <avr/interrupt.h>

#include "chipdef.h"
#include "df4iah_fw_main.h"

#include "df4iah_fw_twi.h"


extern twiStatus_t twiState;
extern uint8_t twiSeq1Adr;
extern uint8_t twiSeq2DataCnt;
extern uint8_t twiSeq2DataIdx;
extern uint8_t twiSeq2DataTxRxBitmaskLSB;
extern uint8_t twiSeq2Data[TWI_DATA_BUFFER_SIZE];


void twi_fw_init()
{
	/* power up this module */
	PRR &= ~(_BV(PRTWI));

	// setting IO pins: pull-up on
	MCUCR     &= ~(_BV(PUD));											// ensure PUD is off --> activation of all pull-ups
	TWI_PORT  |= _BV(TWI_SDA_PNUM) | _BV(TWI_SCL_PNUM);					// SDA and SCL pull-up

	// SCL frequency: using 400 kHz for SCL @20 MHz clock
	TWBR = 17;  // with the prescaler = 1

	// TWI interface enabled
	TWCR = _BV(TWINT) | _BV(TWEN);
}

void twi_fw_close()
{
	// interrupt: clearing Global Interrupt Flag when interrupts are changed
	uint8_t sreg = SREG;
	cli();

	// TWI interface disabled
	TWCR = _BV(TWINT);

	SREG = sreg;

	// setting IO pins: pull-up off
	TWI_PORT  |= ~(_BV(TWI_SDA_PNUM) | _BV(TWI_SCL_PNUM));				// SDA and SCL pull-up off

	/* no more power is needed for this module */
	PRR |= _BV(PRTWI);
}

void twi_fw_start()
{
	if (twiState.doStart && twiSeq2DataCnt) {
		cli();

		twiState.isProcessing	= true;
		twiState.doStart		= false;
		twiState.errStart		= false;
		twiState.adrAck			= false;
		twiState.adrAckValid	= false;
		twiState.dataAck		= false;
		twiState.dataAckValid	= false;
		twiSeq2DataIdx			= 0;

		/* send START */
		TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN) | _BV(TWIE);

		/* next state */
		twiState.state = TWI_STATE_START_SENT;

		sei();
	}
}


/*
 * x	Mnemonics	clocks	resulting clocks
 * ------------------------------------------------
 * 15	push		2		30
 * 1	in			1		 1
 * 1	eor			1		 1
 * 1	lds			2		 2
 * 1	sei			1		 1
 *
 * = 35 clocks --> 1.75 µs until sei() is done
 */
//void twi_ISR_TWI(void) - __vector_24
ISR(TWI_vect, ISR_BLOCK)
{
	/* read the data byte received */
	uint8_t localTwsrState = TWSR & TWI_TWSR_STATE_MASK;

	switch(twiState.state) {
	case TWI_STATE_START_SENT:
	case TWI_STATE_REPEATEDSTART_SENT:
		if (((twiState.state == TWI_STATE_START_SENT) && (localTwsrState == TWI_TWSR_START)) ||
			((twiState.state == TWI_STATE_REPEATEDSTART_SENT) && (localTwsrState == TWI_TWSR_REPEATEDSTART))) {
			/* MASTER: send SLA - slave address */
			TWDR = twiSeq1Adr;  // Write or Read mode depends on (twiSeq1Adr & 0x01);
			TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWIE);

			/* next state */
			twiState.state = TWI_STATE_ADR_SENT;

			sei();

		} else {
			twiState.errStart = true;

			/* next state */
			twiState.state = TWI_STATE_STOP;

			sei();
		}
		break;

	case TWI_STATE_ADR_SENT:
		if (localTwsrState == TWI_TWSR_M_SLAW_ADDR_ACK) {
			twiState.adrAck			= true;
			twiState.adrAckValid	= true;

			if (twiSeq2DataTxRxBitmaskLSB & 0x01) {
				/* send first data */
				TWDR = twiSeq2Data[0];
				TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWIE);

				/* next state */
				twiState.state = TWI_STATE_DATA_SENT;

			} else {
				/* receive first data */
				twiSeq2Data[0] = TWDR;
				TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWIE);

				/* next state */
				twiState.state = TWI_STATE_DATA_RCVD;
			}

			sei();

		} else {
			twiState.adrAck			= false;
			twiState.adrAckValid	= true;

			/* next state */
			twiState.state = TWI_STATE_STOP;

			sei();
		}
		break;

	case TWI_STATE_DATA_SENT:
		if (localTwsrState == TWI_TWSR_M_SLAW_DATA_ACK) {
			++twiSeq2DataIdx;
			twiState.dataAck		= true;
			twiState.dataAckValid	= true;

			/* send data */
			TWDR = twiSeq2Data[twiSeq2DataIdx];

			if (twiSeq2DataIdx >= twiSeq2DataCnt) {
				/* next state */
				twiState.state = TWI_STATE_STOP;

			} else  if (twiSeq2DataTxRxBitmaskLSB & _BV(twiSeq2DataIdx)) {
				TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWIE);

				/* next state */
				twiState.state = TWI_STATE_DATA_SENT;

			} else {
				TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN) | _BV(TWIE);

				/* next state */
				twiState.state = TWI_STATE_REPEATEDSTART_SENT;
			}

			sei();

		} else {
			twiState.dataAck		= false;
			twiState.dataAckValid	= true;

			/* next state */
			twiState.state = TWI_STATE_STOP;

			sei();
		}
		break;

	case TWI_STATE_DATA_RCVD:
		if (localTwsrState == TWI_TWSR_M_SLAR_DATA_ACK) {
			++twiSeq2DataIdx;
			twiState.dataAck		= true;
			twiState.dataAckValid	= true;

			twiSeq2Data[twiSeq2DataIdx] = TWDR;

			if (twiSeq2DataIdx >= twiSeq2DataCnt) {
				TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWIE);  // NAK (TWEA = 0) - sequence ended

				/* next state */
				twiState.state = TWI_STATE_STOP;

			} else if (twiSeq2DataTxRxBitmaskLSB & _BV(twiSeq2DataIdx)) {
				TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWIE);  // NAK - sequence ended and restart for next write job

				/* next state */
				twiState.state = TWI_STATE_REPEATEDSTART_SENT;

			} else {
				TWCR = _BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE);  // ACK - read in sequence

				/* next state */
				twiState.state = TWI_STATE_DATA_RCVD;
			}

			sei();

		} else {
			twiState.dataAck		= false;
			twiState.dataAckValid	= true;

			/* next state */
			twiState.state = TWI_STATE_STOP;

			sei();
		}
		break;

	default:
	case TWI_STATE_STOP:
		twiSeq2DataCnt = 0;

		/* send stop */
		TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN);  // no interrupt TWIE

		/* next state */
		twiState.state = TWI_STATE_READY;

		sei();
		break;
	}
}
