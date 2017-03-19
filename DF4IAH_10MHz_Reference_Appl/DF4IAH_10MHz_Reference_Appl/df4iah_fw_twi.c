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
#include <avr/wdt.h>

#include "chipdef.h"
#include "df4iah_fw_main.h"

#include "df4iah_fw_twi.h"


extern uint8_t usbIsUp;
extern volatile main_bf_t main_bf;
extern volatile twiStatus_t twiState;
extern volatile uint8_t twiSeq1Adr;
extern volatile uint8_t twiSeq2DataCnt;
extern volatile uint8_t twiSeq2DataIdx;
extern volatile uint8_t twiSeq2Data[TWI_DATA_BUFFER_SIZE];

static void waitUntilDone(void) {
	while (twiState.isProcessing) {
		wdt_reset();
		if (usbIsUp) {
			usbPoll();
		}
	}
}


void twi_fw_init(void)
{
	/* power up this module */
	PRR &= ~(_BV(PRTWI));

	uint8_t sreg = SREG;
	cli();

	// setting IO pins: pull-up on
	TWI_DDR   &= ~((_BV(TWI_SDA_PNUM) | _BV(TWI_SCL_PNUM)));	// define SDA and SCL pins as input, as long the TWI alternate port function has not taken over
	TWI_PORT  |=   (_BV(TWI_SDA_PNUM) | _BV(TWI_SCL_PNUM));		// SDA and SCL pull-up

#if 1
	// SCL frequency: using 400 kHz for SCL @20 MHz clock
	TWSR = 0; 							 						// prescaler = 1
	TWBR = 17;													// gives 400 kHz @20 MHz clock
#else
	// SCL frequency: using 100 kHz for SCL @20 MHz clock
	TWSR = _BV(TWPS0); 					 						// prescaler = 4
	TWBR = 23;													// gives 100 kHz @20 MHz clock
#endif

	// needed only when TWI is slave - unused and set to default values
	// TWAR  = 0xfe;
	// TWAMR = 0;

	// TWI interface enabled and interrupt cleared
	TWCR = (_BV(TWINT) | _BV(TWEN));

	SREG = sreg;
}

void twi_fw_close(void)
{
	// TWI interface disabled
	TWCR = 0;

	// setting IO pins: pull-up off
	TWI_PORT  |= ~(_BV(TWI_SDA_PNUM) | _BV(TWI_SCL_PNUM));	// SDA and SCL pull-up off

	/* no more power is needed for this module */
	PRR |= _BV(PRTWI);
}


uint8_t twi_fw_sendCmdSendData1(uint8_t addr, uint8_t cmd, uint8_t data1)
{
	waitUntilDone();

	cli();
	twiSeq1Adr = (addr << 1);
	twiSeq2DataCnt = 0;
	twiSeq2Data[twiSeq2DataCnt++] = cmd;
	twiSeq2Data[twiSeq2DataCnt++] = data1;
	twiState.doStart = true;
	twi_fw_sendStart();
	return twiState.dataAck;
}

uint8_t twi_fw_sendCmdSendData1SendData2(uint8_t addr, uint8_t cmd, uint8_t data1, uint8_t data2)
{
	waitUntilDone();

	cli();
	twiSeq1Adr = (addr << 1);
	twiSeq2DataCnt = 0;
	twiSeq2Data[twiSeq2DataCnt++] = cmd;
	twiSeq2Data[twiSeq2DataCnt++] = data1;
	twiSeq2Data[twiSeq2DataCnt++] = data2;
	twiState.doStart = true;
	twi_fw_sendStart();
	return twiState.dataAck;
}

uint8_t twi_fw_sendCmdReadData1(uint8_t addr, uint8_t cmd)
{
	waitUntilDone();

	cli();
	twiSeq1Adr = (addr << 1);
	twiSeq2DataCnt = 0;
	twiSeq2Data[twiSeq2DataCnt++] = cmd;
	twiState.doStart = true;
	twi_fw_sendStart();

	waitUntilDone();
	if (twiState.adrAck) {
		main_bf.mainIsLcdAttached = true;
		cli();
		twiSeq1Adr = ((addr << 1) | 1);
		twiSeq2DataCnt = 0;
		twiSeq2Data[twiSeq2DataCnt++] = 0;
		twiState.doStart = true;
		twi_fw_sendStart();
		waitUntilDone();
		return twiSeq2Data[0];

	} else {
		//main_bf.mainIsLcdAttached = false;
		return 0;
	}
}

void twi_fw_sendStart(void)
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

		/* send START / REPEATED START */
		TWCR = (_BV(TWINT) | _BV(TWSTA) | _BV(TWEN) | _BV(TWIE));	// start-TWI/rise clock, send START, TWI enabled, TWINT-Interrupt enabled

		/* next state */
		twiState.state = TWI_STATE_START_SENT;

		sei();
	}
}

void isr_sendStop(uint8_t sendStopSignal)
{
	cli();

	twiSeq2DataCnt = 0;

	if (sendStopSignal) {
		/* send stop */
		TWCR = (_BV(TWINT) | _BV(TWSTO) | _BV(TWEN));  		// no interrupt enable (TWIE)
	}

	/* next state */
	twiState.isProcessing = false;
	twiState.state = TWI_STATE_READY;

	sei();
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
			TWDR = twiSeq1Adr;  							// I2C device address - write or read mode depends on (twiSeq1Adr & 0x01);
			TWCR = (_BV(TWINT) | _BV(TWEN) | _BV(TWIE));

			/* next state */
			twiState.state = TWI_STATE_ADR_SENT;
			sei();

		} else {
			twiState.errStart = true;
			isr_sendStop(true);
		}
		break;

	case TWI_STATE_ADR_SENT:
		if (localTwsrState == TWI_TWSR_M_SLAW_ADDR_ACK) {
			twiState.adrAck			= true;
			twiState.adrAckValid	= true;

			if (twiSeq2DataCnt <= 1) {
				/* next state */
				twiState.state = TWI_STATE_STOP;

			} else {
				/* next state */
				twiState.state = TWI_STATE_DATA_SENT;
			}

			/* send command data */
			TWDR = twiSeq2Data[0];							// internal command or address register of the I2C device
			TWCR = (_BV(TWINT) | _BV(TWEN) | _BV(TWIE));
			sei();

		} else if (localTwsrState == TWI_TWSR_M_SLAR_ADDR_ACK)  {
			twiState.adrAck			= true;
			twiState.adrAckValid	= true;

			/* no data transfer */

			/* next state */
			twiState.state = TWI_STATE_DATA_RCVD;

			TWCR = (_BV(TWINT) | _BV(TWEN) | _BV(TWIE));
			sei();

		} else {
			twiState.adrAck			= false;
			twiState.adrAckValid	= true;
			isr_sendStop(true);
		}
		break;

	case TWI_STATE_DATA_SENT:
		if (localTwsrState == TWI_TWSR_M_SLAW_DATA_ACK) {
			++twiSeq2DataIdx;
			twiState.dataAck		= true;
			twiState.dataAckValid	= true;

			if ((twiSeq2DataIdx + 1) >= twiSeq2DataCnt) {
				/* next state */
				twiState.state = TWI_STATE_STOP;

			} else {
				/* next state */
//				twiState.state = TWI_STATE_DATA_SENT;
			}

			/* send data */
			TWDR = twiSeq2Data[twiSeq2DataIdx];

			TWCR = (_BV(TWINT) | _BV(TWEN) | _BV(TWIE));
			sei();

		} else {
			twiState.dataAck		= false;
			twiState.dataAckValid	= true;
			isr_sendStop(true);
		}
		break;

	case TWI_STATE_DATA_RCVD:
		if ((localTwsrState == TWI_TWSR_M_SLAR_DATA_ACK) ||
			(localTwsrState == TWI_TWSR_M_SLAR_DATA_NACK)) {
			/* receive data */
			twiSeq2Data[twiSeq2DataIdx++]	= TWDR;
			twiState.dataAck				= true;
			twiState.dataAckValid			= true;

			if (twiSeq2DataIdx >= twiSeq2DataCnt) {
				/* next state */
				isr_sendStop(false);

				TWCR = (_BV(TWINT) | _BV(TWSTO) | _BV(TWEN) | _BV(TWIE));	// do not acknowledge

			} else {
				/* next state */
//				twiState.state = TWI_STATE_DATA_RCVD;

				TWCR = (_BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE));  	// acknowledge to get next data
			}
			sei();

		} else {
			twiSeq2Data[twiSeq2DataIdx] = localTwsrState;			// TODO remove me !
			twiState.dataAck			= false;
			twiState.dataAckValid		= true;
			isr_sendStop(true);
		}
		break;

	default:
	case TWI_STATE_STOP:
		isr_sendStop(true);
		break;
	}
}
