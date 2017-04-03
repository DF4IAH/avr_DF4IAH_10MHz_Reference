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
#include "df4iah_fw_twi_mcp23017.h"
#include "df4iah_fw_twi_smart_lcd.h"

#include "df4iah_fw_twi.h"


extern uint8_t usbIsUp;
extern volatile main_bf_t main_bf;
extern volatile twiStatus_t twiState;
extern volatile uint8_t twiSeq1Adr;
extern volatile uint8_t twiSeq2DataCnt;
extern volatile uint8_t twiSeq2DataRcvCnt;
extern volatile uint8_t twiSeq2DataIdx;
extern volatile uint8_t twiSeq2Data[TWI_DATA_BUFFER_SIZE];


void waitUntilDone(void) {
	while (twiState.doStart || twiState.isProcessing) {
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

	// SCL frequency: using 400 kHz for SCL @20 MHz clock
	TWSR = 0; 							 						// prescaler = 1
	TWBR = 17;													// gives 400 kHz @20 MHz clock

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


void twi_fw_sendCmdSendData1(uint8_t addr, uint8_t cmd, uint8_t data1)
{
	uint8_t sreg;

	waitUntilDone();
	sreg = SREG;
	cli();
	twiSeq1Adr = addr;
	twiSeq2DataCnt = 0;
	twiSeq2Data[twiSeq2DataCnt++] = cmd;
	twiState.isRepeatedStart = false;
	twiSeq2Data[twiSeq2DataCnt++] = data1;
	twiState.doStart = true;
	SREG = sreg;

	isr_sendStart(false);
}

void twi_fw_sendCmdSendData1SendData2(uint8_t addr, uint8_t cmd, uint8_t data1, uint8_t data2)
{
	uint8_t sreg;

	waitUntilDone();
	sreg = SREG;
	cli();
	twiSeq1Adr = addr;
	twiSeq2DataCnt = 0;
	twiSeq2Data[twiSeq2DataCnt++] = cmd;
	twiState.isRepeatedStart = false;
	twiSeq2Data[twiSeq2DataCnt++] = data1;
	twiSeq2Data[twiSeq2DataCnt++] = data2;
	twiState.doStart = true;
	SREG = sreg;

	isr_sendStart(false);
}

void twi_fw_sendCmdSendData1SendDataVar(uint8_t addr, uint8_t cmd, uint8_t cnt, uint8_t data[])
{
	int i;
	uint8_t sreg;

	waitUntilDone();
	sreg = SREG;
	cli();
	twiSeq1Adr = addr;
	twiSeq2DataCnt = 0;
	twiSeq2Data[twiSeq2DataCnt++] = cmd;
	twiState.isRepeatedStart = false;
	for (i = 0; i < cnt; ++i) {
		twiSeq2Data[twiSeq2DataCnt++] = data[i];
	}
	twiState.doStart = true;
	SREG = sreg;

	isr_sendStart(false);
}

uint8_t twi_fw_sendCmdReadData1(uint8_t addr, uint8_t cmd)
{
	uint8_t sreg;

	waitUntilDone();
	sreg = SREG;
	cli();
	twiSeq1Adr = addr;
	twiSeq2DataCnt = 0;
	twiSeq2Data[twiSeq2DataCnt++] = cmd;
	twiState.isRepeatedStart = true;
	twiSeq2DataRcvCnt = 4;
	twiState.doStart = true;
	SREG = sreg;

	isr_sendStart(false);
	waitUntilDone();
	return twiSeq2Data[0];
}

void isr_sendStart(uint8_t isRepeatedStart)
{
	if ((twiState.doStart || isRepeatedStart) && twiSeq2DataCnt) {
		cli();

		twiState.isProcessing	= true;
		twiState.doStart		= false;
		twiState.errStart		= false;
		twiState.adrAck			= false;
		twiState.adrAckValid	= false;
		twiState.dataAck		= false;
		twiState.dataAckValid	= false;
		twiSeq2DataIdx			= 0;

		/* send START or REPEATED START */
		TWCR = (_BV(TWINT) | _BV(TWSTA) | _BV(TWEN) | _BV(TWIE));	// start-TWI/rise clock, send START, TWI enabled, TWINT-Interrupt enabled
		sei();
	}
}

void isr_sendStop(uint8_t sendStopSignal)
{
	uint8_t sreg = SREG;
	cli();

	twiSeq2DataCnt = 0;
	twiState.isProcessing = false;

	if (sendStopSignal) {
		/* send stop */
		TWCR = (_BV(TWINT) | _BV(TWSTO) | _BV(TWEN) | _BV(TWIE));

	} else {
		TWCR = (_BV(TWINT)              | _BV(TWEN) | _BV(TWIE));
	}
	SREG = sreg;
}


//void twi_ISR_TWI(void) - __vector_24
ISR(TWI_vect, ISR_BLOCK)
{
	/* read the data byte received */
	twiState.state = TWSR & TWI_TWSR_STATE_MASK;

	switch(twiState.state) {
	case TWI_TWSR_START:
		/* MASTER: send SLA - slave address */
		TWDR = (twiSeq1Adr << 1) & ~1;  				// I2C device address - write mode after first start

		TWCR = (_BV(TWINT) | _BV(TWEN) | _BV(TWIE));
		sei();
		break;

	case TWI_TWSR_REPEATEDSTART:
		/* MASTER: send SLA - slave address */
		TWDR = (twiSeq1Adr << 1) | 1; 					// I2C device address - read mode after repeated start

		twiState.isRepeatedStart = false;
		twiSeq2DataCnt = twiSeq2DataRcvCnt;

		isr_sendStart(true);							// send repeated start
		sei();
		break;

	case TWI_TWSR_M_SLAW_ADDR_ACK:
	case TWI_TWSR_M_SLAR_ADDR_ACK:
		twiState.adrAck			= true;
		twiState.adrAckValid	= true;

		if (twiSeq1Adr == TWI_MCP23017_ADDR) {
			main_bf.mainIsLcdAttached = true;
		} else if (twiSeq1Adr == TWI_SMART_LCD_ADDR) {
			main_bf.mainIsSmartAttached = true;
		}

		if (twiState.state == TWI_TWSR_M_SLAW_ADDR_ACK) {
			/* send command data */
			TWDR = twiSeq2Data[0];							// internal command or address register of the I2C device
		}

		TWCR = (_BV(TWINT) | _BV(TWEN) | _BV(TWIE));
		sei();
		break;

	case TWI_TWSR_M_SLAW_ADDR_NACK:
	case TWI_TWSR_M_SLAR_ADDR_NACK:
		twiState.adrAck			= false;
		twiState.adrAckValid	= true;

		if (twiSeq1Adr == TWI_MCP23017_ADDR) {
			main_bf.mainIsLcdAttached = false;
		} else if (twiSeq1Adr == TWI_SMART_LCD_ADDR) {
			main_bf.mainIsSmartAttached = false;
		}

		isr_sendStop(true);
		sei();
		break;

	case TWI_TWSR_M_SLAW_DATA_ACK:
		++twiSeq2DataIdx;
		twiState.dataAck		= true;
		twiState.dataAckValid	= true;

		if (twiSeq2DataIdx < twiSeq2DataCnt) {
			/* send data */
			TWDR = twiSeq2Data[twiSeq2DataIdx];
			TWCR = (_BV(TWINT) | _BV(TWEN) | _BV(TWIE));

		} else {
			if (twiState.isRepeatedStart) {
				isr_sendStart(true);
			} else {
				isr_sendStop(true);
			}
		}
		sei();
		break;

	case TWI_TWSR_M_SLAW_DATA_NACK:
		isr_sendStop(true);
		sei();
		break;

	case TWI_TWSR_M_SLAR_DATA_ACK:
	case TWI_TWSR_M_SLAR_DATA_NACK:
		/* receive data */
		twiSeq2Data[twiSeq2DataIdx++]	= TWDR;
		twiState.dataAck				= true;
		twiState.dataAckValid			= true;

		if ((twiState.state == TWI_TWSR_M_SLAR_DATA_NACK) || (twiSeq2DataIdx >= twiSeq2DataCnt)) {
			isr_sendStop(true);
		} else {
			TWCR = (_BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE));	// acknowledge to get next data
		}
		sei();
		break;

	default:
		isr_sendStop(true);
		sei();
		break;
	}
}
