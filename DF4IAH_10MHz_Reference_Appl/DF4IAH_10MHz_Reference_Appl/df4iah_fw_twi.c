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
#include "df4iah_fw_usb.h"
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


static void s_delay(void)
{
	wdt_reset();
	if (usbIsUp) {
		usbPoll();
		usb_fw_sendInInterrupt();
		workInQueue();
	}
}

void twi_fw_waitUntilDone(uint8_t extraDelay)
{
	while (twiState.doStart || twiState.isProcessing) {
		s_delay();
	}

	/* Give some more time for the TWI slave to process the data */
	for (int cnt = extraDelay; cnt; --cnt) {
		s_delay();
	}
}

void twi_fw_sendCmdSendData1(uint8_t addr, uint8_t cmd, uint8_t data1)
{
	uint8_t sreg;

	twi_fw_waitUntilDone(addr == TWI_SMART_LCD_ADDR ?  10 : 0);

	sreg = SREG;
	cli();
	twiSeq1Adr = addr;
	twiSeq2DataCnt = 0;
	twiSeq2Data[twiSeq2DataCnt++] = cmd;
	twiState.isRepeatedStart = false;
	twiSeq2Data[twiSeq2DataCnt++] = data1;
	twiState.doStart = true;
	SREG = sreg;

	isr_sendStart(true, false);
}

void twi_fw_sendCmdSendData1SendData2(uint8_t addr, uint8_t cmd, uint8_t data1, uint8_t data2)
{
	uint8_t sreg;

	twi_fw_waitUntilDone(addr == TWI_SMART_LCD_ADDR ?  10 : 0);

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

	isr_sendStart(true, false);
}

void twi_fw_sendCmdSendData1SendDataVar(uint8_t addr, uint8_t cmd, uint8_t cnt, uint8_t data[])
{
	int i;
	uint8_t sreg;

	twi_fw_waitUntilDone(addr == TWI_SMART_LCD_ADDR ?  10 : 0);

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

	isr_sendStart(true, false);
}

uint8_t twi_fw_sendCmdReadData1(uint8_t addr, uint8_t cmd)
{
	uint8_t sreg;

	twi_fw_waitUntilDone(addr == TWI_SMART_LCD_ADDR ?  10 : 0);

	sreg = SREG;
	cli();
	twiSeq1Adr = addr;
	twiSeq2DataCnt = 0;
	twiSeq2Data[twiSeq2DataCnt++] = cmd;
	twiState.isRepeatedStart = true;
	twiSeq2DataRcvCnt = 1;
	twiState.doStart = true;
	SREG = sreg;

	isr_sendStart(true, false);
	twi_fw_waitUntilDone(0);

	return twiSeq2Data[0];
}

void isr_sendStart(uint8_t sendSignal, uint8_t isRepeatedStart)
{
	uint8_t sreg = SREG;
	cli();

	if (((!twiState.isProcessing && twiState.doStart) ||
		 ( twiState.isProcessing && isRepeatedStart)) &&
		twiSeq2DataCnt) {

		twiState.isProcessing	= true;
		twiState.doStart		= false;
		twiSeq2DataIdx			= 0;

		if (sendSignal) {
			/* send START or REPEATED START */
			TWCR = (_BV(TWINT) | _BV(TWSTA) | _BV(TWEN) | _BV(TWIE));	// start-TWI/rise clock, send START, TWI enabled, TWINT-Interrupt enabled
		}
	}

	SREG = sreg;
}

void isr_sendStop(uint8_t sendSignal)
{
	uint8_t sreg = SREG;
	cli();

	if (sendSignal && twiState.isProcessing) {
		/* send stop */
		TWCR = (_BV(TWINT) | _BV(TWSTO) | _BV(TWEN) | _BV(TWIE));
	}

	twiSeq2DataCnt = 0;
	twiState.isProcessing = false;

	SREG = sreg;
}


/* Forward declaration */
static uint8_t __vector_24__bottom(uint8_t tws, uint8_t twd, uint8_t twcr_cur);

//void twi_ISR_TWI(void) - __vector_24
ISR(TWI_vect, ISR_BLOCK)
{	/* TWI */
	uint8_t tws = TWSR & (0b11111 << TWS3);
	uint8_t twd = TWDR;
	uint8_t twcr_cur = TWCR;
	uint8_t twcr_new = __vector_24__bottom(tws, twd, twcr_cur);
	TWCR = twcr_new | _BV(TWINT) | _BV(TWEN) | _BV(TWIE);			// TWI interrupt flag reset, TWI enabled and TWINT-Interrupt enabled
}

static uint8_t __vector_24__bottom(uint8_t tws, uint8_t twd, uint8_t twcr_cur)
{
	uint8_t twcr_new = twcr_cur & 0b01000101;

	/* publish the state */
	twiState.state = tws;

	switch(tws) {
	case TWI_TWSR_START:
		/* MASTER: send SLA - slave WRITE address */
		TWDR = (twiSeq1Adr << 1);  						// I2C device address - write mode after first start
		break;

	case TWI_TWSR_REPEATEDSTART:
		/* MASTER: send SLA - slave address */
		TWDR = (twiSeq1Adr << 1) | 1; 					// I2C device address - read mode after repeated start

		twiState.isRepeatedStart = false;
		twiSeq2DataCnt = twiSeq2DataRcvCnt;
		break;

	case TWI_TWSR_M_SLAW_ADDR_ACK:
	case TWI_TWSR_M_SLAR_ADDR_ACK:
		if (twiSeq1Adr == TWI_MCP23017_ADDR) {
			main_bf.mainIsLcdAttached = true;
		} else if (twiSeq1Adr == TWI_SMART_LCD_ADDR) {
			main_bf.mainIsSmartAttached = true;
		}

		if (twiState.state == TWI_TWSR_M_SLAW_ADDR_ACK) {
			/* send command data */
			TWDR = twiSeq2Data[0];							// internal command or address register of the I2C device
		} else {
			// no data byte to store, here
			if ((twiSeq2DataIdx + 1) >= twiSeq2DataCnt) {
				twcr_new &= ~_BV(TWEA);						// NACK next data byte
			} else {
				twcr_new |= _BV(TWEA);						// ACK  next data byte to get further data
			}
		}
		break;

	case TWI_TWSR_M_SLAW_ADDR_NACK:
	case TWI_TWSR_M_SLAR_ADDR_NACK:
		if (twiSeq1Adr == TWI_MCP23017_ADDR) {
			main_bf.mainIsLcdAttached = false;
		} else if (twiSeq1Adr == TWI_SMART_LCD_ADDR) {
			main_bf.mainIsSmartAttached = false;
		}

		isr_sendStop(false);
		twcr_new |= _BV(TWSTO);
		break;

	case TWI_TWSR_M_SLAW_DATA_ACK:
		++twiSeq2DataIdx;
		if (twiSeq2DataIdx < twiSeq2DataCnt) {
			/* send data */
			TWDR = twiSeq2Data[twiSeq2DataIdx];

		} else {
			if (twiState.isRepeatedStart) {
				isr_sendStart(false, true);
				twcr_new |= _BV(TWSTA);
			} else {
				isr_sendStop(false);
				twcr_new |= _BV(TWSTO);
			}
		}
		break;

	case TWI_TWSR_M_SLAW_DATA_NACK:
		isr_sendStop(false);
		twcr_new |= _BV(TWSTO);
		break;

	case TWI_TWSR_M_SLAR_DATA_ACK:
		/* receive data */
		twiSeq2Data[twiSeq2DataIdx++]	= TWDR;
		if (twiSeq2DataIdx >= twiSeq2DataCnt) {
			twcr_new &= ~_BV(TWEA);							// NACK next data byte
		} else {
			twcr_new |= _BV(TWEA);							// ACK  next data byte to get further data
		}
		break;

	case TWI_TWSR_M_SLAR_DATA_NACK:
		if (twiSeq2DataIdx < twiSeq2DataCnt) {
			twiSeq2Data[twiSeq2DataIdx++]	= TWDR;
		}
		isr_sendStop(false);
		twcr_new |= _BV(TWSTO);
		break;

	default:
		isr_sendStop(false);
		twcr_new |= _BV(TWSTO);
		break;
	}

	return twcr_new;
}
