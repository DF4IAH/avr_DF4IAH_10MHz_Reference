/*
 * df4iah_fw_serial.c
 *
 *  Created on: 01.11.2014
 *      Author: DF4IAH, Ulrich Habel
 */
// tabsize: 4

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <avr/interrupt.h>

#include "chipdef.h"
#include "df4iah_fw_main.h"
#include "df4iah_fw_ringbuffer.h"
#include "df4iah_fw_memory_eepromData.h"
#include "df4iah_fw_memory.h"

#include "df4iah_fw_serial.h"


/* to make the compiler happy - @see chipdef.h --> mega32.h */
#ifndef UART_DDR
# define UART_DDR											DDRD
#endif
#ifndef UART_PORT
# define UART_PORT											PORTD
#endif
#ifndef UART_RX_PNUM
# define UART_RX_PNUM										0
#endif
#ifndef UART_TX_PNUM
# define UART_TX_PNUM										1
#endif


extern uint16_t serialCoef_b03_serial_baud;
extern uint16_t serialCoef_b03_bitsParityStopbits;
extern uint16_t serialCoef_b03_gps_comm_mode;

extern uint8_t  serialCtxtRxBufferLen;
extern uint8_t	serialCtxtNmeaRxHookBufIdx;
extern uint8_t  serialCtxtTxBufferLen;
extern uint8_t  serialCtxtTxBufferIdx;

extern uint8_t  serialCtxtBufferState;

extern uchar serialCtxtRxBuffer[SERIALCTXT_RX_BUFFER_SIZE];
extern uchar serialCtxtNmeaRxHookBuf[SERIALCTXT_NMEA_RX_HOOK_SIZE];
extern uchar serialCtxtTxBuffer[SERIALCTXT_TX_BUFFER_SIZE];

extern main_bf_t main_bf;
extern uchar mainFormatBuffer[MAIN_FORMAT_BUFFER_SIZE];


static uint16_t getCommParity(uint16_t val)
{
	return (val & DEFAULT_PARITY_N0_E2_O3_MASK) >> DEFAULT_PARITY_N0_E2_O3_BITPOS;
}

static uint16_t getCommStopBits(uint16_t val)
{
	return (val & DEFAULT_STOPBITS_MASK) >> DEFAULT_STOPBITS_BITPOS;
}

static uint16_t getCommDataBits(uint16_t val)
{
	return (val & DEFAULT_DATABITS_MASK) >> DEFAULT_DATABITS_BITPOS;
}

void serial_fw_init()
{
	/* power up this module */
	PRR &= ~(_BV(PRUSART0));

	// setting IO pins: pull-up on
	MCUCR     &= ~(_BV(PUD));											// ensure PUD is off --> activation of all pull-ups
	UART_PORT |=   _BV(UART_RX_PNUM);									// RX pull-up on

	/* read GPS coefficients */
	if (memory_fw_readEepromValidBlock(mainFormatBuffer, BLOCK_GPS_NR)) {
		eeprom_b03_t* b03 = (eeprom_b03_t*) &mainFormatBuffer;
		serialCoef_b03_serial_baud = b03->b03_serial_baud;
		serialCoef_b03_bitsParityStopbits = b03->b03_serial_bitsParityStopbits;
		serialCoef_b03_gps_comm_mode = b03->b03_gps_comm_mode;
	}

	// setting baud rate
	serial_fw_setCommBaud(serialCoef_b03_serial_baud);

#ifdef UART_DOUBLESPEED
	UART_STATUS = _BV(UART_DOUBLE);										// UCSR0A: U2X0
#endif

	// enabling the Transmitter and the Receiver
	UART_CTRL  =  _BV(RXEN0) 	|										// RXEN0=1,
				  _BV(TXEN0);											// TXEN0=1

	// setting frame format
	UART_CTRL2 = (0b00<<UMSEL00) 															|	// UCSR0C: asynchronous USART,
				 (( getCommParity(serialCoef_b03_bitsParityStopbits)		& 0b11)<<UPM00)	|	// parity 0=off, 2=even, 3=odd, 1=(do not use)
				 (((getCommStopBits(serialCoef_b03_bitsParityStopbits) - 1)	&  0b1)<<USBS0) |	// stop bits == 2
				 (((getCommDataBits(serialCoef_b03_bitsParityStopbits) - 5)	& 0b11)<<UCSZ00);	// bits 5..8

	// this is a dummy operation to clear the RX ready bit
	serialCtxtTxBufferIdx = UDR0;
	serialCtxtTxBufferIdx = 0;

	serial_fw_serRxIsrOn(true);
}

void serial_fw_close()
{
	// interrupt: clearing Global Interrupt Flag when interrupts are changed
	uint8_t sreg = SREG;
	cli();
	UART_CTRL = UART_CTRL & ~(_BV(RXCIE0)					|			// UCSR0B: disable all serial interrupts,
							  _BV(TXCIE0)					|
							  _BV(UDRIE0)					|
							  _BV(RXEN0)					|			// and TX/RX ports
							  _BV(TXEN0));
	SREG = sreg;

#ifdef UART_DOUBLESPEED
		UART_STATUS &= ~(_BV(UART_DOUBLE));
#endif

	// setting IO pins: pull-up off
	UART_PORT &= ~(_BV(UART_RX_PNUM));									// RX pull-up off

	/* no more power is needed for this module */
	PRR |= _BV(PRUSART0);
}

void serial_fw_serRxIsrOn(uint8_t flag)
{
	if (flag) {
		// interrupt: clearing Global Interrupt Flag when interrupts are changed
		uint8_t sreg = SREG;
		cli();
		UART_CTRL |= _BV(RXCIE0);											// UCSR0B: enable interrupts for RX data received
		SREG = sreg;

	} else {
		// interrupt: clearing Global Interrupt Flag when interrupts are changed
		uint8_t sreg = SREG;
		cli();
		UART_CTRL &= ~(_BV(RXCIE0));										// UCSR0B: disable interrupts for RX data received
		SREG = sreg;
	}
}

void serial_fw_setCommBaud(uint16_t baud)
{
	UART_BAUD_HIGH = ((UART_CALC_BAUDRATE(baud)>>8) & 0xff);
	UART_BAUD_LOW  = ( UART_CALC_BAUDRATE(baud)     & 0xff);
}

uint8_t serial_fw_isTxRunning()
{
	return (serialCtxtTxBufferLen > 0) ?  true : false;
}

static void serial_fw_sendNmea()
{
	uint8_t sreg = SREG;
	cli();

	/* clear TRANSMIT COMPLETE */
	UCSR0A &= ~(_BV(TXC0));

	/* initial load of USART data register, after this the ISR will handle it until the serial TX buffer is completed */
	UDR0 = serialCtxtTxBuffer[serialCtxtTxBufferIdx++];

	SREG = sreg;

	/* enable DATA REGISTER EMPTY INTERRUPT - the interrupt will arrive after initial UDSR0 loading */
	UART_CTRL |= _BV(UDRIE0);								// this will shoot an interrupt because UDR0 is ready again to be filled (UDRE0 is true)
}

void serial_fw_copyAndSendNmea(uint8_t isPgm, const uchar inData[], uint8_t len)
{
	if (len) {
		memory_fw_copyBuffer(isPgm, serialCtxtTxBuffer, inData, len);
		serialCtxtTxBufferIdx = 0;
		serialCtxtTxBufferLen = len;
		serial_fw_sendNmea();
	}
}

void serial_fw_pullAndSendNmea_havingSemaphore(uint8_t isSend)
{
	/* check if serial TX buffer is clear and the USART0 is ready for a new character to be sent */
	uint8_t sreg = SREG;
	cli();
	uint8_t isTxRdy = UCSR0A & _BV(UDRE0);
	SREG = sreg;

	if (!serialCtxtTxBufferLen && isTxRdy) {
		/* get message and free semaphore */
		serialCtxtTxBufferLen = ringbuffer_fw_ringBufferPull(isSend, serialCtxtTxBuffer, SERIALCTXT_TX_BUFFER_SIZE - 3);
		ringbuffer_fw_freeSemaphore(isSend);

		/* drop serial TX data if transportation is not activated */
		if (!(main_bf.mainIsSerComm)) {
			serialCtxtTxBufferLen = 0;
		}

		serialCtxtTxBufferIdx = 0;
		if (serialCtxtTxBufferLen) {
			if (serialCtxtTxBuffer[--serialCtxtTxBufferLen]) {  // chop off trailing NULL char
				serialCtxtTxBufferLen++;						// restore length, if not NULL
			}
			serialCtxtTxBuffer[serialCtxtTxBufferLen++] = '\r';	// obligatory NMEA message ends with CR LF
			serialCtxtTxBuffer[serialCtxtTxBufferLen++] = '\n';

			serial_fw_sendNmea();
		}

	} else {  // now we are not ready yet, call us later again
		ringbuffer_fw_freeSemaphore(isSend);
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
//void serial_ISR_RXC0(void) - __vector_18
ISR(USART_RX_vect, ISR_BLOCK)
{
	/* read the data byte received */
	uint8_t rxData = UDR0;

	/* since here we can allow global interrupts again */
	sei();

	if (!serialCtxtBufferState && (serialCtxtRxBufferLen < (SERIALCTXT_RX_BUFFER_SIZE - 3))) {
		serialCtxtBufferState = SERIAL_CTXT_BUFFER_STATE_BLOCK;

		/* if data is at the hook, get it first */
		if (serialCtxtNmeaRxHookBufIdx) {
			uint8_t sreg = SREG;
			cli();

			for (int idx = 0; idx < serialCtxtNmeaRxHookBufIdx; ++idx) {
				serialCtxtRxBuffer[serialCtxtRxBufferLen++] = serialCtxtNmeaRxHookBuf[idx];
			}

			/* hook is processed and cleared */
			serialCtxtNmeaRxHookBufIdx = 0;

			SREG = sreg;
		}

		/* append data to the buffer */
		serialCtxtRxBuffer[serialCtxtRxBufferLen++] = rxData;

		if (serialCtxtRxBufferLen >= (SERIALCTXT_RX_BUFFER_SIZE - 3)) {
			serialCtxtRxBuffer[serialCtxtRxBufferLen++] = '\r';
			rxData = '\n';
			serialCtxtRxBuffer[serialCtxtRxBufferLen++] = rxData;
		}

		/* if the end of a NMEA sentence is detected, send this serial RX buffer to the receive (IN) ring buffer */
		if (rxData == '\n') {  // a NMEA sentence stops with:  <sentence...*checksum\r\n>
			/* mark this job to be done in the main context */
			serialCtxtBufferState = SERIAL_CTXT_BUFFER_STATE_SEND;

		} else {
			/* append more data */
			serialCtxtBufferState = 0;
		}
	} else if (serialCtxtNmeaRxHookBufIdx < SERIALCTXT_NMEA_RX_HOOK_SIZE) {
		uint8_t sreg = SREG;
		cli();
		serialCtxtNmeaRxHookBuf[serialCtxtNmeaRxHookBufIdx++] = rxData;
		SREG = sreg;
	}
}

/*
 * x	Mnemonics	clocks	resulting clocks
 * ------------------------------------------------
 * 7	push		2		14
 * 1	in			1		 1
 * 1	eor			1		 1
 * 1	lds			2		 2
 * 1	andi		1		 1
 * 1	sts			2		 2
 * 1	sei			1		 1
 *
 * = 22 clocks --> 1.10 µs until sei() is done
 */
//void serial_ISR_UDRE0(void) - __vector_19
ISR(USART_UDRE_vect, ISR_BLOCK)
{
	UCSR0B &= ~(_BV(UDRIE0));								// disable interrupt for register empty
	sei();

	/* first look if the serial buffer is filled */
	if (serialCtxtTxBufferIdx < serialCtxtTxBufferLen) {
		cli();
		UDR0 = serialCtxtTxBuffer[serialCtxtTxBufferIdx++];	// UDRE0 becomes cleared
		UCSR0B |= _BV(UDRIE0);								// enables interrupt for register empty
		sei();
	}

	/* then check if job is now done */
	if (serialCtxtTxBufferIdx >= serialCtxtTxBufferLen) {
		/* job is done - turn off data register empty interrupt */
		cli();
		UCSR0B &= ~(_BV(UDRIE0));							// disable interrupt for register empty
		sei();

		/* mark buffer as free */
		serialCtxtTxBufferLen = 0;
	}
}

//void serial_ISR_TXC0(void) - __vector_20
ISR(USART_TX_vect, ISR_NOBLOCK)
{
	// not used yet
}
