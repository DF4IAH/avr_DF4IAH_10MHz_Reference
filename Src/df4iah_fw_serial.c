/*
 * df4iah_serial.c
 *
 *  Created on: 01.11.2014
 *      Author: DF4IAH, Ulrich Habel
 */

#include <stdio.h>
#include <stdint.h>
#include <avr/interrupt.h>

#include "chipdef.h"
#include "main.h"
#include "df4iah_fw_ringbuffer.h"

#include "df4iah_fw_serial.h"


/* to make the compiler happy - @see chipdef.h --> mega32.h */
#ifndef UART_DDR
# define UART_DDR	DDRD
#endif
#ifndef UART_PORT
# define UART_PORT	PORTD
#endif
#ifndef UART_RX_PNUM
# define UART_RX_PNUM	0
#endif
#ifndef UART_TX_PNUM
# define UART_TX_PNUM	1
#endif


extern uint8_t serialCtxtRxBufferLen;
extern uint8_t serialCtxtTxBufferLen;
extern uint8_t serialCtxtTxBufferIdx;

extern uchar serialCtxtRxBuffer[SERIALCTXT_RX_BUFFER_SIZE];
extern uchar serialCtxtTxBuffer[SERIALCTXT_TX_BUFFER_SIZE];


#ifdef RELEASE
__attribute__((section(".df4iah_fw_serial"), aligned(2)))
#endif
void serial_fw_init()
{
	// setting IO pins: pull-up on
	MCUCR = (MCUCR & ~(_BV(PUD)));										// ensure PUD is off --> activation of all pull-ups
	UART_PORT = UART_PORT | _BV(UART_RX_PNUM);							// RX pull-up on, PUD is deactivated in main()

	// setting baud rate
	UART_BAUD_HIGH = ((UART_CALC_BAUDRATE(DEFAULT_BAUDRATE)>>8) & 0xff);
	UART_BAUD_LOW  = ( UART_CALC_BAUDRATE(DEFAULT_BAUDRATE)     & 0xff);

#ifdef UART_DOUBLESPEED
	UART_STATUS = _BV(UART_DOUBLE);										// UCSR0A: U2X0
#endif

	// enabling the Transmitter and the Receiver
	UART_CTRL  = (UART_CTRL & 0b11100000) 					|			// UCSR0B: keep interrupt state (RXCIE0 TXCIE0 RXCIE0),
				  _BV(RXEN0) 								|			// RXEN0=1,
				  _BV(TXEN0);											// TXEN0=1

	// setting frame format
	UART_CTRL2 =  (0b00<<UMSEL00) 							|			// UCSR0C: asynchronous USART,
				  ((DEFAULT_PARITY_N0_E2_O3 & 0b11)<<UPM00)	|			// parity 0=off, 2=even, 3=odd, 1=(do not use)
				 (((DEFAULT_BITS - 5)       & 0b11)<<UCSZ00);			// bits 5..8

	// interrupt: clearing Global Interrupt Flag when interrupts are changed
	cli();
//	UART_CTRL |= _BV(RXCIE0);											// UCSR0B: enable interrupts for RX data received
	sei();
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_serial"), aligned(2)))
#endif
void serial_fw_close()
{
	// interrupt: clearing Global Interrupt Flag when interrupts are changed
	cli();
	UART_CTRL = UART_CTRL & ~(_BV(RXCIE0)					|			// UCSR0B: disable all serial interrupts,
							  _BV(TXCIE0)					|
							  _BV(UDRIE0)					|
							  _BV(RXEN0)					|			// and TX/RX ports
							  _BV(TXEN0));
	sei();

#ifdef UART_DOUBLESPEED
		UART_STATUS &= ~(_BV(UART_DOUBLE));
#endif

	// setting IO pins: pull-up off
	UART_PORT = UART_PORT & ~(_BV(UART_RX_PNUM));						// RX pull-up off
}

#if 0
#ifdef RELEASE
__attribute__((section(".df4iah_fw_serial"), aligned(2)))
#endif
void serial_fw_sendchar(uint8_t data)
{
	while (!(UART_STATUS & _BV(UART_TXREADY)));
	UART_DATA = data;
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_serial"), aligned(2)))
#endif
uint8_t serial_fw_recvchar(void)
{
	while (!(UART_STATUS & _BV(UART_RXREADY)));
	return UART_DATA;
}
#endif

#ifdef RELEASE
__attribute__((section(".df4iah_fw_serial"), aligned(2)))
#endif
void serial_pullAndSendNmea_havingSemaphore(uint8_t isSend)
{
	/* check if serial TX buffer is clear and the USART0 is ready for a new character to be sent */
	if (!serialCtxtTxBufferLen && !serialCtxtTxBufferIdx && (UCSR0A & _BV(UDRE0))) {
		/* get message and free semaphore */
		serialCtxtTxBufferLen = ringBufferPull(isSend, serialCtxtTxBuffer, (uint8_t) sizeof(serialCtxtTxBuffer));
		freeSemaphore(isSend);

		if (serialCtxtTxBufferLen) {
			if (serialCtxtTxBuffer[--serialCtxtTxBufferLen]) {  // chop off trailing NULL char
				serialCtxtTxBufferLen++;						// restore index, if not NULL
			}
			serialCtxtTxBuffer[serialCtxtTxBufferLen++] = '\r';	// obligatory NMEA message ends with CR LF
			serialCtxtTxBuffer[serialCtxtTxBufferLen++] = '\n';

			/* clear TRANSMIT COMPLETE */
			UCSR0A = UCSR0A & ~(_BV(TXC0));

			/* initial load of USART data register, after this the ISR will handle it until the serial TX buffer is completed */
			UDR0 = serialCtxtTxBuffer[serialCtxtTxBufferIdx++];

			/* enable DATA REGISTER EMPTY INTERRUPT - the interrupt will arrive after initial UDSR0 loading */
			UCSR0B |= _BV(UDRIE0);								// this will shoot an interrupt because UDR0 is ready again to be filled (UDRE0 is true)
		}

	} else {  // now we are not ready yet, call us later again
		freeSemaphore(isSend);

#if 1
		int len = sprintf((char*) serialCtxtTxBuffer, "E: can not enter SERIAL SEND - UCSR0A=0x%02x, serialCtxtTxBufferLen=%d, serialCtxtTxBufferIdx=%d\n", UCSR0A, serialCtxtTxBufferLen, serialCtxtTxBufferIdx);
		if (getSemaphore(!isSend)) {
			ringBufferPush(!isSend, false, serialCtxtTxBuffer, len);
			freeSemaphore(!isSend);
		} else {
			ringBufferPushAddHook(!isSend, false, serialCtxtTxBuffer, len);
		}
		serialCtxtTxBufferLen = serialCtxtTxBufferIdx = 0;
#endif
	}
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_serial"), aligned(2)))
#endif
void serial_ISR_RXC0(void)
{
#if 0
	const uint8_t isSend = false;

	/* read the data byte received */
	uint8_t rxData = UDR0;

	/* since here we can allow global interrupts again */
	sei();

	/* append data to the buffer */
	serialCtxtRxBuffer[serialCtxtRxBufferLen++] = rxData;

	/* if the end of a NMEA sentence is detected, send this serial RX buffer to the receive (IN) ring buffer */
	if (rxData == '\n') {  // a NMEA sentence stops with:  <sentence...*checksum\r\n>
		if (getSemaphore(!isSend)) {
			ringBufferPush(!isSend, false, serialCtxtRxBuffer, serialCtxtRxBufferLen);
			freeSemaphore(!isSend);
		} else {
			ringBufferPushAddHook(!isSend, false, serialCtxtRxBuffer, serialCtxtRxBufferLen);
		}
		serialCtxtRxBufferLen = 0;
	}
#endif
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_serial"), aligned(2)))
#endif
void serial_ISR_UDRE0(void)
{
	/* first look if the serial buffer is filled */
	if (serialCtxtTxBufferLen) {
		if (serialCtxtTxBufferIdx < serialCtxtTxBufferLen) {
			UDR0 = serialCtxtTxBuffer[serialCtxtTxBufferIdx++];
		}

		/* check if job is done */
		if (serialCtxtTxBufferIdx == serialCtxtTxBufferLen) {
			/* turn off data register empty interrupt */
			UART_CTRL = UART_CTRL & ~(_BV(UDRIE0));									// UCSR0B: disable interrupt for register empty

			/* mark buffer free */
			serialCtxtTxBufferIdx = serialCtxtTxBufferLen = 0;
		}
	}
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_serial"), aligned(2)))
#endif
void serial_ISR_TXC0(void)
{
	// not used yet
}
