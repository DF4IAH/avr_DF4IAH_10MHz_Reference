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
	// setting IO pins: direction
	UART_DDR  = (UART_DDR  & ~(_BV(UART_RX_PNUM))) | _BV(UART_TX_PNUM);	// RX in, TX out

	// setting IO pins: pull-up
	UART_PORT = (UART_PORT & ~(_BV(UART_TX_PNUM))) | _BV(UART_RX_PNUM);	// RX pull-up on, TX off
	MCUCR = MCUCR & ~(_BV(PUD));										// general activation of all pull-ups, if not already done

	// setting baud rate
	UART_BAUD_HIGH = ((UART_CALC_BAUDRATE(DEFAULT_BAUDRATE)>>8) & 0xff);
	UART_BAUD_LOW  = ( UART_CALC_BAUDRATE(DEFAULT_BAUDRATE)     & 0xff);

#ifdef UART_DOUBLESPEED
	UART_STATUS = _BV(UART_DOUBLE);										// UCSR0A: U2X0
#endif

	// enabling the Transmitter or the Receiver
	UART_CTRL  = (UART_CTRL & 0b11100000) 				| 				// UCSR0B: keep interrupt state (RXCIE0 TXCIE0 RXCIE0),
				  _BV(RXEN0) 							| 				// RXEN0=1,
				  _BV(TXEN0);											// TXEN0=1

	// setting frame format
	UART_CTRL2 =  (0b00<<UMSEL00) 						|				// UCSR0C: asynchronous USART,
				  ((DEFAULT_PARITY_N0_E2_O3)<<UPM00)	|				// parity 0=off, 2=even, 3=odd, 1=(do not use)
				  ((DEFAULT_BITS - 5)		<<UCSZ00);					// bits 5..8

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
	UART_CTRL = UART_CTRL & ~(_BV(RXCIE0)				|				// UCSR0B: disable all serial interrupts,
							  _BV(TXCIE0)				|
							  _BV(UDRIE0)				|
							  _BV(RXEN0)				|				// and TX/RX ports
							  _BV(TXEN0));
	sei();

#ifdef UART_DOUBLESPEED
		UART_STATUS &= ~(_BV(UART_DOUBLE));
#endif

	// setting IO pins: pull-up off
	UART_PORT = UART_PORT & ~(_BV(UART_RX_PNUM) | _BV(UART_TX_PNUM));	// RX pull-up off, PUD is deactivated in main()

	// setting IO pins: direction
	UART_DDR  = UART_DDR  & ~(_BV(UART_RX_PNUM) | _BV(UART_TX_PNUM));	// RX and TX are in, again
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

		/* initial load of USART data register, after this the ISR will handle it until the serial TX buffer is completed */
		UDR0 = serialCtxtTxBuffer[serialCtxtTxBufferIdx++];

		/* clear TRANSMIT COMPLETE */
		UCSR0A = UCSR0A & ~(_BV(TXC0));

		/* enable DATA REGISTER EMPTY INTERRUPT */
		UCSR0B |= _BV(UDRIE0);

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
void serial_ISR_TXC0(void)
{
#if 1
	/* first look if the serial buffer is filled */
	if (serialCtxtTxBufferLen) {
		UDR0 = serialCtxtTxBuffer[serialCtxtTxBufferIdx++];

		/* check if job is done */
		if (serialCtxtTxBufferIdx == serialCtxtTxBufferLen) {
			/* turn off data register empty interrupt */
			UART_CTRL = UART_CTRL & ~(_BV(UDRIE0));									// UCSR0B: disable interrupt for register empty

			/* since here we can allow global interrupts again */
			sei();

			serialCtxtTxBufferIdx = serialCtxtTxBufferLen = 0;
		}

	} else {
		sei();
	}
#endif
}
