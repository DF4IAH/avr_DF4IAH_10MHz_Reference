/*
 * df4iah_serial.c
 *
 *  Created on: 01.11.2014
 *      Author: espero
 */

#include <stdint.h>
#include <avr/interrupt.h>

#include "df4iah_serial.h"

#include "main.h"


#ifdef RELEASE
__attribute__((section(".df4iah_serial"), aligned(2)))
#endif
void init_serial()
{
	// set baud rate
	UART_BAUD_HIGH = ((UART_CALC_BAUDRATE(BAUDRATE)>>8) & 0xFF);
	UART_BAUD_LOW  = ( UART_CALC_BAUDRATE(BAUDRATE)     & 0xFF);

#ifdef UART_DOUBLESPEED
	UART_STATUS = (1<<UART_DOUBLE);
#endif

	UART_CTRL = (UART_CTRL & 0b11100000) | UART_CTRL_DATA;
	UART_CTRL2 = UART_CTRL2_DATA;
}

#ifdef RELEASE
__attribute__((section(".df4iah_serial"), aligned(2)))
#endif
void close_serial()
{
#ifdef UART_DOUBLESPEED
		UART_STATUS &= ~(_BV(UART_DOUBLE));
#endif
}

#ifdef RELEASE
__attribute__((section(".df4iah_serial"), aligned(2)))
#endif
void sendchar_serial(uint8_t data)
{
	while (!(UART_STATUS & _BV(UART_TXREADY)));
	UART_DATA = data;
}

#ifdef RELEASE
__attribute__((section(".df4iah_serial"), aligned(2)))
#endif
uint8_t recvchar_serial(void)
{
	while (!(UART_STATUS & _BV(UART_RXREADY)));
	return UART_DATA;
}
