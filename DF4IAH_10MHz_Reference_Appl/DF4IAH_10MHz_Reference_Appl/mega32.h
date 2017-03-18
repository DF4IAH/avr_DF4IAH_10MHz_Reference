#ifndef _MEGA32_H_
#define _MEGA32_H_

/* Part-Code ISP */
#define DEVTYPE_ISP     	0x72
/* Part-Code Boot */
#define DEVTYPE_BOOT    	0x73

#define SIG_BYTE1			0x1E
#define SIG_BYTE2			0x95
#define SIG_BYTE3			0x02

/* USART */
#define UART_BAUD_HIGH		UBRR0H
#define UART_BAUD_LOW		UBRR0L
#define UART_STATUS			UCSR0A
#define UART_TXREADY		UDRE0
#define UART_RXREADY		RXC0
#define UART_DOUBLE			U2X0
#define UART_CTRL			UCSR0B
#define UART_CTRL_DATA		(_BV(TXEN0 | _BV(RXEN0))	// TX enable, RX enable
#define UART_CTRL2			UCSR0C
#define UART_CTRL2_DATA		(0b11 << UCSZ00)			// 8-bit
#define UART_DATA			UDR0

#endif // #ifndef _MEGA32_H_
