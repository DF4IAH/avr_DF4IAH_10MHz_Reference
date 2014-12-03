#ifndef _MEGA32_H_
#define _MEGA32_H_

/* Part-Code ISP */
#define DEVTYPE_ISP     0x72
/* Part-Code Boot */
#define DEVTYPE_BOOT    0x73

#define SIG_BYTE1	0x1E
#define SIG_BYTE2	0x95
#define SIG_BYTE3	0x02

/* USART */
#define UART_BAUD_HIGH	UBRR0H
#define UART_BAUD_LOW	UBRR0L
#define UART_STATUS	UCSR0A
#define UART_TXREADY	UDRE0
#define UART_RXREADY	RXC0
#define UART_DOUBLE	U2X
#define UART_CTRL	UCSR0B
#define UART_CTRL_DATA	((1<<TXEN0) | (1<<RXEN0))
#define UART_CTRL2	UCSR0C
#define UART_CTRL2_DATA	((1<<UCSZ01) | (1<<UCSZ00))
#define UART_DATA	UDR0

/* Timer-n compare output */
#define DDR_OC1A_REG	DDRB
#define DDR_OC1A	PB1
#define DDR_OC1B_REG	DDRB
#define DDR_OC1B	PB2
#define DDR_OC2_REG	DDRB
#define DDR_OC2	PB3

/*
 * Pin "STARTPIN" on port "STARTPORT" in this port has to grounded
 * (active low) to start the bootloader
 */
#define PROBE_PORT		PORTC
#define PROBE_DDR		DDRC
#define PROBE_PIN		PINC
#define PROBE_PNUM		PINC4		// JP3 BootLoader

/* PWM Debugging toggle pin */
#define PWMTOGGLEPIN_DDR	DDRD
#define PWMTOGGLEPIN_PORT	PORTD
#define PWMTOGGLEPIN_PIN	PIND
#define PWMTOGGLEPIN_PNUM	PIN3	// PD3(INT1) - Pin 5

#endif // #ifndef _MEGA32_H_
