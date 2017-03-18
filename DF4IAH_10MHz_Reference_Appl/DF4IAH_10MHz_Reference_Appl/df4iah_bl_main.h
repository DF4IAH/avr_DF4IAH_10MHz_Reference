/*
 * df4iah_bl_main.h
 *
 *  Created on: 01.11.2014
 *      Author: espero
 */

#ifndef DF4IAH_BL_MAIN_H_
#define DF4IAH_BL_MAIN_H_

#ifndef false
# define false					0
#endif
#ifndef true
# define true					1
#endif


#define VERSION_HIGH 			150
#define VERSION_LOW  			224


#define UART_PORT			PORTD						// port D register
#define UART_DDR			DDRD						// DDR for port D
#define UART_TX_PIN			PIND						// PIN for TX pin at port D
#define UART_TX_PNUM		PIN1						// PIN number for TX pin
#define UART_RX_PIN			PIND						// PIN for RX pin at port D
#define UART_RX_PNUM		PIN0						// PIN number for RX pin

/* Timer-n compare output */
#define DDR_OC0A_REG		DDRD
#define DDR_OC0A			PD6
#define DDR_OC0B_REG		DDRD
#define DDR_OC0B			PD5
#define DDR_OC1A_REG		DDRB
#define DDR_OC1A			PB1
#define DDR_OC1B_REG		DDRB
#define DDR_OC1B			PB2
#define DDR_OC2A_REG		DDRB
#define DDR_OC2A			PB3
#define DDR_OC2B_REG		DDRD
#define DDR_OC2B			PD3

/*
 * Pin "STARTPIN" on port "STARTPORT" in this port has to grounded
 * (active low) to start the bootloader
 */
#define PROBE_DDR			DDRD
#define PROBE_PORT			PORTD
#define PROBE_PIN			PIND
#define PROBE_PNUM			PIN3						// JP3 BootLoader

/* Debugging toggle pin */
#define PWMTOGGLEPIN_DDR	DDRC
#define PWMTOGGLEPIN_PORT	PORTC
#define PWMTOGGLEPIN_PIN	PINC
#define PWMTOGGLEPIN_PNUM	PIN5						// PC5(ADC5/SCL) - Pin 28

/* BOOT token and place of BOOT token as offset before RAMEND */
# define BOOT_TOKEN			0xb00f
# define BOOT_TOKEN_EE_ADR	0x3fa						// @see df4iah_fw_memory.h


/* MCU frequency */
#ifndef F_CPU
// #define F_CPU 				7372800
#define F_CPU 					(7372800 / 2)
#endif

/* Device-Type:
   For AVRProg the BOOT-option is preferred
   which is the "correct" value for a bootloader.
   avrdude may only detect the part-code for ISP */
#define DEVTYPE     			DEVTYPE_BOOT
// #define DEVTYPE     			DEVTYPE_ISP

/*
 * Define if Watchdog-Timer should be disable at startup
 */
#define DISABLE_WDT_AT_STARTUP

/*
 * Watchdog-reset is issued at exit
 * define the timeout-value here (see avr-libc manual)
 */
#define EXIT_WDT_TIME			WDTO_250MS

/* wait-time for START_WAIT mode ( t = WAIT_TIME * 10ms ) */
#define WAIT_VALUE 				100 /* here: 100*10ms = 1000ms = 1sec */

#ifndef GET_LOCK_BITS
# define GET_LOCK_BITS			0x0001
# define GET_LOW_FUSE_BITS		0x0000
# define GET_HIGH_FUSE_BITS		0x0003
# define GET_EXTENDED_FUSE_BITS	0x0002
#endif


// STRINGS IN CODE SECTION
#define gcs_AVR_len  			7
#define gcs_FDL_len  			6
#define gcs_E99_len  			8


void give_away(void);

int main(void);

#endif /* DF4IAH_BL_MAIN_H_ */
