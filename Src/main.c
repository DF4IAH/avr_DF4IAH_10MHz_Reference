/*****************************************************************************
*
* AVRPROG compatible boot-loader
* Version  : 0.85 (Dec. 2008)
* Compiler : avr-gcc 4.1.2 / avr-libc 1.4.6
* size     : depends on features and startup ( minmal features < 512 words)
* by       : Martin Thomas, Kaiserslautern, Germany
*            eversmith@heizung-thomas.de
*            Additional code and improvements contributed by:
*           - Uwe Bonnes
*           - Bjoern Riemer
*           - Olaf Rempel
*           - DF4IAH, Ulrich Habel
*
* License  : Copyright (c) 2006-2008 M. Thomas, U. Bonnes, O. Rempel
*            Free to use. You have to mention the copyright
*            owners in source-code and documentation of derived
*            work. No warranty! (Yes, you can insert the BSD
*            license here)
*
* Tested with ATmega8, ATmega16, ATmega162, ATmega32, ATmega324P,
*             ATmega644, ATmega644P, ATmega128, AT90CAN128
*
* - Initial versions have been based on the Butterfly bootloader-code
*   by Atmel Corporation (Authors: BBrandal, PKastnes, ARodland, LHM)
*
****************************************************************************
*
*  See the makefile and readme.txt for information on how to adapt 
*  the linker-settings to the selected Boot Size (BOOTSIZE=xxxx) and 
*  the MCU-type. Other configurations futher down in this file.
*
*  With BOOT_SIMPLE, minimal features and discarded int-vectors
*  this bootloader has should fit into a a 512 word (1024, 0x400 bytes) 
*  bootloader-section. 
*
****************************************************************************/
// tabsize: 4


#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>   								// required by usbdrv.h

#include "chipdef.h"
#include "usbdrv_fw/usbdrv.h"
#include "df4iah_bl_memory.h"
#include "df4iah_bl_clkPullPwm.h"
#include "df4iah_fw_usb.h"
#include "df4iah_fw_clkPullPwm.h"
#include "df4iah_fw_ringbuffer.h"
#include "df4iah_fw_serial.h"

#include "main.h"


#ifndef BOOT_TOKEN_LO										// should be included from chipdef.h --> mega32.h
# define BOOT_TOKEN_LO										0xb0
# define BOOT_TOKEN_LO_REG									GPIOR1
# define BOOT_TOKEN_HI										0x0f
# define BOOT_TOKEN_HI_REG									GPIOR2
#endif


#define MAINCTXT_BUFFER_SIZE								250


// DATA SECTION

/* main */
void (*jump_to_bl)(void)	 								= (void*) 0x0000;
volatile uint8_t timer0Snapshot 							= 0x00;
volatile uint8_t stopAvr		 							= 0;
volatile uint8_t enterBL		 							= 0;
volatile uint8_t helpConcatNr								= 0;
uint8_t isUsbCommTest 										= false;
volatile uint8_t mainCtxtBufferIdx 							= 0;
usbTxStatus_t usbTxStatus1 									= { 0 },
			  usbTxStatus3 									= { 0 };

/* df4iah_fw_usb */
volatile uint16_t cntRcv 									= 0;
volatile uint16_t cntSend 									= 0;
volatile uint8_t usbIsrCtxtBufferIdx 						= 0;

/* df4iah_fw_ringbuffer */
volatile uint8_t usbRingBufferSendPushIdx 					= 0;
volatile uint8_t usbRingBufferSendPullIdx 					= 0;
volatile uint8_t usbRingBufferRcvPushIdx 					= 0;
volatile uint8_t usbRingBufferRcvPullIdx 					= 0;
volatile uint8_t usbRingBufferSendSemaphore 				= 0;  // semaphore is free
volatile uint8_t usbRingBufferRcvSemaphore 					= 0;  // semaphore is free
volatile uint8_t usbRingBufferHookLen 						= 0;
volatile uint8_t usbRingBufferHookIsSend 					= 0;

/* df4iah_fw_serial */
volatile uint8_t serialCtxtRxBufferLen						= 0;
volatile uint8_t serialCtxtTxBufferLen						= 0;
volatile uint8_t serialCtxtTxBufferIdx						= 0;


// ARRAYS - due to overwriting hazards they are following the controlling variables

/* main */
uchar mainCtxtBuffer[MAINCTXT_BUFFER_SIZE] 					= { 0 };

/* df4iah_fw_usb */
uchar usbIsrCtxtBuffer[USBISRCTXT_BUFFER_SIZE] 				= { 0 };
uchar usbCtxtSetupReplyBuffer[USBSETUPCTXT_BUFFER_SIZE] 	= { 0 };

/* df4iah_fw_ringbuffer */
uchar usbRingBufferSend[RINGBUFFER_SEND_SIZE] 				= { 0 };
uchar usbRingBufferRcv[RINGBUFFER_RCV_SIZE] 				= { 0 };
uchar usbRingBufferHook[RINGBUFFER_HOOK_SIZE] 				= { 0 };

/* df4iah_fw_serial */
uchar serialCtxtRxBuffer[SERIALCTXT_RX_BUFFER_SIZE] 		= { 0 };
uchar serialCtxtTxBuffer[SERIALCTXT_TX_BUFFER_SIZE] 		= { 0 };


// STRINGS IN MEMORY SECTION
const uchar VM_COMMAND_ABORT[]								= "ABORT";
const uchar VM_COMMAND_HELP[]								= "HELP";
const uchar VM_COMMAND_LOAD[]								= "LOAD";
const uchar VM_COMMAND_TEST[]								= "TEST";


// STRINGS IN CODE SECTION
// PROGMEM const char PM_VENDOR[] 							= "DF4IAH";
// const uint8_t PM_VENDOR_len = sizeof(PM_VENDOR);

PROGMEM const uchar PM_INTERPRETER_HELP1[] 					= "\n" \
															  "\n" \
															  "=== HELP ===\n" \
															  "\n" \
															  "$ <NMEA-Message>\t\tsends message to the GPS module.\n" \
															  "\n" \
															  "ABORT\t\t\t\tpowers the device down (sleep mode).\n";
const uint8_t PM_INTERPRETER_HELP1_len 						= sizeof(PM_INTERPRETER_HELP1);

PROGMEM const uchar PM_INTERPRETER_HELP2[] 					= "HELP\t\t\t\tthis message.\n" \
															  "\n" \
															  "LOAD\t\t\t\tenter bootloader.\n"
															  "\n" \
															  "TEST\t\t\t\ttoggles counter test.\n" \
															  "\n" \
															  "===========\n" \
															  "\n" \
															  ">";
const uint8_t PM_INTERPRETER_HELP2_len 						= sizeof(PM_INTERPRETER_HELP2);

PROGMEM const uchar PM_INTERPRETER_UNKNOWN[] 				= "*?* unknown command, try HELP.\n" \
															  "\n" \
															  ">";
const uint8_t PM_INTERPRETER_UNKNOWN_len 					= sizeof(PM_INTERPRETER_UNKNOWN);


// CODE SECTION

/*
 * @see http://www.nongnu.org/avr-libc/user-manual/group__avr__interrupts.html
 */

#if defined(BOOTLOADERHASNOVECTORS)
# warning "This Bootloader does not link interrupt vectors - see makefile"
/* make the linker happy - it wants to see __vector_default */
void __vector_default(void) { ; }
#endif

/*  VECTOR - TABLE
 *
	Address	Labels	Code 					Comments
	0x0000			jmp RESET 				; Reset Handler
	0x0002			jmp EXT_INT0 			; IRQ0 Handler
	0x0004			jmp EXT_INT1 			; IRQ1 Handler
	0x0006			jmp PCINT0 				; PCINT0 Handler
	0x0008			jmp PCINT1 				; PCINT1 Handler
	0x000A			jmp PCINT2 				; PCINT2 Handler
	0x000C			jmp WDT 				; Watchdog Timer Handler
	0x000E			jmp TIM2_COMPA 			; Timer2 Compare A Handler
	0x0010			jmp TIM2_COMPB 			; Timer2 Compare B Handler
	0x0012			jmp TIM2_OVF 			; Timer2 Overflow Handler
	0x0014			jmp TIM1_CAPT 			; Timer1 Capture Handler
	0x0016			jmp TIM1_COMPA 			; Timer1 Compare A Handler
	0x0018			jmp TIM1_COMPB 			; Timer1 Compare B Handler
	0x001A			jmp TIM1_OVF 			; Timer1 Overflow Handler
	0x001C			jmp TIM0_COMPA 			; Timer0 Compare A Handler
	0x001E			jmp TIM0_COMPB 			; Timer0 Compare B Handler
	0x0020			jmp TIM0_OVF 			; Timer0 Overflow Handler
	0x0022			jmp SPI_STC 			; SPI Transfer Complete Handler
	0x0024			jmp USART_RXC 			; USART, RX Complete Handler
	0x0026			jmp USART_UDRE 			; USART, UDR Empty Handler
	0x0028			jmp USART_TXC 			; USART, TX Complete Handler
	0x002A			jmp ADC 				; ADC Conversion Complete Handler
	0x002C			jmp EE_RDY 				; EEPROM Ready Handler
	0x002E			jmp ANA_COMP 			; Analog Comparator Handler
	0x0030			jmp TWI 				; 2-wire Serial Interface Handler
	0x0032			jmp SPM_RDY 			; Store Program Memory Ready Handler
	;
	0x0033	RESET:	ldi r16, high(RAMEND)	; Main program start
	0x0034			out SPH,r16 			; Set Stack Pointer to top of RAM
 *
 */

//EMPTY_INTERRUPT(INT0_vect);
//EMPTY_INTERRUPT(INT1_vect);
//EMPTY_INTERRUPT(PCINT0_vect);
//EMPTY_INTERRUPT(PCINT1_vect);
//EMPTY_INTERRUPT(PCINT2_vect);
//EMPTY_INTERRUPT(WDT_vect);
//EMPTY_INTERRUPT(TIMER2_COMPA_vect);
//EMPTY_INTERRUPT(TIMER2_COMPB_vect);
//EMPTY_INTERRUPT(TIMER2_OVF_vect);
//EMPTY_INTERRUPT(TIMER1_CAPT_vect);
//EMPTY_INTERRUPT(TIMER1_COMPA_vect);
//EMPTY_INTERRUPT(TIMER1_COMPB_vect);
//EMPTY_INTERRUPT(TIMER1_OVF_vect);
//EMPTY_INTERRUPT(TIMER0_COMPA_vect);
//EMPTY_INTERRUPT(TIMER0_COMPB_vect);
//EMPTY_INTERRUPT(TIMER0_OVF_vect);
//EMPTY_INTERRUPT(SPI_STC_vect);
//EMPTY_INTERRUPT(USART_RX_vect);
//EMPTY_INTERRUPT(USART_UDRE_vect);
//EMPTY_INTERRUPT(USART_TX_vect);
//ISR(USART_TX_vect, ISR_ALIASOF(USART_RX_vect));
//EMPTY_INTERRUPT(ADC_vect);
//EMPTY_INTERRUPT(EE_READY_vect);
//EMPTY_INTERRUPT(ANALOG_COMP_vect);
//EMPTY_INTERRUPT(TWI_vect);
//EMPTY_INTERRUPT(SPM_READY_vect);

/* assign interrupt routines to vectors */
ISR(USART_RX_vect) {
	serial_ISR_RXC0();
}

ISR(USART_TX_vect) {
	serial_ISR_TXC0();
}


static inline void vectortable_to_firmware(void) {
	cli();
	asm volatile											// set active vector table into the Firmware section
	(
		"ldi r24, %1\n\t"
		"out %0, r24\n\t"
		"ldi r24, %2\n\t"
		"out %0, r24\n\t"
		:
		: "i" (_SFR_IO_ADDR(MCUCR)),
		  "i" (1<<IVCE),
		  "i" (0)
		: "r24"
	);
}

static inline void wdt_init() {
	cli();
	wdt_reset();
	wdt_disable();
}

static inline void wdt_close() {
	cli();
	wdt_reset();
	wdt_disable();
}

static void doInterpret(uchar msg[], uint8_t len)
{
	const uint8_t isSend = true;

	if (!strncmp((char*) msg, (char*) VM_COMMAND_ABORT, sizeof(VM_COMMAND_ABORT))) {
		/* stop AVR controller and enter sleep state */
		stopAvr = true;

	} else if (!strncmp((char*) msg, (char*) VM_COMMAND_HELP, sizeof(VM_COMMAND_HELP))) {
		/* help information */
		if (getSemaphore(!isSend)) {
			int len = sprintf((char*) mainCtxtBuffer, "\n\n\n=== DF4IAH - 10 MHz Reference Oscillator ===\n=== Ver: %03d%03d\n", VERSION_HIGH, VERSION_LOW);
			ringBufferPush(!isSend, false, mainCtxtBuffer, len);
			ringBufferPush(!isSend, true, (uchar*) PM_INTERPRETER_HELP1, PM_INTERPRETER_HELP1_len);
			freeSemaphore(!isSend);
			helpConcatNr = 1;
		}

	} else if (!strncmp((char*) msg, (char*) VM_COMMAND_LOAD, sizeof(VM_COMMAND_LOAD))) {
		/* enter bootloader */
		enterBL = true;
		stopAvr = true;

	} else if (!strncmp((char*) msg, (char*) VM_COMMAND_TEST, sizeof(VM_COMMAND_TEST))) {
		/* special communication TEST */
		isUsbCommTest = !setTestOn(!isUsbCommTest);

	} else {
		/* unknown command */
		if (getSemaphore(!isSend)) {
			ringBufferPush(!isSend, true, (uchar*) PM_INTERPRETER_UNKNOWN, PM_INTERPRETER_UNKNOWN_len);
			freeSemaphore(!isSend);
		} else {
			ringBufferPushAddHook(!isSend, true, (uchar*) PM_INTERPRETER_UNKNOWN, PM_INTERPRETER_UNKNOWN_len);
		}
	}

#if 0  // XXX REMOVE ME!
	if (len > 0) {
		msg[len] = '#';
		msg[len + 1] = '0' + ((len / 10) % 10);
		msg[len + 2] = '0' + ( len       % 10);
		if (getSemaphore(!isSend)) {
			ringBufferPush(!isSend, false, msg, len + 3);
			freeSemaphore(!isSend);
		} else {
			msg[len] = '!';
			ringBufferPushAddHook(!isSend, false, msg, len + 3);
		}
	}
#endif
}

static void workInQueue()
{
	const uint8_t isSend = true;

#if 0  // XXX REMOVE ME!
	const uchar starString[3] = "<A>";
	const uchar hookString[3] = "<$>";
	static uint32_t cntr = 0;

	if (cntr++ > 100000) {
		cntr = 0;
		if (getSemaphore(!isSend)) {
			ringBufferPush(!isSend, false, starString, sizeof(starString));
			freeSemaphore(!isSend);
		} else {
			ringBufferPushAddHook(!isSend, false, hookString, sizeof(hookString));
		}
	}
#endif

	if (getSemaphore(isSend)) {
		uint8_t isLocked = true;
		enum RINGBUFFER_MSG_STATUS_t statusSend = getStatusNextMsg(isSend);
		enum RINGBUFFER_MSG_STATUS_t statusRcv  = getStatusNextMsg(!isSend);

		if (!helpConcatNr && (statusSend & RINGBUFFER_MSG_STATUS_AVAIL)) {	// if any message is available and not during help printing
			if (statusSend & RINGBUFFER_MSG_STATUS_IS_NMEA) {
				serial_pullAndSendNmea_havingSemaphore(isSend); isLocked = false;

			} else if ((statusSend & RINGBUFFER_MSG_STATUS_IS_MASK) == 0) {	// message from firmware state machine
				mainCtxtBufferIdx = ringBufferPull(isSend, mainCtxtBuffer, (uint8_t) sizeof(mainCtxtBuffer));
				freeSemaphore(isSend); isLocked = false;
				doInterpret(mainCtxtBuffer, mainCtxtBufferIdx);				// message is clean to process
			}

		} else if (helpConcatNr && !(statusRcv & RINGBUFFER_MSG_STATUS_AVAIL)) {  // during help printing, go ahead when receive buffer is empty again
			freeSemaphore(isSend); isLocked = false;

			if (helpConcatNr == 1) {
				if (getSemaphore(!isSend)) {
					ringBufferPush(!isSend, true, (uchar*) PM_INTERPRETER_HELP2, PM_INTERPRETER_HELP2_len);
					freeSemaphore(!isSend);
				}
			}
			helpConcatNr = 0;
		}

		if (isLocked) {
			freeSemaphore(isSend);
		}

#if 0
	} else {
		const uchar noSemString[3] = "<S>";
		const uchar noSemHookString[3] = "<s>";

		if (getSemaphore(!isSend)) {
			ringBufferPush(!isSend, false, noSemString, sizeof(noSemString));
			freeSemaphore(!isSend);
		} else {
			ringBufferPushAddHook(!isSend, false, noSemHookString, sizeof(noSemHookString));
		}
#endif
	}
}

static void sendInitialHelp()
{
	const uint8_t isSend = true;

	if (getSemaphore(isSend)) {
		ringBufferPush(isSend, false, VM_COMMAND_HELP, sizeof(VM_COMMAND_HELP));
		freeSemaphore(isSend);
	} else {
		ringBufferPushAddHook(isSend, false, VM_COMMAND_HELP, sizeof(VM_COMMAND_HELP));
	}
}

static void give_away(void)
{
    wdt_reset();

    usbPoll();
	usb_fw_sendInInterrupt();
	workInQueue();

	clkPullPwm_fw_togglePin();
}


int main(void)
{
	/* init AVR */
	{
		vectortable_to_firmware();
		wdt_init();

		clkPullPwm_fw_init();

		usb_fw_init();
		sei();
	}

	/* enter HELP command in USB host OUT queue */
	sendInitialHelp();

	/* run the chip */
    while (!stopAvr) {
    	give_away();
    }

    /* stop AVR */
    {
		cli();
		wdt_close();
		usb_fw_close();
		serial_fw_close();
		clkPullPwm_fw_close();

		// switch off all pull-up
		MCUCR |= _BV(PUD);										// general deactivation of all pull-ups

		// all pins are set to be input
		DDRB = 0x00;
		DDRC = 0x00;
		DDRD = 0x00;

		// all pull-ups are being switched off
		PORTB = 0x00;
		PORTC = 0x00;
		PORTD = 0x00;

		if (enterBL) {
			/* write BOOT token to SRAM */
			BOOT_TOKEN_LO_REG = BOOT_TOKEN_LO;
			BOOT_TOKEN_HI_REG = BOOT_TOKEN_HI;

			/* enter bootloader */
			jump_to_bl();										// jump to bootloader section

		} else {
			/* enter and keep in sleep mode */
			for (;;) {
				set_sleep_mode(SLEEP_MODE_EXT_STANDBY);
				cli();
				// if (some_condition) {
					sleep_enable();
					sleep_bod_disable();
					sei();
					sleep_cpu();
					sleep_disable();
				// }
				sei();
			}
		}
    }

	return 0;
}
