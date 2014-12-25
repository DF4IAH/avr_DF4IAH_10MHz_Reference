/*****************************************************************************
*
* DF4IAH     10 MHz Reference Oscillator
* Version  : 141.224 (24. Dec. 2014)
*
******************************************************************************/
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
#include "df4iah_fw_memory.h"
#include "df4iah_fw_usb.h"
#include "df4iah_fw_clkPullPwm.h"
#include "df4iah_fw_ringbuffer.h"
#include "df4iah_fw_serial.h"

#include "main.h"


#ifndef BOOT_TOKEN											// should be included from chipdef.h --> mega32.h
# define BOOT_TOKEN											0xb00f
# define BOOT_TOKEN_EE_ADR									0x3fe
#endif


#define MAINCTXT_BUFFER_SIZE								250


// DATA SECTION

/* main */
void (*jump_to_fw)(void)	 								= (void*) 0x0000;
void (*jump_to_bl)(void)	 								= (void*) 0x7000;
volatile uint8_t timer0Snapshot 							= 0x00;
uint8_t jumperBlSet											= false;
volatile uint8_t stopAvr		 							= 0;
volatile enum ENTER_MODE_t enterMode						= ENTER_MODE_SLEEP;
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
uint8_t serialCtxtTxBufferIdx								= 0;


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
const uchar VM_COMMAND_LOADER[]								= "LOADER";
const uchar VM_COMMAND_REBOOT[]								= "REBOOT";
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
															  "LOADER\t\t\t\tenter bootloader.\n"
															  "\n" \
															  "REBOOT\t\t\t\treboot the firmware.\n" \
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
//ISR(INT1_vect, ISR_ALIASOF(INT0_vect));
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
//EMPTY_INTERRUPT(ADC_vect);
//EMPTY_INTERRUPT(EE_READY_vect);
//EMPTY_INTERRUPT(ANALOG_COMP_vect);
//EMPTY_INTERRUPT(TWI_vect);
//EMPTY_INTERRUPT(SPM_READY_vect);

/* assign interrupt routines to vectors */
/* due to optimizations the ISR is set at the function block directly */
//ISR(USART_RX_vect, ISR_BLOCK) {
//	serial_ISR_RXC0();
//}

//ISR(USART_UDRE_vect, ISR_BLOCK) {
//	serial_ISR_UDRE0();
//}

//ISR(USART_TX_vect, ISR_NOBLOCK) {
//	serial_ISR_TXC0();
//}


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
		enterMode = ENTER_MODE_SLEEP;
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

	} else if (!strncmp((char*) msg, (char*) VM_COMMAND_LOADER, sizeof(VM_COMMAND_LOADER))) {
		/* enter bootloader */
		enterMode = ENTER_MODE_BL;
		stopAvr = true;

	} else if (!strncmp((char*) msg, (char*) VM_COMMAND_REBOOT, sizeof(VM_COMMAND_REBOOT))) {
		/* enter firmware (REBOOT) */
		enterMode = ENTER_MODE_FW;
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
}

static void workInQueue()
{
	const uint8_t isSend = true;

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

	clkPullPwm_fw_togglePin();								// XXX for debugging purposes only

	/* go into sleep mode */
	// TODO sleep mode
}


int main(void)
{
	/* init AVR */
	{
		vectortable_to_firmware();
		wdt_init();
		clkPullPwm_fw_init();
		serial_fw_init();
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

		// all pins are set to be input
		DDRB = 0x00;
		DDRC = 0x00;
		DDRD = 0x00;

		// all pull-ups are being switched off
		PORTB = 0x00;
		PORTC = 0x00;
		PORTD = 0x00;

		if (enterMode) {
			if (enterMode == ENTER_MODE_BL) {
				/* write BOOT one time token to the EEPROM to INHIBIT restart into this Firmware again */
				uint16_t tokenVal = BOOT_TOKEN;
				memory_fw_writeEEpromPage((uint8_t*) &tokenVal, sizeof(tokenVal), BOOT_TOKEN_EE_ADR);

				/* enter bootloader */
				jump_to_bl();										// jump to bootloader section

			} else if (enterMode == ENTER_MODE_FW) {
				/* restart firmware */
				jump_to_fw();										// jump to firmware section (REBOOT)
			}

		} else {
			/* enter and keep in sleep mode */
			for (;;) {
				set_sleep_mode(SLEEP_MODE_PWR_DOWN);
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
