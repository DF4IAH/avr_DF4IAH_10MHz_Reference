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
#include <avr/pgmspace.h>   /* required by usbdrv.h */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <string.h>
#include <util/delay.h>

#include "chipdef.h"
#include "usbdrv_fw/usbdrv.h"
#include "main.h"
#include "df4iah_bl_memory.h"
#include "df4iah_bl_clkPullPwm.h"
#include "df4iah_fw_usb.h"
#include "df4iah_fw_clkPullPwm.h"
#include "df4iah_fw_ringbuffer.h"
#include "df4iah_fw_serial.h"


#define MAINCTXT_BUFFER_SIZE			128


// DATA SECTION

/* main */
void (*jump_to_app)(void) = 0x0000;
uint8_t timer0Snapshot = 0x00;
usbTxStatus_t usbTxStatus1 = { 0 }, usbTxStatus3 = { 0 };
uint8_t isUsbCommTest = 0;
uchar mainCtxtBuffer[MAINCTXT_BUFFER_SIZE] = { 0 };
uint8_t mainCtxtBufferIdx = 0;

/* df4iah_fw_usb */
uchar usbIsrCtxtBuffer[USBISRCTXT_BUFFER_SIZE] = { 0 };
uint8_t usbIsrCtxtBufferIdx = 0;
uchar usbCtxtSetupReplyBuffer[USBSETUPCTXT_BUFFER_SIZE] = { 0 };
uint16_t cntRcv = 0;
uint16_t cntSend = 0;

/* df4iah_fw_ringbuffer */
uchar usbRingBufferSend[RINGBUFFER_SEND_SIZE] = { 0 };
uchar usbRingBufferRcv[RINGBUFFER_RCV_SIZE] = { 0 };
uchar usbRingBufferHook[RINGBUFFER_HOOK_SIZE] = { 0 };
uint8_t usbRingBufferSendPushIdx = 0;
uint8_t usbRingBufferSendPullIdx = 0;
uint8_t usbRingBufferRcvPushIdx = 0;
uint8_t usbRingBufferRcvPullIdx = 0;
uint8_t usbRingBufferSendSemaphore = 0;
uint8_t usbRingBufferRcvSemaphore = 0;
uint8_t usbRingBufferHookLen = 0;
uint8_t usbRingBufferHookIsSend = 0;


// STRINGS IN CODE SECTION
// PROGMEM const char VENDOR[VENDOR_len] = { 'D', 'F', '4', 'I', 'A', 'H' };


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


static inline void vectortable_to_firmware(void) {
	asm volatile									// set active vector table into the Firmware section
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

static inline void init_wdt() {
#ifdef DISABLE_WDT_AT_STARTUP
# ifdef WDT_OFF_SPECIAL
#   warning "using target specific watchdog_off"
	bootloader_wdt_off();
# else
	cli();

	wdt_reset();
	wdt_disable();
# endif
#endif
}

static void doInterpret(uchar msg[], uint8_t len)
{
	const uint8_t isSend = true;

	/* special communication TEST */
	if (!strncmp((char*) msg, "TEST", 4)) {
		isUsbCommTest = !setTestOn(!isUsbCommTest);
	}

#if 1  // XXX REMOVE ME!
	if (len > 0) {
		msg[len] = '#';
		msg[len + 1] = '0' + len;
		msg[len + 2] = 0;
		if (getSemaphore(!isSend)) {
			ringBufferPush(!isSend, msg, len + 2);
			freeSemaphore(!isSend);
		} else {
			ringBufferPushAddHook(!isSend, msg, len + 2);
		}
	}
#endif
}

static void workInQueue()
{
	const uchar starString[] = "*";
	const uint8_t isSend = true;
	static uint32_t cntr = 0;

#if 1  // XXX REMOVE ME!
	if (cntr++ > 100000) {
		cntr = 0;
		if (getSemaphore(!isSend)) {
			ringBufferPush(!isSend, starString, sizeof(starString));
			freeSemaphore(!isSend);
		} else {
			ringBufferPushAddHook(!isSend, starString, sizeof(starString));
		}
	}
#endif

	if (getSemaphore(isSend)) {
		enum RINGBUFFER_MSG_STATUS_t status = getStatusNextMsg(isSend);
		if (status & RINGBUFFER_MSG_STATUS_AVAIL) {
			if (status & RINGBUFFER_MSG_STATUS_IS_NMEA) {
				serial_pullAndSendNmea_havingSemaphore(isSend);

			} else if ((status & RINGBUFFER_MSG_STATUS_IS_MASK) == 0) {
				mainCtxtBufferIdx = ringBufferPull(isSend, mainCtxtBuffer, (uint8_t) sizeof(mainCtxtBuffer));
				freeSemaphore(isSend);
				doInterpret(mainCtxtBuffer, mainCtxtBufferIdx);
			}
		}
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
	vectortable_to_firmware();
	init_wdt();

    clkPullPwm_fw_init();

    usb_fw_init();
    sei();

    for(;;) {
    	give_away();
    }

    cli();
    usb_fw_close();
	return 0;
}
