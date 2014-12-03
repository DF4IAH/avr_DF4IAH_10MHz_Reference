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
#include <util/delay.h>

#include "chipdef.h"
#include "main.h"
#include "df4iah_probe.h"
#include "df4iah_usb.h"
#include "df4iah_memory.h"
#include "df4iah_clkPullPwm.h"
#include "usbdrv/usbdrv.h"



// DATA SECTION

uint8_t timer0Snapshot = 0x00;
usbTxStatus_t usbTxStatus1, usbTxStatus3;
void (*jump_to_app)(void) = 0x0000;


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

//ISR(INT1_vect) {
//	send_error_msg();
//}


static inline void vectortable_to_bootloader(void) {
	asm volatile									// set active vector table into the Bootloader section
	(
		"ldi r24, %1\n\t"
		"out %0, r24\n\t"
		"ldi r24, %2\n\t"
		"out %0, r24\n\t"
		:
		: "i" (_SFR_IO_ADDR(MCUCR)),
		  "i" (1<<IVCE),
		  "i" (1<<IVSEL)
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

static inline void app_startup_check()
{
	// check for jumper-setting and for a valid jump-table entry
	if ((!check_jumper()) && (*((unsigned short*) 0x0000) == 0x0c94)) {
		close_probe();
		jump_to_app();								// jump to application sector
	}
}

void give_away(void)
{
    wdt_reset();
	usbPoll();
	debug_togglePin();							// XXX DEBUGGING
}


int main(void)
{
	vectortable_to_bootloader();
	init_wdt();
	init_probe();
 	app_startup_check();

	init_usb();										// starts at 67 ms after power-up, ends at 316 ms after power-up
    sei();											// ENABLE interrupt

	init_clkPullPwm();

    for(;;) {
    	give_away();
    }

	return 0;
}
