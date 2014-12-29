/*****************************************************************************
*
* DF4IAH     10 MHz Reference Oscillator
* Version  : 141.224 (24. Dec. 2014)
*
******************************************************************************/
// tabsize: 4


#include <stdint.h>
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>   								// required by usbdrv.h

#include "chipdef.h"
#include "usbdrv_fw/usbdrv.h"
#include "df4iah_fw_usb.h"
#include "df4iah_bl_memory.h"
#include "df4iah_bl_clkPullPwm.h"
#include "df4iah_fw_memory.h"
#include "df4iah_fw_ringbuffer.h"
#include "df4iah_fw_clkPullPwm.h"
#include "df4iah_fw_clkFastCtr.h"
#include "df4iah_fw_anlgComp.h"
#include "df4iah_fw_clkSlowCtr.h"
#include "df4iah_fw_serial.h"

#include "df4iah_fw_main.h"


#ifndef BOOT_TOKEN											// should be included from chipdef.h --> mega32.h
# define BOOT_TOKEN											0xb00f
#endif


#define MAINCTXT_BUFFER_SIZE								128


// DATA SECTION

/* df4iah_fw_main */
void (*mainJumpToFW)(void)	 								= (void*) 0x0000;
void (*mainJumpToBL)(void)	 								= (void*) 0x7000;
float mainCoef_b01_ref_AREF_V								= 0.0f;
float mainCoef_b01_ref_1V1_V								= 0.0f;
float mainCoef_b01_temp_ofs_adc_25C_steps					= 0.0f;
float mainCoef_b01_temp_k_p1step_adc_K						= 0.0f;
volatile uint8_t  mainCtxtBufferIdx 						= 0;
volatile enum REFCLK_STATE_t mainRefClkState				= REFCLK_STATE_NOSYNC;
volatile int32_t  mainPwmTerminalAdj						= 0;
volatile uint8_t  mainHelpConcatNr							= 0;
volatile uint8_t  mainIsTimerTest							= false;
volatile uint8_t  mainIsJumperBlSet							= false;
volatile uint8_t  mainStopAvr		 						= 0;
volatile enum ENTER_MODE_t mainEnterMode					= ENTER_MODE_SLEEP;
volatile uint8_t  timer0Snapshot 							= 0x00;
usbTxStatus_t usbTxStatus1 									= { 0 },
			  usbTxStatus3 									= { 0 };

/* df4iah_fw_clkPullPwm */
volatile uint16_t pullCoef_b02_pwm_initial					= 0;
volatile uint16_t pullPwmVal								= 0;

/* df4iah_fw_anlgComp (10kHz) */
volatile uint32_t ac_timer_10us								= 0;
//volatile uint8_t  ac_stamp_TCNT2							= 0;
volatile uint8_t  ac_adc_convertNowCh						= 0;
volatile uint8_t  ac_adc_convertNowTempStep					= 0;

/* df4iah_fw_clkSlowCtr (PPS) */
volatile uint32_t csc_timer_s_HI							= 0;
volatile uint8_t  csc_stamp_TCNT2							= 0;
volatile uint32_t csc_stamp_10us							= 0;

/* df4iah_fw_ringbuffer */
volatile uint8_t  usbRingBufferSendPushIdx 					= 0;
volatile uint8_t  usbRingBufferSendPullIdx 					= 0;
volatile uint8_t  usbRingBufferRcvPushIdx 					= 0;
volatile uint8_t  usbRingBufferRcvPullIdx 					= 0;
volatile uint8_t  usbRingBufferHookLen 						= 0;
volatile uint8_t  usbRingBufferHookIsSend 					= 0;
volatile uint8_t  usbRingBufferSendSemaphore 				= 0;  // semaphore is free
volatile uint8_t  usbRingBufferRcvSemaphore 				= 0;  // semaphore is free

/* df4iah_fw_serial */
volatile uint8_t  serialCtxtRxBufferLen						= 0;
volatile uint8_t  serialCtxtTxBufferLen						= 0;
volatile uint8_t  serialCtxtTxBufferIdx						= 0;

/* df4iah_fw_usb */
//volatile uint16_t usbSetupCntr							= 0;
volatile uint16_t cntRcv 									= 0;
volatile uint16_t cntSend 									= 0;
volatile uint8_t  usbIsrCtxtBufferIdx 						= 0;
volatile uint8_t  isSerComm									= true;
volatile uint8_t  isUsbCommTest 							= false;


// ARRAYS - due to overwriting hazards they are following the controlling variables

/* df4iah_fw_main */
uchar mainCtxtBuffer[MAINCTXT_BUFFER_SIZE] 					= { 0 };
uint8_t eepromBlockCopy[sizeof(eeprom_b00_t)]				= { 0 };  // any block has the same size

/* df4iah_fw_anlgComp (10kHz) */
volatile uint16_t ac_adc_ch[AC_ADC_CH_COUNT + 1]			= { 0 };  // plus one for the temperature sensor

/* df4iah_fw_ringbuffer */
uchar usbRingBufferSend[RINGBUFFER_SEND_SIZE] 				= { 0 };
uchar usbRingBufferRcv[RINGBUFFER_RCV_SIZE] 				= { 0 };
uchar usbRingBufferHook[RINGBUFFER_HOOK_SIZE] 				= { 0 };

/* df4iah_fw_serial */
uchar serialCtxtRxBuffer[SERIALCTXT_RX_BUFFER_SIZE] 		= { 0 };
uchar serialCtxtTxBuffer[SERIALCTXT_TX_BUFFER_SIZE] 		= { 0 };

/* df4iah_fw_usb */
uchar usbIsrCtxtBuffer[USBISRCTXT_BUFFER_SIZE] 				= { 0 };
uchar usbCtxtSetupReplyBuffer[USBSETUPCTXT_BUFFER_SIZE] 	= { 0 };


// STRINGS IN MEMORY SECTION
const uchar VM_COMMAND_HALT[]								= "HALT";
const uchar VM_COMMAND_HELP[]								= "HELP";
const uchar VM_COMMAND_INFO[]								= "INFO";
const uchar VM_COMMAND_LOADER[]								= "LOADER";
const uchar VM_COMMAND_REBOOT[]								= "REBOOT";
const uchar VM_COMMAND_SEROFF[]								= "SEROFF";
const uchar VM_COMMAND_SERON[]								= "SERON";
const uchar VM_COMMAND_TEST[]								= "TEST";
const uchar VM_COMMAND_WRITEPWM[]							= "WRITEPWM";
const uchar VM_COMMAND_PLUSSIGN[]							= "+";
const uchar VM_COMMAND_MINUSSIGN[]							= "-";


// STRINGS IN CODE SECTION
// PROGMEM const char PM_VENDOR[] 							= "DF4IAH";
// const uint8_t PM_VENDOR_len = sizeof(PM_VENDOR);

PROGMEM const uchar PM_INTERPRETER_HELP1[] 					= "\n" \
															  "\n" \
															  "=== HELP ===\n" \
															  "\n" \
															  "$ <NMEA-Message>\t\tsends message to the GPS module.\n" \
															  "\n" \
															  "HALT\t\t\t\tpowers the device down (sleep mode).";
const uint8_t PM_INTERPRETER_HELP1_len 						= sizeof(PM_INTERPRETER_HELP1);

PROGMEM const uchar PM_INTERPRETER_HELP2[] 					= "\n" \
															  "HELP\t\t\t\tthis message.\n" \
		  	  	  	  	  	  	  	  	  	  	  	  	  	  "\n" \
															  "INFO\t\t\t\ttoggles additional printed infos.\n" \
															  "\n" \
															  "LOADER\t\t\t\tenter bootloader.";
const uint8_t PM_INTERPRETER_HELP2_len 						= sizeof(PM_INTERPRETER_HELP2);

PROGMEM const uchar PM_INTERPRETER_HELP3[] 					= "\n" \
															  "REBOOT\t\t\t\treboot the firmware.\n" \
		  	  	  	  	  	  	  	  	  	  	  	  	  	  "\n" \
		  	  	  	  	  	  	  	  	  	  	  	  	  	  "SEROFF\t\t\t\tswitch serial communication OFF.\n" \
		  	  	  	  	  	  	  	  	  	  	  	  	  	  "SERON\t\t\t\tswitch serial communication ON.";
const uint8_t PM_INTERPRETER_HELP3_len 						= sizeof(PM_INTERPRETER_HELP3);

PROGMEM const uchar PM_INTERPRETER_HELP4[] 					= "\n" \
															  "TEST\t\t\t\ttoggles counter test.\n" \
															  "\n" \
															  "WRITEPWM\t\t\tstore current PWM as default value." \
															  "\n" \
															  "+/- <PWM value>\t\tcorrection value to be added.";
const uint8_t PM_INTERPRETER_HELP4_len 						= sizeof(PM_INTERPRETER_HELP4);

PROGMEM const uchar PM_INTERPRETER_HELP5[] 					= "\n" \
															  "===========\n" \
		  	  	  	  	  	  	  	  	  	  	  	  	  	  "\n" \
		  	  	  	  	  	  	  	  	  	  	  	  	  	  ">";
const uint8_t PM_INTERPRETER_HELP5_len 						= sizeof(PM_INTERPRETER_HELP5);

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
/* due to optimizations the ISRs are set at the function block directly */
ISR(WDT_vect, ISR_NAKED) {  // vector_6 - nothing to do, resets WDIF bit
	__asm__ __volatile__ ("reti" ::: "memory");
}


#ifdef RELEASE
__attribute__((section(".df4iah_fw_main"), aligned(2)))
#endif
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

#ifdef RELEASE
__attribute__((section(".df4iah_fw_main"), aligned(2)))
#endif
static inline void wdt_init() {
	wdt_disable();
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_main"), aligned(2)))
#endif
static inline void wdt_close() {
	wdt_disable();
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_main"), aligned(2)))
#endif
static void doInterpret(uchar msg[], uint8_t len)
{
	const uint8_t isSend = true;

	if (!strncmp((char*) msg, (char*) VM_COMMAND_HALT, sizeof(VM_COMMAND_HALT))) {
		/* stop AVR controller and enter sleep state */
		isSerComm = false;
		mainIsTimerTest = false;
		isUsbCommTest = false;
		mainEnterMode = ENTER_MODE_SLEEP;
		mainStopAvr = true;

	} else if (!strncmp((char*) msg, (char*) VM_COMMAND_HELP, sizeof(VM_COMMAND_HELP))) {
		/* help information */
		int len = sprintf((char*) mainCtxtBuffer, "\n\n\n=== DF4IAH - 10 MHz Reference Oscillator ===\n=== Ver: %03d%03d\n", VERSION_HIGH, VERSION_LOW);
		ringBufferWaitAppend(!isSend, false, mainCtxtBuffer, len);
		ringBufferWaitAppend(!isSend, true, (uchar*) PM_INTERPRETER_HELP1, PM_INTERPRETER_HELP1_len);
		mainHelpConcatNr = 1;
		mainIsTimerTest = false;
		isUsbCommTest = false;

	} else if (!strncmp((char*) msg, (char*) VM_COMMAND_INFO, sizeof(VM_COMMAND_INFO))) {
		/* timer 2 overflow counter TEST */
		mainIsTimerTest = !mainIsTimerTest;
		if (mainIsTimerTest) {
			isSerComm = false;
			isUsbCommTest = false;
		}

	} else if (!strncmp((char*) msg, (char*) VM_COMMAND_LOADER, sizeof(VM_COMMAND_LOADER))) {
		/* enter bootloader */
		isSerComm = false;
		mainIsTimerTest = false;
		isUsbCommTest = false;
		mainEnterMode = ENTER_MODE_BL;
		mainStopAvr = true;

	} else if (!strncmp((char*) msg, (char*) VM_COMMAND_REBOOT, sizeof(VM_COMMAND_REBOOT))) {
		/* enter firmware (REBOOT) */
		isSerComm = false;
		mainIsTimerTest = false;
		isUsbCommTest = false;
		mainEnterMode = ENTER_MODE_FW;
		mainStopAvr = true;

	} else if (!strncmp((char*) msg, (char*) VM_COMMAND_SEROFF, sizeof(VM_COMMAND_SEROFF))) {
		/* serial communication OFF */
		isSerComm = false;

	} else if (!strncmp((char*) msg, (char*) VM_COMMAND_SERON, sizeof(VM_COMMAND_SERON))) {
		/* serial communication ON */
		isSerComm = true;
		mainIsTimerTest = false;
		isUsbCommTest = false;

	} else if (!strncmp((char*) msg, (char*) VM_COMMAND_TEST, sizeof(VM_COMMAND_TEST))) {
		/* special communication TEST */
		isUsbCommTest = !isUsbCommTest;
		if (isUsbCommTest) {
			isSerComm = false;
			mainIsTimerTest = false;
		}

	} else if (!strncmp((char*) msg, (char*) VM_COMMAND_WRITEPWM, sizeof(VM_COMMAND_WRITEPWM))) {
		/* write current PWM value as the default/startup value to the EEPROM */
		memory_fw_writeEEpromPage((uint8_t*) &pullPwmVal, sizeof(uint16_t), offsetof(eeprom_layout_t, b02.b02_pwm_initial));

	} else if (msg[0] == VM_COMMAND_PLUSSIGN[0]) {
		/* correct the PWM value up */
		int32_t scanVal = 0;
		sscanf((char*) msg + 1, "%ld", &scanVal);
		mainPwmTerminalAdj = scanVal;
		isSerComm = false;
		isUsbCommTest = false;

	} else if (msg[0] == VM_COMMAND_MINUSSIGN[0]) {
		/* correct the PWM value down */
		int32_t scanVal = 0;
		sscanf((char*) msg + 1, "%ld", &scanVal);
		mainPwmTerminalAdj = -scanVal;
		isSerComm = false;
		isUsbCommTest = false;

	} else {
		/* unknown command */
		ringBufferWaitAppend(!isSend, true, (uchar*) PM_INTERPRETER_UNKNOWN, PM_INTERPRETER_UNKNOWN_len);
	}
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_main"), aligned(2)))
#endif
static void workInQueue()
{
	const uint8_t isSend = true;

	if (getSemaphore(isSend)) {
		uint8_t isLocked = true;
		enum RINGBUFFER_MSG_STATUS_t statusSend = getStatusNextMsg(isSend);
		enum RINGBUFFER_MSG_STATUS_t statusRcv  = getStatusNextMsg(!isSend);

		if (!mainHelpConcatNr && (statusSend & RINGBUFFER_MSG_STATUS_AVAIL)) {	// if any message is available and not during help printing
			if (statusSend & RINGBUFFER_MSG_STATUS_IS_NMEA) {
				serial_pullAndSendNmea_havingSemaphore(isSend); isLocked = false;

			} else if ((statusSend & RINGBUFFER_MSG_STATUS_IS_MASK) == 0) {	// message from firmware state machine
				mainCtxtBufferIdx = ringBufferPull(isSend, mainCtxtBuffer, (uint8_t) sizeof(mainCtxtBuffer));
				freeSemaphore(isSend); isLocked = false;
				doInterpret(mainCtxtBuffer, mainCtxtBufferIdx);				// message is clean to process
			}

		} else if (mainHelpConcatNr && !(statusRcv & RINGBUFFER_MSG_STATUS_AVAIL)) {  // during help printing, go ahead when receive buffer is empty again
			freeSemaphore(isSend); isLocked = false;

			switch (mainHelpConcatNr) {
			case 1:
				ringBufferWaitAppend(!isSend, true, (uchar*) PM_INTERPRETER_HELP2, PM_INTERPRETER_HELP2_len);
				mainHelpConcatNr = 2;
				break;

			case 2:
				ringBufferWaitAppend(!isSend, true, (uchar*) PM_INTERPRETER_HELP3, PM_INTERPRETER_HELP3_len);
				mainHelpConcatNr = 3;
				break;

			case 3:
				ringBufferWaitAppend(!isSend, true, (uchar*) PM_INTERPRETER_HELP4, PM_INTERPRETER_HELP4_len);
				mainHelpConcatNr = 4;
				break;

			case 4:
				ringBufferWaitAppend(!isSend, true, (uchar*) PM_INTERPRETER_HELP5, PM_INTERPRETER_HELP5_len);
				// no break
			default:
				mainHelpConcatNr = 0;
				break;
			}
		}

		if (isLocked) {
			freeSemaphore(isSend);
		}
	}
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_main"), aligned(2)))
#endif
static void doJobs()
{
	const uint8_t isSend = false;							// USB Function --> host: USB IN
	static uint8_t isTimerTestPrintCtr = 0;
	static uint32_t refClkSM_last_csc_timer_s = 0;
	uint32_t csc_timer_s = ((uint32_t) (csc_timer_s_HI << 8)) | TCNT0;

	if ((refClkSM_last_csc_timer_s != csc_timer_s) || isTimerTestPrintCtr) {  // a new SPS impulse has arrived ... and some following lines
		/*
		 * ATTENTION: Floating vfprint() and friends needs changes to the linker
		 * @see http://winavr.scienceprog.com/avr-gcc-tutorial/using-sprintf-function-for-float-numbers-in-avr-gcc.html
		 *
		 * 1)	Linker option:		     --Wl,-u,vfprintf  --Wl,-u,vfscanf
		 * 2)	Linker libraries:	-lm  -lprintf_flt      -lscanf_flt
		 */

		float adc_ch0_volts = ( ac_adc_ch[0] * mainCoef_b01_ref_AREF_V) / 1024.f;
		float adc_ch1_volts = ( ac_adc_ch[1] * mainCoef_b01_ref_AREF_V) / 1024.f;
		float adc_ch2_C     = ((ac_adc_ch[2] - mainCoef_b01_temp_ofs_adc_25C_steps) * mainCoef_b01_temp_k_p1step_adc_K) + 25.0f;

		if (refClkSM_last_csc_timer_s != csc_timer_s) {
			/* 10 MHz Ref-Clk State Machine */
			refClkSM_last_csc_timer_s = csc_timer_s;
			ac_adc_convertNowTempStep = 1;  				// prepare for next temperature conversion, once per second
			isTimerTestPrintCtr = 1;						// show 1 timer line per second
		}

		if (mainIsTimerTest && isTimerTestPrintCtr) {
			/* enter this block just n times per second */
			--isTimerTestPrintCtr;

			uint8_t len;
			len = sprintf((char*) mainCtxtBuffer,
					"### csc_timer_s=%09lu\tcsc_stamp_10us=%05lu x10us\t+ csc_stamp_TCNT2=%03u\n",
					csc_timer_s,
					csc_stamp_10us,
					csc_stamp_TCNT2);
			ringBufferWaitAppend(isSend, false, mainCtxtBuffer, len);

			len = sprintf((char*) mainCtxtBuffer,
					"### PWM=%05u\n",
					pullPwmVal);
			ringBufferWaitAppend(isSend, false, mainCtxtBuffer, len);

			len = sprintf((char*) mainCtxtBuffer,
					"### ADC0=%04u (%0.4fmV)\n",
					ac_adc_ch[0],
					adc_ch0_volts);
			ringBufferWaitAppend(isSend, false, mainCtxtBuffer, len);

			len = sprintf((char*) mainCtxtBuffer,
					"### ADC1=%04u (%0.4fmV)\n",
					ac_adc_ch[1],
					adc_ch1_volts);
			ringBufferWaitAppend(isSend, false, mainCtxtBuffer, len);

			len = sprintf((char*) mainCtxtBuffer,
					"### Temp=%04u (%0.1fC).\n\n",
					ac_adc_ch[2],
					adc_ch2_C);
			ringBufferWaitAppend(isSend, false, mainCtxtBuffer, len);
		}
	}

	/* correct PWM with  +/- <value> */
	if (mainPwmTerminalAdj) {
		// make a big signed calculation
		int32_t pwmCalc = ((int32_t) pullPwmVal) + mainPwmTerminalAdj;
		mainPwmTerminalAdj = 0;

		// frame to uint16_t
		if (pwmCalc > 0xffff) {
			pwmCalc = 0xffff;
		} else if (pwmCalc < 0) {
			pwmCalc = 0;
		}

		// write back to the global variable
		pullPwmVal = (uint16_t) pwmCalc;

		// adjust PWM out
		clkPullPwm_fw_setRatio(pullPwmVal);
	}
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_main"), aligned(2)))
#endif
static void sendInitialHelp()
{
	ringBufferAppend(true, false, VM_COMMAND_HELP, sizeof(VM_COMMAND_HELP));
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_main"), aligned(2)))
#endif
void give_away(void)
{
    wdt_reset();

    usbPoll();
	usb_fw_sendInInterrupt();
	workInQueue();
	doJobs();


#if 0
	/* go into sleep mode */
	{
		clkPullPwm_fw_setPin(false);							// XXX for debugging purposes only

		cli();
		WDTCSR |= _BV(WDIE);
		sei();

		wdt_enable(WDTO_15MS);

		set_sleep_mode(SLEEP_MODE_IDLE);
		sleep_mode();											// wake-up by any interrupt or after 15 ms

		cli();
		WDTCSR &= ~(_BV(WDIE));
		sei();

		wdt_disable();
		clkPullPwm_fw_setPin(true);								// XXX for debugging purposes only
	}
#else
	/* due to the fact that the clkFastCtr interrupts every 12.8 µs there is no chance to power down */
	clkPullPwm_fw_togglePin();									// XXX for debugging purposes only
#endif
}


#ifdef RELEASE
__attribute__((section(".df4iah_fw_main"), aligned(2)))
#endif
int main(void)
{

	/* init AVR */
	{
		vectortable_to_firmware();
		wdt_init();

		PRR    = 0xEF;										// disable all modules within the Power Reduction Register
		ACSR  |= _BV(ACD);									// switch on Analog Comparator Disable
		DIDR1 |= (0b11 << AIN0D);							// disable digital input buffers on AIN0 and AIN1

		/* switch off Pull-Up Disable */
		MCUCR &= ~(_BV(PUD));

		clkPullPwm_fw_init();
		clkFastCtr_fw_init();
		anlgComp_fw_init();
		clkSlowCtr_fw_init();
		serial_fw_init();
		usb_fw_init();
		sei();

		/* read sensor coefficients */
		if (memory_fw_readEepromValidBlock(eepromBlockCopy, BLOCK_MEASURING_NR)) {
			eeprom_b01_t* b01 = (eeprom_b01_t*) &eepromBlockCopy;
			mainCoef_b01_ref_AREF_V					= b01->b01_ref_AREF_V;
			mainCoef_b01_ref_1V1_V					= b01->b01_ref_1V1_V;
			mainCoef_b01_temp_ofs_adc_25C_steps		= b01->b01_temp_ofs_adc_25C_steps;
			mainCoef_b01_temp_k_p1step_adc_K		= b01->b01_temp_k_p1step_adc_K;
		}

		/* enter HELP command in USB host OUT queue */
		sendInitialHelp();
	}

	/* run the chip */
    while (!mainStopAvr) {
    	give_away();
    }

    /* stop AVR */
    {
		cli();
		wdt_close();
		usb_fw_close();
		serial_fw_close();
		clkSlowCtr_fw_close();
		anlgComp_fw_close();
		clkFastCtr_fw_close();
		clkPullPwm_fw_close();

		// all pins are set to be input
		DDRB = 0x00;
		DDRC = 0x00;
		DDRD = 0x00;

		// all pull-ups are being switched off
		PORTB = 0x00;
		PORTC = 0x00;
		PORTD = 0x00;

		// switch off Pull-Up Disable
		MCUCR &= ~(_BV(PUD));

		if (mainEnterMode) {
			if (mainEnterMode == ENTER_MODE_BL) {
				/* write BOOT one time token to the EEPROM to INHIBIT restart into this Firmware again */
				uint16_t tokenVal = BOOT_TOKEN;
				memory_fw_writeEEpromPage((uint8_t*) &tokenVal, sizeof(tokenVal), offsetof(eeprom_layout_t, bootMarker));

				/* enter bootloader */
				mainJumpToBL();										// jump to bootloader section

			} else if (mainEnterMode == ENTER_MODE_FW) {
#if 1
				/* restart firmware */
				mainJumpToFW();										// jump to firmware section (REBOOT)
#else
				/* reset with the help of the WDT */
				wdt_enable(WDTO_250MS);
				for (;;) {
			        _delay_ms(1000);
				}
#endif
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
