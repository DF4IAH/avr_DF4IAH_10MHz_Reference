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
#include "df4iah_fw_usb.h"
#include "df4iah_bl_memory.h"
#include "df4iah_bl_clkPullPwm.h"
#include "df4iah_fw_memory.h"
#include "df4iah_fw_memory_eepromData.h"
#include "df4iah_fw_ringbuffer.h"
#include "df4iah_fw_clkPullPwm.h"
#include "df4iah_fw_clkFastCtr.h"
#include "df4iah_fw_anlgComp.h"
#include "df4iah_fw_serial.h"

#include "df4iah_fw_main.h"


#ifndef BOOT_TOKEN											// make Eclipse happy: should be included from chipdef.h --> mega32.h
# define BOOT_TOKEN											0xb00f
#endif
#ifndef DEFAULT_PWM_COUNT									// make Eclipse happy: should be included from chipdef.h --> mega32.h
# define DEFAULT_PWM_COUNT									90
#endif


// STRINGS IN MEMORY SECTION


// STRINGS IN CODE SECTION

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-braces"
PROGMEM const eeprom_defaultValues_layout_t eeprom_defaultValues_content = {
		EEPROM_DEFAULT_CONTENT_B00,
		EEPROM_DEFAULT_CONTENT_B01,
		EEPROM_DEFAULT_CONTENT_B02,
		EEPROM_DEFAULT_CONTENT_B03,
		EEPROM_DEFAULT_CONTENT_B04
};
#pragma GCC diagnostic pop

// PROGMEM const char PM_VENDOR[] 							= "DF4IAH";
// const uint8_t PM_VENDOR_len = sizeof(PM_VENDOR);

PROGMEM const uchar PM_COMMAND_AFCOFF[]						= "AFCOFF";
PROGMEM const uchar PM_COMMAND_AFCON[]						= "AFCON";
PROGMEM const uchar PM_COMMAND_APCOFF[]						= "APCOFF";
PROGMEM const uchar PM_COMMAND_APCON[]						= "APCON";
PROGMEM const uchar PM_COMMAND_HALT[]						= "HALT";
PROGMEM const uchar PM_COMMAND_HELP[]						= "HELP";
PROGMEM const uchar PM_COMMAND_INFO[]						= "INFO";
PROGMEM const uchar PM_COMMAND_LOADER[]						= "LOADER";
PROGMEM const uchar PM_COMMAND_REBOOT[]						= "REBOOT";
PROGMEM const uchar PM_COMMAND_SEROFF[]						= "SEROFF";
PROGMEM const uchar PM_COMMAND_SERON[]						= "SERON";
PROGMEM const uchar PM_COMMAND_SERBAUD[]					= "SERBAUD";
PROGMEM const uchar PM_COMMAND_STACK[]						= "STACK";
PROGMEM const uchar PM_COMMAND_TEST[]						= "TEST";
PROGMEM const uchar PM_COMMAND_WRITEPWM[]					= "WRITEPWM";
PROGMEM const uchar PM_COMMAND_PLUSSIGN[]					= "+";
PROGMEM const uchar PM_COMMAND_MINUSSIGN[]					= "-";

PROGMEM const uchar PM_GPIB_SCM_IDN[]						= "*IDN?";
PROGMEM const uchar PM_GPIB_SCM_RST[]						= "*RST";

PROGMEM const uchar PM_INTERPRETER_HELP01[]					= "\n=== HELP ===" \
															  "\n" \
															  "\n$ <NMEA-Message>\t\tsends message to the GPS module.";

PROGMEM const uchar PM_INTERPRETER_HELP02[] 				= "\nAFCOFF\t\t\t\tswitch AFC (automatic frequency control) off." \
															  "\nAFCON\t\t\t\tswitch AFC (automatic frequency control) on.";

PROGMEM const uchar PM_INTERPRETER_HELP03[] 				= "\nAPCOFF\t\t\t\tswitch APC (automatic phase control) off." \
															  "\nAPCON\t\t\t\tswitch APC (automatic phase control) on.";

PROGMEM const uchar PM_INTERPRETER_HELP04[] 				= "\nHALT\t\t\t\tpowers the device down (sleep mode).";

PROGMEM const uchar PM_INTERPRETER_HELP05[] 				= "\nHELP\t\t\t\tthis message.";

PROGMEM const uchar PM_INTERPRETER_HELP06[] 				= "\nINFO\t\t\t\ttoggles additional printed infos.";

PROGMEM const uchar PM_INTERPRETER_HELP07[] 				= "\nLOADER\t\t\t\tenter bootloader.";

PROGMEM const uchar PM_INTERPRETER_HELP08[] 				= "\nREBOOT\t\t\t\treboot the firmware.";

PROGMEM const uchar PM_INTERPRETER_HELP09[] 				= "\nSERBAUD <baud>\t\t\tsetting serial baud rate.";

PROGMEM const uchar PM_INTERPRETER_HELP10[] 				= "\nSEROFF\t\t\t\tswitch serial communication OFF." \
		  	  	  	  	  	  	  	  	  	  	  	  	  	  "\nSERON\t\t\t\tswitch serial communication ON.";

PROGMEM const uchar PM_INTERPRETER_HELP11[] 				= "\nSTACK\t\t\t\ttoggles stack mung-wall test.";

PROGMEM const uchar PM_INTERPRETER_HELP12[] 				= "\nTEST\t\t\t\ttoggles counter test.";

PROGMEM const uchar PM_INTERPRETER_HELP13[] 				= "\nWRITEPWM\t\t\tstore current PWM as default value.";

PROGMEM const uchar PM_INTERPRETER_HELP14[] 				= "\n+/- <PWM value>\t\tcorrection value to be added.";

PROGMEM const uchar PM_INTERPRETER_HELP15[] 				= "\n===========" \
		  	  	  	  	  	  	  	  	  	  	  	  	  	  "\n>";

PROGMEM const uchar PM_INTERPRETER_UNKNOWN[] 				= "\n*?*  unknown command '%s' received, try HELP." \
															  "\n>";

PROGMEM const uchar PM_FORMAT_VERSION[]						= "\n=== DF4IAH - 10 MHz Reference Oscillator ===\n=== Ver: 20%03d%03d";

PROGMEM const uchar PM_FORMAT_GPS_STBY[]					= "$PMTK161,0*28\r\n";

PROGMEM const uchar PM_FORMAT_TA00[]						= "\n#TA00: =======";
PROGMEM const uchar PM_FORMAT_TA01[]						= "#TA01: ADC0 = %04u (%0.3fV)\n";
PROGMEM const uchar PM_FORMAT_TA02[]						= "#TA02: ADC1 = %04u (%0.3fV)\n";
PROGMEM const uchar PM_FORMAT_TA03[]						= "#TA03: Temp = %04u (%0.1fC)\n";
PROGMEM const uchar PM_FORMAT_TA11[]						= "#TA11: localFastCtr1ms = %09lu, \tlocalFastTCNT1 = %05u\n";
PROGMEM const uchar PM_FORMAT_TA12[]						= "#TA12: ppsStampCtr1ms  = %09lu, \tppsStampICR1   = %05u, \tppsStampCtr1ms_last  = %09lu, \tppsStampICR1_last   = %05u\n";
PROGMEM const uchar PM_FORMAT_TA13[]						= "#TA13: PWM = %03u, \tSub-PWM = %03u\n";
PROGMEM const uchar PM_FORMAT_TA14[]						= "#TA14: mainRefClkState = %u\n";

PROGMEM const uchar PM_FORMAT_ID01[]						= "#ID01: +/- KEY \tmainPwmTerminalAdj = %f, \tpullPwmValBefore    = %03u + fastPwmSubCmpBefore    = %03u\n";
PROGMEM const uchar PM_FORMAT_ID02[]						= "#ID02: +/- KEY \tmainPwmTerminalAdj = %f, \tlocalFastPwmValNext = %03u + localFastPwmSubCmpNext = %03u\n";

PROGMEM const uchar PM_FORMAT_IA01[]						= "#IA01: Clock int20MHzClockDiff       =   %+04liHz @20MHz\n";
PROGMEM const uchar PM_FORMAT_IA02[]						= "#IA02: Clock localMeanFloatClockDiff = %+03.3fHz @20MHz, \tqrgDev_Hz = %+03.3fHz @10MHz, \tppm = %+02.6f\n";
PROGMEM const uchar PM_FORMAT_IA03[]						= "#IA03: QRG   newPwmVal = %03.3f, \tpwmCorSteps         = %+03.3f\n";
PROGMEM const uchar PM_FORMAT_IA11[]						= "#IA11: PHASE phaseErr  = %03.3f°, \t phaseStepsFrequency = %+03.3f, \tphaseStepsPhase = %+03.3f\n";
PROGMEM const uchar PM_FORMAT_IA12[]						= "#IA12: PHASE fastPwmSingleDiff_steps = %+03.3f\n";

PROGMEM const uchar PM_FORMAT_SC01[]						= "#SC01: Stack-Check: mung-wall address: 0x%04x, lowest-stack: 0x%04x\n";

PROGMEM const uchar PM_FORMAT_GPIB_SCM_IDN[] 				= "DF4IAH,%s,%05u,V20%03u%03u.";

PROGMEM const uchar PM_FORMAT_SET_BAUD[]					= "Communication baud rate set to %5u baud.\n";


// DATA SECTION

/* df4iah_fw_main */
void (*const mainJumpToFW)(void)							= (void*) 0x0000;
void (*const mainJumpToBL)(void)							= (void*) 0x7000;
uchar mainCoef_b00_dev_header[16 + 1]						= { 0 };
uint16_t mainCoef_b00_dev_serial							= 0;
uint16_t mainCoef_b00_dev_version							= 0;
float mainCoef_b01_ref_AREF_V								= 0.0f;
float mainCoef_b01_ref_1V1_V								= 0.0f;
float mainCoef_b01_temp_ofs_adc_25C_steps					= 0.0f;
float mainCoef_b01_temp_k_p1step_adc_K						= 0.0f;
float mainCoef_b02_qrg_ofs_minV_25C_ppm						= 0.0f;
float mainCoef_b02_qrg_ofs_maxV_25C_ppm						= 0.0f;
float mainCoef_b02_qrg_k_pPwmStep_25C_ppm					= 0.0f;
float mainCoef_b02_pwm_minV_V								= 0.0f;
float mainCoef_b02_pwm_maxV_V								= 0.0f;

uint8_t mainIsJumperBlSet									= false;
enum REFCLK_STATE_t mainRefClkState							= REFCLK_STATE_NOSYNC;
float mainPwmTerminalAdj									= 0.0f;
volatile uint8_t  timer0Snapshot 							= 0x00;
usbTxStatus_t usbTxStatus1 									= { 0 },
			  usbTxStatus3 									= { 0 };
uint16_t mainSCStackAddr									= 0x1fff;
uint16_t mainSCMungwallAddr									= 0x1fff;
uint32_t ppsStampCtr1ms 									= 0;
uint16_t ppsStampICR1 										= 0;
uint32_t ppsStampCtr1ms_last 								= 0;
uint16_t ppsStampICR1_last 									= 0;
/* bit fields */
main_bf_t main_bf											= {
									/* mainIsAFC			= */	false,
									/* mainIsAPC			= */	false,
									/* mainIsTimerTest		= */	false,
									/* mainIsSerComm		= */	false,
									/* mainIsUsbCommTest	= */	false,
									/* mainStopAvr			= */	false,
									/* mainStackCheck		= */	false,
									/* mainReserved01		= */	false,

									/* mainEnterMode		= */	ENTER_MODE_SLEEP,
									/* mainHelpConcatNr		= */	0
															  };

/* df4iah_fw_clkPullPwm */
uint8_t pullCoef_b02_pwm_initial							= 0;
uint8_t pullCoef_b02_pwm_initial_sub						= 0;

/* df4iah_fw_clkFastCtr (20 MHz clock) */
uint16_t fastStampTCNT1										= 0;
uint32_t fastStampCtr1ms									= 0;
uint32_t fastCtr1ms											= 0;

uint8_t  fastPwmLoopVal										= 0;
uint8_t  fastPwmSubLoopVal									= 0;
uint8_t	 fastPwmSingleLoad									= 0;
uint8_t  fastPwmSingleVal									= 0;
uint8_t  fastPwmSubSingleVal								= 0;
uint8_t  fastPwmSubCmp										= 0;
uint8_t  fastPwmSubCnt										= 0;
float    fastPwmSingleDiffSum									= 0.0f;
uint32_t fastPwmAdcNow										= 0;
uint32_t fastPwmAdcLast										= 0;
int16_t  fastPwmAdcAscendingVal								= 0;

/* df4iah_fw_anlgComp (PPS: 1 Hz GPS pulse) */
uint8_t  acAdcConvertNowState								= 0;
uint8_t  acAdcConvertNowCntr								= 0;

/* df4iah_fw_ringbuffer */
uint8_t  usbRingBufferSendPushIdx 							= 0;
uint8_t  usbRingBufferSendPullIdx 							= 0;
uint8_t  usbRingBufferRcvPushIdx 							= 0;
uint8_t  usbRingBufferRcvPullIdx 							= 0;
uint8_t  usbRingBufferSendSemaphore 						= 0;  // semaphore is free
uint8_t  usbRingBufferRcvSemaphore 							= 0;  // semaphore is free

/* df4iah_fw_serial */
uint16_t serialCoef_b03_serial_baud							= 0;
uint16_t serialCoef_b03_bitsParityStopbits					= 0;
uint16_t serialCoef_b03_gps_comm_mode						= 0;
uint8_t  serialCtxtRxBufferLen								= 0;
uint8_t  serialCtxtTxBufferLen								= 0;
uint8_t  serialCtxtTxBufferIdx								= 0;
uint8_t  serialCtxtBufferState								= 0;

/* df4iah_fw_usb */
//uint16_t usbSetupCntr										= 0;
uint16_t cntRcv 											= 0;
uint16_t cntSend 											= 0;
uint8_t  usbIsrCtxtBufferIdx 								= 0;


// ARRAYS - due to overwriting hazards they are following the controlling variables

/* df4iah_fw_main */
uchar mainInterpreterBuffer[MAIN_FORMAT_BUFFER_SIZE]		= { 0 };
uchar mainPrepareBuffer[MAIN_PREPARE_BUFFER_SIZE] 			= { 0 };
uchar mainFormatBuffer[MAIN_FORMAT_BUFFER_SIZE]				= { 0 };

/* df4iah_fw_clkFastCtr */

/* df4iah_fw_anlgComp (10kHz) */
uint16_t acAdcCh[AC_ADC_CH_COUNT + 1]						= { 0 };  // plus one for the temperature sensor

/* df4iah_fw_ringbuffer */
uchar usbRingBufferSend[RINGBUFFER_SEND_SIZE] 				= { 0 };
uchar usbRingBufferRcv[RINGBUFFER_RCV_SIZE] 				= { 0 };

/* df4iah_fw_serial */
uchar serialCtxtRxBuffer[SERIALCTXT_RX_BUFFER_SIZE] 		= { 0 };
uchar serialCtxtTxBuffer[SERIALCTXT_TX_BUFFER_SIZE] 		= { 0 };

/* df4iah_fw_usb */
uchar usbIsrCtxtBuffer[USBISRCTXT_BUFFER_SIZE] 				= { 0 };
uchar usbCtxtSetupReplyBuffer[USBSETUPCTXT_BUFFER_SIZE] 	= { 0 };

/* LAST IN RAM: Stack Check mung-wall */
uchar stackCheckMungWall[MAIN_STACK_CHECK_SIZE];			// XXX debugging purpose
// mung-wall memory array[0x0300] = 0x05b3 .. 0x08b2
// lowest stack:	0x085b
// mung-wall low:	0x0850
// --> RAM: free abt. 650 bytes
// --> ROM: free abt. 7kB (FW section only)

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
	wdt_disable();
}

static inline void wdt_close() {
	wdt_disable();
}

float main_fw_calcTimerToFloat(uint8_t intVal, uint8_t intSubVal)
{
	/* the fractional part depends on the bit count used for the sub-PWM */
	return intVal + (intSubVal / 256.0f);
}

float main_fw_calcTimerAdj(float pwmAdjust, uint8_t* intVal, uint8_t* intSubVal)
{
	float fltTime = main_fw_calcTimerToFloat(*intVal, *intSubVal);
	float residue = 0.0f;
	const float maxLimit = 255.0f - (1.0f / (1 << FAST_PWM_SUB_BITCNT));	// NOT 256.0f due to the compare unit that catches not around the overflow value

	/* add the time offset to the current PWM settings */
	fltTime += pwmAdjust;

	/* windowing the next value into the 8-Bit range */
	if (fltTime < 0.0f) {
		residue = fltTime;
		fltTime = 0.0f;

	} else if (fltTime > maxLimit) {
		residue = fltTime - maxLimit;
		fltTime = maxLimit;
	}

	/* add rounding value */
	fltTime += 1.0f / 512.0f;

	/* break up into integer and fractional parts */
	*intVal		= (uint8_t) fltTime;
	*intSubVal	= (uint8_t) ((fltTime - floorf(fltTime)) * 256.0f);
	return residue;
}

static void main_fw_calcQrg(int32_t int20MHzClockDiff, float meanFloatClockDiff, float qrgDev_Hz, float ppm)
{
	/* frequency shift calculation */
	uint8_t localIsOffset = false;

	if (mainRefClkState <= REFCLK_STATE_SEARCH_PHASE_CNTR_STABLIZED) {
		/* Help APC to find its phase - when found, stop offset */
		meanFloatClockDiff += 0.1f;							// 0.1 Hz @ 20 MHz below center frequency to let the phase wander and
															// the phase locker find its position to lock in
		localIsOffset = true;
	}

	if ((-1000.0f < meanFloatClockDiff) &&
		( 1000.0f > meanFloatClockDiff)) {
		/* keep measuring window between +/-50ppm */
		ppm -=  (localIsOffset ?  0.005f : 0.0f);			// correct the clock offset

		float pwmCorSteps;
		if ((-CLOCK_DIFF_FAST_FRAME <= int20MHzClockDiff) && (int20MHzClockDiff <= CLOCK_DIFF_FAST_FRAME)) {
			/* fine pitching */
			pwmCorSteps = ((-meanFloatClockDiff / 20.0f) / mainCoef_b02_qrg_k_pPwmStep_25C_ppm) / PWM_COR_STEPS_FINE_DIV_F;

		} else {
			/* coarse pitching */
			pwmCorSteps = ((((float) -int20MHzClockDiff) / 20.0f) / mainCoef_b02_qrg_k_pPwmStep_25C_ppm) / PWM_COR_STEPS_COARSE_DIV_F;  // XXX used also in main_fw_calcPhase()
			mainRefClkState = REFCLK_STATE_NOSYNC;
		}

		/* determine the new state of the FSM */
		if ((-0.015f <= ppm) && (ppm <= 0.015f)) {  // single step tuning with counter stabilizer
			if (mainRefClkState < REFCLK_STATE_SEARCH_PHASE_CNTR_STABLIZED) {
				/* Upgrading: switch on the frequency mean value counter */
				mainRefClkState = REFCLK_STATE_SEARCH_PHASE_CNTR_STABLIZED;
			}
		} else if ((-0.025f <= ppm) && (ppm <= 0.025f)) {  // single step tuning
			if (mainRefClkState < REFCLK_STATE_SEARCH_PHASE) {
				/* Upgrading: hand-over to the phase lock loop */
				mainRefClkState = REFCLK_STATE_SEARCH_PHASE;

			} else if (mainRefClkState <= REFCLK_STATE_SEARCH_PHASE_CNTR_STABLIZED) {
				/* Downgrading: to shaky for the mean value counter */
				//mainRefClkState = REFCLK_STATE_SEARCH_PHASE;
			}

		} else if ((-0.040f <= ppm) && (ppm <= 0.040f)) {  // phase lock loop locked out again
			if (mainRefClkState > REFCLK_STATE_SEARCH_QRG) {
				/* Downgrading: frequency search and lock loop entering QRG area */
				//mainRefClkState = REFCLK_STATE_SEARCH_QRG;
			}

		} else if ((-25.0f <= ppm) && (ppm <= 25.0f)) {	 // entering 10.0 MHz area
			if (mainRefClkState < REFCLK_STATE_SEARCH_QRG) {
				/* Upgrading: frequency search and lock loop entering QRG area */
				mainRefClkState = REFCLK_STATE_SEARCH_QRG;
			} else if (mainRefClkState > REFCLK_STATE_SEARCH_QRG) {
				/* Downgrading: to shaky for the mean value counter */
				//mainRefClkState = REFCLK_STATE_SEARCH_QRG;
			}

		} else {
			/* frequency search and lock loop - out if sync */
			mainRefClkState = REFCLK_STATE_NOSYNC;
		}

		/* windowing and adding of the new PWM value */
		cli();
		uint8_t localFastPwmLoopVal		= fastPwmLoopVal;
		uint8_t localFastPwmSubLoopVal	= fastPwmSubLoopVal;
		sei();

		if (mainRefClkState <= REFCLK_STATE_SEARCH_PHASE_CNTR_STABLIZED) {
			/* adjusting the PWM registers and make the new value public - only when hand-over to Phase Correction is not made */
			(void) main_fw_calcTimerAdj(pwmCorSteps, &localFastPwmLoopVal, &localFastPwmSubLoopVal);

			cli();
			fastPwmLoopVal		= localFastPwmLoopVal;
			fastPwmSubLoopVal	= localFastPwmSubLoopVal;
			sei();
		}

		if (main_bf.mainIsTimerTest) {
			/* monitoring */
			int len;
			memory_fw_copyBuffer(true, mainFormatBuffer, PM_FORMAT_IA01, sizeof(PM_FORMAT_IA01));
			len = sprintf((char*) mainPrepareBuffer, (char*) mainFormatBuffer,
					int20MHzClockDiff);
			ringbuffer_fw_ringBufferWaitAppend(false, false, mainPrepareBuffer, len);

			memory_fw_copyBuffer(true, mainFormatBuffer, PM_FORMAT_IA02, sizeof(PM_FORMAT_IA02));
			len = sprintf((char*) mainPrepareBuffer, (char*) mainFormatBuffer,
					meanFloatClockDiff,
					qrgDev_Hz,
					ppm);
			ringbuffer_fw_ringBufferWaitAppend(false, false, mainPrepareBuffer, len);

			memory_fw_copyBuffer(true, mainFormatBuffer, PM_FORMAT_IA03, sizeof(PM_FORMAT_IA03));
			len = sprintf((char*) mainPrepareBuffer, (char*) mainFormatBuffer,
					main_fw_calcTimerToFloat(localFastPwmLoopVal, localFastPwmSubLoopVal),
					pwmCorSteps);
			ringbuffer_fw_ringBufferWaitAppend(false, false, mainPrepareBuffer, len);
		}

	} else {
		/* frequency search and lock loop - out if sync */
		// mainRefClkState = REFCLK_STATE_NOSYNC;  // single spike should not destroy time base - deactivated
	}
}

// forward declaration
static void main_fw_calcPhaseResidue();
static void main_fw_calcPhase()
{
	static float phaseMeanPhaseErrorSum = 0.0f;
	uint8_t adcPhase = acAdcCh[ADC_CH_PHASE];

	/* APC = automatic phase control */
	if (mainRefClkState >= REFCLK_STATE_SEARCH_PHASE_CNTR_STABLIZED) {
		if ((ADC_PHASE_LO_LOCKING <= adcPhase) && (adcPhase <= ADC_PHASE_HI_LOCKING)) {
			if (mainRefClkState < REFCLK_STATE_LOCKING_PHASE) {
				/* up-grading */
				mainRefClkState = REFCLK_STATE_LOCKING_PHASE;
				phaseMeanPhaseErrorSum	= 0.0f;

				uint8_t sreg = SREG;
				cli();
				fastPwmSingleDiffSum = 0.0f;
				SREG = sreg;

			} else if (mainRefClkState > REFCLK_STATE_LOCKING_PHASE) {
				/* down-grading */
				mainRefClkState = REFCLK_STATE_LOCKING_PHASE;
			}

			if ((ADC_PHASE_LO_INSYNC <= adcPhase) && (adcPhase <= ADC_PHASE_HI_INSYNC)) {
				if (mainRefClkState < REFCLK_STATE_IN_SYNC) {
					mainRefClkState = REFCLK_STATE_IN_SYNC;
				}
			}

		} else {
			/* lost phase: hand-over to AFC */
			if (mainRefClkState >= REFCLK_STATE_LOCKING_PHASE) {
				mainRefClkState = REFCLK_STATE_SEARCH_QRG;
				phaseMeanPhaseErrorSum	= 0.0f;

				uint8_t sreg = SREG;
				cli();
				fastPwmSingleDiffSum = 0.0f;
				SREG = sreg;
			}
		}
	}

	const float PhaseErrAdc		= 450.0f / (((float) ADC_PHASE_HI_LOCKING) - ADC_PHASE_LO_LOCKING);
	float phaseErr				= PhaseErrAdc * (adcPhase - ADC_PHASE_CENTER);
	float phaseStepsPhase		= 0.0f;
	float phaseStepsFrequency	= 0.0f;

	if (mainRefClkState >= REFCLK_STATE_LOCKING_PHASE) {
		/* phase correction */
		phaseStepsPhase = (float) (pow(fabs(phaseErr) * 45.00f, 1.25f));  // magic values
		if (phaseErr < 0.0f) {
			phaseStepsPhase = -phaseStepsPhase;
		}

		if (!fastPwmSingleDiffSum) {
			uint8_t sreg = SREG;
			cli();
			fastPwmSingleDiffSum += phaseStepsPhase;						// phase offset accumulator
			SREG = sreg;

			main_fw_calcPhaseResidue();									// first call - to be called many times during the whole second until next pulse comes
		}
	}

	if (mainRefClkState >= REFCLK_STATE_LOCKING_PHASE) {
		/* frequency drift correction */
		float phaseMeanPhaseErrorDiff = phaseMeanPhaseErrorSum / MEAN_PHASE_CLOCK_STAGES_F;
		phaseMeanPhaseErrorSum += (((float) phaseStepsPhase) - phaseMeanPhaseErrorDiff);
		phaseStepsFrequency = phaseMeanPhaseErrorSum * 0.0000003f; 	// magic value

		cli();
		uint8_t localFastPwmLoopVal		= fastPwmLoopVal;
		uint8_t localFastPwmSubLoopVal	= fastPwmSubLoopVal;
		sei();

		(void) main_fw_calcTimerAdj(phaseStepsFrequency, &localFastPwmLoopVal, &localFastPwmSubLoopVal);

		uint8_t sreg = SREG;
		cli();
		fastPwmLoopVal		= localFastPwmLoopVal;					// single frequency correction
		fastPwmSubLoopVal	= localFastPwmSubLoopVal;
		SREG = sreg;
	}

	if (main_bf.mainIsTimerTest) {
		/* monitoring */
		int len;
		memory_fw_copyBuffer(true, mainFormatBuffer, PM_FORMAT_IA11, sizeof(PM_FORMAT_IA11));
		len = sprintf((char*) mainPrepareBuffer, (char*) mainFormatBuffer,
				phaseErr,
				phaseStepsFrequency,
				phaseStepsPhase);
		ringbuffer_fw_ringBufferWaitAppend(false, false, mainPrepareBuffer, len);

		memory_fw_copyBuffer(true, mainFormatBuffer, PM_FORMAT_IA12, sizeof(PM_FORMAT_IA12));
		len = sprintf((char*) mainPrepareBuffer, (char*) mainFormatBuffer,
				fastPwmSingleDiffSum);
		ringbuffer_fw_ringBufferWaitAppend(false, false, mainPrepareBuffer, len);
	}
}

static void main_fw_calcPhaseResidue()
{
	uint8_t localFastPwmSingleLoad;
	uint8_t localFastPwmSingleVal;
	uint8_t localFastPwmSubSingleVal;

	uint8_t sreg = SREG;
	cli();
	localFastPwmSingleLoad = fastPwmSingleLoad;
	SREG = sreg;

	if (fastPwmSingleDiffSum && (!localFastPwmSingleLoad)) {  // enter only if an offset is accumulated and the last phase correction is loaded
		/* set PWM corrected impulse for one PWM cycle - no frequency corrections done */
		sreg = SREG;
		cli();
		localFastPwmSingleVal		= fastPwmLoopVal;		// make a copy from the current loop settings
		localFastPwmSubSingleVal	= fastPwmSubLoopVal;
		SREG = sreg;

		/* calculation with saturation and residue */
		fastPwmSingleDiffSum = main_fw_calcTimerAdj(fastPwmSingleDiffSum, &localFastPwmSingleVal, &localFastPwmSubSingleVal);

		/* single phase correction */
		sreg = SREG;
		cli();
		fastPwmSingleVal	= localFastPwmSingleVal;
		fastPwmSubSingleVal	= localFastPwmSubSingleVal;
		fastPwmSingleLoad	= true;
		SREG = sreg;
	}
}

int main_fw_strncmp(const unsigned char* msg, const unsigned char* cmpProg, size_t size)
{
	memory_fw_copyBuffer(true, mainFormatBuffer, cmpProg, size);
	return strncmp((const char*) msg, (const char*) mainFormatBuffer, size);
}

int main_fw_memcmp(const unsigned char* msg, const unsigned char* cmpProg, size_t size)
{
	memory_fw_copyBuffer(true, mainFormatBuffer, cmpProg, size);
	return memcmp((const char*) msg, (const char*) mainFormatBuffer, size);
}

static void doInterpret(uchar msg[], uint8_t len)
{
	if (!main_fw_strncmp(msg, PM_GPIB_SCM_IDN, sizeof(PM_GPIB_SCM_IDN))) {
		/* GPIB commands - SCPI/SCM - *IDN? */
		memory_fw_copyBuffer(true, mainFormatBuffer, PM_FORMAT_GPIB_SCM_IDN, sizeof(PM_FORMAT_GPIB_SCM_IDN));
		int len = sprintf((char*) mainPrepareBuffer, (char*) mainFormatBuffer,
				&(mainCoef_b00_dev_header[0]),
				mainCoef_b00_dev_serial,
				mainCoef_b00_dev_version >> 8,
				mainCoef_b00_dev_version & 0xff);
		ringbuffer_fw_ringBufferWaitAppend(false, false, mainPrepareBuffer, len);

	} else if (!main_fw_strncmp(msg, PM_COMMAND_AFCOFF, sizeof(PM_COMMAND_AFCOFF))) {
		/* automatic frequency control OFF */
		main_bf.mainIsAFC = false;

	} else if (!main_fw_strncmp(msg, PM_COMMAND_AFCON, sizeof(PM_COMMAND_AFCON))) {
		/* automatic frequency control ON */
		main_bf.mainIsAFC = true;

	} else if (!main_fw_strncmp(msg, PM_COMMAND_APCOFF, sizeof(PM_COMMAND_APCOFF))) {
		/* automatic phase control OFF */
		main_bf.mainIsAPC = false;

	} else if (!main_fw_strncmp(msg, PM_COMMAND_APCON, sizeof(PM_COMMAND_APCON))) {
		/* automatic phase control ON */
		main_bf.mainIsAPC = true;

	} else if (!main_fw_strncmp(msg, PM_COMMAND_HALT, sizeof(PM_COMMAND_HALT))) {
		/* stop AVR controller and enter sleep state */
		uint8_t cnt = 250;
		main_bf.mainIsTimerTest = false;
		main_bf.mainIsUsbCommTest = false;
		main_bf.mainIsSerComm = true;
		serial_fw_copyAndSendNmea(true, PM_FORMAT_GPS_STBY, sizeof(PM_FORMAT_GPS_STBY));

		/* send message until the send buffer is clear */
		while (serial_fw_isTxRunning()) {
			wdt_reset();
			usbPoll();
		};

		/* give some time for the GPS module before powering down */
		while (--cnt) {
			wdt_reset();
			usbPoll();
			_delay_ms(1);
		}

		main_bf.mainIsSerComm = false;
		main_bf.mainEnterMode = ENTER_MODE_SLEEP;
		main_bf.mainStopAvr = true;

	} else if (!main_fw_strncmp(msg, PM_COMMAND_HELP, sizeof(PM_COMMAND_HELP))) {
		/* help information */
		memory_fw_copyBuffer(true, mainFormatBuffer, PM_FORMAT_VERSION, sizeof(PM_FORMAT_VERSION));
		int len = snprintf((char*) mainPrepareBuffer, sizeof(mainPrepareBuffer) - 1, (char*) mainFormatBuffer, VERSION_HIGH, VERSION_LOW);
		ringbuffer_fw_ringBufferWaitAppend(false, false, mainPrepareBuffer, len);
		ringbuffer_fw_ringBufferWaitAppend(false, true, PM_INTERPRETER_HELP01, sizeof(PM_INTERPRETER_HELP01));
		main_bf.mainHelpConcatNr = 1;
		main_bf.mainIsUsbCommTest = false;

	} else if (!main_fw_strncmp(msg, PM_COMMAND_INFO, sizeof(PM_COMMAND_INFO))) {
		/* timer 2 overflow counter TEST */
		main_bf.mainIsTimerTest = !(main_bf.mainIsTimerTest);
		if (main_bf.mainIsTimerTest) {
			main_bf.mainIsSerComm = false;
			main_bf.mainIsUsbCommTest = false;
		}

	} else if (!main_fw_strncmp(msg, PM_COMMAND_LOADER, sizeof(PM_COMMAND_LOADER))) {
		/* enter bootloader */
		main_bf.mainIsSerComm = false;
		main_bf.mainIsTimerTest = false;
		main_bf.mainIsUsbCommTest = false;
		main_bf.mainEnterMode = ENTER_MODE_BL;
		main_bf.mainStopAvr = true;

	} else if ((!main_fw_strncmp(msg, PM_COMMAND_REBOOT, sizeof(PM_COMMAND_REBOOT))) ||
			   (!main_fw_strncmp(msg, PM_GPIB_SCM_RST, sizeof(PM_GPIB_SCM_RST)))) {
		/* enter firmware (REBOOT) */
		main_bf.mainIsSerComm = false;
		main_bf.mainIsTimerTest = false;
		main_bf.mainIsUsbCommTest = false;
		main_bf.mainEnterMode = ENTER_MODE_FW;
		main_bf.mainStopAvr = true;

	} else if (!main_fw_strncmp(msg, PM_COMMAND_SEROFF, sizeof(PM_COMMAND_SEROFF))) {
		/* serial communication OFF */
		main_bf.mainIsSerComm = false;

	} else if (!main_fw_strncmp(msg, PM_COMMAND_SERON, sizeof(PM_COMMAND_SERON))) {
		/* serial communication ON */
		main_bf.mainIsSerComm = true;
		main_bf.mainIsTimerTest = false;
		main_bf.mainIsUsbCommTest = false;

	} else if (!main_fw_memcmp(msg, PM_COMMAND_SERBAUD, sizeof(PM_COMMAND_SERBAUD) - 1)) {
		/* serial communication baud parameter */
		sscanf((char*) msg + sizeof(PM_COMMAND_SERBAUD), "%d", &serialCoef_b03_serial_baud);
		serial_fw_setCommBaud(serialCoef_b03_serial_baud);

		/* write current baud rate as the default/startup value to the EEPROM */
		memory_fw_writeEEpromPage((uint8_t*) &serialCoef_b03_serial_baud, sizeof(uint16_t), offsetof(eeprom_layout_t, b03.b03_serial_baud));

		/* for any modified block add the CRC seal marker and do a  memory_fw_checkAndInitBlock() */
		uint16_t newCrc = memory_fw_getSealMarker(BLOCK_GPS_NR);
		memory_fw_writeEEpromPage((uint8_t*) &newCrc, sizeof(uint16_t), offsetof(eeprom_layout_t, b03.b03_crc));
		memory_fw_checkAndInitBlock(BLOCK_GPS_NR);

		/* user information */
		memory_fw_copyBuffer(true, mainFormatBuffer, PM_FORMAT_SET_BAUD, sizeof(PM_FORMAT_SET_BAUD));
		len = sprintf((char*) mainPrepareBuffer, (char*) mainFormatBuffer,
				serialCoef_b03_serial_baud);
		ringbuffer_fw_ringBufferWaitAppend(false, false, mainPrepareBuffer, len);

	} else if (!main_fw_strncmp(msg, PM_COMMAND_STACK, sizeof(PM_COMMAND_STACK))) {
		/* Stack Check facility */
		main_bf.mainStackCheck = !(main_bf.mainStackCheck);

	} else if (!main_fw_strncmp(msg, PM_COMMAND_TEST, sizeof(PM_COMMAND_TEST))) {
		/* special communication TEST */
		main_bf.mainIsUsbCommTest = !(main_bf.mainIsUsbCommTest);
		if (main_bf.mainIsUsbCommTest) {
			main_bf.mainIsSerComm = false;
			main_bf.mainIsTimerTest = false;
		}

	} else if (!main_fw_strncmp(msg, PM_COMMAND_WRITEPWM, sizeof(PM_COMMAND_WRITEPWM))) {
		/* write current PWM value as the default/startup value to the EEPROM */
		cli();
		pullCoef_b02_pwm_initial		= fastPwmLoopVal;
		pullCoef_b02_pwm_initial_sub	= fastPwmSubLoopVal;
		sei();

		memory_fw_writeEEpromPage((uint8_t*) &pullCoef_b02_pwm_initial, sizeof(uint8_t), offsetof(eeprom_layout_t, b02.b02_pwm_initial));
		memory_fw_writeEEpromPage((uint8_t*) &pullCoef_b02_pwm_initial_sub, sizeof(uint8_t), offsetof(eeprom_layout_t, b02.b02_pwm_initial_sub));

		/* for any modified block add the CRC seal marker and do a  memory_fw_checkAndInitBlock() */
		uint16_t newCrc = memory_fw_getSealMarker(BLOCK_REFOSC_NR);
		memory_fw_writeEEpromPage((uint8_t*) &newCrc, sizeof(uint16_t), offsetof(eeprom_layout_t, b02.b02_crc));
		memory_fw_checkAndInitBlock(BLOCK_REFOSC_NR);

	} else if (msg[0] == PM_COMMAND_PLUSSIGN[0]) {
		/* correct the PWM value up */
		sscanf(((char*) msg) + 1, "%f", &mainPwmTerminalAdj);

	} else if (msg[0] == PM_COMMAND_MINUSSIGN[0]) {
		/* correct the PWM value down */
		sscanf(((char*) msg) + 1, "%f", &mainPwmTerminalAdj);
		mainPwmTerminalAdj = -mainPwmTerminalAdj;

	} else {
		/* unknown command */
		memory_fw_copyBuffer(true, mainFormatBuffer, PM_INTERPRETER_UNKNOWN, sizeof(PM_INTERPRETER_UNKNOWN));
		int len = snprintf((char*) mainPrepareBuffer, sizeof(mainPrepareBuffer) - 1, (char*) mainFormatBuffer, msg);
		ringbuffer_fw_ringBufferWaitAppend(false, false, mainPrepareBuffer, len);
	}
}

static void workInQueue()
{
	if (ringbuffer_fw_getSemaphore(true)) {
		uint8_t isLocked = true;
		enum RINGBUFFER_MSG_STATUS_t statusSend = ringbuffer_fw_getStatusNextMsg(true);
		enum RINGBUFFER_MSG_STATUS_t statusRcv  = ringbuffer_fw_getStatusNextMsg(false);

		if (!(main_bf.mainHelpConcatNr) && (statusSend & RINGBUFFER_MSG_STATUS_AVAIL)) {		// if any message is available and not during help printing
			if (statusSend & RINGBUFFER_MSG_STATUS_IS_NMEA) {
				serial_fw_pullAndSendNmea_havingSemaphore(true); isLocked = false;

			} else if ((statusSend & RINGBUFFER_MSG_STATUS_IS_MASK) == 0) {						// message from firmware state machine
				uint8_t localMsgLen = ringbuffer_fw_ringBufferPull(true, mainInterpreterBuffer, (uint8_t) (sizeof(mainInterpreterBuffer) - 1));
				ringbuffer_fw_freeSemaphore(true); isLocked = false;

#if 0
				/* debugging */
				{
					const char localReceivedInfo[] = "Got message (length=%d): '%s'\n";
					int len = snprintf((char*) mainPrepareBuffer, sizeof(mainPrepareBuffer) - 1, localReceivedInfo, localMsgLen, mainInterpreterBuffer);
					ringbuffer_fw_ringBufferWaitAppend(false, false, mainPrepareBuffer, len);
				}
#endif

				doInterpret(mainInterpreterBuffer, localMsgLen);								// message is clean to process
			}

		} else if (main_bf.mainHelpConcatNr && !(statusRcv & RINGBUFFER_MSG_STATUS_AVAIL)) {	// during help printing, go ahead when receive buffer is empty again
			ringbuffer_fw_freeSemaphore(true); isLocked = false;

			switch (main_bf.mainHelpConcatNr) {
			case 1:
				ringbuffer_fw_ringBufferWaitAppend(false, true, PM_INTERPRETER_HELP02, sizeof(PM_INTERPRETER_HELP02));
				main_bf.mainHelpConcatNr = 2;
				break;

			case 2:
				ringbuffer_fw_ringBufferWaitAppend(false, true, PM_INTERPRETER_HELP03, sizeof(PM_INTERPRETER_HELP03));
				main_bf.mainHelpConcatNr = 3;
				break;

			case 3:
				ringbuffer_fw_ringBufferWaitAppend(false, true, PM_INTERPRETER_HELP04, sizeof(PM_INTERPRETER_HELP04));
				main_bf.mainHelpConcatNr = 4;
				break;

			case 4:
				ringbuffer_fw_ringBufferWaitAppend(false, true, PM_INTERPRETER_HELP05, sizeof(PM_INTERPRETER_HELP05));
				main_bf.mainHelpConcatNr = 5;
				break;

			case 5:
				ringbuffer_fw_ringBufferWaitAppend(false, true, PM_INTERPRETER_HELP06, sizeof(PM_INTERPRETER_HELP06));
				main_bf.mainHelpConcatNr = 6;
				break;

			case 6:
				ringbuffer_fw_ringBufferWaitAppend(false, true, PM_INTERPRETER_HELP07, sizeof(PM_INTERPRETER_HELP07));
				main_bf.mainHelpConcatNr = 7;
				break;

			case 7:
				ringbuffer_fw_ringBufferWaitAppend(false, true, PM_INTERPRETER_HELP08, sizeof(PM_INTERPRETER_HELP08));
				main_bf.mainHelpConcatNr = 8;
				break;

			case 8:
				ringbuffer_fw_ringBufferWaitAppend(false, true, PM_INTERPRETER_HELP09, sizeof(PM_INTERPRETER_HELP09));
				main_bf.mainHelpConcatNr = 9;
				break;

			case 9:
				ringbuffer_fw_ringBufferWaitAppend(false, true, PM_INTERPRETER_HELP10, sizeof(PM_INTERPRETER_HELP10));
				main_bf.mainHelpConcatNr = 10;
				break;

			case 10:
				ringbuffer_fw_ringBufferWaitAppend(false, true, PM_INTERPRETER_HELP11, sizeof(PM_INTERPRETER_HELP11));
				main_bf.mainHelpConcatNr = 11;
				break;

			case 11:
				ringbuffer_fw_ringBufferWaitAppend(false, true, PM_INTERPRETER_HELP12, sizeof(PM_INTERPRETER_HELP12));
				main_bf.mainHelpConcatNr = 12;
				break;

			case 12:
				ringbuffer_fw_ringBufferWaitAppend(false, true, PM_INTERPRETER_HELP13, sizeof(PM_INTERPRETER_HELP13));
				main_bf.mainHelpConcatNr = 13;
				break;

			case 13:
				ringbuffer_fw_ringBufferWaitAppend(false, true, PM_INTERPRETER_HELP14, sizeof(PM_INTERPRETER_HELP14));
				main_bf.mainHelpConcatNr = 14;
				break;

			case 14:
				ringbuffer_fw_ringBufferWaitAppend(false, true, PM_INTERPRETER_HELP15, sizeof(PM_INTERPRETER_HELP15));
				// no break
			default:
				main_bf.mainHelpConcatNr = 0;
				break;
			}
		}

		if (isLocked) {
			ringbuffer_fw_freeSemaphore(true);
		}
	}
}

static void doJobs()
{
	const uint16_t LocalCtr1sSpanMs = 1000 * DEBUG_DELAY_CNT;// wake up every DEBUG_DELAY_CNT second
	const uint8_t  LocalCtr250msBorderMs = 250;				// 250 ms time span
	static uint8_t localAdcConvertNowCntrLast = 0;
	static uint32_t localFastCtr1ms_next = 0;
	uint32_t localFastCtr1ms;
	uint16_t localFastTCNT1;
	uint16_t localStampCtr1ms;
	uint16_t localStampICR1;
	uint8_t len = 0;
	uint8_t localPpsReceived = false;

	{
		/* get the timers */
		cli();

		/* get the current ms and ticks timer */
		uint8_t localTCNT1L = TCNT1L;						// low byte first
		uint8_t localTCNT1H = TCNT1H;
		localFastCtr1ms = fastCtr1ms;						// make a copy of the running clock

		/* get the last stamped time as ms and ticks */
		uint8_t localICR1L = ICR1L;							// low byte first
		uint8_t localICR1H = ICR1H;
		localStampCtr1ms = fastStampCtr1ms;					// make a copy of the captured / timestamped clock

		sei();

		localFastTCNT1	= localTCNT1L | (localTCNT1H << 8);
		localStampICR1	= localICR1L  | (localICR1H  << 8);
	}

	if (localAdcConvertNowCntrLast != acAdcConvertNowCntr) {  // a new PPS impulse has arrived
		localAdcConvertNowCntrLast = acAdcConvertNowCntr;
		localPpsReceived = true;

		ppsStampCtr1ms_last = ppsStampCtr1ms;
		ppsStampICR1_last   = ppsStampICR1;

		ppsStampCtr1ms = localStampCtr1ms;
		ppsStampICR1   = localStampICR1;

		/* reload timer */
		localFastCtr1ms_next = localFastCtr1ms + LocalCtr1sSpanMs + LocalCtr250msBorderMs;

	} else if (localFastCtr1ms >= localFastCtr1ms_next) {  	//  the timer has elapsed without a PPS impulse
		if ((localFastCtr1ms_next + LocalCtr1sSpanMs) > localFastCtr1ms) {
			/* adjust */
			localFastCtr1ms_next += LocalCtr1sSpanMs;		// +1 second

		} else {
			/* reload / initial timer */
			localFastCtr1ms_next = localFastCtr1ms + LocalCtr1sSpanMs + LocalCtr250msBorderMs;
		}

	} else {
		/* nothing has happened - do some bulk data if a job is ready to be done */

		if (serialCtxtBufferState == SERIAL_CTXT_BUFFER_STATE_SEND) {
			ringbuffer_fw_ringBufferWaitAppend(false, false, serialCtxtRxBuffer, serialCtxtRxBufferLen);
			serialCtxtRxBufferLen = 0;

			/* mark the serial buffer as to be ready again to receive GPS data */
			serialCtxtBufferState = 0;
		}

		/* PWM offset due to phase accumulator */
		main_fw_calcPhaseResidue();

		/* leave here */
		return;
	}

	/* HERE: every second once a second */

	if (main_bf.mainStackCheck) {
		/* do a Stack Check when active */
		for (int idx = 0; idx < MAIN_STACK_CHECK_SIZE; ++idx) {
			if (stackCheckMungWall[idx] != 0x5a) {
				uint16_t localCheckAddr = (uint16_t) (&(stackCheckMungWall[idx]));
				if (mainSCMungwallAddr > localCheckAddr) {
					mainSCMungwallAddr = localCheckAddr;
				}

				/* leave loop body */
				break;
			}
		}

		memory_fw_copyBuffer(true, mainFormatBuffer, PM_FORMAT_SC01, sizeof(PM_FORMAT_SC01));
		len = sprintf((char*) mainPrepareBuffer, (char*) mainFormatBuffer,
				mainSCMungwallAddr,
				mainSCStackAddr);
		ringbuffer_fw_ringBufferWaitAppend(false, false, mainPrepareBuffer, len);
	}

	/*
	 * ATTENTION: Floating vfprint() and friends needs changes to the linker
	 * @see http://winavr.scienceprog.com/avr-gcc-tutorial/using-sprintf-function-for-float-numbers-in-avr-gcc.html
	 *
	 * 1)	Linker option:		--Wl,-u,vfprintf  --Wl,-u,vfscanf
	 * 2)	Linker libraries:	-lm  -lprintf_flt  -lscanf_flt
	 */

	if (main_bf.mainIsTimerTest) {
		/* print timer values */

		memory_fw_copyBuffer(true, mainPrepareBuffer, PM_FORMAT_TA00, sizeof(PM_FORMAT_TA00) + 1);
		ringbuffer_fw_ringBufferWaitAppend(false, false, mainPrepareBuffer, sizeof(PM_FORMAT_TA00) + 1);

		if (localPpsReceived) {
			float adcCh0Volts = ( acAdcCh[ADC_CH_PWMPULL] * mainCoef_b01_ref_AREF_V) / 1024.0f;
			float adcCh1Volts = ( acAdcCh[ADC_CH_PHASE]	  * mainCoef_b01_ref_AREF_V) / 1024.0f;
			float adcCh2C     = ((acAdcCh[ADC_CH_TEMP]	  - mainCoef_b01_temp_ofs_adc_25C_steps) * mainCoef_b01_temp_k_p1step_adc_K) + 25.0f;

			/* print ADC values - only valid when a PPS has arrived */

			memory_fw_copyBuffer(true, mainFormatBuffer, PM_FORMAT_TA01, sizeof(PM_FORMAT_TA01));
			len = sprintf((char*) mainPrepareBuffer, (char*) mainFormatBuffer,
					acAdcCh[ADC_CH_PWMPULL],
					adcCh0Volts);
			ringbuffer_fw_ringBufferWaitAppend(false, false, mainPrepareBuffer, len);

			memory_fw_copyBuffer(true, mainFormatBuffer, PM_FORMAT_TA02, sizeof(PM_FORMAT_TA02));
			len = sprintf((char*) mainPrepareBuffer, (char*) mainFormatBuffer,
					acAdcCh[ADC_CH_PHASE],
					adcCh1Volts);
			ringbuffer_fw_ringBufferWaitAppend(false, false, mainPrepareBuffer, len);

			memory_fw_copyBuffer(true, mainFormatBuffer, PM_FORMAT_TA03, sizeof(PM_FORMAT_TA03));
			len = sprintf((char*) mainPrepareBuffer, (char*) mainFormatBuffer,
					acAdcCh[ADC_CH_TEMP],
					adcCh2C);
			ringbuffer_fw_ringBufferWaitAppend(false, false, mainPrepareBuffer, len);
		}

		memory_fw_copyBuffer(true, mainFormatBuffer, PM_FORMAT_TA11, sizeof(PM_FORMAT_TA11));
		len = sprintf((char*) mainPrepareBuffer, (char*) mainFormatBuffer,
				localFastCtr1ms,
				localFastTCNT1);
		ringbuffer_fw_ringBufferWaitAppend(false, false, mainPrepareBuffer, len);

		memory_fw_copyBuffer(true, mainFormatBuffer, PM_FORMAT_TA12, sizeof(PM_FORMAT_TA12));
		len = sprintf((char*) mainPrepareBuffer, (char*) mainFormatBuffer,
				ppsStampCtr1ms,
				ppsStampICR1,
				ppsStampCtr1ms_last,
				ppsStampICR1_last);
		ringbuffer_fw_ringBufferWaitAppend(false, false, mainPrepareBuffer, len);

		memory_fw_copyBuffer(true, mainFormatBuffer, PM_FORMAT_TA13, sizeof(PM_FORMAT_TA13));
		len = sprintf((char*) mainPrepareBuffer, (char*) mainFormatBuffer,
				fastPwmLoopVal,
				fastPwmSubLoopVal);
		ringbuffer_fw_ringBufferWaitAppend(false, false, mainPrepareBuffer, len);

		memory_fw_copyBuffer(true, mainFormatBuffer, PM_FORMAT_TA14, sizeof(PM_FORMAT_TA14));
		len = sprintf((char*) mainPrepareBuffer, (char*) mainFormatBuffer,
				mainRefClkState);
		ringbuffer_fw_ringBufferWaitAppend(false, false, mainPrepareBuffer, len);
	}

	if (localPpsReceived) {
		/* PPS - 1 Hz Ref.-Clk. - State Machine */

		/* central calculations */
		static float localMeanClockDiffSum = 0.0f;
		float qrgDev_Hz;
		float ppm;
		int32_t local20MHzClockDiff =   (20000L * (((int32_t) ppsStampCtr1ms) - ((int32_t) ppsStampCtr1ms_last)))
							    	  +           (((int32_t) ppsStampICR1)   - ((int32_t) ppsStampICR1_last))
							    	  -  20000000L;

		float localMeanFloatClockDiff = localMeanClockDiffSum / MEAN_QRG_CLOCK_STAGES_F;
		if ((local20MHzClockDiff < -CLOCK_DIFF_OUT) || (CLOCK_DIFF_OUT < local20MHzClockDiff)) {
			/* bad value - ignore */
			local20MHzClockDiff = 0;
			qrgDev_Hz = (localMeanFloatClockDiff / 2.0f);
			ppm = (localMeanFloatClockDiff / 20.0f);

		} else if ((-CLOCK_DIFF_COARSE_FINE < local20MHzClockDiff) && (local20MHzClockDiff < CLOCK_DIFF_COARSE_FINE)) {
			/* fine mode */
			localMeanClockDiffSum += (((float) local20MHzClockDiff) - localMeanFloatClockDiff);
			qrgDev_Hz = (localMeanFloatClockDiff / 2.0f);
			ppm = (localMeanFloatClockDiff / 20.0f);

		} else {
			/* re-init the mean value sum when being in coarse mode */
			localMeanClockDiffSum = 0.0f;
			qrgDev_Hz = (local20MHzClockDiff / 2.0f);
			ppm = (local20MHzClockDiff / 20.0f);
		}

		if (main_bf.mainIsTimerTest && (!main_bf.mainIsAFC)) {
			/* monitoring frequency even when AFC is switched off */
			memory_fw_copyBuffer(true, mainFormatBuffer, PM_FORMAT_IA01, sizeof(PM_FORMAT_IA01));
			int len = sprintf((char*) mainPrepareBuffer, (char*) mainFormatBuffer,
					local20MHzClockDiff);
			ringbuffer_fw_ringBufferWaitAppend(false, false, mainPrepareBuffer, len);

			memory_fw_copyBuffer(true, mainFormatBuffer, PM_FORMAT_IA02, sizeof(PM_FORMAT_IA02));
			len = sprintf((char*) mainPrepareBuffer, (char*) mainFormatBuffer,
					localMeanFloatClockDiff,
					qrgDev_Hz,
					ppm);
			ringbuffer_fw_ringBufferWaitAppend(false, false, mainPrepareBuffer, len);
		}


		/* frequency & phase correction modules */

		if (main_bf.mainIsAFC) {
			/* AFC = automatic frequency calculation */
			main_fw_calcQrg(local20MHzClockDiff, localMeanFloatClockDiff, qrgDev_Hz, ppm);
		}

		if (main_bf.mainIsAPC) {
			/* APC = automatic phase control */
			main_fw_calcPhase();
		}
	}

	if (mainPwmTerminalAdj) {
		/* Manual PWM correction */

		uint8_t localFastPwmLoopValBefore;
		uint8_t localFastPwmSubLoopValBefore;
		uint8_t localFastPwmSubLoopValNext;
		uint8_t localFastPwmLoopValNext;

		{
			/* correct PWM with  +/- <value> */
			cli();
			localFastPwmLoopValBefore		= fastPwmLoopVal;
			localFastPwmSubLoopValBefore	= fastPwmSubLoopVal;
			sei();

			/* calculate next value */
			localFastPwmLoopValNext		= localFastPwmLoopValBefore;
			localFastPwmSubLoopValNext	= localFastPwmSubLoopValBefore;
			(void) main_fw_calcTimerAdj(mainPwmTerminalAdj, &localFastPwmLoopValNext, &localFastPwmSubLoopValNext);

			/* write back the global variables for PWM and sub-PWM */
			cli();
			fastPwmLoopVal		= localFastPwmLoopValNext;
			fastPwmSubLoopVal	= localFastPwmSubLoopValNext;
			sei();
		}

		memory_fw_copyBuffer(true, mainFormatBuffer, PM_FORMAT_ID01, sizeof(PM_FORMAT_ID01));
		uint8_t len = sprintf((char*) mainPrepareBuffer, (char*) mainFormatBuffer,
				mainPwmTerminalAdj,
				localFastPwmLoopValBefore,
				localFastPwmSubLoopValBefore);
		ringbuffer_fw_ringBufferWaitAppend(false, false, mainPrepareBuffer, len);

		memory_fw_copyBuffer(true, mainFormatBuffer, PM_FORMAT_ID02, sizeof(PM_FORMAT_ID02));
		len = sprintf((char*) mainPrepareBuffer, (char*) mainFormatBuffer,
				mainPwmTerminalAdj,
				localFastPwmLoopValNext,
				localFastPwmSubLoopValNext);
		ringbuffer_fw_ringBufferWaitAppend(false, false, mainPrepareBuffer, len);

		// reset data entry
		mainPwmTerminalAdj = 0.0f;
	}
}

void main_fw_sendInitialHelp()
{
	ringbuffer_fw_ringBufferWaitAppend(true, true, PM_COMMAND_HELP, sizeof(PM_COMMAND_HELP));
}

void main_fw_giveAway(void)
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

	clkPullPwm_bl_togglePin();									// XXX for debugging purposes only
#endif
}


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

		// Stack Check init
		for (int idx = MAIN_STACK_CHECK_SIZE; idx;) {
			stackCheckMungWall[--idx] = 0x5a;
		}

		clkPullPwm_fw_init();
		clkFastCtr_fw_init();
		anlgComp_fw_init();
		serial_fw_init();
		usb_fw_init();
		sei();

		/* check CRC of all blocks and update with default values if the data is non-valid */
		memory_fw_checkAndInitAllBlocks();

		/* read MEASURING coefficients */
		if (memory_fw_readEepromValidBlock(mainFormatBuffer, BLOCK_HEADER_NR)) {
			eeprom_b00_t* b00 = (eeprom_b00_t*) &mainFormatBuffer;
			memcpy(mainCoef_b00_dev_header, b00->b00_header, sizeof(mainCoef_b00_dev_header) - 1);
			mainCoef_b00_dev_header[sizeof(mainCoef_b00_dev_header) - 1] = 0;

			mainCoef_b00_dev_serial					= b00->b00_device_serial;
			mainCoef_b00_dev_version				= b00->b00_version;
		}

		/* read MEASURING coefficients */
		if (memory_fw_readEepromValidBlock(mainFormatBuffer, BLOCK_MEASURING_NR)) {
			eeprom_b01_t* b01 = (eeprom_b01_t*) &mainFormatBuffer;
			mainCoef_b01_ref_AREF_V					= b01->b01_ref_AREF_V;
			mainCoef_b01_ref_1V1_V					= b01->b01_ref_1V1_V;
			mainCoef_b01_temp_ofs_adc_25C_steps		= b01->b01_temp_ofs_adc_25C_steps;
			mainCoef_b01_temp_k_p1step_adc_K		= b01->b01_temp_k_p1step_adc_K;
		}

		/* read REFERENCE OSCILLATOR (REFOSC) coefficients */
		if (memory_fw_readEepromValidBlock(mainFormatBuffer, BLOCK_REFOSC_NR)) {
			eeprom_b02_t* b02 = (eeprom_b02_t*) &mainFormatBuffer;
			mainCoef_b02_qrg_ofs_minV_25C_ppm		= b02->b02_qrg_ofs_minV_25C_ppm;
			mainCoef_b02_qrg_ofs_maxV_25C_ppm		= b02->b02_qrg_ofs_maxV_25C_ppm;
			mainCoef_b02_qrg_k_pPwmStep_25C_ppm		= b02->b02_qrg_k_pPwmStep_25C_ppm;
			mainCoef_b02_pwm_minV_V					= b02->b02_pwm_minV_V;
			mainCoef_b02_pwm_maxV_V					= b02->b02_pwm_maxV_V;

			/*	b02_pwm_initial			treated by df4iah_fw_clkPullPwm */
			/* 	b02_pwm_initial_sub		treated by df4iah_fw_clkPullPwm */
		}

		/* enter HELP command in USB host OUT queue */
		main_fw_sendInitialHelp();
	}

	/* run the chip */
    while (!(main_bf.mainStopAvr)) {
    	main_fw_giveAway();
    }

    /* stop AVR */
    {
		cli();

		wdt_close();
		usb_fw_close();
		serial_fw_close();
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

		if (main_bf.mainEnterMode) {
			if (main_bf.mainEnterMode == ENTER_MODE_BL) {
				/* write BOOT one time token to the EEPROM to INHIBIT restart into this Firmware again */
				uint16_t tokenVal = BOOT_TOKEN;
				memory_fw_writeEEpromPage((uint8_t*) &tokenVal, sizeof(tokenVal), offsetof(eeprom_layout_t, bootMarker));

				/* enter bootloader */
				mainJumpToBL();										// jump to bootloader section

			} else if (main_bf.mainEnterMode == ENTER_MODE_FW) {
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
				//	sei();
					sleep_cpu();
					sleep_disable();
				// }
				sei();
			}
		}
    }
	return 0;
}
