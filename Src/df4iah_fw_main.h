/*
 * df4iah_fw_main.h
 *
 *  Created on: 01.11.2014
 *      Author: DF4IAH, Ulrich Habel
 */

#ifndef DF4IAH_FW_MAIN_H_
#define DF4IAH_FW_MAIN_H_


#include "usbdrv_fw/usbdrv.h"

/* VERSION: YYM, MDD */
#define VERSION_HIGH										150
#define VERSION_LOW											221


/* MCU frequency */
#ifndef F_CPU
// #define F_CPU 7372800
#define F_CPU (7372800 / 2)
#endif

/*
 * Define if Watchdog-Timer should be disable at startup
 */
#define DISABLE_WDT_AT_STARTUP

/*
 * Watchdog-reset is issued at exit
 * define the timeout-value here (see avr-libc manual)
 */
#define EXIT_WDT_TIME										WDTO_250MS

#define PWM_HIST_COUNT										8
#define MAIN_CLOCK_DIFF_COUNT								3

#define DEBUG_DELAY_CNT										1

#define MAIN_PREPARE_BUFFER_SIZE							128
#define MAIN_FORMAT_BUFFER_SIZE								128

// PHASE-ADC: 1.00V --> ADC-Value = 229 /1024 (Full-Scale = 4.47V)
#define ADC_PHASE_LO_LOCKING  								 20		// @<0.15V
#define ADC_PHASE_LO_INSYNC   								 80		// @ 0.35V
#define ADC_PHASE_CENTER									137		// @ 0.60V
#define ADC_PHASE_HI_INSYNC   								195		// @ 0.85V
#define ADC_PHASE_HI_LOCKING  								241		// @ 1.05V

#define MEAN_CLOCK_STAGES_F									  5.0f;
#define PWM_COR_STEPS_COARSE_DIV_F							  1.1f;
#define PWM_COR_STEPS_FINE_DIV_F							 10.0f;
#define CLOCK_DIFF_OUT										100l
#define CLOCK_DIFF_COARSE_FINE								 20l
#define CLOCK_DIFF_FAST_FRAME								  3
#define ADC_PHASE_PUSH_BORDER								  2


#ifndef true
# define true 1
#endif
#ifndef false
# define false 0
#endif


typedef struct main_bf_struct
{
     uint8_t  mainIsAFC										: 1;
     uint8_t  mainIsAPC										: 1;
     uint8_t  mainIsTimerTest								: 1;
     uint8_t  mainIsSerComm									: 1;
     uint8_t  mainIsUsbCommTest 							: 1;
     uint8_t  mainStopAvr		 							: 1;
     uint8_t  mainEnterMode									: 2;
     //uint8_t												: 0; // fill to 8 bits

     uint8_t  mainHelpConcatNr								: 4;
     uint8_t												: 4; // fill to 8 bits
} main_bf_t;


enum REFCLK_STATE_t {
	REFCLK_STATE_NOSYNC										= 0b0000,
	REFCLK_STATE_SEARCH_QRG									= 0b0001,
	REFCLK_STATE_SEARCH_PHASE								= 0b0010,
	REFCLK_STATE_SEARCH_PHASE_CNTR_STABLIZED				= 0b0011,
	REFCLK_STATE_LOCKING_PHASE								= 0b0111,
	REFCLK_STATE_IN_SYNC									= 0b1111
};

enum ENTER_MODE_t {
	ENTER_MODE_SLEEP 										= 0,
	ENTER_MODE_BL,
	ENTER_MODE_FW
};


float main_fw_calcTimerToFloat(uint8_t subVal, uint8_t intVal);
uint8_t calcTimerAdj(float pwmAdjust, uint8_t intValBefore, uint8_t* subVal);
float main_fw_calcPwmWghtDiff(float pwmDiff);
void  main_fw_calcPwmWghtAvg();
int   main_fw_strncmp(const unsigned char* msg, const unsigned char* cmpProg, size_t size);
int   main_fw_memcmp(const unsigned char* msg, const unsigned char* cmpProg, size_t size);
void  main_fw_sendInitialHelp();
void  main_fw_giveAway(void);
int   main(void);


// this following table has most of the instructions listed

/*TODO
 * x	Mnemonics	clocks	resulting clocks
 * ------------------------------------------------
 * 8	push		2		16
 * 1	in			1		 1
 * 1	eor			1		 1
 * 2	lds			2		 4
 * 1	sei			1		 1
 * 2	ld (Z)		2		 4
 * 2	ldi			2		 4
 * 1	adiw		2		 2
 * 2	adc			1		 2
 * 2	sts			2		 4
 *
 * = 23 clocks --> 1.15 µs until sei() is done
 */

#endif /* DF4IAH_FW_MAIN_H_ */
