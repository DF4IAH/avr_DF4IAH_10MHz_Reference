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
#define VERSION_LOW											109


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

#define PWM_HIST_COUNT										16
#define MAIN_CLOCK_DIFF_COUNT								3

#define DEBUG_DELAY_CNT										1


#ifndef true
# define true 1
#endif
#ifndef false
# define false 0
#endif


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
uint8_t calcTimerAdj(uint8_t* subVal, uint8_t intValBefore, float pwmAdjust);
float main_fw_calcPwmWghtDiff(float pwmDiff);
void  main_fw_calcPwmWghtAvg();
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
