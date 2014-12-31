/*
 * df4iah_fw_main.h
 *
 *  Created on: 01.11.2014
 *      Author: DF4IAH, Ulrich Habel
 */

#ifndef DF4IAH_FW_MAIN_H_
#define DF4IAH_FW_MAIN_H_

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

#define DEBUG_DELAY_CNT										1

#define VERSION_HIGH										141
#define VERSION_LOW											231


#ifndef true
# define true 1
#endif
#ifndef false
# define false 0
#endif


enum REFCLK_STATE_t {
	REFCLK_STATE_NOSYNC										= 0,
	REFCLK_STATE_SEARCH_QRG									= 0b0010,
	REFCLK_STATE_SEARCH_PHASE								= 0b0011,
	REFCLK_STATE_LOCKED_PHASE								= 0b0111,
	REFCLK_STATE_SYNC										= 0b1111
};

enum ENTER_MODE_t {
	ENTER_MODE_SLEEP 										= 0,
	ENTER_MODE_BL,
	ENTER_MODE_FW
};


void give_away(void);
int main(void);


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