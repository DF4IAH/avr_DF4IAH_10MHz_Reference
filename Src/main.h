/*
 * main.h
 *
 *  Created on: 01.11.2014
 *      Author: espero
 */

#ifndef FW_MAIN_H_
#define FW_MAIN_H_

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
#define EXIT_WDT_TIME	WDTO_250MS


#define VERSION_HIGH '141'
#define VERSION_LOW  '204'


int main(void);

#endif /* FW_MAIN_H_ */
