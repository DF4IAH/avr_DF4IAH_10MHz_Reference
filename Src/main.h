/*
 * main.h
 *
 *  Created on: 01.11.2014
 *      Author: espero
 */

#ifndef MAIN_H_
#define MAIN_H_

/* MCU frequency */
#ifndef F_CPU
// #define F_CPU 7372800
#define F_CPU (7372800 / 2)
#endif

/* Device-Type:
   For AVRProg the BOOT-option is preferred
   which is the "correct" value for a bootloader.
   avrdude may only detect the part-code for ISP */
#define DEVTYPE     DEVTYPE_BOOT
// #define DEVTYPE     DEVTYPE_ISP

/*
 * Define if Watchdog-Timer should be disable at startup
 */
#define DISABLE_WDT_AT_STARTUP

/*
 * Watchdog-reset is issued at exit
 * define the timeout-value here (see avr-libc manual)
 */
#define EXIT_WDT_TIME	WDTO_250MS

/*
 * Select startup-mode
 * SIMPLE-Mode - Jump to bootloader main BL-loop if key is
 *   pressed (Pin grounded) "during" reset or jump to the
 *   application if the pin is not grounded. The internal
 *   pull-up resistor is enabled during the startup and
 *   gets disabled before the application is started.
 * POWERSAVE-Mode - Startup is separated in two loops
 *   which makes power-saving a little easier if no firmware
 *   is on the chip. Needs more memory
 * BOOTICE-Mode - to flash the JTAGICE upgrade.ebn file.
 *   No startup-sequence in this mode. Jump directly to the
 *   parser-loop on reset
 *   F_CPU in BOOTICEMODE must be 7372800 Hz to be compatible
 *   with the org. JTAGICE-Firmware
 * WAIT-mode waits 1 sec for the defined character if nothing
 *    is received then the user prog is started.
 */
#define START_SIMPLE
//#define START_WAIT
//#define START_POWERSAVE
//#define START_BOOTICE

/* character to start the bootloader in mode START_WAIT */
#define START_WAIT_UARTCHAR 'S'

/* wait-time for START_WAIT mode ( t = WAIT_TIME * 10ms ) */
#define WAIT_VALUE 100 /* here: 100*10ms = 1000ms = 1sec */


#if 1
# define USE_USB 1
#else
# define USE_SERIAL 1
#endif


#if defined(USE_SERIAL)
# define sendchar(x)	sendchar_serial(x)
# define recvchar()		recvchar_serial()
# define recvBuffer(x)	recvBuffer_serial(x)

#elif defined(USE_USB)
# define sendchar(x)	sendchar_usb(x)
# define recvchar()		recvchar_usb()
# define recvBuffer(x)	recvBuffer_usb(x)

#else
# error "At least one of USE_SERIAL or USE_USB has to be enabled"
#endif


#define VERSION_HIGH '0'
#define VERSION_LOW  '8'

#ifndef GET_LOCK_BITS
# define GET_LOCK_BITS           0x0001
# define GET_LOW_FUSE_BITS       0x0000
# define GET_HIGH_FUSE_BITS      0x0003
# define GET_EXTENDED_FUSE_BITS  0x0002
#endif


// STRINGS IN CODE SECTION
#define gcs_AVR_len  7
#define gcs_FDL_len  6
#define gcs_E99_len  8


void give_away(void);

int main(void);

#endif /* MAIN_H_ */
