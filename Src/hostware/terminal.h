/*
 * terminal.h
 *
 *  Created on: 15.12.2014
 *      Author: espero
 */

#ifndef TERMINAL_H_
#define TERMINAL_H_

#include <ncurses.h>


#ifdef DEBUG
# define RINGBUFFER_SEND_SIZE	5
# define RINGBUFFER_RCV_SIZE	5
# define RINGBUFFER_HOOK_SIZE	5
# define MSG_PATTERN_NMEA		'$'
typedef unsigned char  uchar;

enum RINGBUFFER_MSG_STATUS_t {
	RINGBUFFER_MSG_STATUS_AVAIL			= 0x01,
	RINGBUFFER_MSG_STATUS_IS_NMEA		= 0x10,
	RINGBUFFER_MSG_STATUS_IS_MASK		= 0xF0
};

#else

# define RINGBUFFER_SEND_SIZE	1024
# define RINGBUFFER_RCV_SIZE	1024

#endif

#define MSGBUFFER_SIZE			128


#define TEST_DATATRANSFER_SLOW								// activate on request
#define TEST_DATATRANSFER_USB_TEST2							// activate on request
// #define TEST_DATATRANSFER_PANEL							// activate on request
// #define TEST_DATATRANSFER_USB							// activate on request


/* -- 8< --  RINGBUFFERS */
int ringBufferPush(char isSend, uchar inData[], int len);
int ringBufferPull(char isSend, uchar outData[], int size);

// DEBUGGING
#ifdef DEBUG
uint8_t fw_getSemaphore(uint8_t isSend);
void fw_freeSemaphore(uint8_t isSend);

uint8_t fw_ringBufferPush(uint8_t isSend, const uchar inData[], uint8_t len);
void fw_ringBufferPushAddHook(uint8_t isSend, const uchar inData[], uint8_t len);
uint8_t fw_ringBufferPull(uint8_t isSend, uchar outData[], uint8_t size);
enum RINGBUFFER_MSG_STATUS_t fw_getStatusNextMsg(uint8_t isSend);
#endif


/* -- 8< --  USB */
//static void usb_controlOut(char inLine[], int len);
//static int usb_controlIn(char outLine[], int size);


/* -- 8< --  NCURSES */
//static void ncurses_init(WINDOW** win_rxborder, WINDOW** win_rx, WINDOW** win_tx);
//static void ncurses_finish(WINDOW** win_rxborder, WINDOW** win_rx, WINDOW** win_tx);
//static void ncurses_rx_print(WINDOW** win_rx, const char string[], int colorPair, attr_t attributes);
//static void ncurses_tx_clear(WINDOW** win_tx);
//static void ncurses_tx_print(WINDOW** win_tx, const char string[]);
//static void ncurses_update(WINDOW** win_rxborder, WINDOW** win_rx, WINDOW** win_tx);


/* -- 8< --  TERMINAL - LOOP */
void terminal();


#endif /* TERMINAL_H_ */
